#include "proximity_warning.h"
#include "platform.h"
#include "gpio.h"
#include "gpt.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/***********************************************************
 *                Static Variables
 ***********************************************************/
static radar_entry_t radar_data[MAX_CSV_ENTRIES];
static uint32_t radar_data_count = 0;
static uint32_t data_index = 0;

static gpt_struct *pwm_left, *pwm_right;
static uint32_t pwm_period = PWM_PERIOD;

static int ip_present = 0;   // 1 = hardware IP detected, 0 = fallback mode

// Fallback mode state variables (matching Verilog)
static uint32_t base_duty;
static uint32_t duty_left, duty_right;
static uint32_t alert_active;
static uint32_t buzzer_active;
static uint32_t status_led_state;
static uint32_t steer_state;        // 0=straight,1=left,2=right
static uint32_t lc_state;           // 0=straight,1=turn,2=hold,3=correct,4=done
static uint32_t lc_timer;
static uint32_t lc_reduce;
static uint32_t steer_dir;          // 0=turn left, 1=turn right

// Timing
static uint32_t system_tick = 0;

/***********************************************************
 *             IP Register Access (with error handling)
 ***********************************************************/
void ip_write_reg(uint32_t offset, uint32_t value)
{
    if (!ip_present) return;
    volatile uint32_t *reg = (volatile uint32_t *)(IP_BASE + offset);
    *reg = value;
}

uint32_t ip_read_reg(uint32_t offset)
{
    if (!ip_present) return 0;
    volatile uint32_t *reg = (volatile uint32_t *)(IP_BASE + offset);
    return *reg;
}

/***********************************************************
 *                  GPIO
 ***********************************************************/
void gpio_init(void)
{
    uint32_t dir = read_word(GPIO_DIRECTION_CNTRL_REG);
    dir |= (LED_ALERT_PIN | LED_STATUS_PIN | BUZZER_PIN |
            IN1_L_PIN | IN2_L_PIN | IN1_R_PIN | IN2_R_PIN);
    write_word(GPIO_DIRECTION_CNTRL_REG, dir);

    write_gpio(LED_ALERT_PIN, 0);
    write_gpio(LED_STATUS_PIN, 0);
    write_gpio(BUZZER_PIN, 0);
    write_gpio(IN1_L_PIN, 0);
    write_gpio(IN2_L_PIN, 0);
    write_gpio(IN1_R_PIN, 0);
    write_gpio(IN2_R_PIN, 0);
}

void write_gpio(unsigned long gpio_pin, int value)
{
    uint32_t data = read_word(GPIO_DATA_REG);
    if (value)
        write_word(GPIO_DATA_REG, data | gpio_pin);
    else
        write_word(GPIO_DATA_REG, data & ~gpio_pin);
}

/***********************************************************
 *                  PWM (GPT) – for fallback mode
 ***********************************************************/
void pwm_init_left(uint8_t module, uint32_t period, uint32_t duty)
{
    gpt_init();
    pwm_left = gpt_get_instance(module);
    pwm_left->gpt_clk_control = GPT_CLK_EN | 0x2000;
    gpt_configure_ctrl_reg(module, pwm, 0, 0, 0);
    pwm_left->gpt_count_reference = period;
    pwm_left->gpt_compare = duty;
    gpt_start(pwm_left);
}

void pwm_init_right(uint8_t module, uint32_t period, uint32_t duty)
{
    pwm_right = gpt_get_instance(module);
    pwm_right->gpt_clk_control = GPT_CLK_EN | 0x2000;
    gpt_configure_ctrl_reg(module, pwm, 0, 0, 0);
    pwm_right->gpt_count_reference = period;
    pwm_right->gpt_compare = duty;
    gpt_start(pwm_right);
}

void pwm_set_duty_left(uint32_t duty)
{
    if (duty > pwm_period) duty = pwm_period;
    pwm_left->gpt_compare = duty;
}

void pwm_set_duty_right(uint32_t duty)
{
    if (duty > pwm_period) duty = pwm_period;
    pwm_right->gpt_compare = duty;
}

/***********************************************************
 *                  Delay and Timing
 ***********************************************************/
void delay_ms(uint32_t ms)
{
    for(uint32_t i = 0; i < ms * 1000; i++)
        asm volatile("nop");
}

uint32_t get_tick_ms(void)
{
    return system_tick;
}

/***********************************************************
 *                  Simple Pseudo‑random Generator
 ***********************************************************/
static uint32_t rand_state = 1;
uint32_t simple_rand(void)
{
    rand_state = rand_state * 1103515245 + 12345;
    return (rand_state / 65536) % 32768;
}

int32_t simple_rand_signed(void)
{
    uint32_t r = simple_rand();
    return (r & 0x8000) ? -(int32_t)(r & 0x7FFF) : (int32_t)r;
}

/***********************************************************
 *                  Example Data Loader (fallback)
 ***********************************************************/
void load_example_data(void)
{
    radar_data_count = 500;
    int32_t prev_dist = 500; // cm
    for (uint32_t i = 0; i < 500; i++) {
        radar_data[i].distance_cm = 50 + (simple_rand() % 951);
        radar_data[i].rpm = 400 + (simple_rand() % 1001);

        // Generate synthetic lateral position (x_mean) in Q16.16 metres
        int32_t x_m_q16 = (simple_rand_signed() % (65536 * 5)) / 2;
        radar_data[i].x_mean_q16 = x_m_q16;

        // Generate synthetic relative velocity (cm/s)
        int32_t delta = (int32_t)radar_data[i].distance_cm - prev_dist;
        radar_data[i].obj_vel_cms = delta * 10;
        prev_dist = radar_data[i].distance_cm;
    }
}

/***********************************************************
 *                  IP Detection
 ***********************************************************/
int detect_ip(void)
{
    volatile uint32_t *version_reg = (volatile uint32_t *)(IP_BASE + IP_VERSION);
    uint32_t version = *version_reg;
    if (version == 0x00030000) {
        return 1;
    }
    return 0;
}

/***********************************************************
 *          Fallback Logic (mirrors Verilog IP)
 ***********************************************************/
static void update_fallback_logic(uint32_t dist_cm, uint32_t rpm, int32_t x_mean_q16, int32_t obj_vel_cms)
{
    // 1. Compute base duty (velocity-aware)
    uint32_t obj_approaching = (obj_vel_cms < 0) && ((uint32_t)(-obj_vel_cms) > 50);
    if (rpm >= RPM_MOVE_THR) {
        if (dist_cm > RANGE_7M)       base_duty = DUTY_100;
        else if (dist_cm > RANGE_5M)  base_duty = obj_approaching ? DUTY_20 : DUTY_50;
        else if (dist_cm > RANGE_3M)  base_duty = obj_approaching ? DUTY_20 : DUTY_50;
        else                          base_duty = DUTY_20;
    } else if (rpm < RPM_IDLE_THR) {
        base_duty = DUTY_0;
    }

    // 2. Lane-change steering state machine
    int32_t x_abs = (x_mean_q16 < 0) ? -x_mean_q16 : x_mean_q16;
    int object_right = (x_mean_q16 > STEER_SOFT_THR);
    int object_left  = (x_mean_q16 < -STEER_SOFT_THR);
    int hard_steer   = (x_abs > STEER_HARD_THR);
    uint32_t reduce_amount = hard_steer ? (PWM_PERIOD * 40 / 100) : (PWM_PERIOD * 20 / 100);

    uint32_t alert_now = (dist_cm < RANGE_7M) ? 1 : 0; // simplified alert

    switch (lc_state) {
        case 0: // LC_STRAIGHT
            steer_state = 0;
            if (alert_now && rpm >= RPM_MOVE_THR && (object_right || object_left)) {
                steer_dir = object_right ? 0 : 1;
                lc_reduce = reduce_amount;
                lc_timer = TURN_DURATION_MS;
                lc_state = 1; // LC_TURN
                steer_state = object_right ? 1 : 2;
            }
            break;
        case 1: // LC_TURN
            if (lc_timer > 0) {
                lc_timer--;
            } else {
                lc_timer = HOLD_DURATION_MS;
                lc_state = 2; // LC_HOLD
                steer_state = 0;
            }
            break;
        case 2: // LC_HOLD
            if (lc_timer > 0) {
                lc_timer--;
            } else {
                lc_timer = CORRECT_DURATION_MS;
                lc_state = 3; // LC_CORRECT
                steer_state = steer_dir ? 1 : 2;
            }
            break;
        case 3: // LC_CORRECT
            if (lc_timer > 0) {
                lc_timer--;
            } else {
                lc_state = 4; // LC_DONE
                steer_state = 0;
            }
            break;
        case 4: // LC_DONE
            lc_state = 0;
            steer_state = 0;
            break;
    }

    // Compute left and right duties based on lane-change state
    duty_left = base_duty;
    duty_right = base_duty;
    if (lc_state == 1) { // TURN phase
        if (steer_dir == 0) {
            if (duty_right > lc_reduce) duty_right -= lc_reduce;
            else duty_right = 0;
        } else {
            if (duty_left > lc_reduce) duty_left -= lc_reduce;
            else duty_left = 0;
        }
    } else if (lc_state == 3) { // CORRECT phase
        if (steer_dir == 0) {
            if (duty_left > lc_reduce) duty_left -= lc_reduce;
            else duty_left = 0;
        } else {
            if (duty_right > lc_reduce) duty_right -= lc_reduce;
            else duty_right = 0;
        }
    }

    // 3. Buzzer pattern (zone and active flag)
    static uint32_t buzzer_timer = 0;
    uint32_t zone = 0;
    if (dist_cm > 0) {
        if (dist_cm > RANGE_7M) zone = 0;
        else if (dist_cm > RANGE_5M) zone = 1;
        else if (dist_cm > RANGE_3M) zone = 2;
        else zone = 3;
    }
    switch (zone) {
        case 0: buzzer_active = 0; break;
        case 1: buzzer_active = 1; break;
        case 2:
            if (buzzer_timer < BUZZ_DISCRETE_ON_MS) buzzer_active = 1;
            else buzzer_active = 0;
            if (++buzzer_timer >= (BUZZ_DISCRETE_ON_MS + BUZZ_DISCRETE_OFF_MS)) buzzer_timer = 0;
            break;
        case 3:
            if (buzzer_timer < BUZZ_RAPID_ON_MS) buzzer_active = 1;
            else buzzer_active = 0;
            if (++buzzer_timer >= (BUZZ_RAPID_ON_MS + BUZZ_RAPID_OFF_MS)) buzzer_timer = 0;
            break;
    }

    // 4. Status LED
    static uint32_t led_blink_timer = 0;
    if (alert_now) {
        status_led_state = 1;
    } else if (dist_cm > 0) {
        if (led_blink_timer < LED_BLINK_HALF_MS) status_led_state = 1;
        else status_led_state = 0;
        if (++led_blink_timer >= (LED_BLINK_HALF_MS * 2)) led_blink_timer = 0;
    } else {
        status_led_state = 0;
    }

    // 5. Motor direction pins (always forward unless braking)
    uint32_t brake = (base_duty == DUTY_0) && (rpm > 5);
    write_gpio(IN1_L_PIN, 1);
    write_gpio(IN2_L_PIN, brake ? 1 : 0);
    write_gpio(IN1_R_PIN, 1);
    write_gpio(IN2_R_PIN, brake ? 1 : 0);

    // Apply PWM duties
    pwm_set_duty_left(duty_left);
    pwm_set_duty_right(duty_right);

    // Update GPIO indicators
    write_gpio(LED_ALERT_PIN, alert_now);
    write_gpio(LED_STATUS_PIN, status_led_state);
}

/***********************************************************
 *                       MAIN
 ***********************************************************/
int main(void)
{
    printf("\n=== Proximity Warning System v3.0 (Dual-Motor) ===\n");

    ip_present = detect_ip();
    if (ip_present) {
        printf("[INFO] ADAS IP detected at 0x%08X (version 0x%08X)\n",
               IP_BASE, ip_read_reg(IP_VERSION));
        // Hardware mode (unchanged)
        ip_write_reg(IP_CTRL, 0x04);
        ip_write_reg(IP_THRESH_PED, 300);
        ip_write_reg(IP_THRESH_VEH, 600);
        ip_write_reg(IP_THRESH_STA, 150);
        ip_write_reg(IP_RPM_THRESH, (90 << 16) | 5);
        ip_write_reg(IP_PWM_CTRL, (1 << 9) | (0 << 8) | 100);
        ip_write_reg(IP_BUZZER_CTRL, 0x00);
        ip_write_reg(IP_STEERING_THRESH, (52428 << 16) | 19660);
        delay_ms(100);
    } else {
        printf("[WARN] ADAS IP not found – running in software emulation mode.\n");
        gpio_init();
        pwm_init_left(0, PWM_PERIOD, 0);
        pwm_init_right(1, PWM_PERIOD, 0);
        load_example_data();
        write_gpio(IN1_L_PIN, 1);
        write_gpio(IN2_L_PIN, 0);
        write_gpio(IN1_R_PIN, 1);
        write_gpio(IN2_R_PIN, 0);
        lc_state = 0;
        base_duty = DUTY_0;
    }

    uint32_t loop_count = 0;
    while (1)
    {
        uint32_t obj_dist, hall_rpm;
        int32_t x_mean_q16, obj_vel_cms;

        if (ip_present) {
            // ---------- Hardware mode (unchanged) ----------
            uint32_t status = ip_read_reg(IP_STATUS);
            obj_dist   = ip_read_reg(IP_RANGE_CM) & 0xFFFF;
            hall_rpm   = ip_read_reg(IP_RPM) & 0xFFFF;
            uint32_t steer_state_reg = ip_read_reg(IP_STEERING_STATE);
            uint32_t ext_stat = ip_read_reg(IP_STATUS_EXT);

            printf("\n[Loop %lu] HW Mode: Range=%lu cm, RPM=%lu\n", loop_count, obj_dist, hall_rpm);
            printf("  Steering: LC_state=%lu dir=%lu act=%lu\n",
                   (steer_state_reg>>3)&0x1F, (steer_state_reg>>2)&1, steer_state_reg&3);
            printf("  PWM_L=%d PWM_R=%d Buzzer=%d Alert=%d\n",
                   (ext_stat>>4)&1, (ext_stat>>5)&1, (ext_stat>>3)&1, (ext_stat>>2)&1);
        } else {
            // ---------- Software emulation mode ----------
            obj_dist   = radar_data[data_index].distance_cm;
            hall_rpm   = radar_data[data_index].rpm;
            x_mean_q16 = radar_data[data_index].x_mean_q16;
            obj_vel_cms= radar_data[data_index].obj_vel_cms;
            data_index = (data_index + 1) % radar_data_count;

            update_fallback_logic(obj_dist, hall_rpm, x_mean_q16, obj_vel_cms);

            // Buzzer tone generation (software PWM)
            static uint32_t buzzer_tone_timer = 0;
            static uint32_t buzzer_tone_state = 0;

            // Compute buzzer zone for frequency selection
            uint32_t zone = 0;
            if (obj_dist > 0) {
                if (obj_dist > RANGE_7M) zone = 0;
                else if (obj_dist > RANGE_5M) zone = 1;
                else if (obj_dist > RANGE_3M) zone = 2;
                else zone = 3;
            }
            uint32_t freq_half_period = (zone == 3) ? (1000 / BUZZ_FREQ_4K / 2) : (1000 / BUZZ_FREQ_2K / 2);

            if (buzzer_active) {
                if (++buzzer_tone_timer >= freq_half_period) {
                    buzzer_tone_timer = 0;
                    buzzer_tone_state = !buzzer_tone_state;
                    write_gpio(BUZZER_PIN, buzzer_tone_state);
                }
            } else {
                write_gpio(BUZZER_PIN, 0);
                buzzer_tone_timer = 0;
                buzzer_tone_state = 0;
            }

            printf("\n[Loop %lu] SW Mode: Dist=%lu cm, RPM=%lu, x_mean=%d (Q16.16)\n",
                   loop_count, obj_dist, hall_rpm, x_mean_q16);
            printf("  BaseDuty=%lu, L_Duty=%lu, R_Duty=%lu, LC_state=%lu, Steer=%lu\n",
                   base_duty, duty_left, duty_right, lc_state, steer_state);
        }

        // Heartbeat LED toggle (separate from status LED)
        write_gpio(LED_STATUS_PIN, (loop_count & 1));
        loop_count++;

        delay_ms(100);
        system_tick += 100;
    }

    return 0;
}
