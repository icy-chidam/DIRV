#ifndef PROXIMITY_WARNING_H
#define PROXIMITY_WARNING_H

#include <stdint.h>

/* ============================================================================
 *  AXI Register Offsets (matches radar_adas_v5_axi4lite_v3.v)
 *  Base address from soc.defines: AXI4LiteBase = 0x00040600
 * ========================================================================== */
#define IP_BASE                 0x00040600

#define IP_CTRL                 0x000
#define IP_STATUS               0x004
#define IP_CLASS                0x008
#define IP_CONFIDENCE           0x00C
#define IP_RANGE_CM             0x010
#define IP_THRESH_PED           0x014
#define IP_THRESH_VEH           0x018
#define IP_THRESH_STA           0x01C
#define IP_IRQ_STATUS           0x020
#define IP_IRQ_ENABLE           0x024
#define IP_SCORE_PED            0x028
#define IP_SCORE_STA            0x02C
#define IP_SCORE_VEH            0x030
#define IP_VERSION              0x034
#define IP_ALERT_CLASS          0x038
#define IP_OBJ_VEL_CMS          0x03C

#define IP_PWM_CTRL             0x180
#define IP_BUZZER_CTRL          0x184
#define IP_RPM                  0x188
#define IP_RPM_THRESH           0x18C
#define IP_RANGE_ZONE           0x190
#define IP_STATUS_EXT           0x194
#define IP_STEERING_STATE       0x1B0
#define IP_STEERING_THRESH      0x1B4

/* ============================================================================
 *  GPIO Pin Definitions (used for manual motor control and indicators)
 * ========================================================================== */
#define LED_ALERT_PIN           (1 << 16)
#define LED_STATUS_PIN          (1 << 18)
#define BUZZER_PIN              (1 << 24)

// Dual motor direction pins (L298N)
#define IN1_L_PIN               (1 << 25)   // Left motor IN1
#define IN2_L_PIN               (1 << 26)   // Left motor IN2
#define IN1_R_PIN               (1 << 27)   // Right motor IN3 (if available)
#define IN2_R_PIN               (1 << 28)   // Right motor IN4 (if available)

/* ============================================================================
 *  Constants (matching Verilog) – MODIFIED FOR HIGH SPEED & FAST TURNING
 * ========================================================================== */
#define PWM_PERIOD              5000        // 20 kHz at 100 MHz

// Duty cycles – now using near‑maximum values for high speed
#define DUTY_100                5000        // 100% (max speed)
#define DUTY_80                 4500        // 90% (was 4000)
#define DUTY_60                 4000        // 80% (was 3000)
#define DUTY_50                 3500        // 70% (was 2500)
#define DUTY_20                 2500        // 50% (was 1000)
#define DUTY_0                  0

#define RANGE_7M                700
#define RANGE_5M                500
#define RANGE_3M                300

// Lower thresholds so the system always assumes the vehicle is moving
#define RPM_IDLE_THR            0           // was 5
#define RPM_MOVE_THR            1           // was 90 (now always active)

// Steering thresholds (Q16.16 metres) – unchanged
#define STEER_SOFT_THR          10000       // 0.30 m
#define STEER_HARD_THR          52428       // 0.80 m

// Lane-change durations (milliseconds) – unchanged
#define TURN_DURATION_MS        500
#define HOLD_DURATION_MS        300
#define CORRECT_DURATION_MS     300

// Buzzer patterns (ms) – unchanged
#define BUZZ_DISCRETE_ON_MS     250
#define BUZZ_DISCRETE_OFF_MS    250
#define BUZZ_RAPID_ON_MS        100
#define BUZZ_RAPID_OFF_MS       100

// Buzzer frequencies (Hz)
#define BUZZ_FREQ_2K            2000
#define BUZZ_FREQ_4K            4000

// Status LED blink half-period (ms)
#define LED_BLINK_HALF_MS       500

/* ============================================================================
 *  Data Structures
 * ========================================================================== */
#define MAX_CSV_ENTRIES         500

typedef struct {
    uint32_t distance_cm;       // object distance
    uint32_t rpm;               // vehicle speed
    int32_t  x_mean_q16;        // lateral position (Q16.16 metres)
    int32_t  obj_vel_cms;       // relative velocity (cm/s, signed)
} radar_entry_t;

/* ============================================================================
 *  Function Prototypes
 * ========================================================================== */
void gpio_init(void);
void write_gpio(unsigned long gpio_pin, int value);

void ip_write_reg(uint32_t offset, uint32_t value);
uint32_t ip_read_reg(uint32_t offset);

void pwm_init_left(uint8_t module, uint32_t period, uint32_t duty);
void pwm_init_right(uint8_t module, uint32_t period, uint32_t duty);
void pwm_set_duty_left(uint32_t duty);
void pwm_set_duty_right(uint32_t duty);

void delay_ms(uint32_t ms);
uint32_t get_tick_ms(void);

#endif // PROXIMITY_WARNING_H
