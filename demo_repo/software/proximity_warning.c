/*************************
 * Project        : Proximity Warning and Speed Control
 * File           : proximity_warning.c
 * Description    : Main application – loads radar & RPM data from CSV file
 *                  on SD card via SPI, feeds to HLS IP, and controls outputs.
 *                  Uses GPTimer0 for PWM on JC2 to drive L298N enable pin.
 * Author         : (Your Name)
 * Email          : (Your Email)
 *
 * Copyright (C) 2025  (Your Organization)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *************************/

/*********************
 *                   Include Files
 *********************/
#include "gpio.h"
#include "platform.h"
#include "proximity_warning.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* SPI drivers (from Shakti BSP) */
#include "pinmux.h"
#include "sspi.h"
#include "utils.h"

/* FatFs header */
#include "ff.h"
#include "diskio.h"



/*********************
 *                Static Variables
 *********************/
static radar_entry_t radar_data[MAX_CSV_ENTRIES];
static uint32_t radar_data_count = 0;
static uint32_t data_index = 0;

/*********************
 *             Low-level Register Access
 *********************/
void ip_write_reg(uint32_t offset, uint32_t value)
{
    volatile uint32_t *reg = (volatile uint32_t *)(IP_BASE + offset);
    *reg = value;
}

uint32_t ip_read_reg(uint32_t offset)
{
    volatile uint32_t *reg = (volatile uint32_t *)(IP_BASE + offset);
    return *reg;
}

/*********************
 *                  GPIO Initialization
 *********************/
void gpio_init(void)
{
    unsigned long dir_reg = read_word(GPIO_DIRECTION_CNTRL_REG);
    dir_reg |= (LED_ALERT_PIN | LED_STATUS_PIN | BUZZER_PIN |
                IN1_PIN | IN2_PIN);
    write_word(GPIO_DIRECTION_CNTRL_REG, dir_reg);

    write_gpio(LED_ALERT_PIN, 0);
    write_gpio(LED_STATUS_PIN, 0);
    write_gpio(BUZZER_PIN, 0);
    write_gpio(IN1_PIN, 0);
    write_gpio(IN2_PIN, 0);
}

void write_gpio(unsigned long gpio_pin, int value)
{
    unsigned long read_data = read_word(GPIO_DATA_REG);
    if (value)
        write_word(GPIO_DATA_REG, read_data | gpio_pin);
    else
        write_word(GPIO_DATA_REG, read_data & ~gpio_pin);
}

/*********************
 *                  GPTimer0 PWM Functions
 *********************/
void gptimer0_pwm_init(uint32_t period)
{
    volatile uint32_t *ctrl   = (uint32_t *)(GPTIMER0_BASE + GPTIMER_CTRL);
    volatile uint32_t *period_reg = (uint32_t *)(GPTIMER0_BASE + GPTIMER_PERIOD);
    volatile uint32_t *compare = (uint32_t *)(GPTIMER0_BASE + GPTIMER_COMPARE);

    // Stop timer
    *ctrl = 0;

    // Set period (PWM frequency = core_clk / (prescaler * period) – adjust prescaler if needed)
    *period_reg = period;

    // Initial duty = 0
    *compare = 0;

    // Enable PWM mode and start timer
    *ctrl = GPTIMER_ENABLE | GPTIMER_MODE_PWM;
}

void gptimer0_set_duty(uint32_t duty)
{
    volatile uint32_t *compare = (uint32_t *)(GPTIMER0_BASE + GPTIMER_COMPARE);
    *compare = duty;
}

/*********************
 *                     Timing
 *********************/
uint32_t get_tick_ms(void)
{
    static uint32_t tick = 0;
    return tick++;
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 1000; i++) {
        asm volatile ("nop");
    }
}

/* SSPI configuration helper */
void sspi_configure(sspi_struct *sspi_instance, uint32_t clk_rate, uint8_t clk_pol,
                    uint8_t clk_phase, uint8_t pins_master, uint8_t mas_slv,
                    uint8_t lsb_first, uint8_t setup_time, uint8_t hold_time)
{
    sspi_enable_txrx(sspi_instance, DISABLE);
    sspi_configure_clock(sspi_instance, clk_rate, clk_pol, clk_phase);
    sspi_configure_clock_in_hz(sspi_instance, clk_rate);
    sspi_configure_clock_pol(sspi_instance, clk_pol);
    sspi_configure_clock_phase(sspi_instance, clk_phase);
    if(pins_master)
        sspi_configure_pins(sspi_instance, 1, 0, 1, 1);
    else
        sspi_configure_pins(sspi_instance, 0, 1, 0, 0);
    if(mas_slv)
        sspi_configure_mas_slv(sspi_instance, SPI_MASTER);
    else
        sspi_configure_mas_slv(sspi_instance, ~SPI_MASTER);
    if(!lsb_first)
        sspi_configure_lsb_first(sspi_instance, MSB_FIRST);
    sspi_configure_tx_setup_time(sspi_instance, setup_time);
    sspi_configure_tx_hold_time(sspi_instance, hold_time);
}

/*********************
 *                  SPI Initialization
 *********************/
void spi_init(void)
{
    *(pinmux_config_reg) = 0x154000;   // Adjust for your board

    /* Note: No need to configure pinmux for GPTimer0 – it is a dedicated output on JC2. */

    sspi_init();

    sspi_clear_fifo(sspi_instance[SSPI_INSTANCE_SDCARD]);
    sspi_notbusy(sspi_instance[SSPI_INSTANCE_SDCARD]);
    sspi_configure(sspi_instance[SSPI_INSTANCE_SDCARD],
                   SSPI_CLOCK_FREQ,
                   CLK_POLARITY_HIGH,
                   CLK_PHASE_HIGH,
                   1, 1, 0,
                   SETUP_TIME_CYCLES,
                   HOLD_TIME_CYCLES);

    printf("SPI initialized for SD card on instance %d\n", SSPI_INSTANCE_SDCARD);
}

/*********************
 *            Load Radar Distances and RPM from CSV File
 *********************/
int load_radar_distances_from_csv(radar_entry_t *buffer, uint32_t max_count)
{
    FIL fil;
    FRESULT fr;
    char line[128];
    int count = 0;

    fr = f_open(&fil, CSV_FILE_PATH, FA_READ);
    if (fr != FR_OK) {
        printf("Failed to open %s (error %d). Using example data.\n", CSV_FILE_PATH, fr);
        goto use_fallback;
    }

    if (f_gets(line, sizeof(line), &fil) == NULL) {
        printf("CSV file is empty. Using example data.\n");
        f_close(&fil);
        goto use_fallback;
    }

    while (count < max_count && f_gets(line, sizeof(line), &fil)) {
        char *token;
        float x = 0, y = 0;
        int rpm = 0;
        int field = 0;

        token = strtok(line, "\t");
        while (token != NULL) {
            switch (field) {
                case 2: break;
                case 3: x = atof(token); break;
                case 4: y = atof(token); break;
                case 9: rpm = atoi(token); break;
                default: break;
            }
            field++;
            token = strtok(NULL, "\t");
        }

        if (field < 10) {
            printf("Warning: line %d has only %d fields, skipping\n", count+2, field);
            continue;
        }

        float dist_m = sqrtf(x*x + y*y);
        uint32_t dist_cm = (uint32_t)(dist_m * 100.0f + 0.5f);

        buffer[count].distance_cm = dist_cm;
        buffer[count].rpm = rpm;
        count++;
    }

    f_close(&fil);
    printf("Loaded %d entries from %s\n", count, CSV_FILE_PATH);
    return count;

use_fallback:
    printf("WARNING: CSV file loading not implemented. Using example data.\n");
    static const uint32_t example_distances[] = {
        1000, 150, 180, 200, 220, 250, 280, 300, 320, 350,
        380, 400, 420, 450, 480, 500, 520, 550, 580, 600,
        620, 650, 680, 700, 720, 750, 780, 800, 820, 850,
        880, 900, 920, 950, 980, 1000, 1020, 1050, 1080, 1100,
        1120, 1150, 1180, 1200, 1220, 1250, 1280, 1300, 1320, 1350,
        1380, 1400, 1420, 1450, 1480, 1500, 1520, 1550, 1580, 1600,
        1620, 1650, 1680, 1700, 1720, 1750, 1780, 1800, 1820, 1850,
        1880, 1900, 1920, 1950, 1980, 2000, 2020, 2050, 2080, 2100,
        2120, 2150, 2180, 2200, 2220, 2250, 2280, 2300, 2320, 2350,
        2380, 2400, 2420, 2450, 2480, 2500, 2520, 2550, 2580, 2600
    };
    uint32_t num_dist = sizeof(example_distances)/sizeof(example_distances[0]);
    if (num_dist > max_count) num_dist = max_count;
    for (uint32_t i = 0; i < num_dist; i++) {
        buffer[i].distance_cm = example_distances[i];
        buffer[i].rpm = 600;
    }
    return num_dist;
}

/*********************
 *                     Main Function
 *********************/
int main()
{
    
    
    uint32_t hall_rpm;
    uint32_t obj_dist;
    uint32_t pwm_duty;
    uint32_t alert;
    uint32_t buzzer;
    
    

    printf("\n\n\tProximity Warning and Speed Control System (CSV data from SD card)\n");

    gpio_init();
    spi_init();

    /* Initialize GPTimer0 for PWM on JC2 */
    gptimer0_pwm_init(5000);   // period = 1000 → adjust frequency as needed

    radar_data_count = load_radar_distances_from_csv(radar_data, MAX_CSV_ENTRIES);
    if (radar_data_count == 0) {
        printf("Failed to load radar data. Halting.\n");
        while (1);
    }
    printf("Loaded %d radar data entries from %s\n", radar_data_count, CSV_FILE_PATH);

    for (int i = 0; i < 5 && i < radar_data_count; i++) {
        printf("Entry %d: distance=%lu, rpm=%lu\n", i, radar_data[i].distance_cm, radar_data[i].rpm);
    }

    printf("Testing GPIO read\n");
    uint32_t gpio_val = read_word(0x00040100);
    printf("GPIO read value: 0x%x\n", gpio_val);

    printf("Before IP_CTRL write\n");
    ip_write_reg(IP_CTRL, 0x00);
    printf("After IP_CTRL write\n");

    /* Set motor direction forward */
    write_gpio(IN1_PIN, 1);
    write_gpio(IN2_PIN, 0);

    uint32_t loop_count = 0;

    while (1)
    {
        printf("\n=== Loop iteration %lu, data_index=%lu ===\n", loop_count++, data_index);

        obj_dist = radar_data[data_index].distance_cm;
        hall_rpm = radar_data[data_index].rpm;
        printf("Read: distance=%lu, rpm=%lu\n", obj_dist, hall_rpm);

        data_index++;
        if (data_index >= radar_data_count) data_index = 0;

        printf("Writing HALL_RPM\n");
        ip_write_reg(IP_HALL_RPM, hall_rpm);

        printf("Writing OBJ_DIST\n");
        ip_write_reg(IP_OBJ_DIST, obj_dist);

        printf("Clearing start...\n");
        ip_write_reg(IP_CTRL, 0x00);
        for (volatile int i = 0; i < 100; i++);
        printf("Setting start...\n");
        ip_write_reg(IP_CTRL, 0x01);

        printf("Waiting for done...\n");
        while(!(ip_read_reg(IP_CTRL) & 0x2));
        printf("IP done.\n");

        printf("Reading PWM_DUTY\n");
        pwm_duty = ip_read_reg(IP_PWM_DUTY);
        printf("PWM_DUTY = %d\n", pwm_duty);
        alert = ip_read_reg(IP_DASH_ALERT);
        buzzer = ip_read_reg(IP_BUZZER);
        printf("Alert = %d, Buzzer = %d\n", alert, buzzer);

        printf("Writing LED_ALERT\n");
        write_gpio(LED_ALERT_PIN, alert);
        printf("Writing BUZZER\n");
        write_gpio(BUZZER_PIN, buzzer);

        /* Set GPTimer0 duty cycle (motor speed) */
        printf("Setting GPTimer0 duty = %d\n", pwm_duty);
        gptimer0_set_duty(pwm_duty);

        write_gpio(LED_STATUS_PIN, 1);
        delay_ms(10);
        write_gpio(LED_STATUS_PIN, 0);

        delay_ms(500);
    }

    return 0;
}



