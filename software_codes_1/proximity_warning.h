
/*************************
 * Project        : Proximity Warning and Speed Control
 * File           : proximity_warning.h
 * Description    : Header file – loads radar & RPM data from CSV file on SD card.
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

#ifndef PROXIMITY_WARNING_H
#define PROXIMITY_WARNING_H

/*********************
 *                IP Register Definitions
 *********************/
#define IP_BASE             0x00040600      // From soc.defines (AXI4Lite slave)

#define IP_CTRL             0x00
#define IP_GIE              0x04
#define IP_IER              0x08
#define IP_ISR              0x0C
#define IP_HALL_RPM         0x10
#define IP_OBJ_DIST         0x18
#define IP_DASH_ALERT       0x20
#define IP_DASH_ALERT_CTRL  0x24
#define IP_BUZZER           0x28
#define IP_BUZZER_CTRL      0x2C
#define IP_PWM_DUTY         0x30
#define IP_PWM_DUTY_CTRL    0x34

/*********************
 *                GPIO Pin Definitions
 *********************/
/* Existing outputs */
#define LED_ALERT_PIN       GPIO16
#define LED_STATUS_PIN      GPIO18
#define BUZZER_PIN          GPIO24

/* L298N Motor Driver Pins (direction only – enable uses GPTimer0) */
#define IN1_PIN             GPIO25  // Direction control 1
#define IN2_PIN             GPIO26  // Direction control 2

/* If GPIO macros are not defined elsewhere, define them as bit masks */
#ifndef GPIO9
#define GPIO9   (1 << 9)
#define GPIO10  (1 << 10)
#define GPIO12  (1 << 12)
#define GPIO11  (1 << 11)
#define GPIO16  (1 << 16)
#define GPIO18  (1 << 18)
#endif

/* GPTimer0 base address (from Shakti memory map – verify with your soc.defines) */
#define GPTIMER0_BASE           0x00020000

/* GPTimer0 register offsets */
#define GPTIMER_CTRL            0x00
#define GPTIMER_PERIOD          0x04
#define GPTIMER_COMPARE         0x08
#define GPTIMER_COUNTER         0x0C

/* Control register bits (adjust according to your BSP) */
#define GPTIMER_ENABLE           (1 << 0)
#define GPTIMER_MODE_PWM         (1 << 1)   // 1 = PWM mode

/*********************
 *                     Constants
 *********************/
/* SPI Configuration */
#define SSPI_INSTANCE_RADAR     0          // Not used if radar data is from CSV
#define SSPI_INSTANCE_SDCARD    1          // SD card on SSPI1
#define SSPI_CLOCK_FREQ         100000     // 100 kHz
#define CLK_POLARITY_HIGH       1
#define CLK_PHASE_HIGH          1
#define SETUP_TIME_CYCLES       0
#define HOLD_TIME_CYCLES        0

/* CSV file containing radar distances and RPM (tab‑separated) */
#define CSV_FILE_PATH           "mmw_demo_output.csv"
#define MAX_CSV_ENTRIES         1000       // Maximum number of rows to load

/*********************
 *                  Data Structures
 *********************/
typedef struct {
    uint32_t distance_cm;   // Object distance in cm (computed from x,y)
    uint32_t rpm;           // RPM value from CSV
} radar_entry_t;

/*********************
 *                  Function Prototypes
 *********************/
void gpio_init(void);
void ip_write_reg(uint32_t offset, uint32_t value);
uint32_t ip_read_reg(uint32_t offset);
void delay_ms(uint32_t ms);
uint32_t get_tick_ms(void);
void write_gpio(unsigned long gpio_pin, int value);

/* GPTimer0 PWM functions */
void gptimer0_pwm_init(uint32_t period);
void gptimer0_set_duty(uint32_t duty);

/* SPI and CSV loading */
void spi_init(void);
int load_radar_distances_from_csv(radar_entry_t *buffer, uint32_t max_count);

#endif // PROXIMITY_WARNING_H
