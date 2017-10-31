/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"

#include <string.h>

// Settings
#define BAUDRATE					115200

// Threads
static THD_FUNCTION(saber_process_thread, arg);
static THD_WORKING_AREA(saber_process_thread_wa, 4096);
static thread_t *process_tp = 0;
static int32_t rpm;

// Variables
static volatile bool is_running = false;

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}

#define DRIVER_MASK (0x80u)
#define DRIVER_ID (0x80u) // TODO: this should be configurable
#define DRIVER_COMM_OFFSET (64) // subtract this value to center on zero
#define MOTOR_RPM_SCALE (126) // TODO: this should be configurable

/*
 * Check if the received byte is intended for this driver
 */
static bool this_driver(uint8_t c)
{
    return ((c & DRIVER_MASK) == DRIVER_ID);
}

/*
 * Extract the rpm in the byte, scale it to an rpm value
 */
static int32_t get_rpm_info_from_saber_char(uint8_t c)
{
    uint8_t byteval = c & (~DRIVER_MASK); /* remove the driver id bit */
    int32_t x = ((int32_t)byteval - (int32_t)DRIVER_COMM_OFFSET) * MOTOR_RPM_SCALE;
    return x;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
    /* Process the characted */
    /* Check the driver id - MSB of c */
	(void)uartp;
    if (this_driver((uint8_t)c)){
        /* extract the rpm info from c */
        rpm = get_rpm_info_from_saber_char((uint8_t)c);
        chEvtSignalI(process_tp, (eventmask_t) 1);
    }
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
		txend1,
		txend2,
		rxend,
		rxchar,
		rxerr,
		BAUDRATE,
		0,
		USART_CR2_LINEN,
		0
};

void app_custom_start(void) {
	if (!is_running) {
		chThdCreateStatic(saber_process_thread_wa, sizeof(saber_process_thread_wa),
				NORMALPRIO, saber_process_thread, NULL);
		is_running = true;
	}

	uartStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
}

void app_custom_stop(void) {
	uartStop(&HW_UART_DEV);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);

	// Notice that the processing thread is kept running in case this call is made from it.
}


void app_custom_configure(app_configuration *conf) {
    (void) conf;
    /* TODO: configure the driver id (MSB mask) and rpm scale */
    
	// uart_cfg.speed = baudrate;

	// if (is_running) {
		// uartStart(&HW_UART_DEV, &uart_cfg);
	// }
}

static THD_FUNCTION(saber_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("Saber simplified");

	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);
        /* a new rpm reference has been set, process it */
        static uint8_t buffer[4] = "c \r\l";
        mc_interface_set_pid_speed(rpm);
        
        while (HW_UART_DEV.txstate == UART_TX_ACTIVE) {
            chThdSleep(1);
        }
        buffer[1] = (uint8_t) rpm;
        uartStartSend(&HW_UART_DEV, 4, buffer);
        // TODO: check if brakes need to be enabled
    }
}

