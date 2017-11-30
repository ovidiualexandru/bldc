/*
    Copyright 2017 Ovidiu Alexandru ovidiu.marius.alexandru@gmail.com

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


/* This file partially implements the Sabertooth Packetized protocol */

/*
= The protocol
--------------

The packetized protocol is composed of 4 byte messages:
    Byte 1: Address (first address is 128)
    Byte 2: Command
    Byte 3: Data
    Byte 4: Checksum ((Address + Command + Data) & 0x7F)
    
The Command characters that are implemented:
 0: Drive forward motor 1 (decimal 0, binary 0b00000000, hex 0h00)
 1: Drive backwards motor 1 (decimal 1, binary 0b00000001, hex 0h01)
 4: Drive forward motor 2 (decimal 4, binary 0b00000100, hex 0h04)
 5: Drive backwards motor 2 (decimal 5, binary 0b00000101, hex 0h05)
 6*: Drive motor 2 7 bit (decimal 6, binary 0b00000110, hex 0h06)
 7*: Drive motor 1 7 bit (decimal 7, binary 0b00000111, hex 0h07)

*The Sabertooth 2x60 user manual contains a documentation bug, where command 6
is documented as being for motor 1 and command 7 for motor 2.

 Since Sabertooth 2x60 is a dual-channel motor driver and VESC is single-channel,
we'll use the VESC CAN ID to indicate if the current VESC is for Motor 1 or Motor 2.
We'll use:
    CAN_ID = 0 -> Saber 128, M1
    CAN_ID = 1 -> Saber 128, M2
    CAN_ID = 2 -> Saber 129, M1
    CAN_ID = 3 -> Saber 129, M2
    and so on...


= Implementation
----------------
We'll use a 4 char circular buffer to store the most recent received chars. When
a valid data packet is detected, it is decoded, checked for the motor id, and the
new values written.

For brushed DC motors we'll use duty-cycle control, and for BLDC or FOC we'll use RPM
control. The RPM value is computed by multiplying the received data packet with a 
constant defined in this file.

To optimize the rx ISR, we'll save the received data to an intermediate value.
Since the best resolution for the packetized protocol is 7 + 1 sign bit, we'll
save the received value to signed 8 bit variable.
To optimize thread behavior, we'll compare new data to the value that is already
saved, and wake-up the thread only if they are different.

== Decode algorithm
-------------------
  ____________________
 | Init 4 char buffer |
 |____________________|
            |
            +------------+
  __________|_________   |
 | Wait receive char  |  |
 |____________________|  |
            |            |
  __________|_________   |
 / Checksum valid     \__|
 \____________________/ F|
            |T           |
  __________|_________   |
 / Motor ID correct   \__|
 \____________________/ F
            |T
  __________|_________
 | Decode cmd and send|
 |_to VESC____________|

TODO: implement E-Stop described below.
== E-Stop
---------
The Saber driver uses the Tx pin as an E-Stop input. If the input is at logic
level '1', the driver is enabled. If it's at '0', it's disabled. If the input is 
disconnected, it defaults to '1' via an internal pull-up.

Instead of using the asyncronous read of the E-Stop pin, we will check it at every
received char. Also, we'll use a pull-down for safety reasons.

TODO: use vesc timeouts properly. In this implementation, the timeout must be set to
a very high value to avoid unwanted stopping. But the timeout should be used for safety
reasons.
*/

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "mc_interface.h"
#include "datatypes.h"

// Settings
#define BAUDRATE 115200

/* checksum mask defined for the saber protocol */
#define CHECKSUM_MASK (0x7Fu)

/* duty-cycle definitions */
#define MOTOR_DUTY_SCALE (1.0 / 127.0) // scale from byte command to floats

/* rpm control definitions */
#define MOTOR_RPM_SCALE (126) // TODO: this should be configurable

/* Circular buffer length */
#define BUFFER_LEN (4u)
/* Increment operation on the buffer pointer */
#define BUFFER_INC_IDX(x) (((x) + 1u) % BUFFER_LEN)
static uint8_t buff[BUFFER_LEN]; /* The circular buffer */
static uint8_t buff_idx; /* Buffer pointer: next index to write in the buffer */

static volatile uint8_t controlled_id; /* can id */
static volatile int8_t crt_command; /* store the current command */
static volatile bool is_running = false;

/* Thread */
static THD_FUNCTION(saber_process_thread, arg);
static THD_WORKING_AREA(saber_process_thread_wa, 4096);
static thread_t *process_tp = 0;

/* Helper functions */

// check if the buffer contains a data packet with a valid checksum
static bool valid_checksum(void)
{
    /* buff_idx indicates where the next char will be saved, but also shows where
       the oldest char is, so we start from there */
    uint8_t idx = buff_idx;
    uint8_t sum = 0u;
    uint8_t i;
    bool chk = false;
    /* Iterate over the first three chars, and verify checksum */
    for (i = 0u; i < (BUFFER_LEN - 1u); i++){
        sum += buff[idx];
        idx = BUFFER_INC_IDX(idx);
    }
    /* compare the local checksum with the received one, idx will point now
    to the latest char in the buffer */
    uint8_t checksum = (uint8_t)(sum & CHECKSUM_MASK);
    if (checksum == buff[idx]){
        chk = true;
    }
    return chk;
}

// add a char into the buffer
static void buff_add_char(uint8_t c)
{
    buff[buff_idx] = c;
    buff_idx = BUFFER_INC_IDX(buff_idx);
}

// compare the saber adress and cmd with the can id to see if this vesc is addressed 
static bool driver_id_correct(uint8_t can_id, uint8_t saber_address, uint8_t cmd)
{
    if (saber_address < 128u) return false; //input sanitization
    uint8_t internal_address = saber_address - 128u;
    uint8_t can_lsb = can_id & 1u; //M1 is for CAN IS LSB = 0, M2 is for CAN ID LSB = 1
    uint8_t can_internal_address = can_id >> 1u; // remove the LSB
    if (internal_address == can_internal_address){
        if ((can_lsb == 0u) && 
                ((cmd == 0u) || (cmd == 1u) || (cmd == 7u))
            ){
            return true;
        }
        if ((can_lsb == 1u) && 
                ((cmd == 4u) || (cmd == 5u) || (cmd == 6u))
            ){
            return true;
        }
    }
    return false;
}

// UART callbacks

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

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
    (void)uartp;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
    /* Process the characted */
    (void)uartp;
    buff_add_char((uint8_t)(c & 0xFFu));
    if(valid_checksum()){
        /* Nice, we received a valid data packet, process it */
        uint8_t id = buff_idx; // now buff_idx will be pointing to the first element of the data packet, the driver address
        uint8_t saber_address = buff[id];
        id = BUFFER_INC_IDX(id);
        uint8_t cmd = buff[id];
        id = BUFFER_INC_IDX(id);
        uint8_t payload = buff[id];
        int8_t next_command = crt_command;
        /* check saber address and motor id, compare with CAN ID */
        if (driver_id_correct(controlled_id, saber_address, cmd)){
            if (payload > 127u) return; // input sanitization
            /* extract the command type and the payload */
            if (cmd == 0 || cmd == 4){ /* Drive forward motor */
                /* 7bit magnitudine, positive */
                next_command = (int8_t)payload; // this should already be 7 bits
            } else if (cmd == 1 || cmd == 5){ /* Drive backwards motor */
                /* 7bit magnitude, negative */
                next_command = -((int8_t)payload);
            } else if (cmd == 6 || cmd == 7){ /* Drive motor 7 bit */
                /* 0 is full reverse, 127 full forward, 64 is stop */
                if (payload > 64u){
                    // add 1 to adjust values to get 100%
                    next_command = ((int8_t)payload - 64) * 2 + 1;
                }
                else{
                    next_command = ((int8_t)payload - 64) * 2;
                }
            }
            else {
                /* Panic! Invalid command. */
                /* Go directly to Jail. Do not pass Go. Do not collect $200. */
                return;
            }
        }
        
        if (next_command != crt_command){
            /* the current command needs to be changed */
            crt_command = next_command;
            chSysLockFromISR();
            chEvtSignalI(process_tp, (eventmask_t) 1);
            chSysUnlockFromISR();
        }
    }
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
    (void)conf;
    controlled_id = conf->controller_id;
    // use the baudrate set from the VESC tool for UART
    uart_cfg.speed = conf->app_uart_baudrate;
    if (is_running) {
        uartStart(&HW_UART_DEV, &uart_cfg);
    }
    /* TODO: save the rpm scale coefficient from conf */
}

static THD_FUNCTION(saber_process_thread, arg) {
    (void)arg;

    chRegSetThreadName("Saber packetized");

    process_tp = chThdGetSelfX();

    for(;;) {
        chEvtWaitAny((eventmask_t) 1);

        /* get a reference to the current motor configuration so we can read the motor type */
        const volatile mc_configuration* mcconf = mc_interface_get_configuration();

        if (crt_command == 0){
            /* set the brakes */
            mc_interface_brake_now();
        }
        else{
            if (mcconf->motor_type == MOTOR_TYPE_DC){
                /* duty-cycle control */
                float duty = ((float)crt_command) * MOTOR_DUTY_SCALE;
                // make sure commands are in a valid range 
                if (duty > 1.0) duty = 1.0;
                else if (duty < -1.0) duty = -1.0;
                mc_interface_set_duty(duty); /* note: the fact that only floats can be used for duty-cycle is very, very sad */
            }
            else {
                /* rpm control */
                int32_t rpm = ((int32_t)crt_command) * (int32_t)MOTOR_RPM_SCALE;
                mc_interface_set_pid_speed(rpm);
            }
            timeout_reset();
        }
    }
}
