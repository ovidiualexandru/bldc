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
 CAN ID = 0 for Motor 1
 CAN ID != 0 for Motor 2
The default address is fixed at 128. A potential extension is described below.
----
    TODO: extend the CAN ID usage for multiple Saber addresses. Example:
    CAN_ID = 0 -> Saber 128, M1
    CAN_ID = 1 -> Saber 128, M2
    CAN_ID = 2 -> Saber 129, M1
    CAN_ID = 3 -> Saber 129, M2
    and so on...
----

= Implementation
----------------
We'll use a 4 char circular buffer to store the most recent received chars. When
a valid data packet is detected, it is decoded, checked for the motor id, and the
new values written.

For brushed DC motors we'll use duty-cycle control, and for BLDC or FOC we'll use RPM
control. The RPM value is computed by multiplying the received data packet with a 
constant defined in this file.

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

== E-Stop
---------
The Saber driver uses the Tx pin as an E-Stop input. If the input is at logic
level '1', the driver is enabled. If it's at '0', it's disabled. If the input is 
disconnected, it defaults to '1' via an internal pull-up.

Instead of using the asyncronous read of the E-Stop pin, we will check it at every
received char. Also, we'll use a pull-down for safety reasons.

*/

/* checksum mask defined for the saber protocol */
#define CHECKSUM_MASK (0x7Fu)

/* Circular buffer length */
#define BUFFER_LEN (4u)
/* Increment operation on the buffer pointer */
#define BUFFER_INC_IDX(x) (((x) + 1u) % BUFFER_LEN)
static uint8_t buff[BUFFER_LEN]; /* The circular buffer */
static uint8_t buff_idx; /* Buffer pointer: next index to write in the buffer */

/* Helper functions */

// check if the buffer contains a data packet with a valid checksum
static bool valid_checksum(void)
{
    /* buff_idx indicates where the next char will be saved, but also shows where
       the oldest char is, so we start from there */
    uint8_t idx = buff_idx;
    uint8_t sum;
    uint8_t i;
    /* Iterate over the first three chars, and verify checksum */
    for (i = 0u; i < (BUFFER_LEN - 1u); i++){
        sum += buff[idx];
        idx = BUFFER_INC_IDX(idx);
    }
    /* compare the local checksum with the received one, idx will point now
    to the latest char in the buffer */
    uint8_t checksum = (uint8_t)(sum & CHECKSUM_MASK);
    if (checksum == buff[idx]){
        return true;
    }
    return false;
}

// add a char into the buffer
static void buff_add_char(uint8_t c)
{
    buff[buff_idx] = c;
    buff_idx = BUFFER_INC_IDX(buff_idx);
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
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
    /* Process the characted */
	(void)uartp;
    buff_add_char((uint8_t)c);
    if(valid_checksum()){
        /* Nice, we received a valid data packet, process it */
        uint8_t id = buff_idx; // now buff_idx will be pointing to the first element of the data packet, the driver address
        uint8_t saber_address = buff[id];
        id = BUFFER_INC_IDX(id);
        uint8_t cmd = buff[id];
        id = BUFFER_INC_IDX(id);
        uint8_t payload = buff[id];
        /* check saber address and motor id, compare with CAN ID */
        if (driver_id_correct(can_id, saber_address, cmd)){
            /* extract the command type and the payload */
            if (cmd == 0 || cmd == 4){ /* Drive forward motor */
                
            } else if (cmd == 1 || cmd == 5){ /* Drive backwards motor */
                
            } else if (cmd == 6 || cmd == 7){ /* Drive motor 7 bit */
                
            }
            else {
                /* Panic! Invalid command. */
                /* Go directly to Jail. Do not pass Go. Do not collect $200. */
                return;
            }
        }
        
        /* write to output and wake up thread */
        
        chSysLockFromISR();
        /*TODO: write eventmask with either 1 or 2, depending on control mode
         (duty-cycle for brushed, RPM for BLDC) */
        chEvtSignalI(process_tp, (eventmask_t) 1);
        chSysUnlockFromISR();
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
}

void app_custom_stop(void) {
}


void app_custom_configure(app_configuration *conf) {
    (void)conf;
}
