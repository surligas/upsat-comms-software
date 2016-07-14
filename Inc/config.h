/*
 * upsat-comms-software: Communication Subsystem Software for UPSat satellite
 *
 *  Copyright (C) 2016, Libre Space Foundation <http://librespacefoundation.org/>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "cc1120_config.h"

static const char __UPSAT_CALLSIGN[] = "ON02GR";
static const uint8_t __UPSAT_SSID = 0;
static const uint8_t __UPSAT_AX25_CTRL = 0x03;
static const char __UPSAT_DEST_CALLSIGN[] = "ABCD";
static const uint8_t __UPSAT_DEST_SSID = 0;
static const char __COMMS_RF_SWITCH_CMD[] = "RF SW CMD";
static const uint32_t __COMMS_RF_SWITCH_ON_CMD[] =
    {0xda4942a9, 0xa7a45d61, 0x413981b, 0xa94ee2d3};
static const uint32_t __COMMS_RF_SWITCH_OFF_CMD[] =
  { 0x593d55df, 0x4d2f84c0, 0x24d60191, 0x9287b5fd };

/**
 * A key indicating that the TX RF of the COMMS is on.
 *
 * We use a random generated key instead of a just a
 * boolean value to be sure that even if some bits of the
 * flash memory are destroyed, the system can recover and
 * properly identify the state of the TX RF.
 */
static const uint32_t __COMMS_RF_ON_KEY = 0x16264e84;

/**
 * A key indicating that the TX RF of the COMMS is OFF.
 *
 * We use a random generated key instead of a just a
 * boolean value to be sure that even if some bits of the
 * flash memory are destroyed, the system can recover and
 * properly identify the state of the TX RF.
 */
static const uint32_t __COMMS_RF_OFF_KEY = 0x669d93a3;

/**
 * The scrambler polynomial
 */
static const uint32_t __SCRAMBLER_POLY = 0x21;

/**
 * The initial seed of the scrambler
 */
static const uint32_t __SCRAMBLER_SEED = 0x00;

/**
 * The order of the LFSR that the scrambler will use
 */
static const uint32_t __SCRAMBLER_ORDER = 16;

/**
 * The interval of the CW beacon
 */
static const uint32_t __CW_INTERVAL_MS = 300000;

/**
 * Enables/disables the UART debug
 */
#define COMMS_UART_DBG_EN 1
#define COMMS_UART_BUF_LEN 512

/**
 * If set to 1, the UART target is the OBC. If set to 0, the UART target
 * is the FTDI debugging dongle
 */
#define COMMS_UART_DEST_OBC 0
/**
 * The default time out period is 4 seconds
 */
#define COMMS_DEFAULT_TIMEOUT_MS 4000

/**
 * The default COMMS statistics polling period
 */
#define COMMS_STATS_PERIOD_MS 1000

/**
 * The WOD (World Orbit Data) period in milliseconds
 */
#define COMMS_WOD_PERIOD_MS  30000

#endif /* CONFIG_H_ */
