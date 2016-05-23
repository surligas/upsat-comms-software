/*
 * config.h
 *
 *  Created on: May 13, 2016
 *      Author: surligas
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "cc1120_config.h"

static const char UPSAT_CALLSIGN[] = "UPSAT";
static const uint8_t UPSAT_SSID = 0;
static const uint8_t UPSAT_AX25_CTRL = 0x03;
static const char UPSAT_DEST_CALLSIGN[] = "ABCD";
static const uint8_t UPSAT_DEST_SSID = 0;

/**
 * Enables/disables the UART debug
 */
#define COMMS_UART_DBG_EN 1
#define COMMS_UART_BUF_LEN 512

/**
 * The default time out period is 4 seconds
 */
#define COMMS_DEFAULT_TIMEOUT_MS 4000

#endif /* CONFIG_H_ */
