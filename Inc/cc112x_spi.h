#ifndef __CC1120X_SPI_H
#define __CC1120X_SPI_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cc_definitions.h"
#include "cc_commands.h"
#include "config.h"

#define CC_EXT_ADD 0x2F00

uint8_t
cc_tx_readReg (uint16_t add, uint8_t *data);
uint8_t
cc_tx_writeReg (uint16_t add, uint8_t data);
int32_t
cc_tx_data (const uint8_t *data, uint8_t size, uint8_t *rec_data,
	    size_t timeout_ms);
uint8_t
cc_tx_cmd (uint8_t CMDStrobe);


uint8_t
cc_rx_readReg (uint16_t add, uint8_t *data);
uint8_t
cc_rx_writeReg (uint16_t add, uint8_t data);
uint8_t
cc_RX_DATA (uint8_t *data, uint8_t *size, uint8_t *rec_data);
int32_t
cc_rx_data (uint8_t *out, size_t len,
	    size_t timeout_ms);
uint8_t
cc_rx_cmd (uint8_t CMDStrobe);

#endif
