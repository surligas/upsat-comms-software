#ifndef __CC1120X_SPI_H
#define __CC1120X_SPI_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "cc_definitions.h"
#include "cc_commands.h"
#include "config.h"

#define CC_EXT_ADD 0x2F00

uint8_t
cc_tx_rd_reg (uint16_t add, uint8_t *data);
uint8_t
cc_tx_wr_reg (uint16_t add, uint8_t data);
HAL_StatusTypeDef
cc_tx_spi_write_fifo(const uint8_t *data, uint8_t *spi_rx_data, size_t len);
int32_t
cc_tx_data (const uint8_t *data, uint8_t size, uint8_t *rec_data,
	    size_t timeout_ms);
uint8_t
cc_tx_cmd (uint8_t CMDStrobe);

HAL_StatusTypeDef
cc_rx_spi_read_fifo(uint8_t *out, size_t len);
uint8_t
cc_rx_rd_reg (uint16_t add, uint8_t *data);
uint8_t
cc_rx_wr_reg (uint16_t add, uint8_t data);
int32_t
cc_rx_data (uint8_t *out, size_t len,
	    size_t timeout_ms);
uint8_t
cc_rx_cmd (uint8_t CMDStrobe);

#endif
