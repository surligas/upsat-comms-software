/*
 * rx_manager.h
 *
 *  Created on: May 24, 2016
 *      Author: surligas
 */

#ifndef INC_RX_MANAGER_H_
#define INC_RX_MANAGER_H_

#include <stdint.h>
#include <stdlib.h>

int32_t
rx_data(uint8_t *out, size_t len, uint8_t *dev_rx_buffer, size_t timeout_ms);

#endif /* INC_RX_MANAGER_H_ */
