/*
 * tx_manager.h
 *
 *  Created on: May 14, 2016
 *      Author: surligas
 */

#ifndef INC_TX_MANAGER_H_
#define INC_TX_MANAGER_H_

#include <stdint.h>
#include <stdlib.h>

int32_t
tx_data(const uint8_t *in, size_t len, uint8_t *dev_rx_buffer,
	size_t timeout_ms);

#endif /* INC_TX_MANAGER_H_ */
