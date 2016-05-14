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

size_t
tx_data(uint8_t *dev_tx_buffer, uint8_t *dev_rx_buffer,
	const uint8_t *in, size_t len);

size_t
tx_large_data(const uint8_t *in, size_t len);

void
tx_fifo_thr_irq_handler ();

#endif /* INC_TX_MANAGER_H_ */
