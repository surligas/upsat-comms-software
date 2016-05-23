/*
 * cc1120_config.h
 *
 *  Created on: May 14, 2016
 *      Author: surligas
 */

#ifndef INC_CC1120_CONFIG_H_
#define INC_CC1120_CONFIG_H_

#define CC1120_TX_MAX_FRAME_LEN 255
#define CC1120_TX_FIFO_SIZE 125
#define CC1120_RX_FIFO_SIZE 128

#define CC1120_TXFIFO_THR 63
#define CC1120_RXFIFO_THR 63
#define CC1120_BYTES_IN_RX_FIF0 (CC1120_RXFIFO_THR + 1)

#endif /* INC_CC1120_CONFIG_H_ */
