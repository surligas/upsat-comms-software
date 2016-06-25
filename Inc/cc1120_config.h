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

#ifndef INC_CC1120_CONFIG_H_
#define INC_CC1120_CONFIG_H_

#define CC1120_TX_MAX_FRAME_LEN 256
#define CC1120_TX_FIFO_SIZE 127
#define CC1120_RX_FIFO_SIZE 128

#define CC1120_TXFIFO_THR 63
#define CC1120_TXFIFO_IRQ_THR (CC1120_TX_FIFO_SIZE - CC1120_TXFIFO_THR)
#define CC1120_RXFIFO_THR 64
#define CC1120_BYTES_IN_RX_FIF0 (CC1120_RXFIFO_THR + 1)

#endif /* INC_CC1120_CONFIG_H_ */
