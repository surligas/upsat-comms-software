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

#ifndef INC_LOG_H_
#define INC_LOG_H_

#include "config.h"
#include <string.h>

static char _log_uart_buffer[COMMS_UART_BUF_LEN];

#if COMMS_UART_DBG_EN
#define LOG_UART_DBG(huart, M, ...) 									\
	snprintf(_log_uart_buffer, COMMS_UART_BUF_LEN, 						\
			"[DEBUG] %s:%d: " M "\n",									\
			 __FILE__, __LINE__, ##__VA_ARGS__);						\
	HAL_UART_Transmit_IT (huart, _log_uart_buffer,							\
					   strlen (_log_uart_buffer));				\

#define LOG_UART_ERROR(huart, M, ...) 									\
	snprintf(_log_uart_buffer, COMMS_UART_BUF_LEN, 						\
			"[ERROR] %s:%d: " M "\n",									\
			 __FILE__, __LINE__, ##__VA_ARGS__);						\
	HAL_UART_Transmit_IT (huart, _log_uart_buffer,							\
					   strlen (_log_uart_buffer));				\

#else
#define LOG_UART_DBG(huart, M, ...)
#define LOG_UART_ERROR(huart, M, ...)
#endif



#endif /* INC_LOG_H_ */
