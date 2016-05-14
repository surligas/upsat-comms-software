/*
 * log.h
 *
 *  Created on: May 14, 2016
 *      Author: surligas
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
	HAL_UART_Transmit (huart, _log_uart_buffer,							\
					   strlen (_log_uart_buffer), 10000);				\

#define LOG_UART_ERROR(huart, M, ...) 									\
	snprintf(_log_uart_buffer, COMMS_UART_BUF_LEN, 						\
			"[ERROR] %s:%d: " M "\n",									\
			 __FILE__, __LINE__, ##__VA_ARGS__);						\
	HAL_UART_Transmit (huart, _log_uart_buffer,							\
					   strlen (_log_uart_buffer), 10000);				\

#else
#define LOG_UART_DBG(huart, M, ...)
#define LOG_UART_ERROR(huart, M, ...)
#endif



#endif /* INC_LOG_H_ */
