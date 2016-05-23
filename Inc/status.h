/*
 * status.h
 *
 *  Created on: May 18, 2016
 *      Author: surligas
 */

#ifndef INC_STATUS_H_
#define INC_STATUS_H_


enum {
  STATUS_TIMEOUT = -4,
  STATUS_NO_DATA = -3,
  STATUS_BUFFER_OVERFLOW = -2,
  STATUS_BUFFER_UNDERFLOW = -1,
  STATUS_OK = 0,
};

#endif /* INC_STATUS_H_ */
