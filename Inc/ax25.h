#ifndef __AX25_H
#define __AX25_H

#include <stdint.h>
#include "utils.h"
#include "config.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

#define AX25_MAX_ADDR_LEN 28
#define AX25_MAX_FRAME_LEN 256
#define AX25_MIN_ADDR_LEN 14
#define AX25_SYNC_FLAG 0x7E
#define AX25_MIN_CTRL_LEN 1
#define AX25_MAX_CTRL_LEN 2
#define AX25_CALLSIGN_MAX_LEN 6

/**
 * AX.25 Frame types
 */
typedef enum
{
  AX25_I_FRAME, //!< AX25_I_FRAME Information frame
  AX25_S_FRAME, //!< AX25_S_FRAME Supervisory frame
  AX25_U_FRAME, //!< AX25_U_FRAME Unnumbered frame
  AX25_UI_FRAME /**!< AX25_UI_FRAME Unnumbered information frame */
} ax25_frame_type_t;

typedef enum
{
  AX25_ENC_FAIL, AX25_ENC_OK
} ax25_encode_status_t;

typedef enum
{
  AX25_DEC_FAIL, AX25_DEC_OK
} ax25_decode_status_t;

typedef struct
{
  uint8_t address[AX25_MAX_ADDR_LEN];
  size_t address_len;
  uint16_t ctrl;
  size_t ctrl_len;
  uint8_t pid;
  uint8_t *info;
  size_t info_len;
  ax25_frame_type_t type;
} ax25_frame_t;


uint16_t
ax25_fcs (uint8_t *buffer, size_t len);

size_t
ax25_create_addr_field (uint8_t *out, const uint8_t *dest_addr, uint8_t dest_ssid,
			const uint8_t *src_addr, uint8_t src_ssid);

size_t
ax25_prepare_frame (uint8_t *out, const uint8_t *info, size_t info_len,
		    ax25_frame_type_t type, uint8_t *addr, size_t addr_len,
		    uint16_t ctrl, size_t ctrl_len);

ax25_encode_status_t
ax25_bit_stuffing (uint8_t *out, size_t *out_len, const uint8_t *buffer,
		   const size_t buffer_len);

ax25_encode_status_t
ax25_nrz_bit_stuffing (float *out, size_t *out_len, const uint8_t *buffer,
		       const size_t buffer_len);

ax25_decode_status_t
ax25_decode (uint8_t *out, size_t *out_len, const uint8_t *ax25_frame,
	     size_t len);

int32_t
ax25_send(uint8_t *out, const uint8_t *in, size_t len);

int32_t
ax25_recv(uint8_t *out, const uint8_t *in, size_t len);

#endif
