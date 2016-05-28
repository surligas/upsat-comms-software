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

#include "ax25.h"
#include "log.h"
#include <string.h>
#include "stm32f4xx_hal.h"
#include "services.h"

#undef __FILE_ID__
#define __FILE_ID__ 669

extern UART_HandleTypeDef huart5;

static const uint8_t AX25_SYNC_FLAG_MAP_BIN[8] = {0, 1, 1, 1, 1, 1, 1, 0};
uint8_t interm_send_buf[AX25_MAX_FRAME_LEN] = {0};
uint8_t tmp_bit_buf[AX25_MAX_FRAME_LEN * 8 + AX25_MAX_FRAME_LEN] = {0};


/**
 * Calculates the FCS of the AX25 frame
 * @param buffer data buffer
 * @param len size of the buffer
 * @return the FCS of the buffer
 */
uint16_t
ax25_fcs (uint8_t *buffer, size_t len)
{
  uint16_t fcs = 0xFFFF;
  while (len--) {
    fcs = (fcs >> 8) ^ crc16_ccitt_table_reverse[(fcs ^ *buffer++) & 0xFF];
  }
  return fcs ^ 0xFFFF;
}


/**
 * Creates the header field of the AX.25 frame
 * @param out the output buffer with enough memory to hold the address field
 * @param dest_addr the destination callsign address
 * @param dest_ssid the destination SSID
 * @param src_addr the callsign of the source
 * @param src_ssid the source SSID
 */
size_t
ax25_create_addr_field (uint8_t *out, const uint8_t  *dest_addr,
			uint8_t dest_ssid,
			const uint8_t *src_addr, uint8_t src_ssid)
{
  uint16_t i = 0;

  for (i = 0; i < strnlen (dest_addr, AX25_CALLSIGN_MAX_LEN); i++) {
    *out++ = dest_addr[i] << 1;
  }
  /*
   * Perhaps the destination callsign was smaller that the maximum allowed.
   * In this case the leftover bytes should be filled with space
   */
  for (; i < AX25_CALLSIGN_MAX_LEN; i++) {
    *out++ = ' ' << 1;
  }
  /* Apply SSID, reserved and C bit */
  /* FIXME: C bit is set to 0 implicitly */
  *out++ = ((0x0F & dest_ssid) << 1) | 0x60;
  //*out++ = ((0b1111 & dest_ssid) << 1) | 0b01100000;

  for (i = 0; i < strnlen (src_addr, AX25_CALLSIGN_MAX_LEN); i++) {
    *out++ = dest_addr[i] << 1;
  }
  for (; i < AX25_CALLSIGN_MAX_LEN; i++) {
    *out++ = ' ' << 1;
  }
  /* Apply SSID, reserved and C bit. As this is the last address field
   * the trailing bit is set to 1.
   */
  /* FIXME: C bit is set to 0 implicitly */
  *out++ = ((0x0F & dest_ssid) << 1) | 0x61;
  //*out++ = ((0b1111 & dest_ssid) << 1) | 0b01100001;
  return (size_t) AX25_MIN_ADDR_LEN;
}


size_t
ax25_prepare_frame (uint8_t *out, const uint8_t *info, size_t info_len,
		    ax25_frame_type_t type, uint8_t *addr, size_t addr_len,
		    uint16_t ctrl, size_t ctrl_len)
{
  uint16_t fcs;
  uint16_t i = 1;
  if (info_len > AX25_MAX_FRAME_LEN) {
    return 0;
  }

  out[0] = AX25_SYNC_FLAG;
  /* Insert address and control fields */
  if (addr_len == AX25_MIN_ADDR_LEN || addr_len == AX25_MAX_ADDR_LEN) {
    memcpy (out + i, addr, addr_len);
    i += addr_len;
  }
  else {
    return 0;
  }

  if (ctrl_len == AX25_MIN_CTRL_LEN || ctrl_len == AX25_MAX_CTRL_LEN) {
    memcpy (out + i, &ctrl, ctrl_len);
    i += ctrl_len;
  }
  else {
    return 0;
  }

  /*
   * Set the PID depending the frame type.
   * FIXME: For now, only the "No layer 3 is implemented" information is
   * inserted
   */
  if (type == AX25_I_FRAME || type == AX25_UI_FRAME) {
    out[i++] = 0xF0;
  }
  memcpy (out + i, info, info_len);
  i += info_len;

  /* Compute the FCS. Ignore the first flag byte */
  fcs = ax25_fcs (out + 1, i - 1);
  /* The MS bits are sent first ONLY at the FCS field */
  out[i++] = (fcs >> 8) & 0xFF;
  out[i++] = fcs & 0xFF;
  out[i++] = AX25_SYNC_FLAG;

  return i;
}

/**
 * Constructs an AX.25 by performing bit stuffing.
 * @param out the output buffer to hold the frame. To keep it simple,
 * each byte of the buffer holds only one bit. Also the size of the
 * buffer should be enough, such that the extra stuffed bits are fitting
 * on the allocated space.
 *
 * @param out_len due to bit stuffing the output size can vary. This
 * pointer will hold the resulting frame size after bit stuffing.
 *
 * @param buffer buffer holding the data that should be encoded.
 * Note that this buffer SHOULD contain the leading and trailing
 * synchronization flag, all necessary headers and the CRC.
 *
 * @param buffer_len the length of the input buffer.
 *
 * @return the resulting status of the encoding
 */
ax25_encode_status_t
ax25_bit_stuffing (uint8_t *out, size_t *out_len, const uint8_t *buffer,
		   const size_t buffer_len)
{
  uint8_t bit;
  uint8_t prev_bit = 0;
  size_t out_idx = 0;
  size_t cont_1 = 0;
  size_t total_cont_1 = 0;
  size_t i;

  /* Leading FLAG field does not need bit stuffing */
  memcpy (out, AX25_SYNC_FLAG_MAP_BIN, 8 * sizeof(uint8_t));
  out_idx = 8;

  /* Skip the leading and trailing FLAG field */
  buffer++;
  for (i = 0; i < 8 * (buffer_len - 2); i++) {
    bit = (buffer[i / 8] >> (i % 8)) & 0x1;
    out[out_idx++] = bit;

    /* Check if bit stuffing should be applied */
    if (bit & prev_bit) {
      cont_1++;
      total_cont_1++;
      if (cont_1 == 4) {
	out[out_idx++] = 0;
	cont_1 = 0;
      }
    }
    else {
      cont_1 = total_cont_1 = 0;
    }
    prev_bit = bit;

    /*
     * If the total number of continuous 1's is 15 the the frame should be
     * dropped
     */
    if (total_cont_1 >= 14) {
      return AX25_ENC_FAIL;
    }
  }

  /* Trailing FLAG field does not need bit stuffing */
  memcpy (out + out_idx, AX25_SYNC_FLAG_MAP_BIN, 8 * sizeof(uint8_t));
  out_idx += 8;

  *out_len = out_idx;
  return AX25_ENC_OK;
}

ax25_decode_status_t
ax25_decode (uint8_t *out, size_t *out_len, const uint8_t *ax25_frame,
	     size_t len)
{
  size_t i;
  size_t frame_start = UINT_MAX;
  size_t frame_stop = UINT_MAX;
  uint8_t res;
  size_t cont_1 = 0;
  size_t received_bytes = 0;
  size_t bit_cnt = 0;
  uint8_t decoded_byte = 0x0;
  uint16_t fcs;
  uint16_t recv_fcs;

  if(len < 2 * sizeof(AX25_SYNC_FLAG_MAP_BIN)) {
    return AX25_DEC_SIZE_ERROR;
  }

  /* Start searching for the SYNC flag */
  for (i = 0; i < len - sizeof(AX25_SYNC_FLAG_MAP_BIN); i++) {
    res = (AX25_SYNC_FLAG_MAP_BIN[0] ^ ax25_frame[i])
	| (AX25_SYNC_FLAG_MAP_BIN[1] ^ ax25_frame[i + 1])
	| (AX25_SYNC_FLAG_MAP_BIN[2] ^ ax25_frame[i + 2])
	| (AX25_SYNC_FLAG_MAP_BIN[3] ^ ax25_frame[i + 3])
	| (AX25_SYNC_FLAG_MAP_BIN[4] ^ ax25_frame[i + 4])
	| (AX25_SYNC_FLAG_MAP_BIN[5] ^ ax25_frame[i + 5])
	| (AX25_SYNC_FLAG_MAP_BIN[6] ^ ax25_frame[i + 6])
	| (AX25_SYNC_FLAG_MAP_BIN[7] ^ ax25_frame[i + 7]);
    /* Found it! */
    if (res == 0) {
      frame_start = i;
      break;
    }
  }

  /* We failed to find the SYNC flag */
  if (frame_start == UINT_MAX) {
   return AX25_DEC_START_SYNC_NOT_FOUND;
  }

  for (i = frame_start + sizeof(AX25_SYNC_FLAG_MAP_BIN);
      i < len - sizeof(AX25_SYNC_FLAG_MAP_BIN) + 1; i++) {
    /* Check if we reached the frame end */
    res = (AX25_SYNC_FLAG_MAP_BIN[0] ^ ax25_frame[i])
	| (AX25_SYNC_FLAG_MAP_BIN[1] ^ ax25_frame[i + 1])
	| (AX25_SYNC_FLAG_MAP_BIN[2] ^ ax25_frame[i + 2])
	| (AX25_SYNC_FLAG_MAP_BIN[3] ^ ax25_frame[i + 3])
	| (AX25_SYNC_FLAG_MAP_BIN[4] ^ ax25_frame[i + 4])
	| (AX25_SYNC_FLAG_MAP_BIN[5] ^ ax25_frame[i + 5])
	| (AX25_SYNC_FLAG_MAP_BIN[6] ^ ax25_frame[i + 6])
	| (AX25_SYNC_FLAG_MAP_BIN[7] ^ ax25_frame[i + 7]);
    /* Found it! */
    if (res == 0) {
      frame_stop = i;
      break;
    }

    if (ax25_frame[i]) {
      cont_1++;
      decoded_byte |= (1 << bit_cnt);
      bit_cnt++;
    }
    else {
      /* If 5 consecutive 1's drop the extra zero*/
      if (cont_1 >= 5) {
	cont_1 = 0;
      }
      else {
	bit_cnt++;
	cont_1 = 0;
      }
    }

    /* Fill the fully constructed byte */
    if (bit_cnt == 8) {
      out[received_bytes++] = decoded_byte;
      bit_cnt = 0;
      decoded_byte = 0x0;
    }
  }

  if (frame_stop == UINT_MAX ) {
    return AX25_DEC_STOP_SYNC_NOT_FOUND;
  }

  if( received_bytes < AX25_MIN_ADDR_LEN ){
    return AX25_DEC_SIZE_ERROR;
  }

  /* Now check the CRC */
  fcs = ax25_fcs (out, received_bytes - sizeof(uint16_t));
  recv_fcs = (((uint16_t) out[received_bytes - 2]) << 8)
      | out[received_bytes - 1];

  if (fcs != recv_fcs) {
    LOG_UART_DBG(&huart5, "Computed: 0x%02x Recv 0x%02x", fcs, recv_fcs)
    return AX25_DEC_CRC_FAIL;
  }

  *out_len = received_bytes - sizeof(uint16_t);
  return AX25_DEC_OK;
}

int32_t
ax25_send(uint8_t *out, const uint8_t *in, size_t len)
{
  ax25_encode_status_t status;
  uint8_t addr_buf[AX25_MAX_ADDR_LEN] = {0};
  size_t addr_len = 0;
  size_t interm_len;
  size_t ret_len;
  size_t i;
  size_t pad_bits = 0;

  /* Create the address field */
  addr_len = ax25_create_addr_field (addr_buf, UPSAT_DEST_CALLSIGN,
				     UPSAT_DEST_SSID, UPSAT_CALLSIGN,
				     UPSAT_SSID);

  /*
   * Prepare address and payload into one frame placing the result in
   * an intermediate buffer
   */
  interm_len = ax25_prepare_frame (interm_send_buf, in, len, AX25_UI_FRAME,
				   addr_buf, addr_len, UPSAT_AX25_CTRL, 1);

  status = ax25_bit_stuffing(tmp_bit_buf, &ret_len, interm_send_buf, interm_len);
  if( status != AX25_ENC_OK){
    return -1;
  }

  memset(out, 0, ret_len/8 * sizeof(uint8_t));
  /* Pack now the bits into full bytes */
  for (i = 0; i < ret_len; i++) {
    out[i/8] |= tmp_bit_buf[i] << (7 - (i % 8));
  }
  pad_bits = 8 - (ret_len % 8);
  ret_len += pad_bits;
  out[ret_len/8] &= (0xFF << pad_bits);
  return ret_len/8;
}

int32_t
ax25_recv(uint8_t *out, const uint8_t *in, size_t len)
{
  size_t i;
  size_t decode_len;
  ax25_decode_status_t status;

  if(len > AX25_MAX_FRAME_LEN) {
    return AX25_DEC_SIZE_ERROR;
  }

  /* Apply one bit per byte for easy decoding */
  for (i = 0; i < len; i++) {
    tmp_bit_buf[8*i] = (in[i] >> 7) & 0x1;
    tmp_bit_buf[8*i + 1] = (in[i] >> 6) & 0x1;
    tmp_bit_buf[8*i + 2] = (in[i] >> 5) & 0x1;
    tmp_bit_buf[8*i + 3] = (in[i] >> 4) & 0x1;
    tmp_bit_buf[8*i + 4] = (in[i] >> 3) & 0x1;
    tmp_bit_buf[8*i + 5] = (in[i] >> 2) & 0x1;
    tmp_bit_buf[8*i + 6] = (in[i] >> 1) & 0x1;
    tmp_bit_buf[8*i + 7] = in[i]  & 0x1;
  }

  /* Perform the actual decoding */
  status = ax25_decode(out, &decode_len, tmp_bit_buf, len * 8);
  if( status != AX25_DEC_OK){
    return status;
  }
  return (size_t) decode_len;
}

/**
 * Checks if the destination field of an AX.25 frame matched a specific address
 * @param ax25_frame an ax.25 frame, decoded using the \p ax25_recv() function.
 * @param frame_len the size of the decoded AX.25 frame
 * @param dest string with the desired address
 * @return 1 if the \p addr matched the destination address of the AX.25 frame,
 * 0 otherwise.
 */
uint8_t
ax25_check_dest_callsign (const uint8_t *ax25_frame, size_t frame_len,
			  const char *dest)
{
  size_t callsign_len;
  size_t i;

  callsign_len = strnlen(dest, AX25_CALLSIGN_MAX_LEN );

  /* Perform some size sanity checks */
  if(callsign_len < AX25_CALLSIGN_MIN_LEN || callsign_len > frame_len) {
    return 0;
  }

  for(i = 0; i < callsign_len; i++){
    if((ax25_frame[i] >> 1) != dest[i]){
      return 0;
    }
  }

  /* All good, this frame was for us */
  return 1;
}

/**
 * This function extracts the AX.25 payload from an AX.25 frame
 * @param out the output buffer
 * @param in the buffer with the AX.25 frame
 * @param frame_len the AX.25 frame size in bytes
 * @param addr_len the AX.25 address length in bytes
 * @return the size of the payload in bytes or appropriate error code
 */
int32_t
ax25_extract_payload(uint8_t *out, const uint8_t *in, size_t frame_len,
		     size_t addr_len, size_t ctrl_len)
{
  if (!C_ASSERT (out != NULL && in != NULL)) {
    return AX25_DEC_FAIL;
  }

  if(addr_len != AX25_MIN_ADDR_LEN && addr_len != AX25_MIN_ADDR_LEN) {
    return AX25_DEC_SIZE_ERROR;
  }

  if(addr_len + ctrl_len >= frame_len || ctrl_len > 2) {
    return AX25_DEC_SIZE_ERROR;
  }

  /* Skip also the control field and the frame type field */
  memcpy(out, in + addr_len + ctrl_len + 1, frame_len - addr_len - ctrl_len -1);
  return frame_len - addr_len - ctrl_len - 1;
}
