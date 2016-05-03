#include "ax25.h"

    const float AX25_SYNC_FLAG_MAP[8] = {-1, 1, 1, 1, 1, 1, 1, -1};
    const uint8_t AX25_SYNC_FLAG_MAP_BIN[8] = {0, 1, 1, 1, 1, 1, 1, 0};





    /**
     * Constructs an AX.25 by performing NRZ encoding and bit stuffing
     * @param out the output buffer to hold the frame. Note that due to
     * the NRZ encoding the output would be [-1, 1]. Also the size of the
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
    static inline ax25_encode_status_t
    ax25_nrz_bit_stuffing (float *out, size_t *out_len, const uint8_t *buffer,
			   const size_t buffer_len)
    {
      uint8_t bit;
      uint8_t prev_bit = 0;
      size_t out_idx = 0;
      size_t bit_idx;
      size_t cont_1 = 0;
      size_t total_cont_1 = 0;
      size_t i;

      /* Leading FLAG field does not need bit stuffing */
      memcpy(out, AX25_SYNC_FLAG_MAP, 8 * sizeof(float));
      out_idx = 8;

      /* Skip the leading and trailing FLAG field */
      buffer++;
      for(i = 0; i < 8 * (buffer_len - 2); i++){
	bit = (buffer[i / 8] >> ( i % 8)) & 0x1;
	out[out_idx++] = bit ? 1.0 : -1.0;

	/* Check if bit stuffing should be applied */
	if(bit & prev_bit){
	  cont_1++;
	  total_cont_1++;
	  if(cont_1 == 4){
	    out[out_idx++] = -1.0;
	    cont_1 = 0;
	  }
	}
	else{
	  cont_1 = total_cont_1 = 0;
	}
	prev_bit = bit;

	/*
	 * If the total number of continuous 1's is 15 the the frame should be
	 * dropped
	 */
	if(total_cont_1 >= 14) {
	  return AX25_ENC_FAIL;
	}
      }

      /* Trailing FLAG field does not need bit stuffing */
      memcpy(out + out_idx, AX25_SYNC_FLAG_MAP, 8 * sizeof(float));
      out_idx += 8;

      *out_len = out_idx;
      return AX25_ENC_OK;
    }

    static inline ax25_decode_status_t
    ax25_decode (uint8_t *out, size_t *out_len,
		 const uint8_t *ax25_frame, size_t len)
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


      /* Start searching for the SYNC flag */
      for(i = 0; i < len - sizeof(AX25_SYNC_FLAG_MAP_BIN); i++) {
	res = (AX25_SYNC_FLAG_MAP_BIN[0] ^ ax25_frame[i]) |
	    (AX25_SYNC_FLAG_MAP_BIN[1] ^ ax25_frame[i + 1]) |
	    (AX25_SYNC_FLAG_MAP_BIN[2] ^ ax25_frame[i + 2]) |
	    (AX25_SYNC_FLAG_MAP_BIN[3] ^ ax25_frame[i + 3]) |
	    (AX25_SYNC_FLAG_MAP_BIN[4] ^ ax25_frame[i + 4]) |
	    (AX25_SYNC_FLAG_MAP_BIN[5] ^ ax25_frame[i + 5]) |
	    (AX25_SYNC_FLAG_MAP_BIN[6] ^ ax25_frame[i + 6]) |
	    (AX25_SYNC_FLAG_MAP_BIN[7] ^ ax25_frame[i + 7]);
	/* Found it! */
	if(res == 0){
	  //std::cout << "Start found at " << i << std::endl;
	  frame_start = i;
	  break;
	}
      }

      /* We failed to find the SYNC flag */
      if(frame_start == UINT_MAX){
	//std::cout << "Frame start was not found" << std::endl;
	return AX25_DEC_FAIL;
      }

      for(i = frame_start + sizeof(AX25_SYNC_FLAG_MAP_BIN);
	  i < len - sizeof(AX25_SYNC_FLAG_MAP_BIN) + 1; i++) {
	/* Check if we reached the frame end */
	res = (AX25_SYNC_FLAG_MAP_BIN[0] ^ ax25_frame[i]) |
	    (AX25_SYNC_FLAG_MAP_BIN[1] ^ ax25_frame[i + 1]) |
	    (AX25_SYNC_FLAG_MAP_BIN[2] ^ ax25_frame[i + 2]) |
	    (AX25_SYNC_FLAG_MAP_BIN[3] ^ ax25_frame[i + 3]) |
	    (AX25_SYNC_FLAG_MAP_BIN[4] ^ ax25_frame[i + 4]) |
	    (AX25_SYNC_FLAG_MAP_BIN[5] ^ ax25_frame[i + 5]) |
	    (AX25_SYNC_FLAG_MAP_BIN[6] ^ ax25_frame[i + 6]) |
	    (AX25_SYNC_FLAG_MAP_BIN[7] ^ ax25_frame[i + 7]);
	/* Found it! */
	if(res == 0){
	  //std::cout << "Stop found at " << i << std::endl;
	  frame_stop = i;
	  break;
	}

	if (ax25_frame[i]) {
	  cont_1++;
	  decoded_byte |= 1 << bit_cnt;
	  bit_cnt++;
	}
	else {
	  /* If 5 consecutive 1's drop the extra zero*/
	  if (cont_1 >= 5) {
	    cont_1 = 0;
	  }
	  else{
	    bit_cnt++;
	    cont_1 = 0;
	  }
	}

	/* Fill the fully constructed byte */
	if(bit_cnt == 8){
	  out[received_bytes++] = decoded_byte;
	  bit_cnt = 0;
	  decoded_byte = 0x0;
	}
      }

      if(frame_stop == UINT_MAX || received_bytes < AX25_MIN_ADDR_LEN){
	//std::cout << "Wrong frame size " << frame_stop <<std::endl;
	return AX25_DEC_FAIL;
      }

      /* Now check the CRC */
      fcs = ax25_fcs (out, received_bytes - sizeof(uint16_t));
      recv_fcs = (((uint16_t) out[received_bytes - 2]) << 8)
      	    | out[received_bytes - 1];

      if(fcs != recv_fcs) {
	//std::cout << "Wrong FCS" << std::endl;
	return AX25_DEC_FAIL;
      }

      *out_len = received_bytes - sizeof(uint16_t);
      return AX25_DEC_OK;

    }
