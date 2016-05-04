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

