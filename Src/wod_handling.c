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
#include "wod_handling.h"
#include "status.h"
#include "comms_manager.h"
#include <string.h>

static comms_wod_t last_wod;

static inline uint8_t
bat_voltage_valid(uint8_t val)
{
  if(val > 129 && val < 197){
    return 1;
  }
  return 0;
}

static inline uint8_t
bat_current_valid(uint8_t val)
{
  if(val > 64){
    return 1;
  }
  return 0;
}

static inline uint8_t
bus_3300mV_current_valid(uint8_t val)
{
  if(val < 115){
    return 1;
  }
  return 0;
}

static inline uint8_t
bus_5000mV_current_valid(uint8_t val){
  return bus_3300mV_current_valid(val);
}

static inline uint8_t
comms_temp_valid(uint8_t val)
{
  if(val > 79){
    return 1;
  }
  return 0;
}

static inline uint8_t
eps_temp_valid(uint8_t val)
{
  return comms_temp_valid(val);
}

static inline uint8_t
bat_temp_valid(uint8_t val)
{
  return comms_temp_valid(val);
}

/**
 * This functions takes a WOD frame from the OBC and transforms it properly
 * in order to transmit it at the Ground Station.
 * @param wod the COMMS internal WOD structure
 * @param obc_wod the buffer with the WOD frame from the OBC
 * @param len the size of the \p obc_wod buffer
 * @return COMMS_STATUS_OK on success or -1 on error.
 */
static int32_t
prepare_wod(comms_wod_t *wod, const uint8_t *obc_wod, size_t len)
{
  size_t i = 0;
  size_t j = 0;
  size_t idx = 0;
  size_t bits_cnt = 0;
  size_t bytes_cnt = 0;
  uint8_t valid;
  uint8_t out_b = 0x0;
  const size_t datasets_num = (len - sizeof(uint32_t)) / WOD_DATASET_SIZE;

  if (wod == NULL || obc_wod == NULL
      || datasets_num == 0|| datasets_num > WOD_MAX_DATASETS) {
    return -1;
  }

  /* The first 4 bytes are the timestamp */
  memcpy(wod->wod, obc_wod, sizeof(uint32_t));

  idx += sizeof(uint32_t);
  bytes_cnt += sizeof(uint32_t);

  /* Iterate through the data sets and check the validity of the measurements */
  for(i = 0; i < datasets_num; i++){
    valid = bat_voltage_valid(obc_wod[idx]);
    valid &= bat_current_valid(obc_wod[idx + 1]);
    valid &= bus_3300mV_current_valid(obc_wod[idx + 2]);
    valid &= bus_5000mV_current_valid(obc_wod[idx + 3]);
    valid &= comms_temp_valid(obc_wod[idx + 4]);
    valid &= eps_temp_valid(obc_wod[idx + 5]);
    valid &= bat_temp_valid(obc_wod[idx + 6]);

    out_b <<= 1;
    out_b |= valid & 0x1;
    bits_cnt++;
    if(bits_cnt == 8 ){
      bits_cnt = 0;
      wod->wod[bytes_cnt++] = out_b;
    }

    for(j = 0; j < WOD_DATASET_SIZE * 8; j++){
      out_b <<= 1;
      out_b |= ((obc_wod[idx + j/8] >> (7 - (j % 8))) & 0x1);
      bits_cnt++;
      if (bits_cnt == 8) {
	bits_cnt = 0;
	wod->wod[bytes_cnt++] = out_b;
      }
    }

    idx += WOD_DATASET_SIZE;
  }

  /* Zero padding may be necessary */
  if(bits_cnt){
    out_b <<= (8 - bits_cnt);
    wod->wod[bytes_cnt++] = out_b;
  }
  wod->len = bytes_cnt;
  return COMMS_STATUS_OK;
}

/**
 * Converts a floating point temperature into a WOD compatible 1 byte value
 * @param val the temperature
 * @return 1 byte with the WOD compatible temperature
 */
uint8_t
wod_convert_temperature(float val)
{
  /* clamp between min and max value */
  val = maxf(val, -15.0);
  val = minf(val, 48.75);
  return (uint8_t)((val + 15.0) / 0.25);
}

void
store_wod_obc(const uint8_t *obc_wod, size_t len)
{
  int32_t ret;
  ret = prepare_wod(&last_wod, obc_wod, len);
  if(ret == COMMS_STATUS_OK){
    last_wod.valid = 1;
    last_wod.tx_cnt = 0;
  }
}

/**
 * Initializes the COMMS WOD structure to the initial defaults
 */
void
comms_wod_init()
{
  memset(&last_wod, 0, sizeof(comms_wod_t));
}

int32_t
comms_wod_tx()
{
  int32_t ret = 0;
  if(last_wod.valid && last_wod.tx_cnt < 6) {
    ret = send_payload(last_wod.wod, last_wod.len, COMMS_DEFAULT_TIMEOUT_MS);
    if(ret > 0){
      last_wod.tx_cnt++;
    }
  }
  else{
    /*TODO: What should be transmitted in this case? */
  }
  return ret;
}
