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

#include "comms_manager.h"
#include "ax25.h"
#include "status.h"
#include "cc_tx_init.h"
#include "cc_rx_init.h"
#include "persistent_mem.h"
#include "utils.h"
#include "log.h"
#include "services.h"
#include "pkt_pool.h"
#include "service_utilities.h"
#include "comms.h"
#include "verification_service.h"
#include "large_data_service.h"
#include "sensors.h"

#undef __FILE_ID__
#define __FILE_ID__ 25

static uint8_t interm_buf[AX25_PREAMBLE_LEN + AX25_POSTAMBLE_LEN + AX25_MAX_FRAME_LEN + 2];
static uint8_t spi_buf[AX25_PREAMBLE_LEN + AX25_POSTAMBLE_LEN + AX25_MAX_FRAME_LEN];
static uint8_t recv_buffer[AX25_MAX_FRAME_LEN];
static uint8_t send_buffer[AX25_MAX_FRAME_LEN];

volatile extern uint8_t rx_sync_flag;

extern UART_HandleTypeDef huart5;
extern IWDG_HandleTypeDef hiwdg;
extern struct _comms_data comms_data;
comms_rf_stat_t comms_stats;
/**
 * Used to delay the update of the internal statistics and save some cycles
 */
static uint32_t delay_cnt;

/**
 * Disables the TX RF
 */
static inline void
rf_tx_shutdown()
{
  comms_write_persistent_word(__COMMS_RF_OFF_KEY);
}

/**
 * Enables the TX RF
 */
static inline void
rf_tx_enable()
{
  comms_write_persistent_word(__COMMS_RF_ON_KEY);
}

/**
 * Checks if the TX is enabled or not.
 * For robustness, this routine does not check the flash value for equality.
 * Instead, it counts the number of same bits and if the number is greater than 16 the
 * stored value is considered as true. Otherwise, false.
 * @return 0 if the TX is disabled, 1 if it is enabled
 */
uint8_t
is_tx_enabled()
{
  uint32_t cnt;
  uint32_t val;

  val = comms_read_persistent_word();
  cnt = bit_count(val ^ __COMMS_RF_ON_KEY);
  if(cnt < 16) {
    return 1;
  }
  return 0;
}

/**
 * Checks if the received frame contains an RF switch command for the COMMS
 * subsystem
 * @param in the payload of the received frame
 * @param len the length of the payload
 * @return 0 in case the received payload was not an RF switch command,
 * 1 if it was.
 */
static inline uint8_t
check_rf_switch_cmd(const uint8_t *in, size_t len)
{
  uint32_t i;
  uint8_t flag = 0;
  uint32_t *id_ptr;
  uint32_t cmd_hdr_len = strlen(__COMMS_RF_SWITCH_CMD);
  uint32_t cmd_id_len_bytes = sizeof(__COMMS_RF_SWITCH_ON_CMD);

  /* Due to length restrictions, this is definitely not an RF switch command */
  if(len < cmd_hdr_len || len < cmd_hdr_len + cmd_id_len_bytes){
    return 0;
  }

  /*
   * RF switch command is intended only for the COMMS subsystem.
   * There is no ECSS structure here.
   */

  /* This was not an RF switch command. Proceed normally */
  if(strncmp((const char *)in, __COMMS_RF_SWITCH_CMD, cmd_hdr_len) != 0){
    return 0;
  }

  /*
   * Perform a second search now, based on the commands IDs.
   * Due to the space harmful environment do not be so strict
   * and accept the command if one of the ID integers are correct.
   */
  id_ptr = (uint32_t *)(in + cmd_hdr_len);
  for(i = 0; i < cmd_id_len_bytes / sizeof(uint32_t); i++) {
    flag |= (id_ptr[i] == __COMMS_RF_SWITCH_ON_CMD[i]);
  }

  if(flag){
    rf_tx_enable();
    return 1;
  }

  /*
   * The previous command was not for switching on the RF. Perhaps it is for
   * shutting it down
   */
  flag = 0;
  for(i = 0; i < cmd_id_len_bytes / sizeof(uint32_t); i++) {
    flag |= (id_ptr[i] == __COMMS_RF_SWITCH_OFF_CMD[i]);
  }

  if(flag) {
    rf_tx_shutdown();
    return 1;
  }
  return 0;
}

/**
 * This function receives a valid frame from the uplink interface and extracts
 * its payload.
 * Valid frames are considered those that have passed the AX.25 CRC-16 and
 * have in the Destination Address field the appropriate destination callsign.
 *
 * @param out the output buffer, large enough to hold a valid payload
 * @param len the size of the buffer
 * @param timeout_ms the timeout limit in milliseconds
 * @return the size of the AX.25 payload or appropriate error code
 */
int32_t
recv_payload(uint8_t *out, size_t len, size_t timeout_ms)
{
  int32_t ret;
  uint8_t check;
  if(len > AX25_MAX_FRAME_LEN) {
    return COMMS_STATUS_BUFFER_OVERFLOW;
  }

  memset(spi_buf, 0, sizeof(spi_buf));
  ret = rx_data(interm_buf, len, timeout_ms);
  if(ret < 1){
    return ret;
  }

  /* Now check if the frame was indented for us */
  check = ax25_check_dest_callsign(interm_buf, (size_t)ret, __UPSAT_CALLSIGN);
  if(!check){
    return COMMS_STATUS_INVALID_FRAME;
  }

  /*
   * NOTE: UPSat frames using only Short Address field.
   */
  ret = ax25_extract_payload(out, interm_buf,
			     (size_t) ret, AX25_MIN_ADDR_LEN, 1);

  /* Now check if the received frame contains an RF swicth ON/OFF command */
  if(ret > 0){
    check = check_rf_switch_cmd(out, ret);
    if(check){
      return COMMS_STATUS_RF_SWITCH_CMD;
    }
  }
  return ret;
}

/**
 * Send a payload using AX.25 encoding
 * @param in the payload buffer
 * @param len the length of the payload in bytes
 * @param timeout_ms the timeout limit in milliseconds
 * @return number of bytes sent or appropriate error code
 */
int32_t
send_payload(const uint8_t *in, size_t len, size_t timeout_ms)
{
  int32_t ret;

  if(len > AX25_MAX_FRAME_LEN) {
    return COMMS_STATUS_BUFFER_OVERFLOW;
  }

  /* Check if the TX is enabled */
  if(!is_tx_enabled()){
    return COMMS_STATUS_RF_OFF;
  }

  memset(spi_buf, 0, sizeof(spi_buf));
  ret = tx_data(in, len, spi_buf, timeout_ms);
  return ret;
}

/**
 * Sends a payload using CW Morse code
 * @param in the payload buffer
 * @param len the length of the payload in bytes
 * @return the number of bytes sent or an appropriate error code
 */
int32_t
send_payload_cw(const uint8_t *in, size_t len)
{
  int32_t ret;

  if(len > AX25_MAX_FRAME_LEN) {
    return COMMS_STATUS_BUFFER_OVERFLOW;
  }

  /* Check if the TX is enabled */
  if(!is_tx_enabled()){
    return COMMS_STATUS_RF_OFF;
  }

  ret = tx_data_cw(in, len);
  return ret;
}

/**
 * Sends a CW beacons based on the internal COMMS statistics tracking mechanism
 * @return a negative number in case of error
 */
int32_t
send_cw_beacon()
{
  size_t i = 0;
  memset(send_buffer, 0, AX25_MAX_FRAME_LEN);
  send_buffer[i++] = 'U';
  send_buffer[i++] = 'P';
  send_buffer[i++] = 'S';
  send_buffer[i++] = 'A';
  send_buffer[i++] = 'T';
  send_buffer[i++] = cw_get_temp_char(&comms_stats);
  send_buffer[i++] = cw_get_uptime_hours_char(&comms_stats);
  send_buffer[i++] = cw_get_uptime_mins_char(&comms_stats);
  send_buffer[i++] = cw_get_cont_errors_char(&comms_stats);
  send_buffer[i++] = cw_get_last_error_char(&comms_stats);
  return send_payload_cw(send_buffer, i);
}

/**
 * This dispatcher checks which communication related task should execute.
 * The task may be:
 * 	1. Serve an RX operation because the corresponding IRQ was triggered
 * 	2. Transmit the CW beacon
 * 	3. Flush and restart the RX operation if the CC1120 reached an invalid
 * 	   state.
 *
 * 	NOTE: Normal FSK TX operations are triggered automatically by the
 * 	ECSS services subsystem and more precisely from the route() function.
 *
 * @param send_cw pointer to a boolean indicating if the COMMS should transmit
 * a CW beacon. It will be reset from this function only if the beacon
 * has been succesfully transmitted or the TX subsystem encountered an error
 * during transmission. This is happening because we give a priority to
 * the RX event.
 * @return COMMS_STATUS_OK on success or an appropriate error code.
 */
int32_t
comms_routine_dispatcher(uint8_t *send_cw)
{
  int32_t ret = COMMS_STATUS_OK;
  uint32_t now;

  /* A frame is received */
  if(rx_sync_flag){
    rx_sync_flag = 0;
    ret = recv_payload(recv_buffer, AX25_MAX_FRAME_LEN,
		       COMMS_DEFAULT_TIMEOUT_MS);
    if(ret > 0) {
      ret = rx_ecss(recv_buffer, ret);
      if(ret == SATR_OK){
	comms_rf_stats_frame_received(&comms_stats, FRAME_OK, 0);
	LOG_UART_DBG(&huart5, "All ok %d", ret);
      }
      else{
	comms_rf_stats_frame_received(&comms_stats, !FRAME_OK, ret);
      }
    }
    else{
      comms_rf_stats_frame_received(&comms_stats, !FRAME_OK, ret);
    }
  }
  else if(*send_cw){
    *send_cw = 0;
    ret = send_cw_beacon();
    LOG_UART_DBG(&huart5, "CW %d", ret);
  }
  else{
    import_pkt (OBC_APP_ID, &comms_data.obc_uart);
    export_pkt (OBC_APP_ID, &comms_data.obc_uart);
  }

  large_data_IDLE();


  /*
   * Update the statistics of the COMMS and reset the watchdog
   * if there are strong reasons to do so.
   */
  now = HAL_GetTick();
  if(now - delay_cnt > COMMS_STATS_PERIOD_MS) {
    delay_cnt = now;

    comms_rf_stats_update(&comms_stats);
    if(comms_stats.rx_failed_cnt < 10 && comms_stats.tx_failed_cnt < 5) {
      HAL_IWDG_Refresh(&hiwdg);
    }
  }

  /* Check the RX FIFO status and act accordingly */
  cc_rx_check_fifo_status();

  return ret;
}

/**
 * Make all the necessary initializations for the COMMS subsystem
 */
void
comms_init ()
{
  uint8_t cc_id_tx;
  uint8_t cc_id_rx;

  /* fetch tx id */
  cc_tx_rd_reg (0x2f8F, &cc_id_tx);

  /* fetch rx id */
  cc_rx_rd_reg (0x2f8F, &cc_id_rx);

  /* Configure TX CC1120 */
  tx_registerConfig ();

  HAL_Delay (10);
  cc_tx_rd_reg (0x2f8F, &cc_id_tx);

  //Configure RX CC1120
  rx_register_config ();

  HAL_Delay (10);
  cc_rx_rd_reg (0x2f8F, &cc_id_rx);

  //Calibrate TX
  tx_manualCalibration ();

  cc_tx_rd_reg (0x2f8F, &cc_id_tx);

  //Calibrate RX
  rx_manual_calibration ();

  cc_rx_rd_reg (0x2f8F, &cc_id_tx);

  /* Initialize the TX and RX routines */
  rx_init();
  cw_init();

  large_data_init();

  /*Initialize the COMMS statistics mechanism */
  comms_rf_stats_init(&comms_stats);

  /* Initialize the CC1120 in RX mode */
  cc_rx_cmd(SRX);

  delay_cnt = HAL_GetTick();

  /*Start the watchdog */
  HAL_IWDG_Start(&hiwdg);

  pkt_pool_INIT ();

  /* Wait a little and we are ready! */
  HAL_Delay(1000);
}
