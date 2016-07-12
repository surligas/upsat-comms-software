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

#include "stats.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include "sensors.h"
#include "status.h"

extern ADC_HandleTypeDef hadc1;

static inline void
update_temperature(comms_rf_stat_t *h)
{
  int32_t ret;
  ret = stm32_get_temp(&hadc1, &(h->temperature));
  if(ret != COMMS_STATUS_OK){
    h->last_error_code = ret;
  }
}

void
comms_rf_stats_init(comms_rf_stat_t *h)
{
  if(h == NULL){
    return;
  }

  memset(h, 0, sizeof(comms_rf_stat_t));
  h->last_tick = HAL_GetTick();
}


static inline void
update_uptime(comms_rf_stat_t *h)
{
  uint32_t now = HAL_GetTick();
  uint32_t diff = now - h->last_tick;

  h->uptime_s += (diff/1000);
  h->uptime_m += h->uptime_s / 60;
  h->uptime_s %= 60;
  h->uptime_h += h->uptime_m / 60;
  h->uptime_m %= 60;
  h->last_tick = now;
}

void
comms_rf_stats_update(comms_rf_stat_t *h)
{
  update_uptime(h);
  update_temperature(h);
}
