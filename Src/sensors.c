#include "sensors.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

struct _temp_sensor
{
  /* ADT7420 Temperature */
  uint16_t temp_raw;
  float temp_c;
  uint32_t timestamp;
};

static struct _temp_sensor temp_sensor = { .temp_raw = 0, .temp_c = 0.0, .timestamp = 0 };

/* Init ADT7420 */
void
init_adt7420 ()
{
  uint8_t i2c_temp[2];

  /* Set to 0 all state values */
  temp_sensor.temp_raw = 0;
  temp_sensor.temp_c = 0;

  i2c_temp[0] = 0;
  i2c_temp[1] = 0;
  HAL_I2C_Mem_Read (&hi2c1, ( ADT7420_ADDRESS << 1), ADT7420_REG_ID, 1,
            i2c_temp, 1, ADT7420_TIMEOUT);
  if (i2c_temp[0] != ADT7420_DEFAULT_ID) {
    ;
  }
  HAL_Delay (10);
  /* Set operation mode */
  i2c_temp[0] = ADT7420_16BIT | ADT7420_OP_MODE_1_SPS;
  i2c_temp[1] = 0x00;
  HAL_I2C_Mem_Write (&hi2c1, ( ADT7420_ADDRESS << 1), ADT7420_REG_CONFIG, 1,
             i2c_temp, 1, ADT7420_TIMEOUT);
}

/* Update values for adt7420 */
float
update_adt7420 ()
{
  uint8_t lsb, msb;
  uint8_t i2c_temp[2];

  HAL_StatusTypeDef resM;
  HAL_StatusTypeDef resL;
  /* Get Temperature */

  i2c_temp[0] = 0;
  i2c_temp[1] = 0;
  HAL_Delay (10);

  resM = HAL_I2C_Mem_Read (&hi2c1, ( ADT7420_ADDRESS << 1), ADT7420_REG_TEMP_MSB, 1,
            i2c_temp, 1, ADT7420_TIMEOUT);
  msb = i2c_temp[0];
  resL = HAL_I2C_Mem_Read (&hi2c1, ( ADT7420_ADDRESS << 1), ADT7420_REG_TEMP_LSB, 1,
            i2c_temp, 1, ADT7420_TIMEOUT);
  lsb = i2c_temp[0];

  if(resM != HAL_OK || resL != HAL_OK ) { return 0; }

  temp_sensor.temp_raw = msb << 8;
  temp_sensor.temp_raw |= lsb;
  if ((temp_sensor.temp_raw >> 15 & 1) == 0) {
    temp_sensor.temp_c = (float) (temp_sensor.temp_raw / 128);
  }
  else {
    temp_sensor.temp_c = (float) (temp_sensor.temp_raw - 65536) / 128;
  }
  temp_sensor.timestamp = HAL_GetTick();

  return temp_sensor.temp_c;
}

uint16_t
get_raw_adt7420 ()
{
    return temp_sensor.temp_raw;
}

float
get_temp_adt7420 ()
{
    return temp_sensor.temp_c;
}

uint32_t
get_timestamp_adt7420 ()
{
    return temp_sensor.timestamp;
}
