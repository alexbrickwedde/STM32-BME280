#include <stm32hal.h>
#include "./bme280.h"
#include <delay.h>
struct identifier
{
    uint8_t dev_addr;
    I2C_HandleTypeDef *hi2c;
    TIM_HandleTypeDef *htim;
};

struct bme280_dev dev;
struct identifier id;


int8_t BME280_user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
  struct identifier *pid = intf_ptr;
  HAL_StatusTypeDef Result = HAL_I2C_Mem_Read(pid->hi2c, pid->dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
  return Result == HAL_OK ? BME280_INTF_RET_SUCCESS : BME280_E_COMM_FAIL;
}

int8_t BME280_user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
  struct identifier *pid = intf_ptr;
  HAL_StatusTypeDef Result = HAL_I2C_Mem_Write(pid->hi2c, pid->dev_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
  return Result == HAL_OK ? BME280_INTF_RET_SUCCESS : BME280_E_COMM_FAIL;
}

void BME280_user_delay_us(uint32_t period, void *intf_ptr)
{
  struct identifier *pid = intf_ptr;
//  delay_usec(pid->htim, period);
  HAL_Delay((period / 1000) + 1);
}

int8_t BME280_Init(uint8_t address, I2C_HandleTypeDef *hi2c, TIM_TypeDef *htim) {

  id.hi2c = hi2c;
  id.dev_addr = BME280_I2C_ADDR_PRIM;
  id.htim = htim;

  dev.intf = BME280_I2C_INTF;
  dev.read = BME280_user_i2c_read;
  dev.write = BME280_user_i2c_write;
  dev.delay_us = BME280_user_delay_us;

  dev.intf_ptr = &id;

  return bme280_init(&dev);
// uint8_t buffer[1];
// buffer[0] = 0x21;
// HAL_I2C_Master_Transmit(hi2c, address << 1, buffer, 1, 1000);
}

uint8_t BME280_Read(uint8_t address, I2C_HandleTypeDef *hi2c, struct bme280_data *comp_data) {
  /* Variable to define the result */
  int8_t rslt = BME280_OK;

  /* Variable to define the selecting sensors */
  uint8_t settings_sel = 0;

  /* Variable to store minimum wait time between consecutive measurement in force mode */
  uint32_t req_delay;

  /* Structure to get the pressure, temperature and humidity values */

  /* Recommended mode of operation: Indoor navigation */
  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
  dev.settings.osr_p = BME280_OVERSAMPLING_16X;
  dev.settings.osr_t = BME280_OVERSAMPLING_2X;
  dev.settings.filter = BME280_FILTER_COEFF_16;

  settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL
      | BME280_FILTER_SEL;

  /* Set the sensor settings */
  rslt = bme280_set_sensor_settings(settings_sel, &dev);
  if (rslt != BME280_OK) {
    return rslt;
  }

  /*Calculate the minimum delay required between consecutive measurement based upon the sensor enabled
   *  and the oversampling configuration. */
  req_delay = bme280_cal_meas_delay(&dev.settings);

  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
  if (rslt != BME280_OK) {
    return rslt;
  }

  dev.delay_us(req_delay, dev.intf_ptr);
  rslt = bme280_get_sensor_data(BME280_ALL, comp_data, &dev);
  return rslt;

//      print_sensor_data(&comp_data);

//  HAL_StatusTypeDef Result = HAL_I2C_Master_Receive (hi2c, address, buff, 2, 100);
//  return Result;
}
