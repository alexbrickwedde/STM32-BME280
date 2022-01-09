#include <stm32hal.h>

//  BME280_addr = 0x23;

void BME280_Init(uint8_t address, I2C_HandleTypeDef* hi2c)
{
  uint8_t buffer[1];
  buffer[0] = 0x21;
  HAL_I2C_Master_Transmit(hi2c, address << 1, buffer, 1, 1000);
}

void BME280_Deinit(uint8_t address, I2C_HandleTypeDef* hi2c)
{
  uint8_t ucInit = 0x0; // powerdown
  HAL_StatusTypeDef Result = HAL_I2C_Master_Transmit (hi2c, address, &ucInit, 1, 100);
}

uint8_t BME280_Read(uint8_t address, I2C_HandleTypeDef* hi2c, uint8_t* buff)
{
  HAL_StatusTypeDef Result = HAL_I2C_Master_Receive (hi2c, address, buff, 2, 100);
  return Result;
}

uint16_t BME280_Value(uint8_t address, I2C_HandleTypeDef* hi2c)
{
  uint8_t buff[2];
  int32_t valf = 0; //float valf=0;
  if(BME280_Read(address << 1, hi2c, buff)==HAL_OK)
  {
    valf=((buff[0]<<8)|buff[1]);
    return (valf);
  }
  return 0;
}
