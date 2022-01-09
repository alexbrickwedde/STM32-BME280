#include <stm32hal.h>
#include "../BME280_driver/bme280.h"
#include "../BME280_driver/bme280_defs.h"

int8_t BME280_Init(uint8_t address, I2C_HandleTypeDef* hi2c, TIM_TypeDef *htim);
uint8_t BME280_Read(uint8_t address, I2C_HandleTypeDef* hi2c);
