#include <stm32hal.h>

#include "../BME280_driver/bme280.h"

int8_t BME280_Init(uint8_t address, I2C_HandleTypeDef* hi2c, TIM_HandleTypeDef *htim);
uint8_t BME280_Read(uint8_t address, I2C_HandleTypeDef* hi2c, struct bme280_data *comp_data);
