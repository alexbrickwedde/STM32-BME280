#include <stm32hal.h>

void BME280_Init(uint8_t address, I2C_HandleTypeDef* hi2c);
void BME280_Deinit(uint8_t address, I2C_HandleTypeDef* hi2c);
uint16_t BME280_Value(uint8_t address, I2C_HandleTypeDef* hi2c);
