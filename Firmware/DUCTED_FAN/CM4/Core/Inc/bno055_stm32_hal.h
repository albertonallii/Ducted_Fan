#ifndef BNO055_STM32_HAL_H
#define BNO055_STM32_HAL_H

#include "stm32h7xx_hal.h"

// Prototipi delle funzioni
void delay_func(uint32_t delay_in_msec);

HAL_StatusTypeDef bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint16_t r_len);
HAL_StatusTypeDef bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t* reg_data, uint16_t wr_len);

/**
 * @brief This function performs the calibration of the BNO055. Calibration is recommended as it
 * 		  improves the reliability of the sensor data and sets an initial reference frame.
 */
void bno055_calibration(void);

#endif // BNO055_STM32_HAL_H
