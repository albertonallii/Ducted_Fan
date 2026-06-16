#include "bno055_stm32_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include <string.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

void delay_func(uint32_t delay_in_msec) {
	HAL_Delay(delay_in_msec);
}

HAL_StatusTypeDef bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t r_len) {
	HAL_StatusTypeDef stato;

	stato = HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg_addr,
	I2C_MEMADD_SIZE_8BIT, reg_data, r_len,
	HAL_MAX_DELAY);

	if (stato != HAL_OK) {
		char messaggio[50];
		sprintf(messaggio, "Errore READ: %d\r\n", stato);
		HAL_UART_Transmit(&huart3, (uint8_t*) messaggio, strlen(messaggio),
		HAL_MAX_DELAY);
	}

	return stato;
}

HAL_StatusTypeDef bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t wr_len) {
	HAL_StatusTypeDef stato;

	stato = HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg_addr,
	I2C_MEMADD_SIZE_8BIT, reg_data, wr_len,
	HAL_MAX_DELAY);

	if (stato != HAL_OK) {
		char messaggio[50];
		sprintf(messaggio, "Errore WRITE: %d\r\n", stato);
		HAL_UART_Transmit(&huart3, (uint8_t*) messaggio, strlen(messaggio),
		HAL_MAX_DELAY);
	}

	return stato;
}

void bno055_calibration(void) {

	unsigned char accel_calib_status = 0;
	unsigned char gyro_calib_status = 0;
	unsigned char mag_calib_status = 0;
	unsigned char sys_calib_status = 0;

	char messaggio[150];

	//lampeggio con i led per comunicare che il sensore si deve calibrare
	for (int i = 0; i < 3; i++) {
		// Accendo tutti e 3 i LED
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_Delay(50); // aspetta 500 ms

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_Delay(50); // aspetta 500 ms
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_Delay(50); // aspetta 500 ms

	do {
		bno055_get_accel_calib_stat(&accel_calib_status);
		bno055_get_mag_calib_stat(&mag_calib_status);
		bno055_get_gyro_calib_stat(&gyro_calib_status);
		bno055_get_sys_calib_stat(&sys_calib_status);

		sprintf(messaggio, "accel_calib_status = %u || gyro_calib_status = %u || mag_calib_status = %u || sys_calib_status = %u  \n\r",
				accel_calib_status, gyro_calib_status, mag_calib_status, sys_calib_status);
		HAL_UART_Transmit(&huart3, (uint8_t*) messaggio, strlen(messaggio),
		HAL_MAX_DELAY);
		HAL_Delay(50);
	} while (accel_calib_status != 3 || gyro_calib_status != 3 || mag_calib_status != 3 || sys_calib_status != 3);

	sprintf(messaggio, "BNO055 calibrato correttamente\n\r");
	HAL_UART_Transmit(&huart3, (uint8_t*) messaggio, strlen(messaggio),
	HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);

}

