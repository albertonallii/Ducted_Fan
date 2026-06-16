/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - OPEN LOOP MODE (MATLAB control)
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include "config.h"
#include "pwm.h"
#include "pid.h"
#include "tof.h"
#include "imu.h"

#include "bno055.h"
#include "bno055_stm32_hal.h"
#include "vl53l1_api.h"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
typedef enum {
    ACK_NONE = 0,
    ACK_ON,
    ACK_OFF,
    ACK_KILL,
    ACK_ERR
} ack_t;

typedef enum {
    STOP_NONE = 0,
    STOP_CMD_TIMEOUT,
    STOP_UART_ERROR,
    STOP_RX_OVERFLOW,
    STOP_MOTOR_OFF,
    STOP_KILL_CMD,
    STOP_BUTTON
} stop_reason_t;

volatile stop_reason_t stop_reason = STOP_NONE;

volatile ack_t ack_pending = ACK_NONE;

volatile bool motor_kill_latched = false;

volatile uint32_t uart_rx_errors = 0;
volatile uint32_t uart_rx_overflows = 0;
volatile uint32_t uart_bad_lines = 0;
volatile uint32_t last_cmd_tick_ms = 0;
volatile uint32_t cmd_timeouts = 0;
#define CMD_TIMEOUT_MS 300000000U

volatile int16_t top_pwm = LOWER_LIMIT_MOTOR;
volatile int16_t bottom_pwm = LOWER_LIMIT_MOTOR;
volatile uint16_t roll_pwm = CENTER_SERVO;
volatile uint16_t pitch_pwm = CENTER_SERVO;

volatile bool motors_enabled = false;
volatile bool actuate_servo_control = false;
volatile bool actuate_motors_control = false;
/* ---------------- TELEMETRY TIMING ----------------
 * TIM3 genera il tick della telemetria.
 * A 50 Hz il periodo teorico è 20 ms.
 */
#define TELEMETRY_PERIOD_MS      20U
#define TELEMETRY_MAX_PENDING    5U

volatile uint32_t telemetry_pending = 0;
volatile uint32_t telemetry_time_ms = 0;
volatile uint32_t telemetry_overrun = 0;

/* ---------------- UART DMA ---------------- */
volatile bool uart_tx_busy = false;
char telemetry_tx_buffer[STANDARD_MESSAGE_LENGTH];

uint8_t rx_byte;
char rx_buffer[128];                  /* esteso per accogliere CMD,xxx,xxx,xxx,xxx */
volatile uint8_t rx_index = 0;

volatile uint8_t current_number_of_toggles = 0;


volatile uint16_t ext_top_pwm    = LOWER_LIMIT_MOTOR;
volatile uint16_t ext_bottom_pwm = LOWER_LIMIT_MOTOR;
volatile uint16_t ext_roll_pwm   = CENTER_SERVO;
volatile uint16_t ext_pitch_pwm  = CENTER_SERVO;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static inline void motor_force_off_immediate(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void motor_force_off_immediate(void)
{
    motors_enabled = false;

    ext_top_pwm    = LOWER_LIMIT_MOTOR;
    ext_bottom_pwm = LOWER_LIMIT_MOTOR;

    top_pwm        = LOWER_LIMIT_MOTOR;
    bottom_pwm     = LOWER_LIMIT_MOTOR;

    set_pwm_motors(LOWER_LIMIT_MOTOR, LOWER_LIMIT_MOTOR);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/* Activate HSEM notification for Cortex-M4*/
	HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
	/*
	 Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
	 perform system initialization (system clock config, external memory configuration.. )
	 */
	HAL_PWREx_ClearPendingEvent();
	HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE,
	PWR_D2_DOMAIN);
	/* Clear HSEM flag */
	__HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	/*-------------------------------------------------------------------------------------------------------*/
	/*		   				    INITIALIZATION OF VARIABLES AND STRUCTURES		   	   			     */
	/*-------------------------------------------------------------------------------------------------------*/

	char uart_msg[STANDARD_MESSAGE_LENGTH];

	/*------------------------------ CONTROL-RELATED STRUCTURES ------------------------------*/

	struct bno055_t myBNO;

	/* PID structures: kept for compatibility, NOT USED in open-loop mode */
	pid_controller_t pid_roll, pid_pitch, pid_motor, pid_yaw;
	median_filter_t  tof_filter = {0};          /* Zero-init is safe */
	imu_angles_t     imu_ref;
	imu_angles_t     imu_now;
	float alt_mm = 0.0f;



	/* Debug timing */

	VL53L1_RangingMeasurementData_t rangingData;
	VL53L1_Dev_t vl53l1_c;
	VL53L1_DEV Dev = &vl53l1_c;

	/*-------------------------------------------------------------------------------------------------------*/
	/*		   				    			  SAFE STARTUP				      			 				 */
	/*-------------------------------------------------------------------------------------------------------*/

	// Ensure button is not held down during power-up
	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
		__WFI();
	}

	HAL_UART_Transmit(&huart3, (uint8_t*) "Waiting safe startup (press user button or sends something to serial)...\n", strlen("Waiting safe startup (press user button or sends something to serial)...\n"),
	HAL_MAX_DELAY);

	// Setup USART to listen for exactly 1 byte (used for start/stop from serial)
	HAL_UART_Receive_IT(&huart3, &rx_byte, 1);


	HAL_UART_Transmit(&huart3, (uint8_t*) "Initialization starting in 5 seconds\n", strlen("Initialization starting in 5 seconds\n"),
	HAL_MAX_DELAY);

	safe_startup(NUMBER_OF_TOGGLES);

	/*-------------------------------------------------------------------------------------------------------*/
	/*		   				    	 SENSOR INITIALIZATION				      			 				 */
	/*-------------------------------------------------------------------------------------------------------*/

	/*------------------------------ VL53L1X INITIALIZATION ------------------------------*/

	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET); // Power the altitude sensor

	// Hardware reset the sensor with XSHUT (PE14 starts LOW from MX_GPIO_Init)
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_Delay(10);

	VL53L1_Error status = VL53L1_total_platform_init(Dev, 0x52, VL53L1_I2C, 400, 20000, 33);

	if (status == VL53L1_ERROR_NONE) {
		HAL_UART_Transmit(&huart3, (uint8_t*) "Altitude sensor (VL53L1X) initialized\n", strlen("Altitude sensor (VL53L1X) initialized\n"),
		HAL_MAX_DELAY);
	} else {
		sprintf(uart_msg, "Altitude sensor (VL53L1X) error: %d\n", status);
		HAL_UART_Transmit(&huart3, (uint8_t*) uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
		return 1;
	}

	/*------------------------------ BNO055 INITIALIZATION ------------------------------*/

	myBNO.bus_read = bus_read;
	myBNO.bus_write = bus_write;
	myBNO.delay_msec = delay_func;
	myBNO.dev_addr = BNO055_I2C_ADDR1;
	bno055_init(&myBNO);

	// Enter config mode to remap axis
	bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);
	HAL_Delay(20);  //BNO055 requires ~19ms to enter CONFIG mode

	// Axis mapping: unchanged (default = 0x24 → X=X, Y=Y, Z=Z)
	bno055_set_axis_remap_value(BNO055_DEFAULT_AXIS);

	// Y e Z inverted to compensate flip of 180° over Y
	bno055_set_remap_x_sign(BNO055_REMAP_AXIS_NEGATIVE);
	bno055_set_remap_y_sign(BNO055_REMAP_AXIS_POSITIVE);
	bno055_set_remap_z_sign(BNO055_REMAP_AXIS_NEGATIVE);

	bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	//bno055_calibration();
	imu_set_reference(&imu_ref);          // Capture "level" orientation
	HAL_UART_Transmit(&huart3, (uint8_t*) "IMU (BNO055) initialized\n", strlen("IMU (BNO055) initialized\n"), HAL_MAX_DELAY);

	/*-------------------------------------------------------------------------------------------------------*/
	/*                                   PID INITIALIZATIONS (kept for compatibility)                       */
	/*                                   NOT ACTIVE: external_control = true by default                     */
	/*-------------------------------------------------------------------------------------------------------*/

	pid_init(&pid_roll,
	         FLIGHT_CFG.servo_roll.kp, FLIGHT_CFG.servo_roll.ki, FLIGHT_CFG.servo_roll.kd,
	         FLIGHT_CFG.servo_roll.sample_time,
	         FLIGHT_CFG.servo_roll.out_min, FLIGHT_CFG.servo_roll.out_max,
	         FLIGHT_CFG.servo_roll.out_offset,
	         FLIGHT_CFG.servo_roll.lpf_alpha,
	         FLIGHT_CFG.servo_roll.deriv_on_measurement);

	pid_init(&pid_pitch,
	         FLIGHT_CFG.servo_pitch.kp, FLIGHT_CFG.servo_pitch.ki, FLIGHT_CFG.servo_pitch.kd,
	         FLIGHT_CFG.servo_pitch.sample_time,
	         FLIGHT_CFG.servo_pitch.out_min, FLIGHT_CFG.servo_pitch.out_max,
	         FLIGHT_CFG.servo_pitch.out_offset,
	         FLIGHT_CFG.servo_pitch.lpf_alpha,
	         FLIGHT_CFG.servo_pitch.deriv_on_measurement);

	pid_init(&pid_motor,
	         FLIGHT_CFG.motor.kp, FLIGHT_CFG.motor.ki, FLIGHT_CFG.motor.kd,
	         FLIGHT_CFG.motor.sample_time,
	         FLIGHT_CFG.motor.out_min, FLIGHT_CFG.motor.out_max,
	         FLIGHT_CFG.motor.out_offset,
	         FLIGHT_CFG.motor.lpf_alpha,
	         FLIGHT_CFG.motor.deriv_on_measurement);

	pid_init(&pid_yaw,
	         FLIGHT_CFG.yaw.kp, FLIGHT_CFG.yaw.ki, FLIGHT_CFG.yaw.kd,
	         FLIGHT_CFG.yaw.sample_time,
	         FLIGHT_CFG.yaw.out_min, FLIGHT_CFG.yaw.out_max,
	         FLIGHT_CFG.yaw.out_offset,
	         FLIGHT_CFG.yaw.lpf_alpha,
	         FLIGHT_CFG.yaw.deriv_on_measurement);

	HAL_UART_Transmit(&huart3, (uint8_t*) "PID structures initialized (OPEN-LOOP MODE: PID inactive)\n",
			strlen("PID structures initialized (OPEN-LOOP MODE: PID inactive)\n"), HAL_MAX_DELAY);

	/*-------------------------------------------------------------------------------------------------------*/
	/*                                   PWM INITIALIZATIONS                                                 */
	/*-------------------------------------------------------------------------------------------------------*/

	start_all_pwm();

	/*-------------------------------------------------------------------------------------------------------*/
	/*		   				    	 ACTUATORS SETUP       				      			 				 */
	/*-------------------------------------------------------------------------------------------------------*/

	// Setup signal for ESC (throttle to the bottom)
	set_pwm_motors(LOWER_LIMIT_MOTOR, LOWER_LIMIT_MOTOR);

	HAL_UART_Transmit(&huart3, (uint8_t*) "Wait 5 seconds for ESC setup (n-beeps and a long beep)...\n", strlen("Wait 5 seconds for ESC setup (n-beeps and a long beep)...\n"), HAL_MAX_DELAY);

	// Delay for ESC setup
	HAL_Delay(5000);

	HAL_UART_Transmit(&huart3, (uint8_t*) "ESC setup completed (ensure no more beeps are emitted)\n",
			strlen("ESC setup completed (ensure no more beeps are emitted)\n"), HAL_MAX_DELAY);

	/*-------------------------------------------------------------------------------------------------------*/
	/*		   				    	 POST INITIALIZATION      				      			 				 */
	/*-------------------------------------------------------------------------------------------------------*/

	HAL_UART_Transmit(&huart3, (uint8_t*) "Initialization completed!\n", strlen("Initialization completed!\n"), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t*) "MODE: OPEN-LOOP (MATLAB control via CMD,top,bottom,roll,pitch)\n",
			strlen("MODE: OPEN-LOOP (MATLAB control via CMD,top,bottom,roll,pitch)\n"), HAL_MAX_DELAY);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); // turn off LD2 (yellow led)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // turn on LD1 (green led)

	// Start measurement LAST because first edge arrives ~25ms from now
	VL53L1_StartMeasurement(Dev);
	/* Start telemetry timing from now, not from boot */

	/*-------------------------------------------------------------------------------------------------------*/
	/*	  TIMER INTERRUPTS START — ONLY AFTER FULL INITIALIZATION (sensors + ESC + PWM all ready)            */
	/*-------------------------------------------------------------------------------------------------------*/

	HAL_TIM_Base_Start_IT(&htim7);  // Servo control actuation @ 50 Hz
	HAL_UART_Transmit(&huart3, (uint8_t*) "Control servos' actuation started\n", strlen("Control servos' actuation started\n"), HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart3, (uint8_t*) "Telemetry actuation started\n", strlen("Telemetry actuation started\n"), HAL_MAX_DELAY);
	/*

	 * Reset contatori telemetria appena prima di avviare TIM3.

	 */

	__disable_irq();

	telemetry_pending = 0;

	telemetry_time_ms = 0;

	telemetry_overrun = 0;

	uart_tx_busy = false;

	__enable_irq();

	/* Avvia TIM3 solo ora */

	HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		/*--------------------------------------------- MOTOR ACTUATION (open-loop) ---------------------------------------------*/
		if (actuate_motors_control) {
			actuate_motors_control = false;
			/* ---- Read ToF (only for telemetry/logging, not used for control in open-loop) ---- */
			VL53L1_GetRangingMeasurementData(Dev, &rangingData);
			VL53L1_ClearInterruptAndStartMeasurement(Dev);

			/* ---- Compute tilt compensation ---- */
			float compensated_alt_mm = tof_compensate_tilt((float)rangingData.RangeMilliMeter, imu_now.roll_deg, imu_now.pitch_deg);

			/* ---- Compute median measurement ---- */
			alt_mm  = median_filter_compute(&tof_filter, compensated_alt_mm);

			/* ---- NO PID in open-loop mode: motor_pwm not computed here ---- */
		}

		/*--------------------------------------------- SERVO-MOTOR ACTUATION (open-loop from MATLAB) ---------------------------------------------*/
		if (actuate_servo_control) {

		    actuate_servo_control = false;

		    /* ---- Read IMU (for telemetry only in open-loop) ---- */
		    imu_read_relative(&imu_ref, &imu_now);

		    /* ----------- OPEN LOOP: usa direttamente i comandi MATLAB ----------- */

		    roll_pwm  = ext_roll_pwm;
		    pitch_pwm = ext_pitch_pwm;

		    /*
		     * Watchdog comandi:
		     * se MATLAB smette di mandare CMD, spegni.
		     */
		    if (motors_enabled && !motor_kill_latched) {
		        if ((HAL_GetTick() - last_cmd_tick_ms) > CMD_TIMEOUT_MS) {
		        	 stop_reason = STOP_CMD_TIMEOUT;
		            motor_force_off_immediate();
		            ack_pending = ACK_OFF;
		        }
		    }


		    if (!motors_enabled || motor_kill_latched) {

		        ext_top_pwm    = LOWER_LIMIT_MOTOR;
		        ext_bottom_pwm = LOWER_LIMIT_MOTOR;

		        top_pwm        = LOWER_LIMIT_MOTOR;
		        bottom_pwm     = LOWER_LIMIT_MOTOR;

		        set_pwm_motors(LOWER_LIMIT_MOTOR, LOWER_LIMIT_MOTOR);

		    } else {

		        top_pwm    = (int16_t) ext_top_pwm;
		        bottom_pwm = (int16_t) ext_bottom_pwm;

		        set_pwm_motors(top_pwm, bottom_pwm);
		    }

		    set_pwm_servos(roll_pwm, pitch_pwm);

		}
		if (ack_pending != ACK_NONE && !uart_tx_busy) {

		    const char *ack_msg = NULL;

		    __disable_irq();
		    ack_t ack = ack_pending;
		    ack_pending = ACK_NONE;
		    __enable_irq();

		    switch (ack) {
		        case ACK_ON:
		            ack_msg = "ACK_ON\n";
		            break;

		        case ACK_OFF:
		            ack_msg = "ACK_OFF\n";
		            break;

		        case ACK_KILL:
		            ack_msg = "ACK_KILL\n";
		            break;

		        case ACK_ERR:
		            ack_msg = "ACK_ERR\n";
		            break;

		        default:
		            ack_msg = NULL;
		            break;
		    }

		    if (ack_msg != NULL) {
		        uart_tx_busy = true;

		        if (HAL_UART_Transmit_DMA(&huart3,
		                                  (uint8_t*)ack_msg,
		                                  (uint16_t)strlen(ack_msg)) != HAL_OK) {
		            uart_tx_busy = false;
		        }
		    }
		}

		/*--------------------------------------------- TELEMETRY FULL ---------------------------------------------*/
		/*
		 * Telemetria a 50 Hz:
		 * time_ms,roll_cdeg,pitch_cdeg,yaw_cdeg,alt_mm,top_pwm,bottom_pwm,
		 * roll_pwm,pitch_pwm,motors_enabled,telemetry_overrun
		 */
		if (telemetry_pending > 0 && !uart_tx_busy && ack_pending == ACK_NONE) {

		    uint32_t pending_snapshot;
		    uint32_t latest_time_snapshot;
		    uint32_t sample_time;

		    int roll_cdeg_snapshot;
		    int pitch_cdeg_snapshot;
		    int yaw_cdeg_snapshot;
		    int alt_mm_snapshot;
		    int16_t top_pwm_snapshot;
		    int16_t bottom_pwm_snapshot;
		    uint16_t roll_pwm_snapshot;
		    uint16_t pitch_pwm_snapshot;
		    int motors_enabled_snapshot;

		    __disable_irq();

		    pending_snapshot = telemetry_pending;
		    latest_time_snapshot = telemetry_time_ms;

		    sample_time = latest_time_snapshot - ((pending_snapshot - 1U) * TELEMETRY_PERIOD_MS);

		    telemetry_pending--;

		    /*
		     * Snapshot dei valori da inviare.
		     * Così lo snprintf lavora su valori già copiati.
		     */
		    roll_cdeg_snapshot  = (int)(imu_now.roll_deg * 100.0f);
		    pitch_cdeg_snapshot = (int)(imu_now.pitch_deg * 100.0f);
		    yaw_cdeg_snapshot   = (int)(imu_now.yaw_deg * 100.0f);

		    alt_mm_snapshot = (int)alt_mm;

		    top_pwm_snapshot = top_pwm;
		    bottom_pwm_snapshot = bottom_pwm;
		    roll_pwm_snapshot = roll_pwm;
		    pitch_pwm_snapshot = pitch_pwm;

		    motors_enabled_snapshot = motors_enabled ? 1 : 0;

		    __enable_irq();

		    int len = snprintf(telemetry_tx_buffer,
		                       sizeof(telemetry_tx_buffer),
							   "%lu,%d,%d,%d,%d,%d,%d,%u,%u,%d,%lu,%lu,%lu,%lu,%lu\n",
		                       (unsigned long)sample_time,
		                       roll_cdeg_snapshot,
		                       pitch_cdeg_snapshot,
		                       yaw_cdeg_snapshot,
		                       alt_mm_snapshot,
		                       top_pwm_snapshot,
		                       bottom_pwm_snapshot,
		                       roll_pwm_snapshot,
		                       pitch_pwm_snapshot,
		                       motors_enabled_snapshot,
							   (unsigned long)uart_rx_errors,
							   (unsigned long)uart_rx_overflows,
							   (unsigned long)uart_bad_lines,
							   (unsigned long)telemetry_overrun,
							   (unsigned long)stop_reason);

		    if (len > 0 && len < (int)sizeof(telemetry_tx_buffer)) {

		        uart_tx_busy = true;

		        if (HAL_UART_Transmit_DMA(&huart3,
		                                  (uint8_t*)telemetry_tx_buffer,
		                                  (uint16_t)len) != HAL_OK) {
		            uart_tx_busy = false;
		        }
		    }
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}

	/*-------------------------------------------------------------------------------------------------------*/
	/*                                   SHUTDOWN                                                            */
	/*-------------------------------------------------------------------------------------------------------*/

	// Stop all actuators immediately
	stop_all_pwm(LOWER_LIMIT_MOTOR, CENTER_SERVO);

	HAL_UART_Transmit_DMA(&huart3, (uint8_t*) "Shutdown\n", strlen("Shutdown\n"));

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Turn off LD1 (green led)
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Turn on LD3 (red led)
  /* USER CODE END 3 */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00301242;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00909FCE;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 14999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1125;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 14999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 14999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 712;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 4999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 7499;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 99;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 14999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, VL53L1X_XSHUT_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(POWER_VL53L1X___BNO055_GPIO_Port, POWER_VL53L1X___BNO055_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : VL52L1X_GPIO1_Pin */
  GPIO_InitStruct.Pin = VL52L1X_GPIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(VL52L1X_GPIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L1X_XSHUT_Pin LD2_Pin */
  GPIO_InitStruct.Pin = VL53L1X_XSHUT_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_VL53L1X___BNO055_Pin */
  GPIO_InitStruct.Pin = POWER_VL53L1X___BNO055_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(POWER_VL53L1X___BNO055_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}


/* USER CODE BEGIN 4 */

/*-------------------------------------------------------------------------------------------------------*/
/*		   				INTERRUPT & CALLBACK FUNCTIONS				      					         */
/*-------------------------------------------------------------------------------------------------------*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM6) {
		// Tim 6 emits at 2 Hz
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_1); // Toggle LD2 (yellow led)
		current_number_of_toggles++;
	}

	else if (htim->Instance == TIM7) {
		actuate_servo_control = true;
	}

	else if (htim->Instance == TIM3) {
	    telemetry_time_ms += TELEMETRY_PERIOD_MS;

	    if (telemetry_pending < TELEMETRY_MAX_PENDING) {
	        telemetry_pending++;
	    } else {
	        telemetry_overrun++;
	    }
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {

        /*
         * Pulsante utente = emergenza fisica.
         * Non deve mai accendere i motori, solo spegnerli.
         */
        motor_kill_latched = true;
        motor_force_off_immediate();
        ack_pending = ACK_KILL;
    }

    if (GPIO_Pin == GPIO_PIN_12) {
        actuate_motors_control = true;
    }
}

// When RX pin reads something
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3) {
        return;
    }

    uint8_t b = rx_byte;

    /*
     * Riarmo subito la ricezione.
     * Così riduci il tempo in cui la UART non sta ascoltando.
     */
    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

    if (b == '\n' || b == '\r') {

        if (rx_index == 0) {
            return;
        }

        rx_buffer[rx_index] = '\0';

        /*
         * MOTOR_OFF: priorità assoluta.
         */
        if (strcmp(rx_buffer, "MOTOR_OFF") == 0) {

        	stop_reason = STOP_MOTOR_OFF;
            motor_kill_latched = false;
            motor_force_off_immediate();
            ack_pending = ACK_OFF;
        }

        /*
         * KILL: spegnimento latched.
         */
        else if (strcmp(rx_buffer, "KILL") == 0) {

        	stop_reason = STOP_KILL_CMD;
            motor_kill_latched = true;
            motor_force_off_immediate();
            ack_pending = ACK_KILL;
        }

        /*
         * MOTOR_ON: unico comando che può riabilitare.
         */
        else if (strcmp(rx_buffer, "MOTOR_ON") == 0) {

            motor_kill_latched = false;

            ext_top_pwm    = LOWER_LIMIT_MOTOR;
            ext_bottom_pwm = LOWER_LIMIT_MOTOR;

            top_pwm        = LOWER_LIMIT_MOTOR;
            bottom_pwm     = LOWER_LIMIT_MOTOR;

            set_pwm_motors(LOWER_LIMIT_MOTOR, LOWER_LIMIT_MOTOR);

            motors_enabled = true;
            last_cmd_tick_ms = HAL_GetTick();

            ack_pending = ACK_ON;
        }

        /*
         * CMD,top,bottom,roll,pitch
         */
        else if (strncmp(rx_buffer, "CMD,", 4) == 0) {

            int top, bottom, roll, pitch;

            if (sscanf(rx_buffer, "CMD,%d,%d,%d,%d", &top, &bottom, &roll, &pitch) == 4) {

                if (top    < LOWER_LIMIT_MOTOR) top    = LOWER_LIMIT_MOTOR;
                if (top    > UPPER_LIMIT_MOTOR) top    = UPPER_LIMIT_MOTOR;
                if (bottom < LOWER_LIMIT_MOTOR) bottom = LOWER_LIMIT_MOTOR;
                if (bottom > UPPER_LIMIT_MOTOR) bottom = UPPER_LIMIT_MOTOR;

                if (roll   < LOWER_LIMIT_SERVO) roll   = LOWER_LIMIT_SERVO;
                if (roll   > UPPER_LIMIT_SERVO) roll   = UPPER_LIMIT_SERVO;
                if (pitch  < LOWER_LIMIT_SERVO) pitch  = LOWER_LIMIT_SERVO;
                if (pitch  > UPPER_LIMIT_SERVO) pitch  = UPPER_LIMIT_SERVO;

                ext_roll_pwm  = (uint16_t) roll;
                ext_pitch_pwm = (uint16_t) pitch;

                /*
                 * I motori accettano CMD solo se sono ON
                 * e se non sei in stato KILL.
                 */
                if (motors_enabled && !motor_kill_latched) {

                    ext_top_pwm    = (uint16_t) top;
                    ext_bottom_pwm = (uint16_t) bottom;
                    last_cmd_tick_ms = HAL_GetTick();

                } else {

                    ext_top_pwm    = LOWER_LIMIT_MOTOR;
                    ext_bottom_pwm = LOWER_LIMIT_MOTOR;

                    top_pwm        = LOWER_LIMIT_MOTOR;
                    bottom_pwm     = LOWER_LIMIT_MOTOR;

                    set_pwm_motors(LOWER_LIMIT_MOTOR, LOWER_LIMIT_MOTOR);
                }

            } else {

                uart_bad_lines++;

            }
        }

        else {
            uart_bad_lines++;
        }

        rx_index = 0;
        memset(rx_buffer, 0, sizeof(rx_buffer));
    }

    else {

        if (rx_index < sizeof(rx_buffer) - 1) {

            rx_buffer[rx_index++] = (char)b;

        } else {

            rx_index = 0;
            memset(rx_buffer, 0, sizeof(rx_buffer));

            uart_rx_overflows++;

            stop_reason = STOP_RX_OVERFLOW;
            motor_force_off_immediate();
            ack_pending = ACK_ERR;
        }
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART3) {
        uart_tx_busy = false;
    }
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {

        uart_rx_errors++;
        stop_reason = STOP_UART_ERROR;
        HAL_UART_AbortTransmit(huart);
        uart_tx_busy = false;

        motor_force_off_immediate();

        rx_index = 0;
        memset(rx_buffer, 0, sizeof(rx_buffer));

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

        ack_pending = ACK_ERR;
    }
}


void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        uart_tx_busy = false;
    }
}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
