#include "pwm.h"
#include "main.h"

extern TIM_HandleTypeDef htim2; // Timer for servos' pwm
extern TIM_HandleTypeDef htim4; // Timer for motors' pwm
extern TIM_HandleTypeDef htim6; // Timer for button interrupt
extern int current_number_of_toggles;


void start_all_pwm(void) {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Start PWM for TIM2-CH2: roll servo
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Start PWM for TIM2-CH2: pitch servo

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // Start PWM for TIM4-CH2: top motor
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // Start PWM for TIM4-CH3: bottom motor


}

void stop_all_pwm(uint16_t lower_limit_motor, uint16_t center_servo) {

	set_pwm_motors(lower_limit_motor, lower_limit_motor);

	set_pwm_servos(center_servo, center_servo);

	HAL_Delay(100); // Let actuators actually get the new value

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Stop PWM for TIM2-CH2: roll servo
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); // Stop PWM for TIM2-CH2: pitch servo

	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2); // Stop PWM for TIM4-CH2: top motor
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3); // Stop PWM for TIM4-CH3: bottom motor

}

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			SERVOS-RELATED FUNCTIONS			      					     */
/*-------------------------------------------------------------------------------------------------------*/

/** Maps a desired physical angle to a Timer CCR value */
uint16_t angle_to_pwm(float target_angle_deg, uint16_t center_servo, float ccr_per_degree, uint16_t upper_limit, uint16_t lower_limit)
{
    /* Calculate raw CCR */
    float target_ccr = center_servo + (target_angle_deg * ccr_per_degree);

    /* Hardware safety clamps (prevents servo binding/stalling) */
    if (target_ccr > upper_limit) return upper_limit;
    if (target_ccr < lower_limit) return lower_limit;

    /* Round to nearest integer */
    return (uint16_t)(target_ccr + 0.5f);
}

void set_pwm_servos(uint16_t ccr_roll, uint16_t ccr_pitch) {

	TIM2->CCR1 = ccr_roll;
	TIM2->CCR2 = ccr_pitch;
}

/*-------------------------------------------------------------------------------------------------------*/
/*					  		 			MOTORS-RELATED FUNCTIONS			      					     */
/*-------------------------------------------------------------------------------------------------------*/

void safe_startup(uint16_t number_of_toggles) {
	// Reset counter before starting
	current_number_of_toggles = 0;

	HAL_TIM_Base_Start_IT(&htim6);

	// Use  __WFI() in all waiting loops to save power and prevents busy-waiting CPU hogs
	while (current_number_of_toggles <= number_of_toggles) {
		__WFI();
	}

	HAL_TIM_Base_Stop_IT(&htim6);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET); // turn on LD2 (yellow led)
}


void set_pwm_motors(uint16_t ccr_top_motor, uint16_t ccr_bottom_motor) {

	TIM4->CCR2 = ccr_top_motor;
	TIM4->CCR3 = ccr_bottom_motor;

}

