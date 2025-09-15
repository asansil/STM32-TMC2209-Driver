/**
 ******************************************************************************
 * @file           	: callbacks.c
 * @author			: asans
 * @brief          	: 
 ******************************************************************************
 */


/* Private includes ----------------------------------------------------------*/
#include "tmc2209.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern TMC2209_HandleTypeDef htmc1;

/* Private function prototypes -----------------------------------------------*/

/* Code ----------------------------------------------------------------------*/


/**
 * @brief  Timer output compare callback for Pulse Generator.
 * @param  htim Pointer to the timer handle that triggered the callback.
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
	if(PulseGen_HalfPulseCallback(htim, &(htmc1.PulseGen)) == PULSEGEN_OK){
		/* Update the TMC driver internal state once the rotation to the target angle has finished */
		if(htmc1.PulseGen.State == PULSEGEN_INACTIVE){
			htmc1.State = TMC2209_ENABLED;
			TMC2209_GetPosition(&htmc1);
		}
	}
}
