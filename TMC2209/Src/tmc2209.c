/**
 ******************************************************************************
 * @file           	: tmc2209.c
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

/* Private function prototypes -----------------------------------------------*/
static inline float deg2rad(float deg);
static inline float rad2deg(float rad);
static inline float NormaliceAngle(float angle);
static inline float Freq2Vel(TMC2209_HandleTypeDef *htmc, uint32_t freq);
static inline uint32_t Vel2Freq(TMC2209_HandleTypeDef *htmc, float vel);
static inline float CalculateStepAngle(TMC2209_HandleTypeDef *htmc);
static inline uint32_t CalculateMaxStepFreq(TMC2209_HandleTypeDef *htmc);
static inline void TMC2209_SetDirPin(TMC2209_HandleTypeDef *htmc);
static TMC2209_Direction_t TMC2209_GetShortestDir(float CurrentAng, float TargetAng);
static float TMC2209_GetRotationDistanceMod(float CurrentAng, float TargetAng, TMC2209_Direction_t direction);

/* Code ----------------------------------------------------------------------*/

/**
 * @brief  Initializes a TMC2209 stepper driver instance. *
 * @param htmc Pointer to the TMC2209 handle structure to initialize.
 * @param htim Pointer to the timer handle used by the internal pulse generator.
 * @param TimChannel Timer channel used for pulse generation (e.g., TIM_CHANNEL_1).
 * @param MinDeltaCCR Minimum CCR increment for the pulse generator (affects frequency range and resolution).
 * @return TMC2209_StatusTypeDef Returns TMC2209_OK if initialization succeeded,
 *         or TMC2209_ERROR if any step failed.
 */
TMC2209_StatusTypeDef TMC2209_Init(TMC2209_HandleTypeDef *htmc, TIM_HandleTypeDef *htim, uint32_t TimChannel, uint32_t MinDeltaCCR){
	/* Init MicroSteps*/
	htmc->Microsteps = htmc->Init.Microsteps;

	/* Init the effective step angle */
	htmc->StepAngle = CalculateStepAngle(htmc);

	/* Calculate the max step frequency needed*/
	uint32_t max_step_freq = CalculateMaxStepFreq(htmc);
	if(!max_step_freq){
		return TMC2209_ERROR;
	}

	/* Init the pulse generator for steps */
	htmc->PulseGen.htim = htim;
	htmc->PulseGen.TimChannel = TimChannel;
	htmc->PulseGen.Init.MaxPulseFreq = max_step_freq;
	htmc->PulseGen.Init.MinDeltaCCR = MinDeltaCCR;

	if(PulseGen_Init(&(htmc->PulseGen)) == PULSEGEN_ERROR){
		return TMC2209_ERROR;
	}

	/* Calculate the maximum angular velocity of the output (rad/s) */
	htmc->MaxAngularVel = fabsf(Freq2Vel(htmc, htmc->PulseGen.MaxPulseFreq));

	/* Init UART configuration*/
	//TODO

	/* Init TMC2209 state*/
	htmc->State = TMC2209_DISABLED;

	/* Init last error */
	htmc->LastError = TMC2209_ERROR_NONE;

	return TMC2209_OK;
}

/**
 * @brief  Enables the TMC2209 stepper motor driver.
 *
 * @param htmc Pointer to the TMC2209 handle structure.
 */
void TMC2209_Enable(TMC2209_HandleTypeDef *htmc) {
    HAL_GPIO_WritePin(htmc->EnPort, htmc->EnPin, GPIO_PIN_RESET);
    htmc->State = TMC2209_ENABLED;
}

/**
 * @brief  Disables the TMC2209 stepper motor driver.
 *
 * @param htmc Pointer to the TMC2209 handle structure.
 */
void TMC2209_Disable(TMC2209_HandleTypeDef *htmc) {
    HAL_GPIO_WritePin(htmc->EnPort, htmc->EnPin, GPIO_PIN_SET);
    htmc->State = TMC2209_DISABLED;
}

/**
 * @brief  Updates the motor's angular velocity and configures the internal pulse generator accordingly.
 *
 * @param htmc Pointer to the TMC2209 handle structure.
 * @param AngVel Desired angular velocity in radians per second.
 * @return TMC2209_StatusTypeDef Returns TMC2209_OK if successful, or TMC2209_ERROR on failure.
 */
TMC2209_StatusTypeDef TMC2209_SetAngVel(TMC2209_HandleTypeDef *htmc, float AngVel){
	/* Check if its a valid angular velocity*/
	if(!AngVel || fabsf(AngVel)>htmc->MaxAngularVel){
		return TMC2209_ERROR;
	}

	/* Set the step frequency*/
	uint32_t step_freq = Vel2Freq(htmc, AngVel);
	if(PulseGen_SetPulseFreq(&(htmc->PulseGen), step_freq) != PULSEGEN_OK){
		return TMC2209_ERROR;
	}

	/* Update the reached angular velocity */
	htmc->AngularVel =  Freq2Vel(htmc, htmc->PulseGen.PulseFreq);

	return TMC2209_OK;
}

/**
 * @brief  Starts continuous rotation of the stepper motor in the specified direction.
 *
 * @param htmc Pointer to the TMC2209 handle structure.
 * @param Direction Rotation direction (DIR_CW, DIR_CCW). DIR_SHORTEST is not allowed.
 * @return TMC2209_StatusTypeDef Returns TMC2209_OK if the motor started successfully,
 *         or TMC2209_ERROR on failure.
 */
TMC2209_StatusTypeDef TMC2209_Move(TMC2209_HandleTypeDef *htmc, TMC2209_Direction_t Direction){
	/* Set the rotating direction of the output */
	if(Direction == DIR_SHORTEST){
		return TMC2209_ERROR;
	} else{
		htmc->Direction = Direction;
	}

	/* Set the Dir pin level*/
	TMC2209_SetDirPin(htmc);

	/* Start sending steps */
	htmc->State = TMC2209_BUSY;
	if(PulseGen_Start(&(htmc->PulseGen)) != PULSEGEN_OK){
		return TMC2209_ERROR;
	}

	return TMC2209_OK;
}

/**
 * @brief  Stops the TMC2209 stepper motor. Halts the motor by stopping pulse generation and updates
 * 		   the current position and driver state.
 *
 * @param htmc Pointer to the TMC2209 handle structure.
 * @return TMC2209_StatusTypeDef Returns TMC2209_OK if the motor was stopped successfully,
 *         or TMC2209_ERROR on failure.
 */
TMC2209_StatusTypeDef TMC2209_Stop(TMC2209_HandleTypeDef *htmc){
	/* Stop pulse generation */
	if(PulseGen_Stop(&(htmc->PulseGen)) != PULSEGEN_OK){
		return TMC2209_ERROR;
	}

	/* Update current position */
	TMC2209_GetPosition(htmc);

	/* Update state */
	if(htmc->State == TMC2209_BUSY) htmc->State = TMC2209_ENABLED;

	return TMC2209_OK;
}

/**
 * @brief  Rotates the TMC2209 stepper motor by a specified angle (in radians) in the appropriate direction
 * 		   based on the sign of the angle.
 *
 * @param htmc Pointer to the TMC2209 handle structure.
 * @param Angle Rotation angle in radians. Positive for counterclockwise, negative for clockwise.
 * @return TMC2209_StatusTypeDef Returns TMC2209_OK if the movement was started successfully,
 *         or TMC2209_ERROR on failure.
 */
TMC2209_StatusTypeDef TMC2209_MoveAngle(TMC2209_HandleTypeDef *htmc, float Angle){
	/* Check if angle is different from current */
	if(!Angle){
		return TMC2209_OK;
	}

	/* Determine the rotating direction of the output */
	if(Angle > 0){
		htmc->Direction = DIR_CCW;
	} else{
		htmc->Direction = DIR_CW;
	}

	/* Set the Dir pin level*/
	TMC2209_SetDirPin(htmc);

	/* Calculate the amount of steps to be done */
	uint32_t steps = Vel2Freq(htmc, Angle);

	/* Start sending steps */
	htmc->State = TMC2209_BUSY;
	if(PulseGen_GeneratePulses(&(htmc->PulseGen), steps) != PULSEGEN_OK){
		return TMC2209_ERROR;
	}

	return TMC2209_OK;
}

/**
 * @brief  Rotates the motor to the specified angle (in radians, normalized to [0, 2π])
 * 		   using the given direction. If DIR_SHORTEST is selected, the motor will rotate
 * 		   along the shortest path to the target.
 *
 * @param htmc Pointer to the TMC2209 handle structure.
 * @param Angle Target angle in radians (normalized to [0, 2π]).
 * @param Direction Rotation direction (DIR_CW, DIR_CCW, or DIR_SHORTEST).
 * @return TMC2209_StatusTypeDef Returns TMC2209_OK if the movement was started successfully,
 *         or TMC2209_ERROR on failure.
 */
TMC2209_StatusTypeDef TMC2209_MoveToNormalized(TMC2209_HandleTypeDef *htmc, float Angle, TMC2209_Direction_t Direction){
	/* Normalize the target angle */
	Angle = NormaliceAngle(Angle);

	/* Check if angle is different from current */
	if(Angle == htmc->CurrentAngle){
		return TMC2209_OK;
	}

	/* Determine the rotating direction of the output */
	if(Direction == DIR_SHORTEST){
		/* Determine the shortest rotation direction */
		htmc->Direction = TMC2209_GetShortestDir(htmc->CurrentAngle, Angle);
	} else{
		htmc->Direction = Direction;
	}

	/* Set the Dir pin level*/
	TMC2209_SetDirPin(htmc);

	/* Calculate the amount of steps to be done */
	float rotation = TMC2209_GetRotationDistanceMod(htmc->CurrentAngle, Angle, htmc->Direction);
	uint32_t steps = Vel2Freq(htmc, rotation);

	/* Start sending steps */
	htmc->State = TMC2209_BUSY;
	if(PulseGen_GeneratePulses(&(htmc->PulseGen), steps) != PULSEGEN_OK){
		return TMC2209_ERROR;
	}

	return TMC2209_OK;
}

/**
 * @brief Determines the shortest rotation direction to reach a target angle.
 *
 * @param CurrentAng Current angle in radians (0 to 2π)
 * @param TargetAng Target angle in radians (0 to 2π)
 * @return TMC2209_Direction_t Rotation direction: DIR_CW for clockwise, DIR_CCW for counterclockwise
 */
static TMC2209_Direction_t TMC2209_GetShortestDir(float CurrentAng, float TargetAng){
	float delta = TargetAng - CurrentAng;

    /* Normalize delta to the range [-pi, pi] */
	if (delta > M_PI)
        delta -= M_TWOPI;
    else if (delta < -M_PI)
        delta += M_TWOPI;

    /* If delta is negative, the shortest path is clockwise; otherwise, counterclockwise */
    return (delta < 0) ? DIR_CW : DIR_CCW;
}

/**
 * @brief Calculates the rotation distance from CurrentAng to TargetAng in the specified direction.
 *
 * @param CurrentAng Current angle in radians (0 to 2π)
 * @param TargetAng Target angle in radians (0 to 2π)
 * @param direction Desired rotation direction (DIR_CW or DIR_CCW)
 * @return Rotation distance in radians (always positive)
 */
static float TMC2209_GetRotationDistanceMod(float CurrentAng, float TargetAng, TMC2209_Direction_t direction) {
    if (direction == DIR_CW) {
        /* Distance clockwise */
        return fmod(CurrentAng - TargetAng + M_TWOPI, M_TWOPI);
    } else {
        /* Distance counterclockwise */
        return fmod(TargetAng - CurrentAng + M_TWOPI, M_TWOPI);
    }
}


/**
 * @brief  Updates and returns the current position of the motor. Calculates the current angle of the motor
 * 		   based on the number of pulses generated by the internal pulse generator. The result is normalized to [0, 2π].
 *
 * @param htmc Pointer to the TMC2209 handle structure.
 * @return float Current angle of the motor in radians.
 *
 * @note This is a __weak function. If an encoder is used, it can be overridden to provide the position directly from
 * 		 the encoder.
 */
__weak float TMC2209_GetPosition(TMC2209_HandleTypeDef *htmc){
	if(htmc->Direction == DIR_CCW){
		htmc->CurrentAngle = NormaliceAngle(htmc->CurrentAngle + Freq2Vel(htmc, htmc->PulseGen.PulseCount));
	} else{
		htmc->CurrentAngle = NormaliceAngle(htmc->CurrentAngle - Freq2Vel(htmc, htmc->PulseGen.PulseCount));
	}

	return htmc->CurrentAngle;
}

/**
 * @brief Converts an angle from degrees to radians.
 *
 * @param deg Angle in degrees.
 * @return Angle in radians.
 */
static inline float deg2rad(float deg) {
    return deg * M_PI / 180.0f;
}

/**
 * @brief Converts an angle from radians to degrees.
 *
 * @param rad Angle in radians.
 * @return Angle in degrees.
 */
static inline float rad2deg(float rad) {
    return rad * 180.0f / M_PI;
}

/**
 * @brief   Normalize an angle to the range [0, 2π).
 *
 * @param   angle   Input angle in radians.
 * @return  Normalized angle in radians within [0, 2π).
 */
static inline float NormaliceAngle(float angle) {
    /* Reduce angle into the range (-2π, 2π) using floating-point remainder */
    angle = fmodf(angle, M_TWOPI);

    /* Shift negative values into the positive range [0, 2π) */
    if (angle < 0) {
        angle += M_TWOPI;
    }

    return angle;
}


/**
 * @brief Calculates the effective step angle of a motor in radians based on its
 * 		  nominal step angle, the configured microsteps, and the gear reduction ratio.
 *
 * @param htmc Pointer to a TMC2209_HandleTypeDef structure containing motor configuration.
 * @return Step angle in radians.
 */
static inline float CalculateStepAngle(TMC2209_HandleTypeDef *htmc){
	float step_angle_deg = htmc->Motor.StepAngle / htmc->Microsteps / htmc->Motor.ReductionRatio;
	return deg2rad(step_angle_deg);
}

/**
 * @brief  Calculates the maximum step input frequency for the TMC2209 driver.
 * 		This function computes the maximum step frequency based on the motor specifications
 * 		(MaxRPM, step angle) and the configured microstepping.
 * 		It ensures that both the full-step frequency (f_FS) and the step frequency (f_STEP)
 * 		respect the timing limits defined in the TMC2209 datasheet:
 *    		- f_FS must not exceed f_CLK / 512
 *    		- f_STEP must not exceed f_CLK / 2
 *
 * @param  htmc Pointer to a TMC2209 handle structure.
 * @retval Maximum step frequency in Hz if within valid limits, otherwise 0U (indicating
 * 		   the configuration exceeds driver limits).
 */
static uint32_t CalculateMaxStepFreq(TMC2209_HandleTypeDef *htmc){
	/* Calculate full step frequency */
	uint32_t f_fs = (uint32_t)(htmc->Motor.MaxRPM * 360 / 60.0f / htmc->Motor.StepAngle);

	/* F_FS must not exceed TMC_F_CLK/512 */
	if(f_fs > TMC2209_CLK/512){
		return 0U;
	}

	/* Calculate step frequency */
	uint32_t f_step = f_fs * htmc->Microsteps;

	/* F_Step must not exceed TMC_F_CLK/2 */
	if(f_step> TMC2209_CLK/2){
		return 0U;
	}

	return f_step;
}


/**
 * @brief  Calculates the maximum achievable output shaft velocity.
 *
 * @param  htmc Pointer to a TMC2209 handle structure.
 * @retval Absolute maximum output shaft velocity in radians per second.
 */
static inline float Freq2Vel(TMC2209_HandleTypeDef *htmc, uint32_t freq){
	return freq * htmc->StepAngle;
}

static inline uint32_t Vel2Freq(TMC2209_HandleTypeDef *htmc, float vel){
	return (uint32_t)(fabsf(vel / htmc->StepAngle));
}

/**
 * @brief Set the DIR pin state based on whether the reduction ratio is inverting and
 * 		  the target rotation direction.
 *
 * 		  Logic table:
 * 		  +-------------------+-------------------+------------+
 * 		  | Step direction    | Target direction  | DirPin     |
 * 		  +-------------------+-------------------+------------+
 * 		  | CCW               | CW                | HIGH       |
 * 		  | CCW               | CCW               | LOW        |
 * 		  | CW                | CW                | LOW        |
 * 		  | CW                | CCW               | HIGH       |
 * 		  +-------------------+-------------------+------------+
 *
 * @param htmc Pointer to TMC2209 handle structure.
 */
static inline void TMC2209_SetDirPin(TMC2209_HandleTypeDef *htmc){
	uint8_t pin_state = (htmc->StepAngle >= 0) ^ (htmc->Direction == DIR_CCW);
	HAL_GPIO_WritePin(htmc->DirPort, htmc->DirPin, pin_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}



