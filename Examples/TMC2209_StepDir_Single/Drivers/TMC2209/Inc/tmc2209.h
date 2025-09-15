/**
 ******************************************************************************
 * @file           	: tmc2209.h
 * @author			: asans
 * @brief          	: 
 ******************************************************************************
 */
#ifndef TMC2209_INC_TMC2209_H_
#define TMC2209_INC_TMC2209_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "pulse_gen.h"
#include <math.h>

/* Exported types ------------------------------------------------------------*/
/**
  * @brief  TMC2209 Status structures definition
  */
typedef enum {
	TMC2209_OK,
	TMC2209_ERROR
} TMC2209_StatusTypeDef;

/**
  * @brief  TMC2209 State structures definition
  */
typedef enum {
	TMC2209_DISABLED,
	TMC2209_ENABLED,
	TMC2209_BUSY
} TMC2209_StateTypeDef;

/**
  * @brief  TMC2209 allowed UART slave addresses definition
  */
typedef enum {
    TMC2209_ADDRESS_0 = 0,
    TMC2209_ADDRESS_1,
    TMC2209_ADDRESS_2,
    TMC2209_ADDRESS_3
} TMC2209_Address_t;

/**
  * @brief  TMC2209 error code definition
  */
typedef enum{
	TMC2209_ERROR_NONE,
	TMC2209_ERROR_CRC
	//TODO
}TMC2209_ErrorTypeDef;

/**
 * @brief  TMC2209 init Structure definition
 */
typedef struct {
	TMC2209_Address_t		Address; 			/*!< UART slave address of the TMC2209 						*/
	uint8_t					Microsteps; 		/*!< Microstep setting (1,2,4,8,16,...,256) 				*/
	uint8_t					IRUN; 				/*!< Run current (0-31) */
	uint8_t					IHOLD; 				/*!< Hold current (0-31) */
	uint8_t					IHOLDDELAY; 		/*!< Delay before switching to hold current (0-15) 			*/
} TMC2209_InitTypeDef;


/**
 * @brief  Structure to define the parameters of a stepper motor.
 */
typedef struct {
    float 		StepAngle;       		/*!< Step angle of the motor in DEGREES 				*/
    float 		ReductionRatio;			/*!< Gear reduction ratio (motor_turns/output_turns).
    										 Negative values indicate that input and output
    										 rotate in opposite directions.						*/
    uint16_t	MaxRPM;					/*!< Maximum rotational speed of the motor shaft in RPM	*/
} Motor_ParamsTypeDef;

/**
 * @brief Motor rotation direction options.
 */
typedef enum {
    DIR_CW = 0,     /*!< Clockwise rotation 							*/
    DIR_CCW,        /*!< Counter-clockwise rotation 					*/
    DIR_SHORTEST    /*!< Rotation along the shortest path to target 	*/
} TMC2209_Direction_t;

/**
 * @brief  TMC2209 handle Structure definition
 * @note Even if you don't like radians, using them later makes calculations much easier 😉
 */
typedef struct __TMC2209_HandleTypeDef{
	TMC2209_InitTypeDef			Init;       		/*!< TMC2209 initialization required parameters 				*/
	Motor_ParamsTypeDef			Motor;				/*!< Structure holding the parameters of the stepper motor 		*/
	PulseGen_HandleTypeDef		PulseGen;			/*!< Associated pulse generator 								*/
	UART_HandleTypeDef 			*huart; 			/*!< UART handle used for communication 						*/
	GPIO_TypeDef 				*DirPort;           /*!< GPIO port of the DIR pin (GPIOA..I)						*/
	uint16_t 					DirPin;             /*!< Specifies the GPIO pin number of the DIR pin.
                           	   	   	   	   	   	   		 @ref GPIO_pins_define 										*/
	GPIO_TypeDef 				*EnPort;            /*!< GPIO port of the ENN pin (GPIOA..I)						*/
	uint16_t 					EnPin;         		/*!< Specifies the GPIO pin number of the ENN pin.
                           	   	   	   	   	   	   		 @ref GPIO_pins_define 										*/
	uint8_t						Microsteps; 		/*!< Microstep setting (1,2,4,8,16,...,256) 					*/
	float						StepAngle;			/*!< Effective step angle of the output shaft in degrees.
														 It represents the effective angle per step after
														 applying microstepping and gear reduction ratio.			*/
	float						TargetAngle;		/*!< Target angle of the output shaft in RADIANS.				*/
	__IO float					CurrentAngle;		/*!< Current angle of the output shaft in RADIANS.				*/
	float						MaxAngularVel;		/*!< Maximum angular velocity of the output shaft (rad/s)		*/
	float						AngularVel;			/*!< Configured angular velocity of the output shaft (rad/s)	*/
	TMC2209_Direction_t			Direction;			/*!< Current rotation direction of the output					*/
	TMC2209_ErrorTypeDef 		LastError; 			/*!< Last error code											*/
	__IO TMC2209_StateTypeDef	State;				/*!< Specifies the current state of the driver					*/
} TMC2209_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
TMC2209_StatusTypeDef TMC2209_Init(TMC2209_HandleTypeDef *htmc, TIM_HandleTypeDef *htim, uint32_t TimChannel, uint32_t MinDeltaCCR);
TMC2209_StatusTypeDef TMC2209_SetAngVel(TMC2209_HandleTypeDef *htmc, float AngVel);
TMC2209_StatusTypeDef TMC2209_Move(TMC2209_HandleTypeDef *htmc, TMC2209_Direction_t Direction);
TMC2209_StatusTypeDef TMC2209_MoveAngle(TMC2209_HandleTypeDef *htmc, float Angle);
TMC2209_StatusTypeDef TMC2209_MoveToNormalized(TMC2209_HandleTypeDef *htmc, float Angle, TMC2209_Direction_t Direction);
TMC2209_StatusTypeDef TMC2209_Stop(TMC2209_HandleTypeDef *htmc);
void TMC2209_Enable(TMC2209_HandleTypeDef *htmc);
void TMC2209_Disable(TMC2209_HandleTypeDef *htmc);
float TMC2209_GetPosition(TMC2209_HandleTypeDef *htmc);

/* Private defines -----------------------------------------------------------*/
#define TMC2209_CLK	12000000U



#ifdef __cplusplus
}
#endif

#endif /* TMC2209_INC_TMC2209_H_ */
