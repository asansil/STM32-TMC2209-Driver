

# STM32 TMC2209 Stepper Driver Library

C library designed to control stepper motors using the **TMC2209** in a **non-blocking** manner on STM32 microcontrollers.

The TMC2209 offers significant advantages over traditional drivers, such as:  
- **Silent and smooth motion** thanks to StealthChop technology.  
- **Precise current control** to protect the motor and reduce energy consumption.  
- **Flexible communication via UART**, allowing monitoring and adjustment of motor parameters in real time without stopping the main application.  

This library efficiently controls the motor without blocking the main program flow, making it easy to integrate into multitasking applications or more complex control systems.



## 🚀 Features  
- **Non-blocking implementation using interrupts**, allowing the main program to continue running while controlling the motor.  
- **Step generation using general-purpose timers in OC mode** via my [PulseGenerator library](https://github.com/asansil/STM32-Pulse-Generator), enabling precise and flexible pulse generation.  
- **Configurable rotational speed**, taking into account the motor step angle and optional `gear reduction`, for fine control over motor movement.  
- **Multiple motion modes**: continuous rotation, move to a normalized angle, or rotate by a specific angle.  
- **Fully compatible with STM32 HAL**, ensuring easy integration with existing projects.  
- **Support for multiple TMC2209 drivers simultaneously**, ideal for multi-axis systems.  
- **UART-based monitoring and parameter adjustment** (*coming soon*), allowing real-time tuning of motor parameters.



## 📂 Adding the Library to STM32CubeIDE Project

1. Copy the [`TMC2209`](./TMC2209) folder into the `Drivers` directory of your STM32CubeIDE project.

2. In STM32CubeIDE, open your project **Properties → C/C++ General → Paths and Symbols** and add the following include path:
	```text
	/${ProjName}/Drivers/TMC2209/Inc
	```


## ⚙️ How to Use  

The basic workflow for using the library is as follows:

1. **Configure timers in STM32CubeIDE**  
	  - 1. Configure the timers to be used for step generation for the driver. For detailed instructions, please refer to the [PulseGenerator library README](https://github.com/asansil/STM32-Pulse-Generator).  																												

2. **Library implementation**
	- Initialize the TMC2209 handle. The required parameters are:  
	     - **Motor configuration** (`Motor_ParamsTypeDef`):  
	       - `MaxRPM` – maximum rotational speed of the motor.  
	       - `StepAngle` – step angle of the motor in degrees.  
	       - `ReductionRatio` – gear reduction ratio.  
	     - **Driver initialization** (`TMC2209_InitTypeDef`):  
	       - `Address` – the TMC2209 device address.  
	       - `Microsteps` – number of microsteps per full step.  
	     - **Pin configuration**:  
	       - `EnPort` / `EnPin` – enable pin.  
	       - `DirPort` / `DirPin` – direction pin.  
	     - **Timer configuration**:  
	       - `htim` – pointer to the STM32 timer used for step generation.  
	       - `TIM_CHANNEL_x` – timer channel used.  
	       - `MinDeltaCCR` – defines the timer resolution at the maximum step frequency. 
        
		   > **Note:** Increasing MaxRPM or decreasing the MinDeltaCCR expands the available output speed range, but reduces the resolution. For a clearer understanding of how these configuration values affect the output, you can refer to the [`TMC2209.xlsx`](./TMC2209.xlsx) spreadsheet.
      
		   > **Important:** For drivers sharing the same timer, `Motor_ParamsTypeDef` and `MinDeltaCCR` must have identical values.

     - Add the callback function `HAL_TIM_OC_DelayElapsedCallback`.
	   The examples provide callback functions for:  
   	     - **Single driver mode**: [`callbacks.c`](./Examples/TMC2209_StepDir_Single/Core/Src/callbacks.c)  
   	     - **Multiple driver mode**: [`callbacks.c`](./Examples/TMC2209_StepDir_Multiple/Core/Src/callbacks.c)
 


## 📊 Usage Examples  
The [`Examples`](./Examples) folder contains ready-to-use reference projects. These examples have been carried out using the STM32F407 Discovery1 development board, and they make use of the Timer 4 channels connected to the board's LEDs, allowing the pulse outputs to be visually observed.
- [TMC2209_StepDir_Single](./Examples/TMC2209_StepDir_Single):  Configures a single driver in simple mode.  
 It blocks execution until the movement is complete before starting the next command.
```c
/* Initialize the TMC2209 driver and internal structures */
APP_TMC2209_Init();
/* Enable the driver and set the target angular velocity to π/2 rad/s */
TMC2209_Enable(&htmc1);
TMC2209_SetAngVel(&htmc1, M_PI_2);
/* Rotate to π radians (180°) counterclockwise */
TMC2209_MoveToNormalized(&htmc1, M_PI, DIR_CCW);
while(htmc1.State == TMC2209_BUSY); // Wait until movement is complete
HAL_Delay(1000);
/* Rotate to π/4 radians (45°) following the shortest path */
TMC2209_MoveToNormalized(&htmc1, M_PI_4, DIR_SHORTEST);
while(htmc1.State == TMC2209_BUSY); // Wait until movement is complete
HAL_Delay(1000);
/* Rotate two full turns clockwise (-2 * 360°) */
TMC2209_MoveAngle(&htmc1, -M_TWOPI*2);
while(htmc1.State == TMC2209_BUSY); // Wait until movement is complete
HAL_Delay(1000);
/* Move back to 0 radians (0°) following the shortest path */
TMC2209_MoveToNormalized(&htmc1, 0.0f, DIR_SHORTEST);
while(htmc1.State == TMC2209_BUSY); // Wait until movement is complete
HAL_Delay(1000);
/* Disable the driver */
TMC2209_Disable(&htmc1);
```
- [TMC2209_StepDir_Multiple](./Examples/TMC2209_StepDir_Multiple):  Configures 3 drivers in simple mode. .
```c
/* Initialize the Pulse Generator application */

/* Enable the driver and set the target angular velocity */


```

## To Do 🔜

 - [ ] **Support for multiple drivers** – enable simultaneous control of several TMC2209 units.
 - [ ] **Advanced UART control and monitoring** – implement real-time parameter tuning and feedback. 
 - [ ] **Acceleration profiles** – allow configuring speed ramps for smoother motion.  
 - [ ]  **DMA support** – offload step generation to DMA for even more efficient, non-blocking operation.

## 📖 Documentation  
Each function is documented. For further information, please check [`TMC2209.h`](./TMC2209/Inc/tmc2209.h) and [`TMC2209.c`](./TMC2209/Src/tmc2209.c) for the full API.  

## 🤝 Contributions  
Contributions are welcome. If you find a bug or want to suggest an improvement, open an *issue* or submit a *pull request*.
