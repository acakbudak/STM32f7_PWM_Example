/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

#include "tim4_driver.h"

TIM4_pwm_t PWM;

/*
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
*/


int main(void)
{

	tim4_init(); //tim4 initialize

	PWM.freq = (uint16_t)0x1FFF; //setting pwm frequency
	pwm_set(&PWM);

    /* Loop forever */

	while(1)
	{
		/*
		 ***********uncommend if you want to use PWM increase and decrease with more values*******

		for(uint16_t i=0xFFFF; i>0; i=i-100)
		{
			PWM.duty_cyle = (uint16_t)i;
			pwm_set(&PWM);

			for(uint32_t x=0; x<50000;x++);	//delay

		}
		for(uint16_t i=0; i<((uint16_t)0xFFFF); i=i+100)
		{
			PWM.duty_cyle = (uint16_t)i;
			pwm_set(&PWM);

			for(uint32_t x=0; x<50000;x++);	//delay

		}
		*/
		PWM.duty_cyle = Duty_Cycle_0;
		pwm_set(&PWM);
		for(uint32_t x=0; x<500000;x++); 	//delay
		PWM.duty_cyle = Duty_Cycle_25;
		pwm_set(&PWM);
		for(uint32_t x=0; x<500000;x++);	//delay
		PWM.duty_cyle = Duty_Cycle_50;
		pwm_set(&PWM);
		for(uint32_t x=0; x<500000;x++);	//delay
		PWM.duty_cyle = Duty_Cycle_100;
		pwm_set(&PWM);
		for(uint32_t x=0; x<500000;x++);	//delay

	}


}