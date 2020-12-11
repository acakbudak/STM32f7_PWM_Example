/**
 ******************************************************************************
 * @file           : main.c
 * @author	   : acakbudak		
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
