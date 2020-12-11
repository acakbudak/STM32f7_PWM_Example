/*
 * tim4_driver.c
 *
 *  Created on: Dec 11, 2020
 *      Author: ahmet
 */

#include "tim4_driver.h"

static void tim4_gpio_init(void);


/*
 *@API: tim4_init();
 *
 *@brief: TIM4 peripheral initialization
 *
 *@param: NONE
 *
 */
void tim4_init(void)
{
	tim4_gpio_init();

	RCC->APB1ENR |= (1<<2);		//enabling tim4 peripheral clock

	TIM4->CR1 |= TIM_CR1_CEN;   //enabling tim4

	/*
	 * PWM initialize
	 */
	TIM4->CCER &= ~TIM_CCER_CC2E;
	//TIM4->CCMR2 &= ~TIM_CCMR1_CC2S_0;				//CC2 channel is configured as output
	TIM4->CCMR1 |= (0x0006UL << TIM_CCMR1_OC2M_Pos); //PWM mode 1 selection
	TIM4->CCMR1 |= TIM_CCMR1_OC2PE;					 //OC2_PE setting
	TIM4->CR1 |= TIM_CR1_ARPE;						 //setting the ARPE bit in the TIM4_CR1 register
	TIM4->EGR |= TIM_EGR_UG;						//etting the UG bit in the TIMx_EGR register
	TIM4->CCER |= TIM_CCER_CC2E;


}


/*
 *@API: tim4_gpio_init();
 *
 *@brief: Driver specific function for GPIOB configurations for TIM4 Channel 2
 *		  Push_pull selected by default
 *		  No pull-up pull_down selected by default
 *
 *@param: NONE
 */
static void tim4_gpio_init(void)
{
	RCC->AHB1ENR |= (1 << 1);					//enabling gpiob peripheral clock

	GPIOB->MODER |= GPIO_MODER_MODER7_1;		//Alternate function mode selection
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1;	//Output High speed selection
	GPIOB->AFR[0] = GPIO_AFRL_AFRL7_1;			//Channel 2 is selected for tim4 alternate function


}

/*
 *@API: pwm_set(TIM4_pwm_t* pHandle);
 *
 *@brief: Settin frequency and duty cyle values for PWM
 *
 *
 *
 *@param: Handle struct temp values
 */

void pwm_set(TIM4_pwm_t* pHandle)
{


	TIM4->ARR = pHandle->freq;
	TIM4->CCR2 = pHandle->duty_cyle;

}
