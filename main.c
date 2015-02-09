#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_misc.h>
#include <math.h>

/*
 * A simple capacitance meter using an external 555 timer in monostable mode
 * along with the input and output capture functionality of one of the
 * internal timers. A small pulse active low pulse is sent to the 555 timer
 * trigger input causing the 555 timer to start charging the capacitor under
 * test. During this, the 555 timer output is high and the internal timer
 * starts counting. Once the 555 timer output then falls from high to low
 * (falling edge), the input capture on the internal timer is triggered,
 * causing an interrupt and indicating the end of the measurement. The value
 * of capacitance is then calculated using the timer settings, processor clock
 * settings and value of external resistor. With a 10k resistor, 48MHz clock
 * and prescaler of 16, the maximum value of capacitance that can be measured
 * is 1.99uF, calculated by: -((16*(2^16-1))/48e6)/(ln(1-2/3)*10k). The minimum
 * measurable value is ~30.34pF though I don't think the accuracy would be
 * particularly good for values this small! If the prescaler is too small and
 * the timer overflows, the result is undefined and will be incorrect.
 *
 * Author: Harris Shallcross
 * Year: 9/2/2015
 *
 *
 *Code and example descriptions can be found on my blog at:
 *www.hsel.co.uk
 *
 *The MIT License (MIT)
 *Copyright (c) 2015 Harris Shallcross
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

//555 trigger pin
#define T_TRIG	GPIO_Pin_0

//555 pulse output
#define T_PLS	GPIO_Pin_1

#define T_TPS	GPIO_PinSource0
#define T_PPS	GPIO_PinSource1
#define T_AF	GPIO_AF_2
#define T_GPIO	GPIOA

//External resistor value in ohms
#define T_EXTR	10000

//Timer prescaler value
#define T_PRESC	0x10

GPIO_InitTypeDef G;
TIM_TimeBaseInitTypeDef TB;
TIM_ICInitTypeDef TI;
TIM_OCInitTypeDef TO;
NVIC_InitTypeDef N;
RCC_ClocksTypeDef RC;

volatile uint8_t TGot = 0;
volatile uint16_t Time = 0;

void TIM2_IRQHandler(void){
	if(TIM_GetITStatus(TIM2, TIM_IT_CC2) == SET){
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		TIM_ITConfig(TIM2, TIM_IT_CC2, DISABLE);

		TO.TIM_OutputState = TIM_OutputState_Disable;
		TIM_OC1Init(TIM2, &TO);

		Time = TIM_GetCapture2(TIM2);
		TGot = 1;
	}
}

int main(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	G.GPIO_Pin = T_TRIG | T_PLS;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_Mode = GPIO_Mode_AF;
	G.GPIO_PuPd = GPIO_PuPd_UP;
	G.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(T_GPIO, &G);

	GPIO_PinAFConfig(T_GPIO, T_PPS, T_AF);
	GPIO_PinAFConfig(T_GPIO, T_TPS, T_AF);

	TIM_Cmd(TIM2, DISABLE);
	TI.TIM_Channel = TIM_Channel_2;
	TI.TIM_ICFilter = 0;
	TI.TIM_ICPolarity = TIM_ICPolarity_Falling;
	TI.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TI.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInit(TIM2, &TI);

	TO.TIM_OCIdleState = TIM_OCIdleState_Set;
	TO.TIM_OCMode = TIM_OCMode_PWM2;
	TO.TIM_OutputState = TIM_OutputState_Enable;
	TO.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TO);

	TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Repetitive);

	TB.TIM_ClockDivision = TIM_CKD_DIV1;
	TB.TIM_CounterMode = TIM_CounterMode_Up;
	TB.TIM_Period = 0xFFFF;
	TB.TIM_Prescaler = T_PRESC;
	TIM_TimeBaseInit(TIM2, &TB);

	TIM_SetCompare1(TIM2, 10);
	TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

	N.NVIC_IRQChannel = TIM2_IRQn;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPriority = 0;
	NVIC_Init(&N);

	RCC_GetClocksFreq(&RC);

	float C, TimerT;
	uint8_t Data = 0;

	while(1)
	{
		TIM_Cmd(TIM2, DISABLE);
		TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);

		TO.TIM_OutputState = TIM_OutputState_Enable;
		TIM_OC1Init(TIM2, &TO);
		TIM_SetCompare1(TIM2, 10);

		TIM_SetCounter(TIM2, 0xFFFF-2);
		TGot = 0;
		TIM_Cmd(TIM2, ENABLE);
		while(!TGot);

		//Successful data
		if(TGot == 1){
			TimerT = (float)(T_PRESC)*(float)Time/(float)RC.PCLK_Frequency;

			//Capacitance in F
			C = (double)-TimerT/(log(1.0 - 2.0/3.0) * (double)T_EXTR);
			Data = 1;
		}
	}
}
