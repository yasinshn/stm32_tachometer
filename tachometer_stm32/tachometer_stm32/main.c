
/*
BaudRate: 115200    EXTI_Pin: PB0     USART2_Pin: PA2=TX  PA3=RX	
*/

#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "stdio.h"
#include "stdbool.h"

static __IO uint32_t millisCounter = 0;
static uint32_t timerValue = 0, timerThreshold, oldTime = 0;
static float speed, speedKm;
static uint8_t IT_FLAG = 0;
static uint16_t bantCounter = 0;
float maxSpeed = 0, maxSpeedKm = 0;
char navBuffer[80];


static void printChar(char a){
	while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
	USART_SendData(USART2, a);
}

static void print(uint8_t *c){
	while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
	while(*c){
		printChar(*c++);
	}
}

static uint16_t scan(){
	while(!USART_GetFlagStatus(USART2, USART_FLAG_RXNE));
	return USART_ReceiveData(USART2);
}


void SysTick_Handler(){
	millisCounter++;
}

static uint32_t millis(){
	return millisCounter;
}

// Not Used For Now -- Delay Function
static void wait(unsigned int nCount)
{
	unsigned int i, j;
	for (i = 0; i < nCount; i++)
		for (j = 0; j < 0x2AFF; j++);
}

static __inline uint8_t debounce(GPIO_TypeDef* port, uint16_t pin)
{
	uint8_t pin_state = GPIO_ReadInputDataBit(port, pin);
	static uint16_t State = 0; // Current debounce status
	State=(State<<1) | !pin_state | 0xe000; // pin_state may change !!!!
	if(State==0xf000)return 1;
	return 0; 
}

void EXTI0_IRQHandler(){
	__disable_irq();
	if(EXTI_GetITStatus(EXTI_Line0) )
	{
		if(debounce(GPIOB, GPIO_Pin_0))
		{
			GPIOB->ODR ^= (GPIO_Pin_14 | GPIO_Pin_15);
			IT_FLAG++;
			oldTime = timerValue;
			timerValue = millis();
			//IT_EN_Counter = millis();
			//DIS_IT;
			//wait(50);
		}
		
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	__enable_irq();
}


static void SysConfig(){
	GPIO_InitTypeDef gpioConf;
	EXTI_InitTypeDef extiConf;
	NVIC_InitTypeDef nvicConf;
	TIM_TimeBaseInitTypeDef timConf;
	USART_InitTypeDef usartConf;
	
	//gpio B0: Input Pullup  
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	gpioConf.GPIO_Mode = GPIO_Mode_IPU;
	gpioConf.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &gpioConf);
	
	// gpio B12 B13 B14 B15 : OUT_PP
	gpioConf.GPIO_Mode = GPIO_Mode_Out_PP;
	gpioConf.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	gpioConf.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &gpioConf);
	
	// EXTI on Line0 (B0)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
	
	extiConf.EXTI_Line = EXTI_Line0;
	extiConf.EXTI_LineCmd = ENABLE;
	extiConf.EXTI_Mode = EXTI_Mode_Interrupt;
	extiConf.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&extiConf);
	
	//NVIC on EXTI0_ IRQHandler
	nvicConf.NVIC_IRQChannel = EXTI0_IRQn;
	nvicConf.NVIC_IRQChannelCmd = ENABLE;
	nvicConf.NVIC_IRQChannelPreemptionPriority = 0;
	nvicConf.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&nvicConf);
	
	//TIM2 -- Kullanilmadi
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	timConf.TIM_ClockDivision = 1;
	timConf.TIM_CounterMode = TIM_CounterMode_Up;
	timConf.TIM_Period = 3999; // Reset Time
	timConf.TIM_Prescaler = 3599; // 72 MHz / 3600 = 20000 Hz = 20 KHz
	TIM_TimeBaseInit(TIM2, &timConf);
	TIM_Cmd(TIM2, ENABLE);
	
	// USART 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	usartConf.USART_BaudRate = 115200;
	usartConf.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usartConf.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usartConf.USART_Parity = USART_Parity_No;
	usartConf.USART_StopBits = USART_StopBits_1;
	usartConf.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &usartConf);
	USART_Cmd(USART2, ENABLE);
	
	
	// USART Tx Rx Pins
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	gpioConf.GPIO_Mode = GPIO_Mode_AF_PP;
	gpioConf.GPIO_Pin = GPIO_Pin_2;
	gpioConf.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpioConf);
	
	gpioConf.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	gpioConf.GPIO_Pin = GPIO_Pin_3;
	gpioConf.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &gpioConf);
	
	// Tick on each 1 ms ( For millis)
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
	
}


static float max (float a, float b){
	return (a > b)? a : b;
}

int main(){
	SysConfig();
	uint32_t millisPrev = 0;
	while(1){
		
		if(millis() - millisPrev > 500){
			GPIOB->ODR ^= GPIO_Pin_13;
			millisPrev = millis();
		}
		if(IT_FLAG){
			bantCounter++;
			timerThreshold = timerValue - oldTime + 50;
			if(timerThreshold > 0)
				speed = 4000.0 / timerThreshold;
			maxSpeed = max(maxSpeed, speed);
			speedKm = speed * 3.6;
			maxSpeedKm = max(maxSpeedKm, speedKm);
			sprintf(navBuffer,"Hiz: %.2f -- Bant No: %d -- Hiz(Km): %.2f -- Max Hiz: %.2f km\n",
			speed, bantCounter, speedKm, maxSpeedKm);
			print(navBuffer);
			IT_FLAG--;
		}
		
		/*
		if(timerMillis == 0)
			GPIOB->ODR ^= GPIO_Pin_14;
		*/
	}
}