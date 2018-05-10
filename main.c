// define time delay for leds
#define DELAY_TIME 50
// define time when button pressed for long
#define PRESSED_TIME 1000
// define baudrate
#define BAUD_RATE 9600


#include "stm32f30x.h"
#include "stm32f30x_conf.h"
#include "stm32f3_discovery.h"
#include "system_stm32f30x.h"

// array of all leds on board (on circle)
uint32_t LEDS[] = {GPIO_ODR_9, GPIO_ODR_10,
                   GPIO_ODR_11, GPIO_ODR_12, GPIO_ODR_13,
                   GPIO_ODR_14, GPIO_ODR_15, GPIO_ODR_8};

// functions to send data trough uart1
void sendToUart(char chr){
   while (!(USART1->ISR & USART_ISR_TC));
   USART1->TDR = chr;
}
void sendStringToUart (char* str){
       uint8_t i = 0;
       while (str[i])
					sendToUart(str[i++]);
}
									 
// block for system timer
volatile uint32_t timestamp = 0;
void SysTick_Handler(void){
  timestamp++;
}

// var to keep state of delay
uint8_t  isDelayed       = 0;

// delay that count ms milliseconds
void delay(uint32_t ms){
	isDelayed = 1;
  uint32_t start = timestamp;
  while((timestamp - start) < ms);
	isDelayed = 0;
}

// var to keep amount of leds in use
uint8_t  a       = 8;
// var to keep state of leds blinking
uint8_t  state   = 0; 
// var to keep index of led, that we need to switch on
uint8_t  i       = 0;


// handler of user button press
void EXTI0_IRQHandler(){
	// clearing pending bits of interrupt
	EXTI->PR |= EXTI_PR_PR0;
	// clear all leds
	GPIOE->ODR = 0x0;
	// changing state
  state = (state + 1)%5;
	// setting index of using led to zero
	i = 0;
}

// handler for usart
void USART1_IRQHandler(){
	if(USART1->ISR & USART_ISR_RXNE) {
		USART1->ISR &= USART_ISR_RXNE;
		char t = USART1->RDR;
		if (t == '0')
			state = 0;
		else if (t == '1')
			state = 1;
		else if (t == '2')
			state = 2;
		else if (t == '3')
			state = 3;
	}
}


int main(){	
  // call time handler every 1 ms
  SysTick_Config(SystemCoreClock / 1E4);
   
	// starting tact on syscfg and usart1 register
  RCC->APB2ENR      |= RCC_APB2ENR_SYSCFGEN | 
											 RCC_APB2ENR_USART1EN;
	// switching on exti0 to pa
  SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
  // starting tact on GPIOE (leds) and GPIOA (user button)
	RCC->AHBENR       |= RCC_AHBENR_GPIOEEN | 
											 RCC_AHBENR_GPIOAEN;
  
	// enable user button
  GPIOA->MODER &= ~(GPIO_MODER_MODER0);
	// enable leds
  GPIOE->MODER |= GPIO_MODER_MODER8_0 | 
                  GPIO_MODER_MODER9_0 | 
                  GPIO_MODER_MODER10_0 | 
                  GPIO_MODER_MODER11_0 | 
                  GPIO_MODER_MODER12_0 | 
                  GPIO_MODER_MODER13_0 | 
                  GPIO_MODER_MODER14_0 | 
                  GPIO_MODER_MODER15_0;
	
	// enable PA9 as push-pull in alternate, clk = 50 MHz
	// and PA10 as input
	GPIOA->MODER |= GPIO_MODER_MODER9_1; // AF for pin 9
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9; // push-pull
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR9_0; // pull-up
	GPIOA->AFR[0] |= GPIO_AFRL_AFRL7 & (7U<<8); // alternate function #7
	
	GPIOA->MODER &= ~GPIO_MODER_MODER10; // reciever, 	input mode
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR10;
	//GPIOA->AFR[0] |= GPIO_AFRL_AFRL7 & 
	
	
	
	// setting UART
	NVIC_EnableIRQ(USART1_IRQn); // doesn't work :C
	//double brr = (SystemCoreClock / 16.0) / 9600.0;
	//USART1->BRR = (unsigned)(brr * 16.0);
	USART1->BRR = 0x1d4c;
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | // enable trans and recieve
								 USART_CR1_RXNEIE | USART_CR1_TXEIE; // enable interrupts
	USART1->CR1 |= USART_CR1_UE; // let usart1 work
	
	
  // setting mask on EXTI
  EXTI->IMR     |= EXTI_IMR_MR0;
	// setting falling trigger event on line 0
  EXTI->FTSR    |= EXTI_FTSR_TR0; 
	// enabling interrupts
	NVIC_EnableIRQ(EXTI0_IRQn);
	
	// main loooooop
  while(1){
		switch (state){
		case 0:
		case 2:// state when only one led is on
      GPIOE->ODR = LEDS[i]; break;
    case 1:
		case 3:// state when each led is switching on/off every DELAY_TIME
			GPIOE->ODR ^= LEDS[i]; break;
		case 4: //sendStringToUart("Hello, world!\r\n");
			break;
		
		}
		if (!isDelayed){
				// inc\decrementing index of led in use
			  i = (state != 2 && state != 3) ? (i + 1)%a : (i - 1 + a)%a;
				// delay to see leds blinking		
				delay(DELAY_TIME);
		}
  };
  
  return 0;
}
