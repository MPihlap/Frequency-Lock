#define USE_STDPERIPH_DRIVER
#include "stm32f4xx.h"
//#include "stm32f4xx_usart.h"
 

//Quick hack, approximately 1ms delay
void ms_delay(int ms)
{
   while (ms-- > 0) {
      volatile int x=5971;
      while (x-- > 0) {
         __asm("nop");
      }
   }
}

USART_TypeDef uartType;
USART_InitTypeDef uartInit;
USART_ClockInitTypeDef uartClock;

//Flash orange LED at about 1hz
int main(void)
{
    USART_StructInit(&uartInit);
    USART_Init(USART1, &uartInit);
    uartInit.USART_BaudRate = 115200;

    USART_ClockStructInit(&uartClock);
    USART_ClockInit(USART1, &uartClock);
    USART_Cmd(USART1, ENABLE);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable the clock to GPIOD
    __asm("dsb");                         // stall instruction pipeline, until instruction completes, as
                                          //    per Errata 2.1.13, "Delay after an RCC peripheral clock enabling"
    GPIOD->MODER = (1 << 26);             // set pin 13 to be general purpose output

    for (;;) {
       ms_delay(500);
       GPIOD->ODR ^= (1 << 13);           // Toggle the pin 
    }
}
