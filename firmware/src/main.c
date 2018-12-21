//#define USE_STDPERIPH_DRIVER
#define ARM_MATH_CM4

#include <math.h>
#include <stdio.h>
#include "arm_math.h" 

#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_spi.h"

#include "pdm2pcm_glo.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
#include "main.h"
//#include "waveplayer.h"

uint16_t  PDM_Input_Buffer[PDM_Input_Buffer_SIZE]; // buffer for PDM samples
uint16_t PCM_Output_Buffer[PCM_Output_Buffer_SIZE]; // 1st buffer for PCM samples
uint16_t PCM_Output_Buffer1[PCM_Output_Buffer_SIZE]; // 2nd buffer for PCM samples(used when data from first is being saved)
uint16_t buffer_input[Buffer_Input_SIZE]; // 1st buffer that aggregates PCM samples
uint16_t buffer_input1[Buffer_Input_SIZE]; // 2nd buffer that aggregates PCM samples(used when the first is busy)
uint32_t PDM_internal_memory[PDM2PCM_INTERNAL_MEMORY_SIZE];

uint16_t buf_idx = 0, buf_idx1 =0;

uint16_t* AudioRecBuf; // pointer to Audio Recording Buffer, it's set to PCM_Output_Buffer or PCM_Output_Buffer2
uint16_t* WriteBuf; // pointer to buffer, from which bytes are being sent

float32_t maxvalue; // in case you use FFT
uint32_t  maxvalueindex; // in case you use FFT
uint8_t   mode;
float32_t f;
float32_t f2;
int sw = 0; // var to switch buffers

char      text[100];

arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32 S_CFFT;
PDM_Filter_Handler_t Filter;

//Quick hack, approximately 1ms delay
void ms_delay(int ms)
{
   while (ms-- > 0) {
      volatile int x=971;
      while (x-- > 0) {
         __asm("nop");
      }
   }
}

/*
USART2 example, with IRQ

I am using a CP2102 USB-to-USART converter.
Wiring connections:
	STM32F4 			CP2102
	PA2 (USART2 Tx) ->	Rx
	PA3 (USART2 Rx) ->	Tx
*/


volatile uint32_t msTicks; /* counts 1ms timeTicks       */
void SysTick_Handler(void) {
	msTicks++;
}

//  Delays number of Systicks (happens every 1 ms)
static void Delay(__IO uint32_t dlyTicks){                                              
  uint32_t curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}

void setSysTick(){
	// ---------- SysTick timer (1ms) -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}
}

void setup_Periph(){
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// Enable the APB1 periph clock for USART2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	// Enable the GPIOA clock, used by pins PA2, PA3
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC |
		RCC_AHB1Periph_CRC, ENABLE);

	/********/
	/* APB1 */
	/********/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	RCC_PLLI2SCmd(ENABLE);

	//USB
	/* Always remember to turn on the peripheral clock...  If not, you may be up till 3am debugging... */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Setup the GPIO pins for Tx and Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Connect PA2 and PA3 with the USART2 Alternate Function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStructure);

	/* Enable the USART2 receive interrupt and configure
		the interrupt controller to jump to USART2_IRQHandler()
		if the USART2 receive interrupt occurs
	*/
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Finally enable the USART2 peripheral
	USART_Cmd(USART2, ENABLE);

	//NVIC_InitTypeDef NVIC_InitStructure;

	// Configure the interrupt priority grouping
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	// Configure the SPI2 interrupt channel
	NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	I2S_InitTypeDef I2S_InitStructure;

	SPI_I2S_DeInit(SPI2);
	I2S_InitStructure.I2S_AudioFreq = 32000; // Audio is recorded from two channels, so this value must be twice the value in the PDM filter
	I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
	I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
	I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
	I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
	I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
	I2S_Init(SPI2, &I2S_InitStructure);

  // Enable the Rx buffer not empty interrupt
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);


}

void USART_puts(USART_TypeDef *USARTx, volatile char *str){
	while(*str){
		// Wait for the TC (Transmission Complete) Flag to be set
		// while(!(USARTx->SR & 0x040));
		while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
		USART_SendData(USARTx, *str);
		*str++;
	}
}



//Flash orange LED at about 1hz
int main(void) {
	setSysTick();
	setup_Periph();
	// Initialize PDM filter
	/*
	Filter.Fs = 16000;
	Filter.HP_HZ = 10;
	Filter.LP_HZ = 8000;
	Filter.In_MicChannels = 1;
	Filter.Out_MicChannels = 1;
	*/
	Filter.bit_order = PDM_FILTER_BIT_ORDER_LSB;
	Filter.endianness = PDM_FILTER_ENDIANNESS_BE;
	Filter.high_pass_tap = 10;
	Filter.in_ptr_channels = 1;
	Filter.out_ptr_channels = 1;
	// Filter.pInternalMemory = PDM_internal_memory;

	PDM_Filter_Config_t filterConf;
	filterConf.decimation_factor = PDM_FILTER_DEC_FACTOR_64;
	filterConf.output_samples_number = PCM_Output_Buffer_SIZE;
	filterConf.mic_gain = 50;
	PDM_Filter_Init(&Filter);
	PDM_Filter_setConfig(&Filter, &filterConf);

	// Initialize FFT for 2048 samples
	arm_rfft_init_f32(&S, &S_CFFT, 2048, 0, 1);

    //RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable the clock to GPIOD
    //__asm("dsb");                         // stall instruction pipeline, until instruction completes, as
                                          //    per Errata 2.1.13, "Delay after an RCC peripheral clock enabling"
    GPIOD->MODER = (1 << 26);             // set pin 13 to be general purpose output
	uint16_t dataOut = 0xA55A;
	GPIOD->ODR ^= (1 << 6);          
    for (;;) {
		ms_delay(500);
		GPIOD->ODR ^= (1 << 13);           // Toggle the pin 
		 
       
		USART_SendData(USART2, dataOut);

    	USART_puts(USART2, "ayy lmao\n");
    }
}
