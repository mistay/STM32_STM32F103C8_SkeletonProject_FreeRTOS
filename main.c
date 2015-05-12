/*
 * A skeleton main.c
 * Add your own code!
 */

// Load CMSIS and peripheral library and configuration
#include "stm32f10x.h"
//y//
//#include "enc28j60.h"
//#include "enc28j60.c"
//#include "stm32f10x_spi.h"
//#include "simple_server.c"
//#include "a.h"
//#include "ENC28J60.H"

//const unsigned char enc28j60_MAC[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
#include "FreeRTOS.h"
#include "task.h"
//#include "timers.h"
//#include "semphr.h"
//y//


// Peripheral configuration functions
void GPIO_Config();

// A simple busy wait loop
void Delay(volatile unsigned long delay);


/*
void eth() {
	int rev = 0;

//    SAMDK_Init();
    
    //__enable_irq();

    //simple_server();

    enc28j60Init((unsigned char *)enc28j60_MAC);

    rev = enc28j60getrev();

    return rev;
}*/


#define configLCD_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )
void prvLCDTask( void *pvParameters )
{

	for(;;) {
        GPIO_SetBits(GPIOC, GPIO_Pin_15);
        vTaskDelay(500);
        GPIO_ResetBits(GPIOC, GPIO_Pin_15);
        vTaskDelay(500);
        
        /*Delay(0xFFFFF);

        GPIO_ResetBits(GPIOC, GPIO_Pin_15);
        Delay(0xFFFFF);*/
    }
}

void prvLCDTask2( void *pvParameters )
{

	for(;;) {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        vTaskDelay(500);
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
        vTaskDelay(500);
        
        
        /*GPIO_SetBits(GPIOC, GPIO_Pin_14);
        
        Delay(0xFFFFF);

        GPIO_ResetBits(GPIOC, GPIO_Pin_14);
        Delay(0xFFFFF);*/
    }
}


void vApplicationTickHook( void );


void arminblink() {
	for(;;) {
        GPIO_SetBits(GPIOC, GPIO_Pin_15);
        Delay(0xFFFFF);

        GPIO_ResetBits(GPIOC, GPIO_Pin_15);
        Delay(0xFFFFF);
    }
}


static void prvSetupHardware( void )
{
	/* Start with the clocks in their expected state. */
	RCC_DeInit();

	/* Enable HSE (high speed external clock). */
	RCC_HSEConfig( RCC_HSE_ON );

	/* Wait till HSE is ready. */
	while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
	{
	}

	/* 2 wait states required on the flash. */
	*( ( unsigned long * ) 0x40022000 ) = 0x02;

	/* HCLK = SYSCLK */
	RCC_HCLKConfig( RCC_SYSCLK_Div1 );

	/* PCLK2 = HCLK */
	RCC_PCLK2Config( RCC_HCLK_Div1 );

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config( RCC_HCLK_Div2 );

	/* PLLCLK = 12MHz * 6 = 72 MHz. */
	RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_6 );

	/* Enable PLL. */
	RCC_PLLCmd( ENABLE );

	/* Wait till PLL is ready. */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );

	/* Wait till PLL is used as system clock source. */
	while( RCC_GetSYSCLKSource() != 0x08 )
	{
	}

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
							| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

	/* SPI2 Periph clock enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );


	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

	/* Misc initialisation, including some of the CircleOS features.  Note
	that CircleOS itself is not used. */
	//vParTestInitialise();
	//MEMS_Init();
	//POINTER_Init();
	//POINTER_SetMode( 4 ); // POINTER_RESTORE_LESS
}



void armin() {



	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG,DISABLE);



	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	prvSetupHardware();

	//xTaskCreate( prvLCDTask, "LCD", configLCD_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL );
	//vTaskStartScheduler();
	
	int ucParameterToPass=0;
	TaskHandle_t xHandle = NULL;
	TaskHandle_t xHandle2 = NULL;


	xTaskCreate( prvLCDTask, "green", configMINIMAL_STACK_SIZE + 50, &ucParameterToPass, 3, &xHandle );
	xTaskCreate( prvLCDTask2, "red",  configMINIMAL_STACK_SIZE + 50, &ucParameterToPass, 4, &xHandle2 );

    // -> geht aba wozu?  vSetupTimerTest();

	vTaskStartScheduler();
  
}

void armin3() {
	int i;
	i++;
	return;
}
void armin4() {
	int i;
	i++;
	i++;
	return;
}

int main(void) {
	//eth();
	
	
	
    // Setup STM32 system (clock, PLL and Flash configuration)
    //SystemInit();

    // Setup the GPIOs
    GPIO_Config();
    
    armin();
    
    for(;;) {
    
        GPIO_SetBits(GPIOC, GPIO_Pin_15);
        Delay(0xFFFFF);

        GPIO_ResetBits(GPIOC, GPIO_Pin_15);
        Delay(0xFFFFF);
    }
}


void Delay(volatile unsigned long delay) {
    for(; delay; --delay );
}

void GPIO_Config() {
    GPIO_InitTypeDef	GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}
