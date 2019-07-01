/**
  ******************************************************************************
  * @file    bsp.c
  * @author  Vipul Panchal
  * @brief   This file contains the board related functions
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "bsp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private constants----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO uint32_t SystemTicks = 0;

/* Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  
/* Extern declarations -------------------------------------------------------*/
 
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Configures the system clock.
  * @param  None
  * @retval None
  */
static void SysClock_Config(void)
{
  __IO uint32_t StartUpCounter = 0;
  
  /* Enable Prefetch Buffer */
  FLASH->ACR |= FLASH_ACR_PRFTBE;

  /* Flash wait states
     0WS for 0  < SYSCLK <= 24 MHz
     1WS for 24 < SYSCLK <= 48 MHz
     2WS for 48 < SYSCLK <= 72 MHz */
  FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
  FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;    

  /* PLL configuration
     PLLCLK = 64 MHz (HSI/2 * 16) */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
  RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL16
                        | RCC_CFGR_HPRE_DIV1    /* HCLK   = 64 MHz */
                        | RCC_CFGR_PPRE2_DIV1   /* PCLK2  = 64 MHz */
                        | RCC_CFGR_PPRE1_DIV2); /* PCLK1  = 32 MHz */    
                                                /* USBCLK = 42.667 MHz (64 MHz / 1.5) --> USB NOT POSSIBLE */

  /* Enable PLL */
  RCC->CR |= RCC_CR_PLLON;

  /* Wait till PLL is ready */
  while((RCC->CR & RCC_CR_PLLRDY) == 0)
  {
  }

  /* Select PLL as system clock source */
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

  /* Wait till PLL is used as system clock source */
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
  {
  }
}

/**
  * @brief  Configures the Peripherals power and clock.
  * @param  None
  * @retval None
  */
static void RCC_Configuration(void)
{
  /* ADCCLK = PCLK2/8, 8MHz when PCLK2 = SYSCLK = 64MHz */
  RCC_ADCCLKConfig(RCC_PCLK2_Div8);
  
  /* Enable peripheral clocks ------------------------------------------------*/

  /* Enable Clock for following peripherals on APB2 
     All GPIO Ports
     Enable ADC1 and ADC2
  */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO |
                          RCC_APB2Periph_GPIOA |
                          RCC_APB2Periph_GPIOB |
                          RCC_APB2Periph_GPIOC |
                          RCC_APB2Periph_GPIOD |
                          RCC_APB2Periph_USART1,
                          ENABLE );

  /* Enable clock for following peripherals on APB1 
     USART2 - Customer Display
     USART3 - User Interface
     power control
  */
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2 |
                          RCC_APB1Periph_PWR,
                          ENABLE);
  
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
}

/**
  * @brief  Configures the Alternate Function port pins.
  * @param  None
  * @retval None
  */
static void AFIO_Configuration(void)
{
  /* Remap JNTRST , JTDI and JTDO pins */
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  /* Remap OSC_IN and OSC_OUT Pins */
  GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
static void GPIO_Configuration(void)
{
  
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_StructInit(&GPIO_InitStructure);

  /* Set all GPIO Output speed to be 50MHz */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  
  /* Configure LCD_RST pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_RST_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_RST_GPIO_PIN;
  GPIO_Init(LCD_RST_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_BKL pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_BKL_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_BKL_GPIO_PIN;
  GPIO_Init(LCD_BKL_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure DBG_USART_TX pin I/O as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = DBG_USART_TX_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = DBG_USART_TX_GPIO_PIN;
  GPIO_Init(DBG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* Configure DBG_USART_RX pin I/O as input floating */
  GPIO_InitStructure.GPIO_Mode = DBG_USART_RX_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = DBG_USART_RX_GPIO_PIN;
  GPIO_Init(DBG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure KPD_SCAN0 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = KPD_SCAN0_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = KPD_SCAN0_GPIO_PIN;
  GPIO_Init(KPD_SCAN0_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure KPD_SCAN1 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = KPD_SCAN1_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = KPD_SCAN1_GPIO_PIN;
  GPIO_Init(KPD_SCAN1_GPIO_PORT, &GPIO_InitStructure);

  /* Configure KPD_SCAN2 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = KPD_SCAN2_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = KPD_SCAN2_GPIO_PIN;
  GPIO_Init(KPD_SCAN2_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure KPD_SCAN3 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = KPD_SCAN3_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = KPD_SCAN3_GPIO_PIN;
  GPIO_Init(KPD_SCAN3_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure LCD_CS pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_CS_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_CS_GPIO_PIN;
  GPIO_Init(LCD_CS_GPIO_PORT, &GPIO_InitStructure);

  /* Configure COM_USART_TX pin I/O as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = COM_USART_TX_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = COM_USART_TX_GPIO_PIN;
  GPIO_Init(COM_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* Configure COM_USART_RX pin I/O as input floating */
  GPIO_InitStructure.GPIO_Mode = COM_USART_RX_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = COM_USART_RX_GPIO_PIN;
  GPIO_Init(COM_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure LCD_RS pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_RS_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_RS_GPIO_PIN;
  GPIO_Init(LCD_RS_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_WR pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_WR_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_WR_GPIO_PIN;
  GPIO_Init(LCD_WR_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure SWDIO pin I/O as input floating  */
  GPIO_InitStructure.GPIO_Mode = SWDIO_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = SWDIO_GPIO_PIN;
  GPIO_Init(SWDIO_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure SWCLK pin I/O as input floating */
  GPIO_InitStructure.GPIO_Mode = SWCLK_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = SWCLK_GPIO_PIN;
  GPIO_Init(SWCLK_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_RD pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_RD_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_RD_GPIO_PIN;
  GPIO_Init(LCD_RD_GPIO_PORT, &GPIO_InitStructure);
    
  /* Configure LCD_DB0 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB0_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB0_GPIO_PIN;
  GPIO_Init(LCD_DB0_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_DB1 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB1_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB1_GPIO_PIN;
  GPIO_Init(LCD_DB1_GPIO_PORT, &GPIO_InitStructure);
    
  /* Configure LCD_DB2 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB2_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB2_GPIO_PIN;
  GPIO_Init(LCD_DB2_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure LCD_DB3 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB3_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB3_GPIO_PIN;
  GPIO_Init(LCD_DB3_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_DB4 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB4_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB4_GPIO_PIN;
  GPIO_Init(LCD_DB4_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_DB5 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB5_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB5_GPIO_PIN;
  GPIO_Init(LCD_DB5_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_DB6 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB6_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB6_GPIO_PIN;
  GPIO_Init(LCD_DB6_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure LCD_DB7 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB7_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB7_GPIO_PIN;
  GPIO_Init(LCD_DB7_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure LCD_DB8 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB8_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB8_GPIO_PIN;
  GPIO_Init(LCD_DB8_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure LCD_DB9 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB9_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB9_GPIO_PIN;
  GPIO_Init(LCD_DB9_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_DB10 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB10_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB10_GPIO_PIN;
  GPIO_Init(LCD_DB10_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_DB11 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB11_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB11_GPIO_PIN;
  GPIO_Init(LCD_DB11_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_DB12 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB12_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB12_GPIO_PIN;
  GPIO_Init(LCD_DB12_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_DB13 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB13_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB13_GPIO_PIN;
  GPIO_Init(LCD_DB13_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_DB14 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB14_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB14_GPIO_PIN;
  GPIO_Init(LCD_DB14_GPIO_PORT, &GPIO_InitStructure);

  /* Configure LCD_DB15 pin I/O as digital output push-pull */
  GPIO_InitStructure.GPIO_Mode = LCD_DB15_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = LCD_DB15_GPIO_PIN;
  GPIO_Init(LCD_DB15_GPIO_PORT, &GPIO_InitStructure);
   
  /* Configure KPD_RET2 pin I/O as digital input pull-up */
  GPIO_InitStructure.GPIO_Mode = KPD_RET2_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = KPD_RET2_GPIO_PIN;
  GPIO_Init(KPD_RET2_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure KPD_RET1 pin I/O as digital input pull-up */
  GPIO_InitStructure.GPIO_Mode = KPD_RET1_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = KPD_RET1_GPIO_PIN;
  GPIO_Init(KPD_RET1_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure KPD_RET0 pin I/O as digital input pull-up */
  GPIO_InitStructure.GPIO_Mode = KPD_RET0_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = KPD_RET0_GPIO_PIN;
  GPIO_Init(KPD_RET0_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure KPD_RET3 pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = KPD_RET3_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = KPD_RET3_GPIO_PIN;
  GPIO_Init(KPD_RET3_GPIO_PORT, &GPIO_InitStructure); 
  
  /* Configure KPD_RET4 pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = KPD_RET4_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = KPD_RET4_GPIO_PIN;
  GPIO_Init(KPD_RET4_GPIO_PORT, &GPIO_InitStructure); 
}

/**
  * @brief  Configures Nested Vector Interrupt for all the configured Interrupts
  * @param  None
  * @retval None
  */
void NVIC_Config(void)
{
//  NVIC_InitTypeDef NVIC_InitStructure;
//  
//  /* Configure and enable DMA1_Channel1 interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  Initialize Debug 
  * @param  None
  * @retval None
  */
static void DBG_USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
       
  /* USARTx configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


  /* Enable GPIO clock */
//  DBG_USART_GPIO_CLK_CMD(ENABLE);
  /* Enable USART Peripheral clock */
//  DBG_USART_PERIPH_CLK_CMD(ENABLE); 

  /* USART configuration */
  USART_Init(DBG_USART, &USART_InitStructure);
    
  /* Enable USART */
  USART_Cmd(DBG_USART, ENABLE);
}

/* Interrupt Handlers---------------------------------------------------------*/
/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    SystemTicks++;
}

/* Public functions ----------------------------------------------------------*/
/* TODO - Refer PAL project to convert this to interrupt based printf */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(DBG_USART, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(DBG_USART, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

void BSP_Init(void)
{
  /* Configure PLL with HSI as clock source to generate 64MHz */
  SysClock_Config();
  SystemCoreClockUpdate();

  /* Configure Peripheral Clocks and Enable Power to Peripherals */  
  RCC_Configuration();
  
  /* Initialize 1 millisecond system timer */
  SystemTicks = 0;
  SysTick_Config(SystemCoreClock / 1000);
  
  /* Configure the AFIO Port Pins */
  AFIO_Configuration();
  
  /* Configure the GPIO Port Pins */
  GPIO_Configuration();
  
  /* Configure Debug Usart */
  DBG_USART_Config();
  
  /* Configure Nested Vector Interrupt Controller */  
  NVIC_Config();
}

uint32_t BSP_GetSysTicks(void)
{
  return SystemTicks;
}

/******************************************************************************/
