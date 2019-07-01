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
static __IO uint32_t CounterValue = 0;

/* Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
  
/* Extern declarations -------------------------------------------------------*/
extern void DMA1_Channel1Config(void);
extern void ADC_Config(void);
  
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
                          RCC_APB2Periph_GPIOE |
                          RCC_APB2Periph_GPIOF |
                          RCC_APB2Periph_GPIOG |
                          RCC_APB2Periph_ADC1 |
                          RCC_APB2Periph_ADC2 |
                          RCC_APB2Periph_TIM1 |
                          RCC_APB2Periph_USART1,
                          ENABLE );

  /* Enable clock for following peripherals on APB1 
     USART2 - Customer Display
     USART3 - User Interface
     power control
  */
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_USART2 |
                          RCC_APB1Periph_USART3 |
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
  
  /* Configure UV_IN pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = UV_IN_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = UV_IN_GPIO_PIN;
  GPIO_Init(UV_IN_GPIO_PORT, &GPIO_InitStructure);

  /* Configure MG_IN pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = MG_IN_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = MG_IN_GPIO_PIN;
  GPIO_Init(MG_IN_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure CD_USART_TX pin I/O as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = CD_USART_TX_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = CD_USART_TX_GPIO_PIN;
  GPIO_Init(CD_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* Configure CD_USART_RX pin I/O as input floating */
  GPIO_InitStructure.GPIO_Mode = CD_USART_RX_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = CD_USART_RX_GPIO_PIN;
  GPIO_Init(CD_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure BLUE_R pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = BLUE_R_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = BLUE_R_GPIO_PIN;
  GPIO_Init(BLUE_R_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure GREEN_R pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = GREEN_R_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = GREEN_R_GPIO_PIN;
  GPIO_Init(GREEN_R_GPIO_PORT, &GPIO_InitStructure);

  /* Configure RED_R pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = RED_R_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = RED_R_GPIO_PIN;
  GPIO_Init(RED_R_GPIO_PORT, &GPIO_InitStructure);

  /* Configure BLUE_L pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = BLUE_L_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = BLUE_L_GPIO_PIN;
  GPIO_Init(BLUE_L_GPIO_PORT, &GPIO_InitStructure);

  /* Configure DBG_USART_TX pin I/O as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = DBG_USART_TX_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = DBG_USART_TX_GPIO_PIN;
  GPIO_Init(DBG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* Configure DBG_USART_RX pin I/O as input floating */
  GPIO_InitStructure.GPIO_Mode = DBG_USART_RX_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = DBG_USART_RX_GPIO_PIN;
  GPIO_Init(DBG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure UV_EN pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = UV_EN_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = UV_EN_GPIO_PIN;
  GPIO_Init(UV_EN_GPIO_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(UV_EN_GPIO_PORT, UV_EN_GPIO_PIN, Bit_RESET);

  /* Configure RGB_EN pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = RGB_EN_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = RGB_EN_GPIO_PIN;
  GPIO_Init(RGB_EN_GPIO_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(RGB_EN_GPIO_PORT, RGB_EN_GPIO_PIN, Bit_RESET);
  
  /* Configure SWDIO pin I/O as input floating  */
  GPIO_InitStructure.GPIO_Mode = SWDIO_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = SWDIO_GPIO_PIN;
  GPIO_Init(SWDIO_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure SWCLK pin I/O as input floating */
  GPIO_InitStructure.GPIO_Mode = SWCLK_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = SWCLK_GPIO_PIN;
  GPIO_Init(SWCLK_GPIO_PORT, &GPIO_InitStructure);

  /* Configure WD_INH pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = WD_INH_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = WD_INH_GPIO_PIN;
  GPIO_Init(WD_INH_GPIO_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(WD_INH_GPIO_PORT, WD_INH_GPIO_PIN, Bit_SET);  
  
  /* Configure THICK_L pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = THICK_L_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = THICK_L_GPIO_PIN;
  GPIO_Init(THICK_L_GPIO_PORT, &GPIO_InitStructure);

  /* Configure THICK_R pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = THICK_R_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = THICK_R_GPIO_PIN;
  GPIO_Init(THICK_R_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure OUT4 pin I/O as input floating */
  GPIO_InitStructure.GPIO_Mode = OUT4_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = OUT4_GPIO_PIN;
  GPIO_Init(OUT4_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure WD_CLK pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = WD_CLK_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = WD_CLK_GPIO_PIN;
  GPIO_Init(WD_CLK_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(WD_CLK_GPIO_PORT, WD_CLK_GPIO_PIN, Bit_RESET);  
  
  /* Configure WD_SO pin I/O as input floating */
  GPIO_InitStructure.GPIO_Mode = WD_SO_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = WD_SO_GPIO_PIN;
  GPIO_Init(WD_SO_GPIO_PORT, &GPIO_InitStructure); 
  
  /* Configure WD_SI pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = WD_SI_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = WD_SI_GPIO_PIN;
  GPIO_Init(WD_SI_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(WD_SI_GPIO_PORT, WD_SI_GPIO_PIN, Bit_RESET);  
  
  /* Configure RGB_L_SCL pin I/O as alternate function open-drain */
  GPIO_InitStructure.GPIO_Mode = RGB_L_SCL_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = RGB_L_SCL_GPIO_PIN;
  GPIO_Init(RGB_L_SCL_GPIO_PORT, &GPIO_InitStructure); 
  
  /* Configure RGB_L_SDA pin I/O as alternate function open-drain */
  GPIO_InitStructure.GPIO_Mode = RGB_L_SDA_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = RGB_L_SDA_GPIO_PIN;
  GPIO_Init(RGB_L_SDA_GPIO_PORT, &GPIO_InitStructure); 

  /* Configure WD_LD pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = WD_LD_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = WD_LD_GPIO_PIN;
  GPIO_Init(WD_LD_GPIO_PORT, &GPIO_InitStructure);
  GPIO_WriteBit(WD_LD_GPIO_PORT, WD_LD_GPIO_PIN, Bit_RESET);  
  
  /* Configure COUNTER_EN pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = COUNTER_EN_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = COUNTER_EN_GPIO_PIN;
  GPIO_Init(COUNTER_EN_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(COUNTER_EN_GPIO_PORT, COUNTER_EN_GPIO_PIN, Bit_RESET);
  
  /* Configure RGB_R_SCL pin I/O as alternate function open-drain */
  GPIO_InitStructure.GPIO_Mode = RGB_R_SCL_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = RGB_R_SCL_GPIO_PIN;
  GPIO_Init(RGB_R_SCL_GPIO_PORT, &GPIO_InitStructure); 
  
  /* Configure RGB_R_SDA pin I/O as alternate function open-drain */
  GPIO_InitStructure.GPIO_Mode = RGB_R_SDA_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = RGB_R_SDA_GPIO_PIN;
  GPIO_Init(RGB_R_SDA_GPIO_PORT, &GPIO_InitStructure); 
  
  /* Configure CD_SPI_NSS pin I/O as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = CD_SPI_NSS_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = CD_SPI_NSS_GPIO_PIN;
  GPIO_Init(CD_SPI_NSS_GPIO_PORT, &GPIO_InitStructure); 
  
  /* Configure CD_SPI_SCK pin I/O as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = CD_SPI_SCK_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = CD_SPI_SCK_GPIO_PIN;
  GPIO_Init(CD_SPI_SCK_GPIO_PORT, &GPIO_InitStructure); 
  
  /* Configure CD_SPI_MISO pin I/O as input floating */
  GPIO_InitStructure.GPIO_Mode = CD_SPI_MISO_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = CD_SPI_MISO_GPIO_PIN;
  GPIO_Init(CD_SPI_MISO_GPIO_PORT, &GPIO_InitStructure); 

  /* Configure CD_SPI_MOSI pin I/O as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = CD_SPI_MOSI_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = CD_SPI_MOSI_GPIO_PIN;
  GPIO_Init(CD_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure); 
  
  /* Configure STACKER_EN pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = STACKER_EN_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = STACKER_EN_GPIO_PIN;
  GPIO_Init(STACKER_EN_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(STACKER_EN_GPIO_PORT, STACKER_EN_GPIO_PIN, Bit_RESET);
  
  /* Configure STACKER_IN pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = STACKER_IN_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = STACKER_IN_GPIO_PIN;
  GPIO_Init(STACKER_IN_GPIO_PORT, &GPIO_InitStructure); 
  
  /* Configure FEEDER_EN pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = FEEDER_EN_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = FEEDER_EN_GPIO_PIN;
  GPIO_Init(FEEDER_EN_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(FEEDER_EN_GPIO_PORT, FEEDER_EN_GPIO_PIN, Bit_RESET);
  
  /* Configure FEEDER_IN pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = FEEDER_IN_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = FEEDER_IN_GPIO_PIN;
  GPIO_Init(FEEDER_IN_GPIO_PORT, &GPIO_InitStructure); 
  
  /* Configure GREEN_L pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = GREEN_L_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = GREEN_L_GPIO_PIN;
  GPIO_Init(GREEN_L_GPIO_PORT, &GPIO_InitStructure);

  /* Configure RED_L pin I/O as analog input */
  GPIO_InitStructure.GPIO_Mode = RED_L_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = RED_L_GPIO_PIN;
  GPIO_Init(RED_L_GPIO_PORT, &GPIO_InitStructure);

  /* Configure DC_MOTOR pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = DC_MOTOR_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = DC_MOTOR_GPIO_PIN;
  GPIO_Init(DC_MOTOR_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(DC_MOTOR_GPIO_PORT, DC_MOTOR_GPIO_PIN, Bit_RESET);
  
  /* Configure ACM_MAIN pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = ACM_MAIN_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = ACM_MAIN_GPIO_PIN;
  GPIO_Init(ACM_MAIN_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(ACM_MAIN_GPIO_PORT, ACM_MAIN_GPIO_PIN, Bit_RESET);
  
  /* Configure ACM_AUX pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = ACM_AUX_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = ACM_AUX_GPIO_PIN;
  GPIO_Init(ACM_AUX_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(ACM_AUX_GPIO_PORT, ACM_AUX_GPIO_PIN, Bit_RESET);
  
  /* Configure BUZZER pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = BUZZER_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = BUZZER_GPIO_PIN;
  GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, Bit_RESET);

  /* Configure UI_USART_TX pin I/O as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = UI_USART_TX_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = UI_USART_TX_GPIO_PIN;
  GPIO_Init(UI_USART_TX_GPIO_PORT, &GPIO_InitStructure);

  /* Configure UI_USART_RX pin I/O as input floating */
  GPIO_InitStructure.GPIO_Mode = UI_USART_RX_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = UI_USART_RX_GPIO_PIN;
  GPIO_Init(UI_USART_RX_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure UI_BOOT0 pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = UI_BOOT0_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = UI_BOOT0_GPIO_PIN;
  GPIO_Init(UI_BOOT0_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(UI_BOOT0_GPIO_PORT, UI_BOOT0_GPIO_PIN, Bit_RESET);
  
  /* Configure WD_LED_EN pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = WD_LED_EN_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = WD_LED_EN_GPIO_PIN;
  GPIO_Init(WD_LED_EN_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(WD_LED_EN_GPIO_PORT, WD_LED_EN_GPIO_PIN, Bit_RESET);
  
  /* Configure CD_BOOT0 pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = CD_BOOT0_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = CD_BOOT0_GPIO_PIN;
  GPIO_Init(CD_BOOT0_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(CD_BOOT0_GPIO_PORT, CD_BOOT0_GPIO_PIN, Bit_RESET);
  
  /* Configure CD_NRST pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = CD_NRST_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = CD_NRST_GPIO_PIN;
  GPIO_Init(CD_NRST_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(CD_NRST_GPIO_PORT, CD_NRST_GPIO_PIN, Bit_SET);

  /* Configure TP2 pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = TP2_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = TP2_GPIO_PIN;
  GPIO_Init(TP2_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(TP2_GPIO_PORT, TP2_GPIO_PIN, Bit_RESET);
  
  /* Configure TP1 pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = TP1_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = TP1_GPIO_PIN;
  GPIO_Init(TP1_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(TP1_GPIO_PORT, TP1_GPIO_PIN, Bit_RESET);
  
  /* Configure UI_NRST pin I/O as output push-pull */
  GPIO_InitStructure.GPIO_Mode = UI_NRST_GPIO_MODE;
  GPIO_InitStructure.GPIO_Pin = UI_NRST_GPIO_PIN;
  GPIO_Init(UI_NRST_GPIO_PORT, &GPIO_InitStructure); 
  GPIO_WriteBit(CD_NRST_GPIO_PORT, CD_NRST_GPIO_PIN, Bit_SET);
}

#define COUNTER_TIM
#define COUNTER_TIx_EXT_CLK_SRC
#define COUNTER_TIx_EXT_CLK_POL
#define COUNTER_TIx_EXT_CLK_FILTER

/**
  * @brief  Configures the counter input as a timer clock input.
  * @param  None
  * @retval None
  */
static void CTR_TIM_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
  
  TIM_ETRClockMode1Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);
 
  TIM_TIxExternalClockConfig(TIM1, TIM_TIxExternalCLK1Source_TI1, TIM_ICPolarity_Rising, 0x00);
  
  TIM_Cmd(TIM1,ENABLE);
}

/**
  * @brief  Configures Nested Vector Interrupt for all the configured Interrupts
  * @param  None
  * @retval None
  */
void NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure and enable DMA1_Channel1 interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
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
  
  /* Configure ADC1 and ADC2 in DUAL ADC Mode with 
     Regular Simultaneous Conversion and transfer using DMA1 Channel1 */
  DMA1_Channel1Config();
  ADC_Config();

  /* Configure the Counter Timer to count the external pulses */
  CTR_TIM_Config();
  
  /* Configure Debug Usart */
  DBG_USART_Config();
  
  /* Configure Nested Vector Interrupt Controller */  
  NVIC_Config();
}

uint32_t BSP_GetSysTicks(void)
{
  return SystemTicks;
}

void BSP_FeederCommand(uint8_t enable)
{
  GPIO_WriteBit(FEEDER_EN_GPIO_PORT, FEEDER_EN_GPIO_PIN, 
                (enable == BSP_ENABLE) ? Bit_SET : Bit_RESET);
}

uint8_t BSP_GetFeederStatus(void)
{
  uint8_t status;
  
  status = (Bit_SET == GPIO_ReadInputDataBit(FEEDER_IN_GPIO_PORT, FEEDER_IN_GPIO_PIN))?
        BSP_ACTIVE : BSP_INACTIVE;
  
  return status;
}


void BSP_StackerCommand(uint8_t enable)
{
  GPIO_WriteBit(STACKER_EN_GPIO_PORT, STACKER_EN_GPIO_PIN, 
                (enable == BSP_ENABLE) ? Bit_SET : Bit_RESET);
}

uint8_t BSP_GetStackerStatus(void)
{
  uint8_t status;
  
  status = (Bit_SET == GPIO_ReadInputDataBit(STACKER_IN_GPIO_PORT, STACKER_IN_GPIO_PIN))?
          BSP_ACTIVE : BSP_INACTIVE;
  
  return status;
}

void BSP_CounterCommand(uint8_t enable)
{
  GPIO_WriteBit(COUNTER_EN_GPIO_PORT, COUNTER_EN_GPIO_PIN, 
                (enable == BSP_ENABLE) ? Bit_SET : Bit_RESET);
}

uint8_t BSP_GetCounterStatus(void)
{
  uint8_t status;
  
  status = (Bit_SET == GPIO_ReadInputDataBit(COUNTER_IN_GPIO_PORT, COUNTER_IN_GPIO_PIN))?
            BSP_ACTIVE : BSP_INACTIVE;
  
  return status;
}

void BSP_BuzzerCommand(uint8_t enable)
{
  GPIO_WriteBit(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, 
                (enable == BSP_ENABLE) ? Bit_SET : Bit_RESET);
}

void BSP_DCMotorCommand(uint8_t enable)
{
  GPIO_WriteBit(DC_MOTOR_GPIO_PORT, DC_MOTOR_GPIO_PIN, 
                (enable == BSP_ENABLE) ? Bit_SET : Bit_RESET);
}

void BSP_ACMotorMainCommand(uint8_t enable)
{
  GPIO_WriteBit(ACM_MAIN_GPIO_PORT, ACM_MAIN_GPIO_PIN, 
                (enable == BSP_ENABLE) ? Bit_SET : Bit_RESET);
}

void BSP_ACMotorAuxCommand(uint8_t enable)
{
  GPIO_WriteBit(ACM_AUX_GPIO_PORT, ACM_AUX_GPIO_PIN, 
                (enable == BSP_ENABLE) ? Bit_SET : Bit_RESET);
}

void BSP_UVLedCommand(uint8_t enable)
{
  GPIO_WriteBit(UV_EN_GPIO_PORT, UV_EN_GPIO_PIN, 
                (enable == BSP_ENABLE) ? Bit_SET : Bit_RESET);
}

void BSP_CounterClear(void)
{
  /* TODO - Implement */
}

void BSP_CounterStart(void)
{
  /* TODO - Implement */
}

void BSP_CounterStop(void)
{
  /* TODO - Implement */
}
