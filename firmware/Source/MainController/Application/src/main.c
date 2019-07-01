/**
  ******************************************************************************
  * @file    USART/Printf/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "bsp.h"

#include "sys_timer_utils.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void ReadWidthData(uint32_t *value)
{
  uint32_t count = 0;
  
  /* Always Drive SER Pin Low */
  GPIO_WriteBit(WD_SI_GPIO_PORT, WD_SI_GPIO_PIN, Bit_RESET);
  SYSTMR_DelayMs(5);
  
  /* Drive SH/LD# pin High to enable shifting of data and 
     disable Latching */
  GPIO_WriteBit(WD_LD_GPIO_PORT, WD_LD_GPIO_PIN, Bit_SET);
  SYSTMR_DelayMs(5);
  
  /* Drive CLK_INH pin Low to enable clock */
  GPIO_WriteBit(WD_INH_GPIO_PORT, WD_INH_GPIO_PIN, Bit_RESET);
  SYSTMR_DelayMs(5);
  
  /* Loop for number of data bits to be read */
  for(count = 0; count < 64; count++)
  {
  
//    if (count & 0x1)
//    {
//      GPIO_WriteBit(WD_SI_GPIO_PORT, WD_SI_GPIO_PIN, Bit_RESET);
//    }
//    else
//    {
//      GPIO_WriteBit(WD_SI_GPIO_PORT, WD_SI_GPIO_PIN, Bit_SET);
//    }
    
    /* Read QH pin */
    if(Bit_SET == GPIO_ReadInputDataBit(WD_SO_GPIO_PORT, WD_SO_GPIO_PIN))
    {
      value[count / 32] |= 1 << (count %32);
    }
    else
    {
      value[count / 32] &= ~(1 << (count %32));
    }
    
    /* Drive CLK pin High */
    GPIO_WriteBit(WD_CLK_GPIO_PORT, WD_CLK_GPIO_PIN, Bit_SET);
    SYSTMR_DelayMs(5);
    /* Drive CLK pin Low */
    GPIO_WriteBit(WD_CLK_GPIO_PORT, WD_CLK_GPIO_PIN, Bit_RESET);
    SYSTMR_DelayMs(5);  
  }
  /* Drive CLK_INH pin High to disable clock */
  GPIO_WriteBit(WD_INH_GPIO_PORT, WD_INH_GPIO_PIN, Bit_SET);
  SYSTMR_DelayMs(5);
  /* Drive SH/LD# pin Low to disable shifting of data and 
     enable Latching */
  GPIO_WriteBit(WD_LD_GPIO_PORT, WD_LD_GPIO_PIN, Bit_RESET);
  SYSTMR_DelayMs(5);
}

void TEST_WD_IO(void)
{
  GPIO_WriteBit(WD_INH_GPIO_PORT, WD_INH_GPIO_PIN, Bit_SET);
  GPIO_WriteBit(WD_SI_GPIO_PORT, WD_SI_GPIO_PIN, Bit_SET);
  GPIO_WriteBit(WD_LD_GPIO_PORT, WD_LD_GPIO_PIN, Bit_SET);
  GPIO_WriteBit(WD_CLK_GPIO_PORT, WD_CLK_GPIO_PIN, Bit_SET);
  SYSTMR_DelayMs(1000);
  
  GPIO_WriteBit(WD_INH_GPIO_PORT, WD_INH_GPIO_PIN, Bit_RESET);
  GPIO_WriteBit(WD_SI_GPIO_PORT, WD_SI_GPIO_PIN, Bit_RESET);
  GPIO_WriteBit(WD_LD_GPIO_PORT, WD_LD_GPIO_PIN, Bit_RESET);
  GPIO_WriteBit(WD_CLK_GPIO_PORT, WD_CLK_GPIO_PIN, Bit_RESET);
  SYSTMR_DelayMs(1000);

}

void Test (void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  volatile uint32_t delay_cnt = 0;
  
  /* GPIOA & GPIOB Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  
    /* Configure PA15 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  
  /* Configure PB3, PB4, PB5 and PB8 in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  while(1)
  {
    GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_SET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_SET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_SET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_SET);
    for(delay_cnt = 0; delay_cnt < 29999; delay_cnt++);
    
    GPIO_WriteBit(GPIOA, GPIO_Pin_15, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
    GPIO_WriteBit(GPIOB, GPIO_Pin_8, Bit_RESET);
    for(delay_cnt = 0; delay_cnt < 29999; delay_cnt++);
  }
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  uint32_t width_value[2] = 0;
  //Test();
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
       
  BSP_Init();
  
  printf("\n\r Power Up \n\r");
  //BSP_CounterCommand(BSP_ENABLE);
  //BSP_FeederCommand(BSP_ENABLE);
  //BSP_StackerCommand(BSP_ENABLE);
  BSP_UVLedCommand(BSP_ENABLE);
  while (1)
  {
    //ReadWidthData(&width_value[0]);
    
    //printf("Width Value[0] =  0x%X\n\r", width_value[0]);
    //printf("Width Value[1] =  0x%X\n\r", width_value[1]);
    //TEST_WD_IO();
    
//    if(BSP_ACTIVE == BSP_GetCounterStatus())
//    {
//      printf("\n\r ON \n\r");
//    }
//    else
//    {
//      printf("\n\r OFF \n\r");
//    }
//    GPIO_WriteBit(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, Bit_RESET);
//    SYSTMR_DelayMs(1000);
//    GPIO_WriteBit(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, Bit_SET);
//    SYSTMR_DelayMs(1000);
  }
}




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
