/**
  ******************************************************************************
  * @file    main.c 
  * @brief   Main program body
  * @details Contains main() function body
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "bsp.h"
#include "bsp_keypad.h"

#include "sys_timer_utils.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External functions Declaration --------------------------------------------*/
extern void Test_Keypad(void);

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
       
  BSP_Init();
  LCD8080_Init();
  GPIO_SetBits(LCD_BKL_GPIO_PORT, LCD_BKL_GPIO_PIN);
  ILI9341_Init();
  
  printf("\n\r Power Up \n\r");
  while (1)
  {
    LCD_FillArea(0, 239, 0, 319, 0xF800);
    SYSTMR_DelayMs(1000);
    LCD_FillArea(0, 239, 0, 319, 0x07E0);
    SYSTMR_DelayMs(1000);
    LCD_FillArea(0, 239, 0, 319, 0x001F);
    SYSTMR_DelayMs(1000);
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

/* END OF FILE ****************************************************************/
