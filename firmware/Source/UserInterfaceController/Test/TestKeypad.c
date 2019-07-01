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
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void KeypadCallback(uint8_t key, uint8_t state)
{
  printf("key %d, state %d\n\r", (int)key, (int)state);
}

/* Public functions ----------------------------------------------------------*/
void Test_Keypad(void)
{
	uint32_t previousTick;
	
	KPD_Init(20, KeypadCallback);
  previousTick = BSP_GetSysTicks();
  while (1)
  {
    if(previousTick != BSP_GetSysTicks())
    {
      KPD_Scan();
    }
  }
}
/* END OF FILE ****************************************************************/
