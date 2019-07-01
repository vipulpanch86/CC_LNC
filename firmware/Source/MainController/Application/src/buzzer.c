/**
  ******************************************************************************
  * @file    buzzer.c
  * @author  Vipul Panchal
  * @brief   This file contains the buzzer related functions
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "bsp.h"
#include "sys_timer_utils.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BUZZER_TICKS_PER_MSEC     100
#define BUZZER_MSEC_TO_TICKS(ms)  (ms / BUZZER_TICKS_PER_MSEC)

/* Private macro -------------------------------------------------------------*/
/* Private constants----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t  BuzzerEnable = BSP_ENABLE;
static uint32_t BuzzerOnTicks = 0;

/* Private function prototypes -----------------------------------------------*/
/* Extern declarations -------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Executes Buzzer state machine
  * @param  None
  * @retval None
  */
void BUZZ_Exec(void)
{
  static uint32_t BackupSysTime = 0;
  uint32_t currentSysTime = SYSTMR_GetCurrentTime();
  
  if(SYSTMR_HasIntervalExpired(currentSysTime, BackupSysTime, BUZZER_TICKS_PER_MSEC))
  {
    BackupSysTime = currentSysTime;
    
    if(0 < BuzzerOnTicks)
    {
      if(BuzzerEnable != BSP_ENABLE)
      {
        BSP_BuzzerCommand(BSP_ENABLE);
        BuzzerEnable = BSP_ENABLE;
      }
      
      BuzzerOnTicks--;
    }
    else
    {
      if(BuzzerEnable != BSP_DISABLE)
      {
        BSP_BuzzerCommand(BSP_DISABLE);
        BuzzerEnable = BSP_DISABLE;
      }
    }
  }
}

/**
  * @brief  Enables Buzzer
  * @param  buzzTime - Time for which the buzzer remains ON
  * @retval None
  */
void BUZZ_Enable(uint32_t buzzTimeMs)
{
  BuzzerOnTicks = BUZZER_MSEC_TO_TICKS(buzzTimeMs);
}

/**
  * @brief  Disables Buzzer
  * @param  None
  * @retval None
  */
void BUZZ_Disable(void)
{
  BuzzerOnTicks = 0;
}
