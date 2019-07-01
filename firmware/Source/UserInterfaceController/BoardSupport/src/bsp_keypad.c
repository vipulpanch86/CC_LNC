/**
  ******************************************************************************
  * @file    key.c
  * @author  Vipul Panchal
  * @brief   Contains the functions to handle matrix keypad,
  *          Initialization, Scanning and Status update
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Board Files Includes */
#include "bsp.h"
#include "bsp_keypad.h"

/* Private define ------------------------------------------------------------*/
/* Keypad Matrix Definations */
#define NB_RETURN_LINES_DEF             (5)
#define NB_SCAN_LINES_DEF               (4)
#define NB_KEYS_DEF                \
            (NB_SCAN_LINES_DEF * NB_RETURN_LINES_DEF)

/* Private Type Definations --------------------------------------------------*/
typedef struct
{
  uint8_t  state;
  uint8_t  press;
//  uint8_t  debounce;
  uint16_t debounce_ticks;
}KEY_INFO_T;

/* External Declaration-------------------------------------------------------*/
/* Private constants----------------------------------------------------------*/
/* List of Keypad Scan Lines GPIO Port */
static  GPIO_TypeDef* const SCAN_GPIO_PORT[NB_SCAN_LINES_DEF] = 
{
  KPD_SCAN0_GPIO_PORT,
  KPD_SCAN1_GPIO_PORT,
  KPD_SCAN2_GPIO_PORT,
  KPD_SCAN3_GPIO_PORT
};

/* List of Keypad Scan Lines GPIO Port Pins */
static const uint16_t SCAN_GPIO_PIN[NB_SCAN_LINES_DEF] = 
{
  KPD_SCAN0_GPIO_PIN,
  KPD_SCAN1_GPIO_PIN,
  KPD_SCAN2_GPIO_PIN,
  KPD_SCAN3_GPIO_PIN
};

/* List of Keypad Scan Lines GPIO Port */
static  GPIO_TypeDef* const RETURN_GPIO_PORT[NB_RETURN_LINES_DEF] = 
{
  KPD_RET0_GPIO_PORT,
  KPD_RET1_GPIO_PORT,
  KPD_RET2_GPIO_PORT,
  KPD_RET3_GPIO_PORT,
  KPD_RET4_GPIO_PORT
};

/* List of Keypad Return Lines GPIO Port Pins */
static const uint16_t RETURN_GPIO_PIN[NB_RETURN_LINES_DEF] = 
{
  KPD_RET0_GPIO_PIN,
  KPD_RET1_GPIO_PIN,
  KPD_RET2_GPIO_PIN,
  KPD_RET3_GPIO_PIN,
  KPD_RET4_GPIO_PIN
};

/* Private variables ---------------------------------------------------------*/
static  KEY_INFO_T KeyInfo[NB_KEYS_DEF];
static  uint8_t    DebounceTicks   = 0;
static  void (* KeypadCB)(uint8_t, uint8_t);

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure Scan Lines for I/O Function
  *         Enable the GPIO Clock, Initialize GPIO as Output for
  *         each Scan Line
  * @param  None
  * @retval None
  */
inline static void ConfigScanLinesIO(void)
{
  /* Configure Keypad scan pins as Output
     Default State is High 
  */
}
/**
  * @brief  Configure Return Lines for I/O Function
  *         Enable the GPIO Clock, Initialize GPIO as Input for
  *         each Return Line
  * @param  None
  * @retval None
  */
inline static void ConfigReturnLinesIO(void)
{
  /* Configure Keypad return pins as Input
     return lines are pulled up
  */
}

/**
  * @brief  Set a Scan Line to Logic Level High
  * @param  nb - ranges form 0 to NB_SC_4_RT_DEF - 1
  * @retval None
  */
inline static void SetScanLine(uint8_t nb)
{
  GPIO_SetBits(SCAN_GPIO_PORT[nb], SCAN_GPIO_PIN[nb]);
}

/**
  * @brief  Clear a Scan Line to Logic Level Low
  * @param  nb - ranges form 0 to NB_SC_4_RT_DEF - 1
  * @retval None
  */
inline static void ClrScanLine(uint8_t nb)
{
  GPIO_ResetBits(SCAN_GPIO_PORT[nb], SCAN_GPIO_PIN[nb]);
}

/**
  * @brief  Get the Logic Level on a Return Line
  * @param  nb - ranges form 0 to MAX_RT_LINES - 1
  * @retval returns HIGH / LOW status of the Logic Level on Return Line
  */
inline static uint8_t GetRetLine(uint8_t nb)
{
  uint8_t val;
  val = GPIO_ReadInputDataBit(RETURN_GPIO_PORT[nb], RETURN_GPIO_PIN[nb]);
  
  return (val == Bit_SET) ? BSP_HIGH : BSP_LOW;
}

/**
  * @brief  Set the current state of the key
  *         Change the current key status depending on the previous
  *         state of the key
  * @param  key - ranges form 0 to NB_KEYS_DEF - 1
  * @param  pressState - the key press state OPEN / CLOSED
  * @retval None
  */
static void SetKeyState(uint8_t key, uint8_t pressState)
{
  if(key < NB_KEYS_DEF)
  {
    /* Waiting for a keypress */
    if(KeyInfo[key].state == KEY_STATE_IDLE)
    {
      if(pressState == BSP_CLOSED)
      {
        /* Update the state to pressed */
        KeyInfo[key].state = KEY_STATE_PRESSED;

        /* Generate a callback */
        if(KeypadCB != NULL)
        {
          KeypadCB(key, KeyInfo[key].state);
        }
      }
    }
    else if(KeyInfo[key].state == KEY_STATE_PRESSED)
    {
      if(pressState == BSP_OPEN)
      {
        KeyInfo[key].state = KEY_STATE_RELEASED;

        /* Generate a callback */
        if(KeypadCB != NULL)
        {
          KeypadCB(key, KeyInfo[key].state);
        }
      }
    }
    else
    {
      KeyInfo[key].state = KEY_STATE_IDLE;
    }
  }
}

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Initialize the Keypad
  *         Initialize Row & Column Lines of the Matrix,
  *         Set key debounce value,
  *         Install call back function.
  * @param  debounce - key debounce value in milliseconds
  * @param  cb - callback function when a key state changes
  * @retval None
  */
void KPD_Init(uint8_t debounce, void (* cb)(uint8_t, uint8_t))
{
  uint8_t key;

  ConfigScanLinesIO();
  ConfigReturnLinesIO();

  debounce /= NB_SCAN_LINES_DEF;
  DebounceTicks = (uint8_t)((debounce < 1) ? (1) : (debounce));
  KeypadCB = cb;

  for(key = 0; key < NB_KEYS_DEF; key++)
  {
    KeyInfo[key].state = KEY_STATE_IDLE;
    KeyInfo[key].press = BSP_OPEN;
    KeyInfo[key].debounce_ticks = 0;
  }
}

/**
  * @brief  Scan the Keypad
  *         scans each key sequentially and updates the key status
  * @note   Must be called in a continious loop
  * @param  None
  * @retval None
  */
void KPD_Scan(void)
{
  static uint8_t keyScanLineNb = 0; 
  uint8_t keyRetLineNb = 0;

  ClrScanLine(keyScanLineNb);

  for(keyRetLineNb = 0; keyRetLineNb < NB_RETURN_LINES_DEF; keyRetLineNb++)
  {
    uint8_t  keyScanNo, keyPressState;

    keyPressState = (uint8_t)((GetRetLine(keyRetLineNb) == BSP_LOW) ? 
                                BSP_CLOSED : BSP_OPEN);
    
    /* Map the Key code generated from scan & return lines to
       Key Value Index */
    keyScanNo = (NB_RETURN_LINES_DEF * keyScanLineNb) + keyRetLineNb;
    
    /* Check if the key state is changed */
    if(keyPressState != KeyInfo[keyScanNo].press)
    {
      /* Check if the debounce verification is done */
      if(KeyInfo[keyScanNo].debounce_ticks == DebounceTicks)
      {
          /* change the key press state */
          KeyInfo[keyScanNo].press = (uint8_t)(
            KeyInfo[keyScanNo].press == BSP_OPEN ? BSP_CLOSED : BSP_OPEN);
          KeyInfo[keyScanNo].debounce_ticks = 0;
      }
      else
      {
        /* Pre debounce check
           Increment the debounce ticks
         */
        KeyInfo[keyScanNo].debounce_ticks++;
      }
    }
    else
    {
      KeyInfo[keyScanNo].debounce_ticks = 0;
    }

    SetKeyState(keyScanNo, KeyInfo[keyScanNo].press);
  }

  SetScanLine(keyScanLineNb);

  keyScanLineNb++;
  keyScanLineNb %= NB_SCAN_LINES_DEF;
  
  ClrScanLine(keyScanLineNb);

}

/**
  * @brief  Checks if a key is pressed
  *         Checks the key buffer status for KEY_STATE_PRESSED & KEY_STATE_HOLD
  * @param  key - The Key whose status needs to be checked
  * @retval TRUE if key is pressed, FALSE if key is released
  */
uint8_t KPD_IsPressed(uint8_t key)
{

  if(KeyInfo[key].state == KEY_STATE_PRESSED)
  {
    return KEY_PRESSED;
  }
  else
  {
    return KEY_RELEASED;
  }
}
/**
  * @brief  Sets the key debounce time
  *         Sets the key debounce time in milliseconds
  * @param  debounce - key debounce value in milliseconds
  * @retval None
  */
void KPD_SetDebounceTicks(uint8_t debounce)
{
  DebounceTicks = (uint8_t)((debounce < 1) ? (1) : (debounce));
}

/**
  * @brief  Gets the key debounce time in milliseconds
  * @param  None
  * @retval Returns set debounce time in milliseconds
  */
uint8_t KPD_GetDebounceTicks(void)
{
  return DebounceTicks;
}

/**
  * @brief  Installs a callback for a keypad event change
  * @param  cb - pointer to the callback function
  * @retval None
  */
void KPD_AddEventListener(void (* cb)(uint8_t, uint8_t))
{
  KeypadCB = cb;
}
/***********************END OF FILE************************/
