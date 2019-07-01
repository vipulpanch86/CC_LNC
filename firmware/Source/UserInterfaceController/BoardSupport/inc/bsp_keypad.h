/**
  ******************************************************************************
  * @file    bsp_keypad.h
  * @author  Vipul Panchal
  * @brief   This file contains definitions for keypad resources.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_KEYPAD_H
#define __BSP_KEYPAD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Key Press Status */
#define KEY_RELEASED                   (0)
#define KEY_PRESSED                    (1)

/* Key Status Definition */
#define KEY_STATE_IDLE                 (0)
#define KEY_STATE_PRESSED              (1)
#define KEY_STATE_RELEASED             (2)

/* Keypad Key List */
typedef enum
{
  /* TODO - Create a List of Keys */
  KPD_KEY_MAX
} KPD_KEY_LIST_T;  

void KPD_Init(uint8_t debounce, void (* cb)(uint8_t, uint8_t));
void KPD_Scan(void);
uint8_t KPD_IsPressed(uint8_t keyNo);
void KPD_SetDebounceTicks(uint8_t debounce);
uint8_t KPD_GetDebounceTicks(void);
void KPD_AddEventListener(void (* cb)(uint8_t, uint8_t));

#ifdef __cplusplus
}
#endif

#endif /* __BSP_KEYPAD_H */
/******************************************************************************/
