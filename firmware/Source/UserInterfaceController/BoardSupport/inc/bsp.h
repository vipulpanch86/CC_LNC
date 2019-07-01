/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define BIT(n)                (1<<(n))

#define BSP_DISABLE           (uint8_t)0
#define BSP_ENABLE            (uint8_t)1
   
#define BSP_INACTIVE          (uint8_t)0
#define BSP_ACTIVE            (uint8_t)1

#define BSP_OPEN              (uint8_t)0
#define BSP_CLOSED            (uint8_t)1

#define BSP_LOW               (uint8_t)0
#define BSP_HIGH              (uint8_t)1

#define LCD_RST_GPIO_PORT       GPIOA
#define LCD_RST_GPIO_PIN        GPIO_Pin_0
#define LCD_RST_GPIO_CLK        RCC_APB2Periph_GPIOA  
#define LCD_RST_GPIO_MODE       GPIO_Mode_Out_PP
   
#define LCD_BKL_GPIO_PORT       GPIOA
#define LCD_BKL_GPIO_PIN        GPIO_Pin_1
#define LCD_BKL_GPIO_CLK        RCC_APB2Periph_GPIOA  
#define LCD_BKL_GPIO_MODE       GPIO_Mode_Out_PP

#define DBG_USART_TX_GPIO_PORT  GPIOA
#define DBG_USART_TX_GPIO_PIN   GPIO_Pin_2
#define DBG_USART_TX_GPIO_CLK   RCC_APB2Periph_GPIOA
#define DBG_USART_TX_GPIO_MODE  GPIO_Mode_AF_PP

#define DBG_USART_RX_GPIO_PORT  GPIOA
#define DBG_USART_RX_GPIO_PIN   GPIO_Pin_3
#define DBG_USART_RX_GPIO_CLK   RCC_APB2Periph_GPIOA
#define DBG_USART_RX_GPIO_MODE  GPIO_Mode_IN_FLOATING

#define KPD_SCAN0_GPIO_PORT     GPIOA
#define KPD_SCAN0_GPIO_PIN      GPIO_Pin_4
#define KPD_SCAN0_GPIO_CLK      RCC_APB2Periph_GPIOA
#define KPD_SCAN0_GPIO_MODE     GPIO_Mode_Out_PP

#define KPD_SCAN1_GPIO_PORT     GPIOA
#define KPD_SCAN1_GPIO_PIN      GPIO_Pin_5
#define KPD_SCAN1_GPIO_CLK      RCC_APB2Periph_GPIOA
#define KPD_SCAN1_GPIO_MODE     GPIO_Mode_Out_PP

#define KPD_SCAN2_GPIO_PORT     GPIOA
#define KPD_SCAN2_GPIO_PIN      GPIO_Pin_6
#define KPD_SCAN2_GPIO_CLK      RCC_APB2Periph_GPIOA
#define KPD_SCAN2_GPIO_MODE     GPIO_Mode_Out_PP

#define KPD_SCAN3_GPIO_PORT     GPIOA
#define KPD_SCAN3_GPIO_PIN      GPIO_Pin_7
#define KPD_SCAN3_GPIO_CLK      RCC_APB2Periph_GPIOA
#define KPD_SCAN3_GPIO_MODE     GPIO_Mode_Out_PP

#define LCD_CS_GPIO_PORT        GPIOA
#define LCD_CS_GPIO_PIN         GPIO_Pin_8
#define LCD_CS_GPIO_CLK         RCC_APB2Periph_GPIOA
#define LCD_CS_GPIO_MODE        GPIO_Mode_Out_PP

#define COM_USART_TX_GPIO_PORT  GPIOA
#define COM_USART_TX_GPIO_PIN   GPIO_Pin_9
#define COM_USART_TX_GPIO_CLK   RCC_APB2Periph_GPIOA
#define COM_USART_TX_GPIO_MODE  GPIO_Mode_AF_PP

#define COM_USART_RX_GPIO_PORT  GPIOA
#define COM_USART_RX_GPIO_PIN   GPIO_Pin_10
#define COM_USART_RX_GPIO_CLK   RCC_APB2Periph_GPIOA
#define COM_USART_RX_GPIO_MODE  GPIO_Mode_IN_FLOATING

#define LCD_RS_GPIO_PORT        GPIOA
#define LCD_RS_GPIO_PIN         GPIO_Pin_11
#define LCD_RS_GPIO_CLK         RCC_APB2Periph_GPIOA  
#define LCD_RS_GPIO_MODE        GPIO_Mode_Out_PP

#define LCD_WR_GPIO_PORT        GPIOA
#define LCD_WR_GPIO_PIN         GPIO_Pin_12
#define LCD_WR_GPIO_CLK         RCC_APB2Periph_GPIOA  
#define LCD_WR_GPIO_MODE        GPIO_Mode_Out_PP

#define SWDIO_GPIO_PORT         GPIOA
#define SWDIO_GPIO_PIN          GPIO_Pin_13
#define SWDIO_GPIO_CLK          RCC_APB2Periph_GPIOA  
#define SWDIO_GPIO_MODE         GPIO_Mode_IN_FLOATING

#define SWCLK_GPIO_PORT         GPIOA
#define SWCLK_GPIO_PIN          GPIO_Pin_14
#define SWCLK_GPIO_CLK          RCC_APB2Periph_GPIOA  
#define SWCLK_GPIO_MODE         GPIO_Mode_IN_FLOATING

#define LCD_RD_GPIO_PORT        GPIOA
#define LCD_RD_GPIO_PIN         GPIO_Pin_15
#define LCD_RD_GPIO_CLK         RCC_APB2Periph_GPIOA 
#define LCD_RD_GPIO_MODE        GPIO_Mode_Out_PP

#define LCD_DB0_GPIO_PORT       GPIOB
#define LCD_DB0_GPIO_PIN        GPIO_Pin_0
#define LCD_DB0_GPIO_CLK        RCC_APB2Periph_GPIOB 
#define LCD_DB0_GPIO_MODE       GPIO_Mode_Out_PP

#define LCD_DB1_GPIO_PORT       GPIOB
#define LCD_DB1_GPIO_PIN        GPIO_Pin_1
#define LCD_DB1_GPIO_CLK        RCC_APB2Periph_GPIOB 
#define LCD_DB1_GPIO_MODE       GPIO_Mode_Out_PP

#define LCD_DB2_GPIO_PORT       GPIOB
#define LCD_DB2_GPIO_PIN        GPIO_Pin_2
#define LCD_DB2_GPIO_CLK        RCC_APB2Periph_GPIOB 
#define LCD_DB2_GPIO_MODE       GPIO_Mode_Out_PP

#define LCD_DB3_GPIO_PORT       GPIOB
#define LCD_DB3_GPIO_PIN        GPIO_Pin_3
#define LCD_DB3_GPIO_CLK        RCC_APB2Periph_GPIOB 
#define LCD_DB3_GPIO_MODE       GPIO_Mode_Out_PP

#define LCD_DB4_GPIO_PORT       GPIOB
#define LCD_DB4_GPIO_PIN        GPIO_Pin_4
#define LCD_DB4_GPIO_CLK        RCC_APB2Periph_GPIOB 
#define LCD_DB4_GPIO_MODE       GPIO_Mode_Out_PP

#define LCD_DB5_GPIO_PORT       GPIOB
#define LCD_DB5_GPIO_PIN        GPIO_Pin_5
#define LCD_DB5_GPIO_CLK        RCC_APB2Periph_GPIOB 
#define LCD_DB5_GPIO_MODE       GPIO_Mode_Out_PP

#define LCD_DB6_GPIO_PORT       GPIOB
#define LCD_DB6_GPIO_PIN        GPIO_Pin_6
#define LCD_DB6_GPIO_CLK        RCC_APB2Periph_GPIOB 
#define LCD_DB6_GPIO_MODE       GPIO_Mode_Out_PP

#define LCD_DB7_GPIO_PORT       GPIOB
#define LCD_DB7_GPIO_PIN        GPIO_Pin_7
#define LCD_DB7_GPIO_CLK        RCC_APB2Periph_GPIOB 
#define LCD_DB7_GPIO_MODE       GPIO_Mode_Out_PP

#define LCD_DB8_GPIO_PORT       GPIOB
#define LCD_DB8_GPIO_PIN        GPIO_Pin_8
#define LCD_DB8_GPIO_CLK        RCC_APB2Periph_GPIOB 
#define LCD_DB8_GPIO_MODE       GPIO_Mode_Out_PP

#define LCD_DB9_GPIO_PORT       GPIOB
#define LCD_DB9_GPIO_PIN        GPIO_Pin_9
#define LCD_DB9_GPIO_CLK        RCC_APB2Periph_GPIOB 
#define LCD_DB9_GPIO_MODE       GPIO_Mode_Out_PP

#define LCD_DB10_GPIO_PORT      GPIOB
#define LCD_DB10_GPIO_PIN       GPIO_Pin_10
#define LCD_DB10_GPIO_CLK       RCC_APB2Periph_GPIOB 
#define LCD_DB10_GPIO_MODE      GPIO_Mode_Out_PP

#define LCD_DB11_GPIO_PORT      GPIOB
#define LCD_DB11_GPIO_PIN       GPIO_Pin_11
#define LCD_DB11_GPIO_CLK       RCC_APB2Periph_GPIOB 
#define LCD_DB11_GPIO_MODE      GPIO_Mode_Out_PP

#define LCD_DB12_GPIO_PORT      GPIOB
#define LCD_DB12_GPIO_PIN       GPIO_Pin_12
#define LCD_DB12_GPIO_CLK       RCC_APB2Periph_GPIOB 
#define LCD_DB12_GPIO_MODE      GPIO_Mode_Out_PP

#define LCD_DB13_GPIO_PORT      GPIOB
#define LCD_DB13_GPIO_PIN       GPIO_Pin_13
#define LCD_DB13_GPIO_CLK       RCC_APB2Periph_GPIOB 
#define LCD_DB13_GPIO_MODE      GPIO_Mode_Out_PP

#define LCD_DB14_GPIO_PORT      GPIOB
#define LCD_DB14_GPIO_PIN       GPIO_Pin_14
#define LCD_DB14_GPIO_CLK       RCC_APB2Periph_GPIOB 
#define LCD_DB14_GPIO_MODE      GPIO_Mode_Out_PP

#define LCD_DB15_GPIO_PORT      GPIOB
#define LCD_DB15_GPIO_PIN       GPIO_Pin_15
#define LCD_DB15_GPIO_CLK       RCC_APB2Periph_GPIOB 
#define LCD_DB15_GPIO_MODE      GPIO_Mode_Out_PP

#define KPD_RET2_GPIO_PORT      GPIOC
#define KPD_RET2_GPIO_PIN       GPIO_Pin_13
#define KPD_RET2_GPIO_CLK       RCC_APB2Periph_GPIOA
#define KPD_RET2_GPIO_MODE      GPIO_Mode_IPU

#define KPD_RET1_GPIO_PORT      GPIOC
#define KPD_RET1_GPIO_PIN       GPIO_Pin_14
#define KPD_RET1_GPIO_CLK       RCC_APB2Periph_GPIOA
#define KPD_RET1_GPIO_MODE      GPIO_Mode_IPU

#define KPD_RET0_GPIO_PORT      GPIOC
#define KPD_RET0_GPIO_PIN       GPIO_Pin_15
#define KPD_RET0_GPIO_CLK       RCC_APB2Periph_GPIOA
#define KPD_RET0_GPIO_MODE      GPIO_Mode_IPU

#define KPD_RET3_GPIO_PORT      GPIOD
#define KPD_RET3_GPIO_PIN       GPIO_Pin_0
#define KPD_RET3_GPIO_CLK       RCC_APB2Periph_GPIOD
#define KPD_RET3_GPIO_MODE      GPIO_Mode_IPU

#define KPD_RET4_GPIO_PORT      GPIOD
#define KPD_RET4_GPIO_PIN       GPIO_Pin_1
#define KPD_RET4_GPIO_CLK       RCC_APB2Periph_GPIOD
#define KPD_RET4_GPIO_MODE      GPIO_Mode_IPU

/**
 * @brief Definition for Communication USART port, connected to USART1
 */ 
#define COM_USART                        USART1
#define COM_USART_PERIPH_CLK             RCC_APB2Periph_USART1
#define COM_USART_IRQn                   USART1_IRQn
#define COM_USART_PERIPH_CLK_CMD(x)      \
    RCC_APB2PeriphClockCmd(COM_USART_PERIPH_CLK, x)
#define COM_USART_GPIO_CLK_CMD(x)        \
    RCC_APB2PeriphClockCmd(COM_USART_TX_GPIO_CLK | COM_USART_RX_GPIO_CLK, x)
    
/**
 * @brief Definition for Debug USART port, connected to USART2
 */ 
#define DBG_USART                         USART2
#define DBG_USART_PERIPH_CLK              RCC_APB1Periph_USART2
#define DBG_USART_IRQn                    USART2_IRQn
#define DBG_USART_PERIPH_CLK_CMD(x)       \
    RCC_APB1PeriphClockCmd(DBG_USART_PERIPH_CLK, x)
#define DBG_USART_GPIO_CLK_CMD(x)         \
    RCC_APB2PeriphClockCmd(DBG_USART_TX_GPIO_CLK | DBG_USART_RX_GPIO_CLK, x)


/* Exported functions ------------------------------------------------------- */
void BSP_Init (void);
uint32_t BSP_GetSysTicks(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H */
