/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"

/* Exported types ------------------------------------------------------------*/
/* A/D channels */
typedef enum 
{
  ADC_STACKER_IN,   /* Stacker Input channel */
  ADC_FEEDER_IN,    /* Feeder Input channel */
  ADC_UV_IN,        /* UV Input channel */
  ADC_MG_IN,        /* MG Input channel */
  ADC_THICK_L,      /* Thickness Left channel */
  ADC_THICK_R,      /* Thickness Right channel */
  ADC_RED_L,        /* RGB Red - Left Channel */
  ADC_RED_R,        /* RGB Red - Right Channel */
  ADC_GREEN_L,      /* RGB Green - Left Channel */
  ADC_GREEN_R,      /* RGB Green - Right Channel */
  ADC_BLUE_L,       /* RGB Blue - Left Channel */
  ADC_BLUE_R,       /* RGB Blue - Right Channel */
  ADC_MAX_CHANNELS, /* Number of A/D channels */
} ADC_VALUE_E;    

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define BSP_DISABLE           (uint8_t)0
#define BSP_ENABLE            (uint8_t)1
   
#define BSP_INACTIVE          (uint8_t)0
#define BSP_ACTIVE            (uint8_t)1

   
#define UV_IN_GPIO_PORT         GPIOA
#define UV_IN_GPIO_PIN          GPIO_Pin_0
#define UV_IN_GPIO_CLK          RCC_APB2Periph_GPIOA  
#define UV_IN_GPIO_MODE         GPIO_Mode_AIN
   
#define MG_IN_GPIO_PORT         GPIOA
#define MG_IN_GPIO_PIN          GPIO_Pin_1
#define MG_IN_GPIO_CLK          RCC_APB2Periph_GPIOA  
#define MG_IN_GPIO_MODE         GPIO_Mode_AIN

#define CD_USART_TX_GPIO_PORT   GPIOA
#define CD_USART_TX_GPIO_PIN    GPIO_Pin_2
#define CD_USART_TX_GPIO_CLK    RCC_APB2Periph_GPIOA
#define CD_USART_TX_GPIO_MODE   GPIO_Mode_AF_PP

#define CD_USART_RX_GPIO_PORT   GPIOA
#define CD_USART_RX_GPIO_PIN    GPIO_Pin_3
#define CD_USART_RX_GPIO_CLK    RCC_APB2Periph_GPIOA
#define CD_USART_RX_GPIO_MODE   GPIO_Mode_IN_FLOATING

#define BLUE_R_GPIO_PORT        GPIOA
#define BLUE_R_GPIO_PIN         GPIO_Pin_4
#define BLUE_R_GPIO_CLK         RCC_APB2Periph_GPIOA
#define BLUE_R_GPIO_MODE        GPIO_Mode_AIN

#define GREEN_R_GPIO_PORT       GPIOA
#define GREEN_R_GPIO_PIN        GPIO_Pin_5
#define GREEN_R_GPIO_CLK        RCC_APB2Periph_GPIOA
#define GREEN_R_GPIO_MODE       GPIO_Mode_AIN

#define RED_R_GPIO_PORT         GPIOA
#define RED_R_GPIO_PIN          GPIO_Pin_6
#define RED_R_GPIO_CLK          RCC_APB2Periph_GPIOA
#define RED_R_GPIO_MODE         GPIO_Mode_AIN

#define BLUE_L_GPIO_PORT        GPIOA
#define BLUE_L_GPIO_PIN         GPIO_Pin_7
#define BLUE_L_GPIO_CLK         RCC_APB2Periph_GPIOA
#define BLUE_L_GPIO_MODE        GPIO_Mode_AIN

#define COUNTER_IN_GPIO_PORT    GPIOA
#define COUNTER_IN_GPIO_PIN     GPIO_Pin_8
#define COUNTER_IN_GPIO_CLK     RCC_APB2Periph_GPIOA
#define COUNTER_IN_GPIO_MODE    GPIO_Mode_IN_FLOATING

#define DBG_USART_TX_GPIO_PORT  GPIOA
#define DBG_USART_TX_GPIO_PIN   GPIO_Pin_9
#define DBG_USART_TX_GPIO_CLK   RCC_APB2Periph_GPIOA
#define DBG_USART_TX_GPIO_MODE  GPIO_Mode_AF_PP

#define DBG_USART_RX_GPIO_PORT  GPIOA
#define DBG_USART_RX_GPIO_PIN   GPIO_Pin_10
#define DBG_USART_RX_GPIO_CLK   RCC_APB2Periph_GPIOA
#define DBG_USART_RX_GPIO_MODE  GPIO_Mode_IN_FLOATING

#define UV_EN_GPIO_PORT         GPIOA
#define UV_EN_GPIO_PIN          GPIO_Pin_11
#define UV_EN_GPIO_CLK          RCC_APB2Periph_GPIOA  
#define UV_EN_GPIO_MODE         GPIO_Mode_Out_PP

#define RGB_EN_GPIO_PORT        GPIOA
#define RGB_EN_GPIO_PIN         GPIO_Pin_12
#define RGB_EN_GPIO_CLK         RCC_APB2Periph_GPIOA  
#define RGB_EN_GPIO_MODE        GPIO_Mode_Out_PP

#define SWDIO_GPIO_PORT         GPIOA
#define SWDIO_GPIO_PIN          GPIO_Pin_13
#define SWDIO_GPIO_CLK          RCC_APB2Periph_GPIOA  
#define SWDIO_GPIO_MODE         GPIO_Mode_IN_FLOATING

#define SWCLK_GPIO_PORT         GPIOA
#define SWCLK_GPIO_PIN          GPIO_Pin_14
#define SWCLK_GPIO_CLK          RCC_APB2Periph_GPIOA  
#define SWCLK_GPIO_MODE         GPIO_Mode_IN_FLOATING

#define WD_INH_GPIO_PORT        GPIOA
#define WD_INH_GPIO_PIN         GPIO_Pin_15
#define WD_INH_GPIO_CLK         RCC_APB2Periph_GPIOA 
#define WD_INH_GPIO_MODE        GPIO_Mode_Out_PP

#define THICK_L_GPIO_PORT       GPIOB
#define THICK_L_GPIO_PIN        GPIO_Pin_0
#define THICK_L_GPIO_CLK        RCC_APB2Periph_GPIOB 
#define THICK_L_GPIO_MODE       GPIO_Mode_AIN

#define THICK_R_GPIO_PORT       GPIOB
#define THICK_R_GPIO_PIN        GPIO_Pin_1
#define THICK_R_GPIO_CLK        RCC_APB2Periph_GPIOB
#define THICK_R_GPIO_MODE       GPIO_Mode_AIN

#define OUT4_GPIO_PORT          GPIOB
#define OUT4_GPIO_PIN           GPIO_Pin_2
#define OUT4_GPIO_CLK           RCC_APB2Periph_GPIOB
#define OUT4_GPIO_MODE          GPIO_Mode_IN_FLOATING

#define WD_CLK_GPIO_PORT        GPIOB
#define WD_CLK_GPIO_PIN         GPIO_Pin_3
#define WD_CLK_GPIO_CLK         RCC_APB2Periph_GPIOB
#define WD_CLK_GPIO_MODE        GPIO_Mode_Out_PP

#define WD_SO_GPIO_PORT         GPIOB
#define WD_SO_GPIO_PIN          GPIO_Pin_4
#define WD_SO_GPIO_CLK          RCC_APB2Periph_GPIOB
#define WD_SO_GPIO_MODE         GPIO_Mode_IN_FLOATING

#define WD_SI_GPIO_PORT         GPIOB
#define WD_SI_GPIO_PIN          GPIO_Pin_5
#define WD_SI_GPIO_CLK          RCC_APB2Periph_GPIOB
#define WD_SI_GPIO_MODE         GPIO_Mode_Out_PP

#define RGB_L_SCL_GPIO_PORT     GPIOB
#define RGB_L_SCL_GPIO_PIN      GPIO_Pin_6
#define RGB_L_SCL_GPIO_CLK      RCC_APB2Periph_GPIOB
#define RGB_L_SCL_GPIO_MODE     GPIO_Mode_AF_OD

#define RGB_L_SDA_GPIO_PORT     GPIOB
#define RGB_L_SDA_GPIO_PIN      GPIO_Pin_7
#define RGB_L_SDA_GPIO_CLK      RCC_APB2Periph_GPIOB
#define RGB_L_SDA_GPIO_MODE     GPIO_Mode_AF_OD

#define WD_LD_GPIO_PORT         GPIOB
#define WD_LD_GPIO_PIN          GPIO_Pin_8
#define WD_LD_GPIO_CLK          RCC_APB2Periph_GPIOB
#define WD_LD_GPIO_MODE         GPIO_Mode_Out_PP

#define COUNTER_EN_GPIO_PORT    GPIOB
#define COUNTER_EN_GPIO_PIN     GPIO_Pin_9
#define COUNTER_EN_GPIO_CLK     RCC_APB2Periph_GPIOB
#define COUNTER_EN_GPIO_MODE    GPIO_Mode_Out_PP

#define RGB_R_SCL_GPIO_PORT     GPIOB
#define RGB_R_SCL_GPIO_PIN      GPIO_Pin_10
#define RGB_R_SCL_GPIO_CLK      RCC_APB2Periph_GPIOB
#define RGB_R_SCL_GPIO_MODE     GPIO_Mode_AF_OD

#define RGB_R_SDA_GPIO_PORT     GPIOB
#define RGB_R_SDA_GPIO_PIN      GPIO_Pin_11
#define RGB_R_SDA_GPIO_CLK      RCC_APB2Periph_GPIOB
#define RGB_R_SDA_GPIO_MODE     GPIO_Mode_AF_OD

#define CD_SPI_NSS_GPIO_PORT    GPIOB
#define CD_SPI_NSS_GPIO_PIN     GPIO_Pin_12
#define CD_SPI_NSS_GPIO_CLK     RCC_APB2Periph_GPIOB
#define CD_SPI_NSS_GPIO_MODE    GPIO_Mode_AF_PP

#define CD_SPI_SCK_GPIO_PORT    GPIOB
#define CD_SPI_SCK_GPIO_PIN     GPIO_Pin_13
#define CD_SPI_SCK_GPIO_CLK     RCC_APB2Periph_GPIOB
#define CD_SPI_SCK_GPIO_MODE    GPIO_Mode_AF_PP

#define CD_SPI_MISO_GPIO_PORT   GPIOB
#define CD_SPI_MISO_GPIO_PIN    GPIO_Pin_14
#define CD_SPI_MISO_GPIO_CLK    RCC_APB2Periph_GPIOB
#define CD_SPI_MISO_GPIO_MODE   GPIO_Mode_IN_FLOATING

#define CD_SPI_MOSI_GPIO_PORT   GPIOB
#define CD_SPI_MOSI_GPIO_PIN    GPIO_Pin_15
#define CD_SPI_MOSI_GPIO_CLK    RCC_APB2Periph_GPIOB
#define CD_SPI_MOSI_GPIO_MODE   GPIO_Mode_AF_PP

#define STACKER_EN_GPIO_PORT    GPIOC
#define STACKER_EN_GPIO_PIN     GPIO_Pin_0
#define STACKER_EN_GPIO_CLK     RCC_APB2Periph_GPIOC  
#define STACKER_EN_GPIO_MODE    GPIO_Mode_Out_PP

#define STACKER_IN_GPIO_PORT    GPIOC
#define STACKER_IN_GPIO_PIN     GPIO_Pin_1
#define STACKER_IN_GPIO_CLK     RCC_APB2Periph_GPIOC  
#define STACKER_IN_GPIO_MODE    GPIO_Mode_AIN

#define FEEDER_EN_GPIO_PORT     GPIOC
#define FEEDER_EN_GPIO_PIN      GPIO_Pin_2
#define FEEDER_EN_GPIO_CLK      RCC_APB2Periph_GPIOC  
#define FEEDER_EN_GPIO_MODE     GPIO_Mode_Out_PP

#define FEEDER_IN_GPIO_PORT     GPIOC
#define FEEDER_IN_GPIO_PIN      GPIO_Pin_3
#define FEEDER_IN_GPIO_CLK      RCC_APB2Periph_GPIOC  
#define FEEDER_IN_GPIO_MODE     GPIO_Mode_AIN

#define GREEN_L_GPIO_PORT       GPIOC
#define GREEN_L_GPIO_PIN        GPIO_Pin_4
#define GREEN_L_GPIO_CLK        RCC_APB2Periph_GPIOC  
#define GREEN_L_GPIO_MODE       GPIO_Mode_AIN

#define RED_L_GPIO_PORT         GPIOC
#define RED_L_GPIO_PIN          GPIO_Pin_5
#define RED_L_GPIO_CLK          RCC_APB2Periph_GPIOC  
#define RED_L_GPIO_MODE         GPIO_Mode_AIN

#define DC_MOTOR_GPIO_PORT      GPIOC
#define DC_MOTOR_GPIO_PIN       GPIO_Pin_6
#define DC_MOTOR_GPIO_CLK       RCC_APB2Periph_GPIOC  
#define DC_MOTOR_GPIO_MODE      GPIO_Mode_Out_PP

#define ACM_MAIN_GPIO_PORT      GPIOC
#define ACM_MAIN_GPIO_PIN       GPIO_Pin_7
#define ACM_MAIN_GPIO_CLK       RCC_APB2Periph_GPIOC  
#define ACM_MAIN_GPIO_MODE      GPIO_Mode_Out_PP

#define ACM_AUX_GPIO_PORT       GPIOC
#define ACM_AUX_GPIO_PIN        GPIO_Pin_8
#define ACM_AUX_GPIO_CLK        RCC_APB2Periph_GPIOC  
#define ACM_AUX_GPIO_MODE       GPIO_Mode_Out_PP

#define BUZZER_GPIO_PORT        GPIOC
#define BUZZER_GPIO_PIN         GPIO_Pin_9
#define BUZZER_GPIO_CLK         RCC_APB2Periph_GPIOC  
#define BUZZER_GPIO_MODE        GPIO_Mode_Out_PP

#define UI_USART_TX_GPIO_PORT   GPIOC
#define UI_USART_TX_GPIO_PIN    GPIO_Pin_10
#define UI_USART_TX_GPIO_CLK    RCC_APB2Periph_GPIOC
#define UI_USART_TX_GPIO_MODE   GPIO_Mode_AF_PP

#define UI_USART_RX_GPIO_PORT   GPIOC
#define UI_USART_RX_GPIO_PIN    GPIO_Pin_11
#define UI_USART_RX_GPIO_CLK    RCC_APB2Periph_GPIOC
#define UI_USART_RX_GPIO_MODE   GPIO_Mode_IN_FLOATING

#define UI_BOOT0_GPIO_PORT      GPIOC
#define UI_BOOT0_GPIO_PIN       GPIO_Pin_12
#define UI_BOOT0_GPIO_CLK       RCC_APB2Periph_GPIOC
#define UI_BOOT0_GPIO_MODE      GPIO_Mode_Out_PP

#define WD_LED_EN_GPIO_PORT     GPIOC
#define WD_LED_EN_GPIO_PIN      GPIO_Pin_13
#define WD_LED_EN_GPIO_CLK      RCC_APB2Periph_GPIOC
#define WD_LED_EN_GPIO_MODE     GPIO_Mode_Out_PP

#define CD_BOOT0_GPIO_PORT      GPIOC
#define CD_BOOT0_GPIO_PIN       GPIO_Pin_14
#define CD_BOOT0_GPIO_CLK       RCC_APB2Periph_GPIOC
#define CD_BOOT0_GPIO_MODE      GPIO_Mode_Out_PP

#define CD_NRST_GPIO_PORT       GPIOC
#define CD_NRST_GPIO_PIN        GPIO_Pin_15
#define CD_NRST_GPIO_CLK        RCC_APB2Periph_GPIOC
#define CD_NRST_GPIO_MODE       GPIO_Mode_Out_PP

#define TP2_GPIO_PORT           GPIOD
#define TP2_GPIO_PIN            GPIO_Pin_0
#define TP2_GPIO_CLK            RCC_APB2Periph_GPIOD
#define TP2_GPIO_MODE           GPIO_Mode_Out_PP

#define TP1_GPIO_PORT           GPIOD
#define TP1_GPIO_PIN            GPIO_Pin_1
#define TP1_GPIO_CLK            RCC_APB2Periph_GPIOD
#define TP1_GPIO_MODE           GPIO_Mode_Out_PP

#define UI_NRST_GPIO_PORT       GPIOD
#define UI_NRST_GPIO_PIN        GPIO_Pin_2
#define UI_NRST_GPIO_CLK        RCC_APB2Periph_GPIOD
#define UI_NRST_GPIO_MODE       GPIO_Mode_Out_PP

/**
 * @brief Definition for Debug USART port, connected to USART1
 */ 
#define DBG_USART                        USART1
#define DBG_USART_PERIPH_CLK             RCC_APB2Periph_USART1
#define DBG_USART_IRQn                   USART1_IRQn
#define DBG_USART_PERIPH_CLK_CMD(x)      \
    RCC_APB2PeriphClockCmd(DBG_USART_PERIPH_CLK, x)
#define DBG_USART_GPIO_CLK_CMD(x)        \
    RCC_APB2PeriphClockCmd(DBG_USART_TX_GPIO_CLK | DBG_USART_RX_GPIO_CLK, x)
    
/**
 * @brief Definition for Customer Dispay USART port, connected to USART2
 */ 
#define CD_USART                         USART2
#define CD_USART_PERIPH_CLK              RCC_APB1Periph_USART2
#define CD_USART_IRQn                    USART2_IRQn
#define CD_USART_PERIPH_CLK_CMD(x)       \
    RCC_APB1PeriphClockCmd(CD_USART_PERIPH_CLK, x)
#define CD_USART_GPIO_CLK_CMD(x)         \
    RCC_APB2PeriphClockCmd(CD_USART_TX_GPIO_CLK | CD_USART_RX_GPIO_CLK, x)


/**
 * @brief Definition for UI USART port, connected to USART3 (USART3 pins remapped on GPIOC)
 */ 
#define UI_USART                         USART3
#define UI_USART_PERIPH_CLK              RCC_APB1Periph_USART3
#define UI_USART_IRQn                    USART3_IRQn
#define UI_USART_PERIPH_CLK_CMD(x)       \
    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE); \
    RCC_APB1PeriphClockCmd(UI_USART_PERIPH_CLK, x)
#define UI_USART_GPIO_CLK_CMD(x)         \
    RCC_APB2PeriphClockCmd(UI_USART_TX_GPIO_CLK | UI_USART_RX_GPIO_CLK, x)

/**
 * @brief ADC Minimum and Maximum Values for 12-bit ADC
 */
#define ADC_MIN_VALUE                 ((uint16_t)0)
#define ADC_MAX_VALUE                 ((uint16_t)0xFFF)

/* Exported functions ------------------------------------------------------- */
void BSP_Init (void);
uint32_t BSP_GetSysTicks(void);
void BSP_FeederCommand(uint8_t enable);
uint8_t BSP_GetFeederStatus(void);
void BSP_StackerCommand(uint8_t enable);
uint8_t BSP_GetStackerStatus(void);
void BSP_CounterCommand(uint8_t enable);
uint8_t BSP_GetCounterStatus(void);
void BSP_BuzzerCommand(uint8_t enable);
void BSP_DCMotorCommand(uint8_t enable);
void BSP_ACMotorMainCommand(uint8_t enable);
void BSP_ACMotorAuxCommand(uint8_t enable);
void BSP_UVLedCommand(uint8_t enable);

void BSP_CounterClear(void);
void BSP_CounterStart(void);
void BSP_CounterStop(void);


void ADC_Config(void);
/* TODO - move the DMA config to ADC config such that DMA Config, 
   ADC1 Config and ADC Config are static to file and called by ADC config */
void DMA1_Channel1Config( void );
void ADC_GetAvgData( uint16_t *pAvg );

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H */
