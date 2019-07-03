/******************************************************************************
 *      CANOPUS INSTRUMENTS (KALYAN)
 ******************************************************************************
 * File Name   : BSP_LCD8080Interface.c
 * Purpose     : Port routines for 8080 interface, 8 bit data bus
 * Description : Contains routines to implement the interface of LCD with MCU
 *               on parallel 8080 bus
 *-----------------------------------------------------------------------------
 * Created By  : Vipul Panchal
 * Date        : 25/09/2012
 *****************************************************************************/

/******************************************************************************
 *      Include Files
 *****************************************************************************/
/* Microcontroller Includes */
//#include "stm32f0xx_conf.h"

/* Board Files Includes */
#include "bsp.h"
/******************************************************************************
 *      Pre-Defines
 *****************************************************************************/
/* GPIO Direction Definition Macros */
#define MODE_IN       GPIO_Mode_IN_FLOATING
#define MODE_OUT      GPIO_Mode_Out_PP

/* Mapping of LCD Interface Signals according to the hardware */
//DATA BUS PIN AND PORT, STANDARD GPIO
#define DB_GPIO_PORT  GPIOB
#define DB_SHIFT      8
#define DB_MASK       0xFF

/* Controlling Signals for LCD GPIO */
#if 0

#define Clr_RS()      GPIO_WriteBit(LCD_RS_GPIO_PORT, LCD_RS_GPIO_PIN, Bit_RESET)
#define Set_RS()      GPIO_WriteBit(LCD_RS_GPIO_PORT, LCD_RS_GPIO_PIN, Bit_SET)
#define Clr_WR()      GPIO_WriteBit(LCD_WR_GPIO_PORT, LCD_WR_GPIO_PIN, Bit_RESET)
#define Set_WR()      GPIO_WriteBit(LCD_WR_GPIO_PORT, LCD_WR_GPIO_PIN, Bit_SET)
#define Clr_RD()      GPIO_WriteBit(LCD_RD_GPIO_PORT, LCD_RD_GPIO_PIN, Bit_RESET)
#define Set_RD()      GPIO_WriteBit(LCD_RD_GPIO_PORT, LCD_RD_GPIO_PIN, Bit_SET)
#define Clr_CS()      GPIO_WriteBit(LCD_CS_GPIO_PORT, LCD_CS_GPIO_PIN, Bit_RESET)
#define Set_CS()      GPIO_WriteBit(LCD_CS_GPIO_PORT, LCD_CS_GPIO_PIN, Bit_SET)

#else

#define Clr_RS()      LCD_RS_GPIO_PORT->BRR  = LCD_RS_GPIO_PIN
#define Set_RS()      LCD_RS_GPIO_PORT->BSRR = LCD_RS_GPIO_PIN
#define Clr_WR()      LCD_WR_GPIO_PORT->BRR  = LCD_WR_GPIO_PIN
#define Set_WR()      LCD_WR_GPIO_PORT->BSRR = LCD_WR_GPIO_PIN
#define Clr_RD()      LCD_RD_GPIO_PORT->BRR  = LCD_RD_GPIO_PIN
#define Set_RD()      LCD_RD_GPIO_PORT->BSRR = LCD_RD_GPIO_PIN
#define Clr_CS()      LCD_CS_GPIO_PORT->BRR  = LCD_CS_GPIO_PIN
#define Set_CS()      LCD_CS_GPIO_PORT->BSRR = LCD_CS_GPIO_PIN

#endif
/******************************************************************************
 *      Type Definations
 *****************************************************************************/
/******************************************************************************
 *      Externals
 *****************************************************************************/
/******************************************************************************
 *      Constant Tables
 *****************************************************************************/
/******************************************************************************
 *      Private Variables
 *****************************************************************************/
/******************************************************************************
 *      Public Variables
 *****************************************************************************/
/******************************************************************************
 *      Private Code
 *****************************************************************************/
void Config_DataBus(GPIOMode_TypeDef mode)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Configure Data Bus in output pushpull mode */
  GPIO_InitStructure.GPIO_Pin = DB_MASK << DB_SHIFT;
  GPIO_InitStructure.GPIO_Mode = mode;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(DB_GPIO_PORT, &GPIO_InitStructure);
}

static uint8_t ReadDataBus(void)
{
  uint16_t in = GPIO_ReadInputData(DB_GPIO_PORT);
  return (uint8_t)((in >> DB_SHIFT) & DB_MASK);
}

#if 0
static void WriteDataBus(uint8_t data)
{
  uint16_t out = GPIO_ReadOutputData(DB_GPIO_PORT);
  out = (out & ~(DB_MASK << DB_SHIFT)) | (data << DB_SHIFT);
  GPIO_Write(DB_GPIO_PORT, out);
}
#endif

#define WriteDataBus(data)  \
DB_GPIO_PORT->ODR = (DB_GPIO_PORT->ODR & ~(DB_MASK << DB_SHIFT)) | (data << DB_SHIFT)

static void WriteLcd8(uint8_t data)
{
  WriteDataBus(data);
  Clr_CS();
  Clr_WR();
  Set_WR();
  Set_CS();
}

static uint8_t ReadLcd8(void)
{
  uint8_t data = 0;
  
  Config_DataBus(MODE_IN);
  Clr_CS();
  Clr_RD();
  data = ReadDataBus();
  Set_RD();
  Set_CS();
  Config_DataBus(MODE_OUT);
  
  return data;
}

/******************************************************************************
 *      Public Code
 *****************************************************************************/
/******************************************************************************
 * Function    : LCD8080_Init
 * Input       : None
 * Output      : None
 * Description : Initialize the port pins interfaced to LCD to their initial 
 *               values
 *****************************************************************************/
void LCD8080_Init(void)
{
  /* GPIO Intialized at BSP Level */ 
}

/******************************************************************************
 * Function    : LCD8080_Write8_A0
 * Input       : c - data byte to be written
 * Output      : None
 * Description : Write a bytes to controller, with RS = 0
 *****************************************************************************/
void LCD8080_Write8_A0(uint8_t c)
{
  Clr_RS();
  WriteLcd8(c);
}

/******************************************************************************
 * Function    : LCD8080_Write8_A1
 * Input       : c - data byte to be written
 * Output      : None
 * Description : Write a bytes to controller, with RS = 1
 *****************************************************************************/
void LCD8080_Write8_A1(uint8_t c)
{
  Set_RS();
  WriteLcd8(c);
}

/******************************************************************************
 * Function    : LCD8080_Write8_A0
 * Input       : pData - pointer to the data buffer to be written
 *               numBytes - number of bytes to be written
 * Output      : None
 * Description : Write multiple bytes to controller, with RS = 0
 *****************************************************************************/
void LCD8080_WriteM8_A0(uint8_t * pData, uint16_t numBytes)
{
  Clr_RS();
  for (; numBytes; numBytes--)
  {
    WriteLcd8(*pData);
    pData++;
  }
}

/******************************************************************************
 * Function    : LCD8080_Write8_A1
 * Input       : pData - pointer to the data buffer to be written
 *               numBytes - number of bytes to be written
 * Output      : None
 * Description : Write multiple bytes to controller, with RS = 1
 *****************************************************************************/
void LCD8080_WriteM8_A1(uint8_t * pData, uint16_t numBytes)
{
  Set_RS();
  for (; numBytes; numBytes--)
  {
    WriteLcd8(*pData);
    pData++;
  }
}

/******************************************************************************
 * Function    : LCD8080_Read8_A0
 * Input       : None
 * Output      : Data byte read from controller
 * Description : Read from controller, with RS = 0
 *****************************************************************************/
uint8_t LCD8080_Read8_A0(void)
{
  uint8_t c;
  
  Clr_RS();
  c = ReadLcd8();
  
  return c;
}

/******************************************************************************
 * Function    : LCD8080_Read8_A1
 * Input       : None
 * Output      : Data byte read from controller
 * Description : Read from controller, with RS = 1
 *****************************************************************************/
uint8_t LCD8080_Read8_A1(void)
{
  uint8_t c;
  
  Set_RS();
  c = ReadLcd8();
  
  return c;
}

/******************************************************************************
 * Function    : LCD8080_ReadM8_A1
 * Input       : pData - pointer to the data buffer, where the data needs to 
 *               be read
 *               numBytes - number of bytes to be written
 * Output      : Data byte read from controller
 * Description : Read from controller, with RS = 1
 *****************************************************************************/
void LCD8080_ReadM8_A1(uint8_t * pData, uint16_t numBytes)
{
  Set_RS();
  for (; numBytes; numBytes--)
  {
    *pData = ReadLcd8();
    pData++;
  }
}
/******************************************************************************
 *      End Of File
 *****************************************************************************/
