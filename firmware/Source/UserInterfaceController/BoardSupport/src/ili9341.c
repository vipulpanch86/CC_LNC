#include "bsp.h"

#define ILI9341_NOP                                      0x00
#define ILI9341_RESET                                    0x01
#define ILI9341_READ_DISPLAY_IDENTIFICATION_INFORMATION  0x04
#define ILI9341_READ_DISPLAY_STATUS                      0x09
#define ILI9341_READ_DISPLAY_POWER_MODE                  0x0A
#define ILI9341_READ_DISPLAY_MADCTL                      0x0B
#define ILI9341_READ_DISPLAY_PIXEL_FORMAT                0x0C
#define ILI9341_READ_DISPLAY_IMAGE_FORMAT                0x0D
#define ILI9341_READ_DISPLAY_SIGNAL_MODE                 0x0E
#define ILI9341_READ_DISPLAY_SELF_DIAGNOSTIC_RESULT      0x0F
#define ILI9341_ENTER_SLEEP_MODE                         0x10
#define ILI9341_SLEEP_OUT                                0x11
#define ILI9341_PARTIAL_MODE_ON                          0x12
#define ILI9341_NORMAL_DISPLAY_MODE_ON                   0x13
#define ILI9341_DISPLAY_INVERSION_OFF                    0x20
#define ILI9341_DISPLAY_INVERSION_ON                     0x21
#define ILI9341_GAMMA                                    0x26
#define ILI9341_DISPLAY_OFF                              0x28
#define ILI9341_DISPLAY_ON                               0x29
#define ILI9341_COLUMN_ADDR                              0x2A
#define ILI9341_PAGE_ADDR                                0x2B
#define ILI9341_GRAM                                     0x2C
#define ILI9341_COLOR_SET                                0x2D
#define ILI9341_MEMORY_READ                              0x2E
#define ILI9341_PARTIAL_AREA                             0x30
#define ILI9341_VERTICAL_SCROLLING_DEFINITION            0x33
#define ILI9341_TEARING_EFFECT_LINE_OFF                  0x34
#define ILI9341_TEARING_EFFECT_LINE_ON                   0x35
#define ILI9341_MAC                                      0x36
#define ILI9341_VERTICAL_SCROLLING_START_ADDRESS         0x37
#define ILI9341_IDLE_MODE_OFF                            0x38
#define ILI9341_IDLE_MODE_ON                             0x39
#define ILI9341_PIXEL_FORMAT                             0x3A
#define ILI9341_WMC                                      0x3C
#define ILI9341_RMC                                      0x3E
#define ILI9341_SET_TEAR_SCANLINE                        0x44
#define ILI9341_WDB                                      0x51
#define ILI9341_READ_DISPLAY_BRIGHTNESS                  0x52
#define ILI9341_WCD                                      0x53
#define ILI9341_READ_CTRL_DISPLAY                        0x54
#define ILI9341_WCABC                                    0x55
#define ILI9341_RCABC                                    0x56
#define ILI9341_WCABCMB                                  0x5E
#define ILI9341_RCABCMB                                  0x5F
#define ILI9341_RGB_INTERFACE                            0xB0
#define ILI9341_FRC                                      0xB1
#define ILI9341_FRAME_CTRL_NM                            0xB2
#define ILI9341_FRAME_CTRL_IM                            0xB3
#define ILI9341_FRAME_CTRL_PM                            0xB4
#define ILI9341_BPC                                      0xB5
#define ILI9341_DFC                                      0xB6
#define ILI9341_ENTRY_MODE_SET                           0xB7
#define ILI9341_BACKLIGHT_CONTROL_1                      0xB8
#define ILI9341_BACKLIGHT_CONTROL_2                      0xB9
#define ILI9341_BACKLIGHT_CONTROL_3                      0xBA
#define ILI9341_BACKLIGHT_CONTROL_4                      0xBB
#define ILI9341_BACKLIGHT_CONTROL_5                      0xBC
#define ILI9341_BACKLIGHT_CONTROL_6                      0xBD
#define ILI9341_BACKLIGHT_CONTROL_7                      0xBE
#define ILI9341_BACKLIGHT_CONTROL_8                      0xBF
#define ILI9341_POWER1                                   0xC0
#define ILI9341_POWER2                                   0xC1
#define ILI9341_VCOM1                                    0xC5
#define ILI9341_VCOM2                                    0xC7
#define ILI9341_POWERA                                   0xCB
#define ILI9341_POWERB                                   0xCF
#define ILI9341_READ_ID1                                 0xDA
#define ILI9341_READ_ID2                                 0xDB
#define ILI9341_READ_ID3                                 0xDC
#define ILI9341_PGAMMA                                   0xE0
#define ILI9341_NGAMMA                                   0xE1
#define ILI9341_DTCA                                     0xE8
#define ILI9341_DTCB                                     0xEA
#define ILI9341_POWER_SEQ                                0xED
#define ILI9341_3GAMMA_EN                                0xF2
#define ILI9341_INTERFACE                                0xF6
#define ILI9341_PRC                                      0xF7


//#define BLACK       0x0000      
//#define NAVY        0x000F      
//#define DARKGREEN   0x03E0      
//#define DARKCYAN    0x03EF      
//#define MAROON      0x7800      
//#define PURPLE      0x780F      
//#define OLIVE       0x7BE0      
//#define LIGHTGREY   0xC618      
//#define DARKGREY    0x7BEF      
//#define BLUE        0x001F      
//#define GREEN       0x07E0      
//#define CYAN        0x07FF      
//#define RED         0xF800     
//#define MAGENTA     0xF81F      
//#define YELLOW      0xFFE0      
//#define WHITE       0xFFFF      
//#define ORANGE      0xFD20      
//#define GREENYELLOW 0xAFE5     
//#define PINK        0xF81F

#define SCREEN_VERTICAL_1			0
#define SCREEN_HORIZONTAL_1		1
#define SCREEN_VERTICAL_2			2
#define SCREEN_HORIZONTAL_2		3

volatile uint16_t LCD_HEIGHT = 240;
volatile uint16_t LCD_WIDTH	 = 320;

extern void SYSTMR_DelayMs(__IO uint32_t nTime);

extern void LCD8080_Write8_A1 (uint8_t c);
extern void LCD8080_Write8_A0 (uint8_t c);
extern void LCD8080_WriteM8_A1(uint8_t * pData, uint32_t numBytes);
extern void LCD8080_WriteM8_A0(uint8_t * pData, uint32_t numBytes);
  
extern uint8_t LCD8080_Read8_A0(void);
extern uint8_t LCD8080_Read8_A1(void);
extern void LCD8080_ReadM8_A1 (uint8_t * pData, uint32_t numBytes);


//#define LCM_CS_H()	  GPIO_WriteBit(LCD_CS_PORT, LCD_CS_PIN, Bit_SET)
//#define LCM_CS_L()	  GPIO_WriteBit(LCD_CS_PORT, LCD_CS_PIN, Bit_RESET)

#define LCD_WriteData     LCD8080_Write8_A1
#define LCD_WriteCommand  LCD8080_Write8_A0


/*HARDWARE RESET*/
//void LCD_Reset(void)
//{
//  GPIO_WriteBit(LCD_RST_PORT, LCD_RST_PIN, Bit_SET);	
//  SYSTMR_DelayMs(200);
//  GPIO_WriteBit(LCD_RST_PORT, LCD_RST_PIN, Bit_RESET);
//  SYSTMR_DelayMs(200);
//  GPIO_WriteBit(LCD_RST_PORT, LCD_RST_PIN, Bit_SET);	
//  SYSTMR_DelayMs(200);
//}

/*Ser rotation of the screen - changes x0 and y0*/
void LCD_SetRotation(uint8_t Rotation) 
{
  uint8_t screen_rotation = Rotation;

  LCD_WriteCommand(ILI9341_MAC);
  SYSTMR_DelayMs(1);

  switch(screen_rotation) 
  {
    case SCREEN_VERTICAL_1:
      LCD_WriteData(0x40|0x08);
      LCD_WIDTH = 240;
      LCD_HEIGHT = 320;
      break;
    case SCREEN_HORIZONTAL_1:
      LCD_WriteData(0x20|0x08);
      LCD_WIDTH  = 320;
      LCD_HEIGHT = 240;
      break;
    case SCREEN_VERTICAL_2:
      LCD_WriteData(0x80|0x08);
      LCD_WIDTH  = 240;
      LCD_HEIGHT = 320;
      break;
    case SCREEN_HORIZONTAL_2:
      LCD_WriteData(0x40|0x80|0x20|0x08);
      LCD_WIDTH  = 320;
      LCD_HEIGHT = 240;
      break;
    default:
      //EXIT IF SCREEN ROTATION NOT VALID!
      break;
  }
}
#if 1
/*Initialize LCD Controller */
void ILI9341_Init(void)
{
//  LCD_IO_Init();
//  LCD_Reset();

  //SOFTWARE RESET
  LCD_WriteCommand(ILI9341_RESET);
  SYSTMR_DelayMs(1000);

  //DISPLAY OFF
  LCD_WriteCommand(ILI9341_DISPLAY_OFF); 
	
  //POWER CONTROL A
  LCD_WriteCommand(ILI9341_POWERA);
  LCD_WriteData(0x39);
  LCD_WriteData(0x2C);
  LCD_WriteData(0x00);
  LCD_WriteData(0x34);
  LCD_WriteData(0x02);

  //POWER CONTROL B
  LCD_WriteCommand(ILI9341_POWERB);
  LCD_WriteData(0x00);
  LCD_WriteData(0xC1);
  LCD_WriteData(0x30);

  //DRIVER TIMING CONTROL A
  LCD_WriteCommand(ILI9341_DTCA);
  LCD_WriteData(0x85);
  LCD_WriteData(0x00);
  LCD_WriteData(0x78);

  //DRIVER TIMING CONTROL B
  LCD_WriteCommand(ILI9341_DTCB);
  LCD_WriteData(0x00);
  LCD_WriteData(0x00);

  //POWER ON SEQUENCE CONTROL
  LCD_WriteCommand(ILI9341_POWER_SEQ);
  LCD_WriteData(0x64);
  LCD_WriteData(0x03);
  LCD_WriteData(0x12);
  LCD_WriteData(0x81);

  //PUMP RATIO CONTROL
  LCD_WriteCommand(ILI9341_PRC);
  LCD_WriteData(0x20);

  //POWER CONTROL,VRH[5:0]
  LCD_WriteCommand(ILI9341_POWER1);
  LCD_WriteData(0x23);

  //POWER CONTROL,SAP[2:0];BT[3:0]
  LCD_WriteCommand(ILI9341_POWER2);
  LCD_WriteData(0x10);

  //VCM CONTROL
  LCD_WriteCommand(ILI9341_VCOM1);
  LCD_WriteData(0x3E);
  LCD_WriteData(0x28);

  //VCM CONTROL 2
  LCD_WriteCommand(ILI9341_VCOM2);
  LCD_WriteData(0x86);

  //MEMORY ACCESS CONTROL
  LCD_WriteCommand(ILI9341_MAC);
  LCD_WriteData(0x48);

  //PIXEL FORMAT
  LCD_WriteCommand(ILI9341_PIXEL_FORMAT);
  LCD_WriteData(0x55);

  //FRAME RATIO CONTROL, STANDARD RGB COLOR
  LCD_WriteCommand(ILI9341_FRC);
  LCD_WriteData(0x00);
  LCD_WriteData(0x18);

  //DISPLAY FUNCTION CONTROL
  LCD_WriteCommand(ILI9341_DFC);
  LCD_WriteData(0x08);
  LCD_WriteData(0x82);
  LCD_WriteData(0x27);

  //3GAMMA FUNCTION DISABLE
  LCD_WriteCommand(ILI9341_3GAMMA_EN);
  LCD_WriteData(0x00);

	LCD_WriteCommand(ILI9341_COLUMN_ADDR);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0xEF);

	LCD_WriteCommand(ILI9341_PAGE_ADDR);
	LCD_WriteData(0x00);
	LCD_WriteData(0x00);
	LCD_WriteData(0x01);
	LCD_WriteData(0x3F);
		
  //GAMMA CURVE SELECTED
  LCD_WriteCommand(ILI9341_GAMMA);
  LCD_WriteData(0x01);

  //POSITIVE GAMMA CORRECTION
  LCD_WriteCommand(ILI9341_PGAMMA);
  LCD_WriteData(0x0F);
  LCD_WriteData(0x31);
  LCD_WriteData(0x2B);
  LCD_WriteData(0x0C);
  LCD_WriteData(0x0E);
  LCD_WriteData(0x08);
  LCD_WriteData(0x4E);
  LCD_WriteData(0xF1);
  LCD_WriteData(0x37);
  LCD_WriteData(0x07);
  LCD_WriteData(0x10);
  LCD_WriteData(0x03);
  LCD_WriteData(0x0E);
  LCD_WriteData(0x09);
  LCD_WriteData(0x00);

  //NEGATIVE GAMMA CORRECTION
  LCD_WriteCommand(ILI9341_NGAMMA);
  LCD_WriteData(0x00);
  LCD_WriteData(0x0E);
  LCD_WriteData(0x14);
  LCD_WriteData(0x03);
  LCD_WriteData(0x11);
  LCD_WriteData(0x07);
  LCD_WriteData(0x31);
  LCD_WriteData(0xC1);
  LCD_WriteData(0x48);
  LCD_WriteData(0x08);
  LCD_WriteData(0x0F);
  LCD_WriteData(0x0C);
  LCD_WriteData(0x31);
  LCD_WriteData(0x36);
  LCD_WriteData(0x0F);

  //INVERT OFF
  LCD_WriteCommand(0x20);
  SYSTMR_DelayMs(120);

  //EXIT SLEEP
  LCD_WriteCommand(ILI9341_SLEEP_OUT);
  SYSTMR_DelayMs(120);

  //TURN ON DISPLAY
  LCD_WriteCommand(ILI9341_DISPLAY_ON);

  //STARTING ROTATION
  //LCD_SetRotation(SCREEN_HORIZONTAL_1);
	
	LCD_WriteCommand(ILI9341_GRAM);
}

#else

//void LCD_Init(void)
//{ 
//  LCD_IO_Init();
//  LCD_Reset();

//	LCM_CS_H();
//	LCM_CS_L();
//	LCD_WriteCommand (0x11);
//	LCM_CS_H();
//	SYSTMR_DelayMs(120);
//	
//	LCM_CS_L();
//	LCD_WriteCommand(0xCF);
//	LCD_WriteData(0x00);
//	LCD_WriteData(0x81);
//	LCD_WriteData(0X30);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0xED);
//	LCD_WriteData(0x64);
//	LCD_WriteData(0x03);
//	LCD_WriteData(0X12);
//	LCD_WriteData(0X81);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0xE8);
//	LCD_WriteData(0x85);
//	LCD_WriteData(0x10);
//	LCD_WriteData(0x78);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0xCB);
//	LCD_WriteData(0x39);
//	LCD_WriteData(0x2C);
//	LCD_WriteData(0x00);
//	LCD_WriteData(0x34);
//	LCD_WriteData(0x02);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0xF7);
//	LCD_WriteData(0x20);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0xEA);
//	LCD_WriteData(0x00);
//	LCD_WriteData(0x00);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0xB1);
//	LCD_WriteData(0x00);
//	LCD_WriteData(0x18);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0xB6); // Display Function Control
//	LCD_WriteData(0x0A);
//	LCD_WriteData(0xA2);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0xC0); //Power control
//	LCD_WriteData(0x21); //VRH[5:0]
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0xC1); //Power control
//	LCD_WriteData(0x11); //SAP[2:0];BT[3:0]
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0xC5); //VCM control
//	LCD_WriteData(0x3e);
//	LCD_WriteData(0x31);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0xC7); //VCM control2
//	LCD_WriteData(0Xaa);

//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0x36); // Memory Access Control
//	LCD_WriteData(0x08);//00
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);

//	LCM_CS_L();
//	LCD_WriteCommand(0xF2); // 3Gamma Function Disable
//	LCD_WriteData(0x00);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0x26); //Gamma curve selected
//	LCD_WriteData(0x01);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);

//	LCM_CS_L();
//	LCD_WriteCommand(0x3A);
//	LCD_WriteData(0x55);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);

//	LCM_CS_L();
//	LCD_WriteCommand(0xE0); //Set Gamma
//	LCD_WriteData(0x0F);
//	LCD_WriteData(0x26);
//	LCD_WriteData(0x24);
//	LCD_WriteData(0x0B);
//	LCD_WriteData(0x0E);
//	LCD_WriteData(0x09);
//	LCD_WriteData(0x54);
//	LCD_WriteData(0XA8);
//	LCD_WriteData(0x46);
//	LCD_WriteData(0x0C);
//	LCD_WriteData(0x17);
//	LCD_WriteData(0x09);
//	LCD_WriteData(0x0F);
//	LCD_WriteData(0x07);
//	LCD_WriteData(0x00);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0XE1); //Set Gamma
//	LCD_WriteData(0x00);
//	LCD_WriteData(0x19);
//	LCD_WriteData(0x1B);
//	LCD_WriteData(0x04);
//	LCD_WriteData(0x10);
//	LCD_WriteData(0x07);
//	LCD_WriteData(0x2A);
//	LCD_WriteData(0x47);
//	LCD_WriteData(0x39);
//	LCD_WriteData(0x03);
//	LCD_WriteData(0x06);
//	LCD_WriteData(0x06);
//	LCD_WriteData(0x30);
//	LCD_WriteData(0x38);
//	LCD_WriteData(0x0F);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	//	Set Display area
//	LCM_CS_L();
//	LCD_WriteCommand(0x2a);
//	LCD_WriteData(0x00);
//	LCD_WriteData(0x00);
//	LCD_WriteData(0x00);
//	LCD_WriteData(0xef);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	LCM_CS_L();
//	LCD_WriteCommand(0x2B);
//	LCD_WriteData(0x00);
//	LCD_WriteData(0x00);
//	LCD_WriteData(0x01);
//	LCD_WriteData(0x3F);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);

//	LCM_CS_L();
//	LCD_WriteCommand(0x11); //Exit Sleep
//	LCM_CS_H();
//	SYSTMR_DelayMs(120);
//	LCM_CS_L();
//	LCD_WriteCommand(0x29); //Display on
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);

//	LCM_CS_L();
//	LCD_WriteCommand (0xB0);     //RGB interfcace SIGNGA CONTROL
//	LCD_WriteData (0xc0);  //41
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);

//	LCM_CS_L();
//	LCD_WriteCommand(0xf6);     //RGB interfcace CONTROL
//	LCD_WriteData (0x01);
//	LCD_WriteData (0x00);
//	LCD_WriteData (0x00);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);

//	LCM_CS_L();
//	LCD_WriteCommand(0xb5);     //RGB interfcace CONTROL
//	LCD_WriteData (0x06);
//	LCD_WriteData (0x06);
//	LCD_WriteData (0x0a);
//	LCD_WriteData (0x14);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	
//	LCM_CS_L();
//	LCD_WriteCommand(0x13);		//normal display on
//	LCM_CS_H();
//	SYSTMR_DelayMs(2);
//	
//	LCM_CS_L();
//	LCD_WriteCommand(0x2c);
//	LCM_CS_H();
//	SYSTMR_DelayMs(2); 

//	SYSTMR_DelayMs(4000);

//}

#endif
/* Set Address - Location block - to draw into */
void LCD_SetAddress(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2)
{
  LCD_WriteCommand(ILI9341_COLUMN_ADDR);
  LCD_WriteData((X1>>8) & 0xFF);
  LCD_WriteData((X1>>0) & 0xFF);
  LCD_WriteData((X2>>8) & 0xFF);
  LCD_WriteData((X2>>0) & 0xFF);

  LCD_WriteCommand(ILI9341_PAGE_ADDR);
  LCD_WriteData((Y1>>8) & 0xFF);
  LCD_WriteData((Y1>>0) & 0xFF);
  LCD_WriteData((Y2>>8) & 0xFF);
  LCD_WriteData((Y2>>0) & 0xFF);

  LCD_WriteCommand(ILI9341_GRAM);
}

////DRAW PIXEL AT XY POSITION WITH SELECTED COLOUR
////
////Location is dependant on screen orientation. x0 and y0 locations change with orientations.
////Using pixels to draw big simple structures is not recommended as it is really slow
////Try using either rectangles or lines if possible
////
//void LCD_DrawPixel(uint16_t X, uint16_t Y, uint16_t Color) 
//{
//  if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;	//OUT OF BOUNDS!
//  
//  LCD_SetAddress(X, Y, X+1, Y+1);
//  LCD_WriteData(Color);
//	
//}

//FILL THE ENTIRE SCREEN WITH SELECTED COLOUR (either #define-d ones or custom 16bit)
/*Sets address (entire screen) and Sends Height*Width ammount of colour information to LCD*/
void LCD_FillArea(uint16_t X0, uint16_t X1, uint16_t Y0, uint16_t Y1, uint16_t Color)
{
  uint32_t pixel;
  LCD_SetAddress(X0, Y0, X1, Y1);	
  
  for(pixel = 0; pixel < (X1 - X0 + 1)*(Y1 - Y0 + 1); pixel++)
  {
    LCD_WriteData(Color>>8);
    LCD_WriteData(Color>>0);
  }
}


