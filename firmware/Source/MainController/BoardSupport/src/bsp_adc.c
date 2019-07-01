/**
  ******************************************************************************
  * @file    bsp.c
  * @author  Vipul Panchal
  * @brief   This file contains the board related functions
  ******************************************************************************
  */

/*******************************************************************************

  Description:  ADC driver with interface functions and DMA support.  

 *  There are a total of twelve channels that get converted; 6 of them
 *  get converted by ADC1 and the other 6 get converted by ADC2 in dual
 *  ADC regular simultaneous mode (6 channels each) using DMA1 channel1 to
 *  transfer the results into an ADC results block.
 *
 *  See ADC_Config for ADC1 and ADC2 channel setup.
 *
 *    Note that ADC1 and ADC2 values are converted simultaneously,
 *    so for example ADC_CHAN_STACKER_IN and ADC_CHAN_FEEDER_IN are converted first
 *    at the same time, then ADC_CHAN_UV_IN and ADC_CHAN_MG_IN, and so on.
 *
 *  The maximum allowed ADC clock is 14MHz. Since the ADCs are fed
 *  with the PCLK2, which is running at 64MHz, a prescaler value of 8
 *  gives an ADC clock of 64MHz/8 = 8MHz. 
 *  The total ADC conversion time is calculated as follows:
 *  Tconv = Sampling time + 12.5 cycles
 *  with Sampling time = 239.5 cycles, Tconv = 252 cycles
 *  with cycle time = (1 / 8MHz) 0.125us, single channel conversion time = 31.5us
 *
 *  The ADC1/ADC2 conversions are initiated by the software. 
 *  The DMA is configured for transfering the conversion values into a memory block
 *  which generates an interrupt once the transfer is complete.
 *  Two sets of memory blocks are used to store ADC results alternately, the DMA 
 *  destination register is swapped in the DMA ISR and ADC is restarted.
 *  The ADC converted values are copied to a circular buffer queue of length 32 
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "bsp.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* ADC channel definition */
#define ADC_CHAN_STACKER_IN ADC_Channel_11 /* Stacker Input A/D channel */
#define ADC_CHAN_UV_IN      ADC_Channel_0  /* UV Input A/D channel */
#define ADC_CHAN_THICK_L    ADC_Channel_8  /* Thickness Left  A/D channel */
#define ADC_CHAN_RED_L      ADC_Channel_15 /* RGB Red - Left A/D channel */
#define ADC_CHAN_GREEN_L    ADC_Channel_14 /* RGB Green - Left A/D channel */
#define ADC_CHAN_BLUE_L     ADC_Channel_7  /* RGB Blue - Left A/D channel */

#define ADC_CHAN_FEEDER_IN  ADC_Channel_13 /* Feeder Input A/D channel */
#define ADC_CHAN_MG_IN      ADC_Channel_1  /* MG Input A/D channel */
#define ADC_CHAN_THICK_R    ADC_Channel_9  /* Thickness Right A/D channel */
#define ADC_CHAN_RED_R      ADC_Channel_6  /* RGB Red - Right A/D channel */
#define ADC_CHAN_GREEN_R    ADC_Channel_5  /* RGB Green - Right A/D channel */
#define ADC_CHAN_BLUE_R     ADC_Channel_4  /* RGB Blue - Right A/D channel */

/* Default ADC Sample time configuration */
#define ADC_SAMPLETIME      ADC_SampleTime_239Cycles5

/* Number of values we're converting */
#define NUM_ADC_VALUES      ((int32_t)ADC_MAX_CHANNELS)

/* Number of conversions each ADC does */
#define NUM_ADC_CONVERSIONS (NUM_ADC_VALUES/2)

/* Number of ADC samples included in moving average */
#define CIRC_BUFFER_DEPTH     (32)

/* used to mask sample counter after it is incremented */
#define SAMPLE_COUNT_MASK   (CIRC_BUFFER_DEPTH - 1)

/* Private macro -------------------------------------------------------------*/
/* Private constants----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Buffers to contain the A/D results.
 * The results are interleaved, each element of the array contains
 * 16 bits of ADC1 value and 16 bits of ADC2 value.
 * Two sets of ADC buffers are used as ping-pong buffers by DMA
 */
/* static */ __IO uint16_t AdcBufferA[ ADC_MAX_CHANNELS ] __attribute__((aligned (4)));
/* static */ __IO uint16_t AdcBufferB[ ADC_MAX_CHANNELS ] __attribute__((aligned (4)));

/* circ buffer holds CIRC_BUFFER_DEPTH of raw data acquired from ADC */
/* static */ uint16_t ADCRawData[ADC_MAX_CHANNELS][CIRC_BUFFER_DEPTH];
/* static */ uint16_t circBufferIndex = 0;

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  This function is called to copy freshly read ADC data to buffer.
  *         Data is arranged in buffer as:
  *           Channel_0:  Sample 0, Sample 1, Sample 2...Sample n
  *           Channel_1:  Sample 0, Sample 1, Sample 2...Sample n
  *           Channel_2:  Sample 0, Sample 1, Sample 2...Sample n
  *              .           .         .         .          .
  *              .           .         .         .          .
  *           Channel_n:  Sample 0, Sample 1, Sample 2...Sample n
  * @param  *pSrc = pointer to buffer of data read from the ADC
  * @retval None
  */
static void CopyADCDataToBuffer( uint16_t *pSrc )
{
    /* pointer to the next location in the circular raw data buffer */
    uint16_t *pDst;                     
    uint16_t channel;
    
    /* save new data starting at row 0 (CH0), and the current sample */
    pDst = &ADCRawData[0][circBufferIndex++]; // dest address for new sample on channel 0
  circBufferIndex = circBufferIndex > CIRC_BUFFER_DEPTH ? 0 : circBufferIndex;

    // sequence through all channels (rows in the data array)
    for ( channel = 0; channel < ADC_MAX_CHANNELS; channel++ )
    {
        // write the new sample into the circular raw data buffer
        *pDst = *pSrc++;
        // increment the pointer to the next channel location in the buffer
        pDst += CIRC_BUFFER_DEPTH;
    }
}

/* Interrupt Handlers---------------------------------------------------------*/
/**
  * @brief  DMA1 Channel1 interrupt handler, generated when the DMA subsystem 
  *          copys the ADC data to the memory block.
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void) 
{
  __IO uint16_t *pBuffer;

  /* Clear the Interrupt Flag */
  //DMA_ClearFlag(DMA1_FLAG_GL1);
  DMA_ClearFlag(DMA1_FLAG_TC1);
  
  /* Disable the channel to change the Destination Memory Address */
  DMA_Cmd(DMA1_Channel1, DISABLE);
  /* All the even samples are transferred in AdcBufferA and 
     odd samples in AdcBufferB */
  if ((circBufferIndex % 2) == 0) 
  {
    /* Even Sample, read data from AdcBufferA
       and set DMA Destination buffer as AdcBufferB for odd sample */
    pBuffer = AdcBufferA;
    DMA1_Channel1->CMAR = (uint32_t)AdcBufferB;
    
  } 
  else 
  {
    /* Odd Sample, read data from AdcBufferB
       and set DMA Destination buffer as AdcBufferA for even sample */
    pBuffer = AdcBufferB;
    DMA1_Channel1->CMAR = (uint32_t)AdcBufferA;
  }
  
  DMA_SetCurrDataCounter(DMA1_Channel1, NUM_ADC_CONVERSIONS);
  
  /* Re-enable the DMA Channel */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
  /* copy ADC data into circular buffer */
  CopyADCDataToBuffer( (uint16_t *)pBuffer );
}

/* Public functions ----------------------------------------------------------*/
/**
  * @brief  Low level initialization of DMA for the A/D converters
  * @param  None
  * @retval None
  */
void DMA1_Channel1Config( void )
{
    DMA_InitTypeDef DMA_InitStructure;

    /* DMA1 channel1 configuration */
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AdcBufferA;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = NUM_ADC_CONVERSIONS;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; //DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* Enable transfer-complete interrupt */
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

    /* Enable DMA1 Channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);
}

/**
  * @brief  Configure ADC1 and ADC2 in Regular Simultaneous 
  *         Dual Conversion Mode
  * @param  None
  * @retval None
  */
void ADC_Config(void)
{
  ADC_InitTypeDef ADC_InitStructure;

  /* ADC1 configuration ---------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; /* ENABLE; */
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = NUM_ADC_CONVERSIONS;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channels configuration */
  ADC_RegularChannelConfig(ADC1, ADC_CHAN_STACKER_IN, 1, ADC_SAMPLETIME);
  ADC_RegularChannelConfig(ADC1, ADC_CHAN_UV_IN, 2, ADC_SAMPLETIME);
  ADC_RegularChannelConfig(ADC1, ADC_CHAN_THICK_L, 3, ADC_SAMPLETIME);
  ADC_RegularChannelConfig(ADC1, ADC_CHAN_RED_L, 4, ADC_SAMPLETIME);
  ADC_RegularChannelConfig(ADC1, ADC_CHAN_GREEN_L, 5, ADC_SAMPLETIME);
  ADC_RegularChannelConfig(ADC1, ADC_CHAN_BLUE_L, 6, ADC_SAMPLETIME);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* ADC2 configuration --------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_RegSimult;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = NUM_ADC_CONVERSIONS;
  ADC_Init(ADC2, &ADC_InitStructure);

  /* ADC2 regular channels configuration */
  ADC_RegularChannelConfig(ADC2, ADC_CHAN_FEEDER_IN, 1, ADC_SAMPLETIME);
  ADC_RegularChannelConfig(ADC2, ADC_CHAN_MG_IN, 2, ADC_SAMPLETIME);
  ADC_RegularChannelConfig(ADC2, ADC_CHAN_THICK_R, 3, ADC_SAMPLETIME);
  ADC_RegularChannelConfig(ADC2, ADC_CHAN_RED_R, 4, ADC_SAMPLETIME);
  ADC_RegularChannelConfig(ADC2, ADC_CHAN_GREEN_R, 5, ADC_SAMPLETIME);
  ADC_RegularChannelConfig(ADC2, ADC_CHAN_BLUE_R, 6, ADC_SAMPLETIME);

  /* Enable ADC2 external trigger conversion */
  ADC_ExternalTrigConvCmd(ADC2, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
    
  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
  
  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);

  /* Enable ADC2 reset calibration register */   
  ADC_ResetCalibration(ADC2);
  /* Check the end of ADC2 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC2));

  /* Start ADC2 calibration */
  ADC_StartCalibration(ADC2);
  /* Check the end of ADC2 calibration */
  while(ADC_GetCalibrationStatus(ADC2));

  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
  * @brief  Compute an average of the ADC data and write to pointer.
  * @param  *pAvg - pointer to array to hold avg data
  * @retval None
  */
void ADC_GetAvgData( uint16_t *pAvg )
{
    uint16_t index;
    uint16_t channel;
    uint16_t *pBuf = ADCRawData[0];                     
    uint32_t sum32;
    
    /* sequence through all channels (rows in the data array) */
    for ( channel = 0; channel < ADC_MAX_CHANNELS; channel++ )
    {
        /* compute sum of all samples for an ADC channel */
        sum32 = 0;
        for (index = 0; index < CIRC_BUFFER_DEPTH; index++)
        {
            sum32 += *pBuf++;
        }
        
        /* save average to the ADCnAvgData array pointed to by pAvg */
        *pAvg++ = sum32 / CIRC_BUFFER_DEPTH;
    }
}

