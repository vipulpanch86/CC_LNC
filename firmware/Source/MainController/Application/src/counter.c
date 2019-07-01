#include <stdint.h>
#include <stdbool.h>

#include "bsp.h"
#include "sys_timer_utils.h"

#include "counter.h"

//typedef enum
//{
//  COUNTER_STATE_DISABLED,
//  COUNTER_STATE_ENABLED,
//  
//  COUNTER_STATE_MAX_NB
//  
//} COUNTER_STATE_LIST_T;

/** @brief The definitions related to Thickness Sensor Values
  */
/* Thickness Minimum and Maximum Values */
#define THICKNESS_VALUE_MIN           ((uint16_t)0)
#define THICKNESS_VALUE_MAX           ((uint16_t)100)
/* Thickness Value Valid Range */
#define THICKNESS_VALUE_RANGE_MIN     ((uint16_t)10)
#define THICKNESS_VALUE_RANGE_MAX     ((uint16_t)90)
/* Thickness Value Nominal Range for True Currency */
#define THICKNESS_VALUE_NOM_MIN       ((uint16_t)25)
#define THICKNESS_VALUE_NOM_MAX       ((uint16_t)75)
/* Thickness Value unavailable */
#define THICKNESS_VALUE_UNAVAILABLE   ((uint16_t)255)

/** @brief The definitions related to Feeder Sensor Values
  */
/* Feeder Minimum and Maximum Values */
#define FEEDER_VALUE_MIN              ((uint16_t)0)
#define FEEDER_VALUE_MAX              ((uint16_t)100)
/* Feeder Value Valid Range */
#define FEEDER_VALUE_RANGE_MIN        ((uint16_t)10)
#define FEEDER_VALUE_RANGE_MAX        ((uint16_t)90)
/* Feeder Threshold value for Feeder Empty / Not-Empty
   Feeder value below Empty threshold represents Feeder state as Empty 
   Feeder value above Not-Empty threshold represents Feeder state as Not-Empty
   For any value between Empty threshold and Not-Empty threshold,
   Feeder state would be Not-Empty if the previous state was Not-Empty,
   else the feeder state would be considered Empty.
*/
#define FEEDER_VALUE_EMPTY_THLD       ((uint16_t)30)
#define FEEDER_VALUE_NOT_EMPTY_THLD   ((uint16_t)70)
/* Feeder Value unavailable */
#define FEEDER_VALUE_UNAVAILABLE      ((uint16_t)255)

/** @brief The definitions related to Stacker Sensor Values
  */
/* Stacker Minimum and Maximum Values */
#define STACKER_VALUE_MIN             ((uint16_t)0)
#define STACKER_VALUE_MAX             ((uint16_t)100)
/* Stacker Value Valid Range */
#define STACKER_VALUE_RANGE_MIN       ((uint16_t)10)
#define STACKER_VALUE_RANGE_MAX       ((uint16_t)90)
/* Stacker Threshold value for Stacker Empty / Not-Empty
   Stacker value below Not-Empty threshold represents Stacker state as Not-Empty 
   Stacker value above Empty threshold represents Stacker state as Empty
   For any value between Not-Empty threshold and Empty threshold,
   Stacker state would be Empty if the previous state was Empty,
   else the Stacker state would be considered Not-Empty.
*/
#define STACKER_VALUE_NOT_EMPTY_THLD  ((uint16_t)30)
#define STACKER_VALUE_EMPTY_THLD      ((uint16_t)70)
/* Stacker Value unavailable */
#define STACKER_VALUE_UNAVAILABLE     ((uint16_t)255)

/** @brief The definitions related to UV Sensor Values
  */
/* UV Minimum and Maximum Values */
#define UV_VALUE_MIN                  ((uint16_t)0)
#define UV_VALUE_MAX                  ((uint16_t)100)
/* UV Value Valid Range */
#define UV_VALUE_RANGE_MIN            ((uint16_t)10)
#define UV_VALUE_RANGE_MAX            ((uint16_t)90)
/* UV Threshold value for UV Detected / Not-Detected
   UV value below Not-Detected threshold represents UV state as Not-Detected
   UV value above Detected threshold represents UV state as Detected
   For any value between Not-Detected threshold and Detected threshold,
   UV state would be Detected if the previous state was Detected,
   else the UV state would be considered Not-Detected.
*/
#define UV_VALUE_NO_DETECT_THLD       ((uint16_t)30)
#define UV_VALUE_DETECT_THLD          ((uint16_t)70)
/* UV Value unavailable */
#define UV_VALUE_UNAVAILABLE          ((uint16_t)255)

/** @brief The definitions related to MG Sensor Values
  */
/* MG Minimum and Maximum Values */
#define MG_VALUE_MIN                  ((uint16_t)0)
#define MG_VALUE_MAX                  ((uint16_t)100)
/* MG Value Valid Range */
#define MG_VALUE_RANGE_MIN            ((uint16_t)10)
#define MG_VALUE_RANGE_MAX            ((uint16_t)90)
/* MG Threshold value for MG Detected / Not-Detected
   MG value below Not-Detected threshold represents MG state as Not-Detected
   MG value above Detected threshold represents MG state as Detected
   For any value between Not-Detected threshold and Detected threshold,
   MG state would be Detected if the previous state was Detected,
   else the MG state would be considered Not-Detected.
*/
#define MG_VALUE_NO_DETECT_THLD       ((uint16_t)30)
#define MG_VALUE_DETECT_THLD          ((uint16_t)70)
/* MG Value unavailable */
#define MG_VALUE_UNAVAILABLE          ((uint16_t)255)

/** @brief The definitions related to Bill Detecting Sensor Values
  */
/* Bill Detector Minimum and Maximum Values */
#define DETECT_VALUE_MIN               ((uint16_t)0)
#define DETECT_VALUE_MAX               ((uint16_t)100)
/* Bill Detector Value Valid Range */
#define DETECT_VALUE_RANGE_MIN         ((uint16_t)10)
#define DETECT_VALUE_RANGE_MAX         ((uint16_t)90)
/* Bill Detector Threshold value for Bill Detected / Not-Detected
   Bill Detector value below Detected threshold represents Bill Detector state as Detected
   Bill Detector value above Not-Detected threshold represents Bill Detector state as Not-Detected
   For any value between Detected threshold and Not-Detected threshold,
   Bill Detector  state would be Not-Detected if the previous state was Not-Detected,
   else the Bill Detector  state would be considered Detected.
*/
#define DETECT_VALUE_BLOCKED_THLD      ((uint16_t)30)
#define DETECT_VALUE_NOT_BLOCKED_THLD  ((uint16_t)70)
/* Bill Detector Value unavailable */
#define DETECT_VALUE_UNAVAILABLE       ((uint16_t)255)

#define ADC_TO_THICKNESS_VALUE(adc)   \
((((THICKNESS_VALUE_MAX - THICKNESS_VALUE_MIN) * ((adc) - ADC_MIN_VALUE))/(ADC_MAX_VALUE - ADC_MIN_VALUE)) + THICKNESS_VALUE_MIN)
#define ADC_TO_FEEDER_VALUE(adc)      \
((((FEEDER_VALUE_MAX - FEEDER_VALUE_MIN) * ((adc) - ADC_MIN_VALUE))/(ADC_MAX_VALUE - ADC_MIN_VALUE)) + FEEDER_VALUE_MIN)
#define ADC_TO_STACKER_VALUE(adc)     \
((((STACKER_VALUE_MAX - STACKER_VALUE_MIN) * ((adc) - ADC_MIN_VALUE))/(ADC_MAX_VALUE - ADC_MIN_VALUE)) + STACKER_VALUE_MIN)
#define ADC_TO_UV_VALUE(adc)          \
((((UV_VALUE_MAX - UV_VALUE_MIN) * ((adc) - ADC_MIN_VALUE))/(ADC_MAX_VALUE - ADC_MIN_VALUE)) + UV_VALUE_MIN)
#define ADC_TO_MG_VALUE(adc)          \
((((MG_VALUE_MAX - MG_VALUE_MIN) * ((adc) - ADC_MIN_VALUE))/(ADC_MAX_VALUE - ADC_MIN_VALUE)) + MG_VALUE_MIN)
#define ADC_TO_BILL_DETECT_VALUE(adc)       \
((((DETECT_VALUE_MAX - DETECT_VALUE_MIN) * ((adc) - ADC_MIN_VALUE))/(ADC_MAX_VALUE - ADC_MIN_VALUE)) + DETECT_VALUE_MIN)

typedef struct
{
  uint16_t ThicknessValueLeft;
  uint16_t ThicknessValueRight;
  uint16_t FeederValue;
  uint16_t StackerValue;
  uint16_t UVValue;
  uint16_t MGValue;
  uint16_t BillDetectorValueLeft;
  uint16_t BillDetectorValueRight;
  
  THICKNESS_SENSOR_STATE_E ThicknessStateLeft;
  THICKNESS_SENSOR_STATE_E ThicknessStateRight;
  FEEDER_SENSOR_STATE_E FeederState;
  STACKER_SENSOR_STATE_E StackerState;
  UV_SENSOR_STATE_E UVState;
  MG_SENSOR_STATE_E MGState;
  
  BILL_DETECTOR_STATE_E BillDetectorStateLeft;
  BILL_DETECTOR_STATE_E BillDetectorStateRight;
  
  uint16_t BillCount; /* Store the total number of currency Bills counted */
  BILL_SCANNER_STATE_E BillScannerState; /* maintains the state of the bill scanner */
  
  BILL_STATE_E BillState; /* Bill state in the bill scanner */ 
  
}COUNTER_DATA_T;


typedef enum
{
  COUNTER_STATE_IDLE,
  COUNTER_STATE_READY,
  COUNTER_STATE_PREPARE_START,
  COUNTER_STATE_STARTING,
  COUNTER_STATE_COUNTING,
  COUNTER_STATE_STOPPING,
  COUNTER_STATE_ERROR,
  
  COUNTER_STATE_MAX_NB
  
} COUNTER_STATE_LIST_T;

typedef enum
{
  COUNTER_CMD_NONE,
  COUNTER_CMD_START,
  COUNTER_CMD_STOP,
  COUNTER_CMD_BRAKE,
  COUNTER_CMD_CLEAR,
  
  COUNTER_CMD_MAX_NB
  
} COUNTER_CMD_LIST_T;

COUNTER_DATA_T Counter;
uint8_t CounterState, CounterCommand;
//uint32_t CounterValue = 0;

void COUNTER_Init(void)
{
  //CounterValue = 0;
  CounterCommand = COUNTER_CMD_NONE;
  CounterState = COUNTER_STATE_IDLE;
  
  //BSP_CounterStop();
}

uint8_t COUNTER_Command(uint8_t command)
{
  if(COUNTER_CMD_NONE == CounterCommand)
  {
    /* TODO -  test if the command is valid for the current counter state */
    CounterCommand = command;
    
    return SUCCESS;
  }
  
  return ERROR;
}

//void COUNTER_StateMachine(void)
//{
//  
//  switch(CounterState)
//  {
//    case COUNTER_STATE_DISABLED:
//    {
//      /* Counter is in Stopped State,
//         Accept the Counter Start command and Counter Reset Command
//      */
//      if(CounterCommand == COUNTER_CMD_START)
//      {
//        CounterCommand = COUNTER_CMD_NONE;
//        
//        /* Start the Hardware Counter to count pulses */
//        BSP_CounterStart();
//        
//        CounterState = COUNTER_STATE_ENABLED;
//      }
//      
//      if(CounterCommand == COUNTER_CMD_RESET)
//      {
//        CounterCommand = COUNTER_CMD_NONE;
//        
//        /* Enable the Hardware Counter to count pulses */
//        BSP_CounterClear();
//        CounterValue = 0;
//      }
//    }
//    break;
//    
//    case COUNTER_STATE_ENABLED:
//    {
//      /* Counter is in Enabled State,
//         Accept the Counter Stop command
//      */
//      if(CounterCommand == COUNTER_CMD_STOP)
//      {
//        CounterCommand = COUNTER_CMD_NONE;
//        
//        /* Stop the Hardware Counter */
//        BSP_CounterStop();
//        
//        CounterState = COUNTER_STATE_DISABLED;
//      }
//    }
//    break;
//    
//    default:
//    {
//      COUNTER_Init();
//    }
//  }
//}
static uint8_t EnableSensors(void)
{
  /* TODO
  Enable Counter
  Enable Thickness sensor
  Enable UV Sensor
  Enable MG Sensor
  */
  BSP_TachLedCommand(BSP_ENABLE);
  BSP_ThickLedCommand(BSP_ENABLE);
  BSP_UVLedCommand(BSP_ENABLE);
  
  return SUCCESS;
}

static uint8_t DisableSensors(void)
{
  /* TODO
  Disable Counter
  Disable Thickness sensor
  Disable UV Sensor
  Disable MG Sensor
  */
  BSP_TachLedCommand(BSP_DISABLE);
  BSP_ThickLedCommand(BSP_DISABLE);
  BSP_UVLedCommand(BSP_DISABLE);
  
  return SUCCESS;
}

static uint8_t StartMotors(void)
{
  /* TODO
  Turn ON DC Motor
  Turn ON AC Motor
  */
  BSP_DCMotorCommand(BSP_ENABLE);
  if(SUCCESS != ACM_Command(ACM_CMD_RUN))
  {
    return ERROR;
  }
  
  return SUCCESS;
}

static uint8_t StopMotors(void)
{
  /* TODO
  Turn OFF DC Motor
  Turn OFF AC Motor
  */
  BSP_DCMotorCommand(BSP_DISABLE);
  if(SUCCESS != ACM_Command(ACM_CMD_COAST))
  {
    return ERROR;
  }
  
  return SUCCESS;
}

static uint8_t BrakeMotors(void)
{
  /* TODO
  Turn OFF DC Motor
  Brake AC Motor
  */  
  BSP_DCMotorCommand(BSP_DISABLE);
  if(SUCCESS != ACM_Command(ACM_CMD_BRAKE))
  {
    return ERROR;
  }
  
  return SUCCESS;
}

static bool IsCountingStarted(void)
{
  /* TODO
  Check if the AC Motor Started Rotating and 
  Check if the first bill is received by thickness sensor
  */
  return (/* (MainMotorRunning == Counter.MainMotorState) && */
          (BillScannerOccupied == Counter.BillScannerState));
          
}

static bool IsCountingStopped(void)
{
  /* TODO 
  Check if the AC Motor is stopped 
  */
  return (/* (MainMotorStopped == Counter.MainMotorState) && */
        (BillScannerEmpty == Counter.BillScannerState));
}

#define SENSOR_ENABLE_WAIT_MS       500
#define COUNTER_START_TIMEOUT_MS    200
#define COUNTER_STOP_TIMEOUT_MS     500
#define COUNTER_BRAKE_TIMEOUT_MS    200


void COUNTER_StateMachine(void)
{
  static uint32_t BackUpSystemTimer = 0, TimeoutInterval = 0;
  
  switch(CounterState)
  {
    case COUNTER_STATE_IDLE:
    {
      /* TODO - Check the Counter Health and change the state to 
         READY State - if counter healthy
         ERROR State - if faulty */
      CounterCommand = COUNTER_CMD_NONE;
      CounterState = COUNTER_STATE_READY;
    }
    break;
    
    case COUNTER_STATE_READY:
    {
      /* Counter will wait in the READY state,
         until it receives the command to start the counter
      */
      if(COUNTER_CMD_START == CounterCommand)
      {
        CounterCommand = COUNTER_CMD_NONE;
        CounterState = COUNTER_STATE_STARTING;
        
        /* TODO 
           Command the sensors to start
           Command the counter to start
        */
        EnableSensors();
        
        /* Configure timer to wait for sensors to enable and stabilize */
        BackUpSystemTimer = SYSTMR_GetCurrentTime();
        TimeoutInterval = SENSOR_ENABLE_WAIT_MS;
      }
      else
      {
        /* TODO - after a timeout move to idle state and check the counter health status */
      }
    }
    break;
    
    case COUNTER_STATE_PREPARE_START:
    {
      /* Wait for sensors to stabilize before starting the motors */
      if (TimeoutInterval <= SYSTMR_GetElapsedTime(BackUpSystemTimer))
      {
        /* TODO
        Check if all the sensors are enabled and healthy
        */
        
        /* TODO
        Reset the sensor values to start a new counting cycle
        */
        ResetSensors();
        /* TODO
        Enable the DC Motor, Start the AC Motor
        */
        if (SUCCESS != StartMotors())
        {
          DisableSensors();
        /* TODO - Update the Error Register with the type of Error */
        CounterState = COUNTER_STATE_ERROR;
        }
        
        /* TODO
        Check if the scanner is already occupied, 
        move to the clean up state to wait for scanner to become empty
        */
        /* Configure to check counter start timeout */
        BackUpSystemTimer = SYSTMR_GetCurrentTime();
        TimeoutInterval = COUNTER_START_TIMEOUT_MS;
      }
    }
    break;
    
    case COUNTER_STATE_STARTING:
    {
      /* Check if the Counter has started counting,
         change the state to COUNTING
      */
      if(BillStateEntered == Counter.BillState)
      {
        CounterState = COUNTER_STATE_COUNTING;
        
        BackUpSystemTimer = SYSTMR_GetCurrentTime();
      }
      /* Wait for the timeout period to expire before
         counter to be stopped
      */
      /*
      else if (TimeoutInterval <= SYSTMR_GetElapsedTime(BackUpSystemTimer))
      {
      */
      /* Wait for the stop command from the application 
         after the start timeout */
      else if(COUNTER_CMD_STOP == CounterCommand) 
      {
        /* Counter stop command received */ 
        CounterCommand = COUNTER_CMD_NONE;
        CounterState = COUNTER_STATE_STOPPING;
        
        BackUpSystemTimer = SYSTMR_GetCurrentTime();
        TimeoutInterval = COUNTER_STOP_TIMEOUT_MS;
        
        /* TODO 
           command the DC Motor to stop
           command AC Motor to Stop (Coast)
           command Sensors to Disable
        */
        StopMotors();
      }
      /* else - counting yet not started,  waiting for counting to start / timeout */
    }
    break;
    
    case COUNTER_STATE_COUNTING:
    {
      /* Counter is currently counting, 
         it would be stopped explicitly by brake command or 
         stop command generated by feeder empty timeout
      */
      if(COUNTER_CMD_STOP == CounterCommand) 
      {
       /* Counter stop command received */ 
        CounterCommand = COUNTER_CMD_NONE;
        CounterState = COUNTER_STATE_STOPPING;
        
        BackUpSystemTimer = SYSTMR_GetCurrentTime();
        TimeoutInterval = COUNTER_STOP_TIMEOUT_MS;
        
        /* TODO 
           command the DC Motor to stop
           command AC Motor to Stop (Coast)
           command Sensors to Disable
        */
        StopMotors();
      }
      else if(COUNTER_CMD_BRAKE == CounterCommand) 
      {
       /* Counter brake command received */ 
        CounterCommand = COUNTER_CMD_NONE;
        CounterState = COUNTER_STATE_STOPPING;
        
        BackUpSystemTimer = SYSTMR_GetCurrentTime();
        TimeoutInterval = COUNTER_BRAKE_TIMEOUT_MS;
        
        /* TODO 
           command the DC Motor to stop
           command AC Motor to Brake
           command Sensors to Disable
        */
        BrakeMotors();        
      }
      /* Counter is counting, monitor the health of the counter */
      else 
      {
        /* TODO - Monitor the health of the counter */
        /* Generate a counting stop command if counter timesout without any currency bills in the feeder */
        
      }
    }
    break;
    
    case COUNTER_STATE_STOPPING:
    {
      /* check if the counter has stopped counting,
         change the state to IDLE
      */
      if(true == IsCountingStopped())
      {
        CounterState = COUNTER_STATE_IDLE;
        
        DisableSensors();
      }
      /* Wait for the timeout period to expire before
         counter error to be flagged
      */
      else if (TimeoutInterval <= SYSTMR_GetElapsedTime(BackUpSystemTimer))
      {
        DisableSensors();
        
        /* TODO - Update the Error Register with the type of Error */
        CounterState = COUNTER_STATE_ERROR;
      }
    }
    break;
    
    case COUNTER_STATE_ERROR:
    {
      if(COUNTER_CMD_CLEAR == CounterCommand)
      {
        CounterCommand = COUNTER_CMD_NONE;
        CounterState = COUNTER_STATE_IDLE;
        
        /* TODO - re-initialize counter */
      }
    }
    break;
    
    default:
    {
        CounterCommand = COUNTER_CMD_NONE;
        CounterState = COUNTER_STATE_IDLE;
        
        /* TODO - re-initialize counter */
    }
    break;
    
  }
}  

uint16_t COUNTER_GetRPM(void)
{
  /* TODO - implement */
  return 0;
}


THICKNESS_SENSOR_STATE_E PopulateThicknessSensorState(uint16_t thicknessValue)
{
  THICKNESS_SENSOR_STATE_E thicknessSensorState;
    
  if ( THICKNESS_VALUE_UNAVAILABLE == thicknessValue )
  {
    thicknessSensorState = ThicknessUnknown;
  }
  else if ( THICKNESS_VALUE_RANGE_MIN > thicknessValue )
  {
    thicknessSensorState = ThicknessInValid;
  }
  else if ( THICKNESS_VALUE_NOM_MIN > thicknessValue )
  {
    thicknessSensorState = ThicknessUnderRange;
  }
  else if ( THICKNESS_VALUE_NOM_MAX > thicknessValue )
  {
    thicknessSensorState = ThicknessWithinRange;
  }
  else if  ( THICKNESS_VALUE_RANGE_MAX > thicknessValue )
  {
    thicknessSensorState = ThicknessOverRange;
  }
  else
  {
    thicknessSensorState = ThicknessInValid;
  }
  
  return thicknessSensorState;
}

FEEDER_SENSOR_STATE_E PopulateFeederSensorState(FEEDER_SENSOR_STATE_E prevState, uint16_t feederValue)
{
  FEEDER_SENSOR_STATE_E feederSensorState;
  
  /* Feeder Value is Unavaiable */  
  if ( FEEDER_VALUE_UNAVAILABLE == feederValue )
  {
    feederSensorState = FeederUnknown;
  }
  /* Feeder Value is less than minimum range */
  else if ( FEEDER_VALUE_RANGE_MIN > feederValue )
  {
    feederSensorState = FeederInValid;
  }
  /* Feeder Value is less than empty threshold */
  else if ( FEEDER_VALUE_EMPTY_THLD > feederValue )
  {
    feederSensorState = FeederEmpty;
  }
  /* Feeder Value is more than empty threshold, but less than not-empty threshold
     if the previous state was not-empty a hysterisis is applied until the 
     value falls below feeder empty threshold value */
  else if  ( FEEDER_VALUE_NOT_EMPTY_THLD > feederValue )
  {
    feederSensorState = (prevState == FeederNotEmpty) ? FeederNotEmpty : FeederEmpty;
  }
  /* Feeder value is more than feeder not-empty threshold */
  else if  ( FEEDER_VALUE_RANGE_MAX > feederValue )
  {
    feederSensorState = FeederNotEmpty;
  }
  /* Feeder value is more than maximum range */
  else
  {
    feederSensorState = FeederInValid;
  }
  
  return feederSensorState;
}


STACKER_SENSOR_STATE_E PopulateStackerSensorState(STACKER_SENSOR_STATE_E prevState, uint16_t stackerValue)
{
  STACKER_SENSOR_STATE_E stackerSensorState;
  
  /* Stacker Value is Unavaiable */  
  if ( STACKER_VALUE_UNAVAILABLE == stackerValue )
  {
    stackerSensorState = StackerUnknown;
  }
  /* Stacker Value is less than minimum range */
  else if ( STACKER_VALUE_RANGE_MIN > stackerValue )
  {
    stackerSensorState = StackerInValid;
  }
  /* Stacker Value is less than not-empty threshold */
  else if ( STACKER_VALUE_NOT_EMPTY_THLD > stackerValue )
  {
    stackerSensorState = StackerNotEmpty;
  }
  /* Stacker Value is more than not-empty threshold, but less than empty threshold
     if the previous state was empty a hysterisis is applied until the 
     value falls below stacker not-empty threshold value */
  else if  ( STACKER_VALUE_EMPTY_THLD > stackerValue )
  {
    stackerSensorState = (prevState == StackerEmpty) ? StackerEmpty : StackerNotEmpty;
  }
  /* Stacker value is more than stacker empty threshold */
  else if  ( STACKER_VALUE_RANGE_MAX > stackerValue )
  {
    stackerSensorState = StackerEmpty;
  }
  /* Stacker value is more than maximum range */
  else
  {
    stackerSensorState = StackerInValid;
  }
  
  return stackerSensorState;
}


UV_SENSOR_STATE_E PopulateUVSensorState(UV_SENSOR_STATE_E prevState, uint16_t UVValue)
{
  UV_SENSOR_STATE_E UVSensorState;
  
  /* UV Value is Unavaiable */  
  if ( UV_VALUE_UNAVAILABLE == UVValue )
  {
    UVSensorState = UVUnknown;
  }
  /* UV Value is less than minimum range */
  else if ( UV_VALUE_RANGE_MIN > UVValue )
  {
    UVSensorState = UVInValid;
  }
  /* UV Value is less than no-detection threshold */
  else if ( UV_VALUE_NO_DETECT_THLD > UVValue )
  {
    UVSensorState = UVNotDetected;
  }
  /* UV Value is more than no-detection threshold, but less than detection threshold
     if the previous state was detected a hysterisis is applied until the 
     value falls below UV no-detection value */
  else if  ( UV_VALUE_DETECT_THLD > UVValue )
  {
    UVSensorState = (prevState == UVDetected) ? UVDetected : UVNotDetected;
  }
  /* UV value is more than UV detection threshold */
  else if  ( UV_VALUE_RANGE_MAX > UVValue )
  {
    UVSensorState = UVDetected;
  }
  /* UV value is more than maximum range */
  else
  {
    UVSensorState = UVInValid;
  }
  
  return UVSensorState;
}


MG_SENSOR_STATE_E PopulateMGSensorState(MG_SENSOR_STATE_E prevState, uint16_t MGValue)
{
  MG_SENSOR_STATE_E MGSensorState;
  
  /* MG Value is Unavaiable */  
  if ( MG_VALUE_UNAVAILABLE == MGValue )
  {
    MGSensorState = MGUnknown;
  }
  /* MG Value is less than minimum range */
  else if ( MG_VALUE_RANGE_MIN > MGValue )
  {
    MGSensorState = MGInValid;
  }
  /* MG Value is less than no-detection threshold */
  else if ( MG_VALUE_NO_DETECT_THLD > MGValue )
  {
    MGSensorState = MGNotDetected;
  }
  /* MG Value is more than no-detection threshold, but less than detection threshold
     if the previous state was detected a hysterisis is applied until the 
     value falls below MG no-detection value */
  else if  ( MG_VALUE_DETECT_THLD > MGValue )
  {
    MGSensorState = (prevState == MGDetected) ? MGDetected : MGNotDetected;
  }
  /* MG value is more than MG detection threshold */
  else if  ( MG_VALUE_RANGE_MAX > MGValue )
  {
    MGSensorState = MGDetected;
  }
  /* MG value is more than maximum range */
  else
  {
    MGSensorState = MGInValid;
  }
  
  return MGSensorState;
}

BILL_DETECTOR_STATE_E PopulateBillDetectorState(BILL_DETECTOR_STATE_E prevState, uint16_t BillDetectorValue)
{
  BILL_DETECTOR_STATE_E BillDetectorState;
  
  /* Bill Detector Value is Unavaiable */  
  if ( DETECT_VALUE_UNAVAILABLE == BillDetectorValue )
  {
    BillDetectorState = BillDetectorUnknown;
  }
  /* Bill Detector Value is less than minimum range */
  else if ( DETECT_VALUE_RANGE_MIN > BillDetectorValue )
  {
    BillDetectorState = BillDetectorInValid;
  }
  /* Bill Detector Value is less than blocked threshold */
  else if ( DETECT_VALUE_BLOCKED_THLD > BillDetectorValue )
  {
    BillDetectorState = BillDetectorBlocked;
  }
  /* Bill Detector Value is more than blocked threshold, but less than not-blocked threshold
     if the previous state was not-blocked a hysterisis is applied until the 
     value falls below Bill Detector blocked value */
  else if  ( DETECT_VALUE_NOT_BLOCKED_THLD > BillDetectorValue )
  {
    BillDetectorState = (prevState == BillDetectorNotBlocked) ? BillDetectorNotBlocked : BillDetectorBlocked;
  }
  /* Bill Detector value is more than Bill Detector not-blocked threshold */
  else if  ( DETECT_VALUE_RANGE_MAX > BillDetectorValue )
  {
    BillDetectorState = BillDetectorNotBlocked;
  }
  /* Bill Detector value is more than maximum range */
  else
  {
    BillDetectorState = BillDetectorInValid;
  }
  
  return BillDetectorState;
}

void ReadSensorValuesAndPopulateSensorStates(void)
{
  uint16_t adcValues[ADC_MAX_CHANNELS];
  
  /* TODO - Check if th ADC readings are ready before reading the ADC Values
  */ 
  ADC_GetAvgData(&adcValues[0]);
  
  /* Convert ADC Values to respective Sensor Values*/
  Counter.ThicknessValueLeft = ADC_TO_THICKNESS_VALUE(adcValues[ADC_THICK_L]);
  Counter.ThicknessValueRight = ADC_TO_THICKNESS_VALUE(adcValues[ADC_THICK_R]);
  Counter.FeederValue = ADC_TO_FEEDER_VALUE(adcValues[ADC_FEEDER_IN]);
  Counter.StackerValue = ADC_TO_STACKER_VALUE(adcValues[ADC_STACKER_IN]);
  Counter.UVValue = ADC_TO_UV_VALUE(adcValues[ADC_UV_IN]);
  Counter.MGValue = ADC_TO_MG_VALUE(adcValues[ADC_MG_IN]);
  Counter.BillDetectorValueLeft = ADC_TO_BILL_DETECT_VALUE(adcValues[ADC_THICK_L]);
  Counter.BillDetectorValueRight = ADC_TO_BILL_DETECT_VALUE(adcValues[ADC_THICK_R]);
  
  /* Populate Sensor States based on Sensor Values */
  Counter.ThicknessStateLeft = PopulateThicknessSensorState(Counter.ThicknessValueLeft);
  Counter.ThicknessStateRight = PopulateThicknessSensorState(Counter.ThicknessValueRight);
  Counter.FeederState = PopulateFeederSensorState(Counter.FeederState, Counter.FeederValue);
  Counter.StackerState = PopulateStackerSensorState(Counter.StackerState, Counter.StackerValue);
  Counter.UVState = PopulateUVSensorState(Counter.UVState, Counter.UVValue);
  Counter.MGState = PopulateMGSensorState(Counter.MGState, Counter.MGValue);
  Counter.BillDetectorStateLeft = PopulateBillDetectorState(Counter.BillDetectorStateLeft, Counter.BillDetectorValueLeft);
  Counter.BillDetectorStateRight = PopulateBillDetectorState(Counter.BillDetectorStateRight, Counter.BillDetectorValueRight); 
}

void PopulateBillScannerState(void)
{
  /* Populate the Bill Scanner Occupied State */
  /* If any of the Bill detector sensors (left / right) are blocked, 
     we can say  bill scanner is occupied */
  if((BillDetectorBlocked == Counter.BillDetectorStateLeft)  || 
     (BillDetectorBlocked == Counter.BillDetectorStateRight) )
  {
    Counter.BillScannerState = BillScannerOccupied;
  }
  /* If both the Bill detector sensors (left / right) are unblocked, 
     we can say  bill scanner is empty */
  else if ((BillDetectorNotBlocked == Counter.BillDetectorStateLeft) && 
           (BillDetectorNotBlocked == Counter.BillDetectorStateRight))
  {
    Counter.BillScannerState = BillScannerEmpty;
  }
  /* If both the Bill detectors sensors (left / right) are either in
     invalid / unknown state, we can say bill scanner state is unknown */
  else
  {
    Counter.BillScannerState = BillScannerUnknown;
  }
}

void PopulateBillState(void)
{
  static BILL_SCANNER_STATE_E PrevBillScannerState = BillScannerUnknown;
  
  /* Check if the Bill moved in to the bill scanner  or
     moved out of the bill scanner */
  if(BillScannerEmpty == Counter.BillScannerState)
  {
    if(BillScannerOccupied == PrevBillScannerState)
    {
      /* bill moved out of the bill scanner */
      Counter.BillState = BillStateExited;
      /* TODO - based on the bill validation, increment the bill count / stop the counter */
    }
    else if(BillScannerEmpty == PrevBillScannerState)
    {
       /* waiting for the bill */
      Counter.BillState = BillStateNotPresent;
    }
    else /* if (BillScannerUnknown == PrevBillScannerState) */
    {
      Counter.BillState = BillStateNotPresent;
    }
  }
  else if (BillScannerOccupied == Counter.BillScannerState)
  {
    if(BillScannerEmpty == PrevBillScannerState)
    {
      /* bill moved in to the bill scanner */
      Counter.BillState = BillStateEntered;
      /* TODO - clear the sensors prev state and enable them to start fresh DAQ */
    }
    else if (BillScannerOccupied == PrevBillScannerState)
    {
      /* bill is still in the scanner */
      Counter.BillState = BillStatePresent;
      /* collect data  */
    }
    else /* if (BillScannerUnknown == PrevBillScannerState) */
    {
      /* This conditon can occur when the counting sensors were previously faulty or 
        counting sensors are just initialized */
       Counter.BillState = BillStatePresent; /* TODO - Check if state valid */
    }
  }
  else /* if (BillScannerUnknown == Counter.BillScannerState) */ 
  {
    if(BillScannerUnknown != PrevBillScannerState)
    {
      /* The bill counting sensors went faulty */
      Counter.BillState = BillStateFault;
    }
    else /* if (BillScannerUnknown == PrevBillScannerState) */
    {
      Counter.BillState = BillStateUnknown;
    }
  }

  PrevBillScannerState = Counter.BillScannerState;
}  

void COUNTER_Exec (void)
{
  /* Populate the counter sensor states based on the counter sensor values */
  ReadSensorValuesAndPopulateSensorStates();
  /* Populate Bill Scanner occupied state based on Bill detector state */
  PopulateBillScannerState();
  /* Populate Bill state based on Bill Scanner current state and previous state */
  PopulateBillState();
  /* Execute the counter state machine */
  COUNTER_StateMachine();
}
