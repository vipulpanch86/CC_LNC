#include <stdint.h>
#include <stdbool.h>

#include "bsp.h"
#include "sys_timer_utils.h"

#define COUNTER_RPM_TO_ACM_RPM(x) ((x*2)/3) /* TODO - Fix formula */ 

#define ACM_RPM_RAMP_UP_THLD  (500) /* TODO - Fix */
#define ACM_RPM_RUNNING_MIN   (1200) /* TODO - Fix */
#define ACM_RPM_RUNNING_MAX   (1800) /* TODO - Fix */
#define ACM_RPM_BRAKE_THLD    (200)  /* TODO - Fix */
#define ACM_RPM_COAST_THLD    (100)  /* TODO - Fix */


#define ACM_RAMP_UP_MAX_INTERVAL      (1000) /* TODO - Fix */
#define ACM_BRAKE_DEAD_TIME_INTERVAL  (100) /* TODO - Fix */
#define ACM_BRAKE_MAX_INTERVAL        (2000)  /* TODO - Fix */
#define ACM_COAST_MAX_INTERVAL        (3000)  /* TODO - Fix */

typedef enum
{
  ACM_STATE_IDLE,
  ACM_STATE_RAMPING,
  ACM_STATE_RUNNING,
#if (ACM_BRAKE_DEAD_TIME_INTERVAL > 0)
  ACM_STATE_WAIT_DEAD_TIME_INTERVAL,
#endif
  ACM_STATE_BRAKING,
  ACM_STATE_COASTING,
  ACM_STATE_FAULT,
  
  ACM_STATE_MAX_NB
  
} ACM_STATE_LIST_T;

typedef enum
{
  ACM_CMD_NONE,
  ACM_CMD_RUN,
  ACM_CMD_COAST,
  ACM_CMD_BRAKE,
  ACM_CMD_CLEAR,
  
  ACM_CMD_MAX_NB
  
} ACM_CMD_LIST_T;

uint8_t ACMotorState, ACMotorCommand;


extern uint16_t COUNTER_GetRPM(void);

uint8_t ACM_Command(uint8_t command)
{
  if(ACM_CMD_NONE == ACMotorCommand)
  {
    /* TODO -  test if the command is valid for the current motor state */
    ACMotorCommand = command;
    
    return SUCCESS;
  }
  
  return ERROR;
}

void ACM_StateMachine(void)
{
  static uint32_t BackupSystemTimer = 0;
  
  switch(ACMotorState)
  {
    case ACM_STATE_IDLE:
    {  
      /* Motor is in Idle State,
         Accept the Motor RUN command
      */
      if(ACMotorCommand == ACM_CMD_RUN)
      {
        ACMotorCommand = ACM_CMD_NONE;
        
        /* Enable the Main TRIAC */
        BSP_ACMotorAuxCommand(BSP_DISABLE);
        BSP_ACMotorMainCommand(BSP_ENABLE);
        
        /* Save the system timer time-stamp to check ramp up time period */
        BackupSystemTimer = SYSTMR_GetCurrentTime();
        ACMotorState = ACM_STATE_RAMPING;
      }
    }
    break;
    
    case ACM_STATE_RAMPING:
    {
      /* Check if the desired RPM is acheived with in the ramp timeout period 
         If yes, changed the state to running.
         If no, stop the motor and change the state to fault state (Motor 
         Cable may be disconnected / broken ) 
      */
      uint16_t acmRPM = COUNTER_RPM_TO_ACM_RPM(COUNTER_GetRPM());
      if(ACM_RPM_RAMP_UP_THLD <= acmRPM)
      {
        ACMotorState = ACM_STATE_RUNNING;
      }
      else if(ACM_RAMP_UP_MAX_INTERVAL <= SYSTMR_GetElapsedTime(BackupSystemTimer))
      {
        BSP_ACMotorAuxCommand(BSP_DISABLE);
        BSP_ACMotorMainCommand(BSP_DISABLE);
        
        ACMotorState = ACM_STATE_FAULT;
      }
    }
    break;
    
    case ACM_STATE_RUNNING:
    {
      /* Continiously monitor if the motor is running with desired RPM,
         Change the state to fault state if the RPM out of range.
         Monitor AC motor commands - Brake,  Coast
      */
      uint16_t acmRPM = COUNTER_RPM_TO_ACM_RPM(COUNTER_GetRPM());
      if(ACM_RPM_RUNNING_MIN <= acmRPM && ACM_RPM_RUNNING_MAX <= acmRPM)
      {
        if(ACMotorCommand == ACM_CMD_BRAKE)
        {
          ACMotorCommand = ACM_CMD_NONE;
          
          BSP_ACMotorMainCommand(BSP_DISABLE);
          
        #if (ACM_BRAKE_DEAD_TIME_INTERVAL > 0)
          /* Save the system timer time-stamp to check dead time before brake */
          BackupSystemTimer = SYSTMR_GetCurrentTime();
          ACMotorState = ACM_STATE_WAIT_DEAD_TIME_INTERVAL;
        #else
          BSP_ACMotorAuxCommand(BSP_ENABLE);
          /* Save the system timer time-stamp to check ramp down time period */
          BackupSystemTimer = SYSTMR_GetCurrentTime();
          ACMotorState = ACM_STATE_BRAKING;
        #endif
        }
        
        if(ACMotorCommand == ACM_CMD_COAST)
        {
          ACMotorCommand = ACM_CMD_NONE;
          
          BSP_ACMotorMainCommand(BSP_DISABLE);
          
          /* Save the system timer time-stamp to check motor coast time period */
          BackupSystemTimer = SYSTMR_GetCurrentTime();
          
          ACMotorState = ACM_STATE_COASTING;
        }
      }
    }
    break;
    
  #if (ACM_BRAKE_DEAD_TIME_INTERVAL > 0)
    case ACM_STATE_WAIT_DEAD_TIME_INTERVAL:
    {
      /* Wait for a dead time interval before activating brake */
      uint32_t currSysTimer = SYSTMR_GetCurrentTime();
      if(ACM_BRAKE_DEAD_TIME_INTERVAL <= SYSTMR_GetElapsedTime(BackupSystemTimer))
      {
        BSP_ACMotorAuxCommand(BSP_ENABLE);
        
        ACMotorState = ACM_STATE_BRAKING;
      }
    }
    break;
  #endif
    
    case ACM_STATE_BRAKING:
    {
      /* Read RPM from Counter Sensor and wait for RPM to drop below a limit 
         If RPM drops below threshold, disable brake and change the state to idle.
         else disable brake and change the state to fault (Brake pin connection may be broken)
      */
      uint16_t acmRPM = COUNTER_RPM_TO_ACM_RPM(COUNTER_GetRPM());
      if(ACM_RPM_BRAKE_THLD >= acmRPM)
      {
        BSP_ACMotorAuxCommand(BSP_DISABLE);
        ACMotorState = ACM_STATE_IDLE;
      }
      else if(ACM_BRAKE_MAX_INTERVAL <= SYSTMR_GetElapsedTime(BackupSystemTimer))
      {
        BSP_ACMotorAuxCommand(BSP_DISABLE);
        BSP_ACMotorMainCommand(BSP_DISABLE);
        
        ACMotorState = ACM_STATE_FAULT;
      }
    }      
    break;
    
    case ACM_STATE_COASTING:
    {
      /* Read RPM from Counter Sensor and wait for RPM to drop below a limit 
         If RPM drops below threshold, change the state to idle.
         else change the state to fault (Brake pin connection may be broken)
      */
      uint16_t acmRPM = COUNTER_RPM_TO_ACM_RPM(COUNTER_GetRPM());
      if(ACM_RPM_COAST_THLD >= acmRPM)
      {
        ACMotorState = ACM_STATE_IDLE;
      }
      else if(ACM_COAST_MAX_INTERVAL <= SYSTMR_GetElapsedTime(BackupSystemTimer))
      {
        BSP_ACMotorAuxCommand(BSP_DISABLE);
        BSP_ACMotorMainCommand(BSP_DISABLE);
        
        ACMotorState = ACM_STATE_FAULT;
      }
   }
    break;
    
    case ACM_STATE_FAULT:
    {
      /* Motor is in Fault State,
         If Clear command received,
         Change the state to IDLE and Disable the AC Motor TRIAC outputs
      */
      if(ACMotorCommand == ACM_CMD_CLEAR)
      {
        ACMotorCommand = ACM_CMD_NONE;
        ACMotorState = ACM_STATE_IDLE;
        
        BSP_ACMotorAuxCommand(BSP_DISABLE);
        BSP_ACMotorMainCommand(BSP_DISABLE);
      }        
    }
    break;
    
    default:
    {
      /* Motor is in Unknown State,
         Change the state to IDLE and Disable the AC Motor TRIAC outputs */
        ACMotorCommand = ACM_CMD_NONE;
        ACMotorState = ACM_STATE_IDLE;
      
        BSP_ACMotorAuxCommand(BSP_DISABLE);
        BSP_ACMotorMainCommand(BSP_DISABLE);
    }
    break;
  }
}
