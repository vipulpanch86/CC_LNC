/*!
@file    sys_timer_utils.c
@brief    Implementation of utility functions for system timer

@details    32 bit system timer in milliseconds - elapsed time calculation includes handling of one rollover.
 */

#include <stdint.h>
#include <stdbool.h>
#include "bsp.h"
#include "sys_timer_utils.h"



/**
  * @brief Get the Current System Time
  * 
  * @return uint32_t
  */
uint32_t SYSTMR_GetCurrentTime(void)
{
    /* 32-bit time, units of 1 millisecond per tick */
    return BSP_GetSysTicks();
}

/*!
 * @brief Get the Elapsed Time
 * 
 * @param startTime 
 * @return uint32_t 
 */
uint32_t SYSTMR_GetElapsedTime(uint32_t startTime)
{
  uint32_t  stopTime = SYSTMR_GetCurrentTime();

  return    SYSTMR_CalcElapsedTime(startTime, stopTime);
}

/*!
 * @brief Calculate the Elapsed Time
 * @param startTime
 * @param endTime
 * 
 * @return uint32_t value of Elapsed Time
 */
uint32_t SYSTMR_CalcElapsedTime(uint32_t startTime, uint32_t endTime)
{
    uint32_t    rVal;

    if (endTime >= startTime)
    {
        /* no rollover, simple subtraction */
        rVal = endTime - startTime;
    }
    else
    {
        /* startTime > endTime */
        uint32_t diff = startTime - endTime;
        /* diff must be greater than 0.  So rVal cannot overflow UINT32_MAX. */
        rVal = UINT32_MAX - diff + 1;
    }

    return rVal;
}

/*!
 * @brief Helper function to decide if an interval has expired
 *
 * @detail Determination that an interval has expired needs to include handling for rollover
 *         of the 32-bit tick, plus should recover quickly if the time is missed or
 *         the previousTick gets badly unsynchronized with the current tick value.
 *
 * @param currentTick - the current system tick value
 * @param previousTick  - the tick at the start of the interval
 * @param interval    - the tick interval
 *
 * @return TRUE if interval has expired

 */
bool SYSTMR_HasIntervalExpired(uint32_t currentTick, uint32_t previousTick, uint32_t interval)
{
    bool rVal;

    /* targetTick and previousTick divide the timeline into three regions 
       which need to be handled differently depending on which one has the lower value */
    uint32_t targetTick = previousTick + interval;

    /* no rollover during this interval */
    if (previousTick < targetTick)
    {
        /* 'top' of the timeline - wait time has expired */
        if (currentTick >= targetTick)
        {
            rVal = true;
        }

        /* 'bottom' of the timeline - wait time has expired (rollover) */
        else if (currentTick < previousTick)
        {
            rVal = true;
        }

        /* 'middle of the timeline - current time is in wait interval */
        /* between last tick and next tick, not there yet
           (currentTick < targetTick) && (currentTick >= previousTick) */
        else
        {
            rVal = false;
        }
    }

    /* rollover occurred during this interval */
    else
    {
        /* 'bottom' of the timeline - current time is in wait interval between 
           last tick and next tick, not there yet */
        if (currentTick < targetTick)
        {
            rVal = false;
        }

        /* 'top' of the timeline - current time is in wait interval between
           last tick and next tick, not there yet */
        else if (currentTick >= previousTick)
        {
            rVal = false;
        }

        /* 'middle' of the timeline - outside of wait interval, go
           (currentTick > targetTick) && (currentTick < previousTick) */
        else
        {
            rVal = true;
        }
    }

    return rVal;
}

void SYSTMR_DelayMs(uint32_t delayMs)
{
  uint32_t startTime = SYSTMR_GetCurrentTime();

  while(SYSTMR_GetElapsedTime(startTime) < delayMs);
}
