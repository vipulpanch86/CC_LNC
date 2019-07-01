/*!
@file sys_timer_utils.h
@brief Declarations for utility functions for system timer

@details Note that actual tick timing is set in BSP.c

 */

#ifndef __SYS_TIMER_UTILS_H__
#define __SYS_TIMER_UTILS_H__


/*!
 * @brief number of milliseconds per tick
 *
 * @details must match setting in BSP.c
 */
#define MILLISECONDS_PER_TICK        (1)


/*!
 * @brief converts milliseconds into ticks
 */
#define MSECONDS(x_)    ((uint32_t)((x_)/MILLISECONDS_PER_TICK))


/*!
 * @brief converts seconds into ticks
 */
#define SECONDS(x_)     ((uint32_t)((x_)*(1000/MILLISECONDS_PER_TICK)))


/*!
 * @brief converts minutes into ticks
 */
#define MINUTES(x_)     ((uint32_t)(60*(x_)*(1000/MILLISECONDS_PER_TICK)))


/*!
 * @brief get standard system time in milliseconds
 *
 * @details rollover every 2^32 * 1 msec = 49.7 days
 *
 * @returns os time in units of 1 msec.
 */
uint32_t SYSTMR_GetCurrentTime(void);

/*!
 * @brief Calculate elapsed time given a starting time
 *
 * @details Automatically handles counter rollover
 * - get current Tick as end time
 * - rollover every 2^32 * 1 msec = 49.7 days
 *
 * @param startTime time stamp at the start of interval
 *
 * @return elapsed system time in units of 1 milliseconds
 */
uint32_t SYSTMR_GetElapsedTime(uint32_t startTime);


/*!
 * @brief Calculate elapsed time given two time stamps 
 *
 * @details Automatically handles (one) counter rollover
 * @details rollover every 2^32 * 1 msec = 49.7 days
 *
 * @param startTime - time stamp at the start of interval
 * @param endTime - time stamp at the end of interval
 *
 * @return elapsed system time in units of 1 millisecond
 */
uint32_t SYSTMR_CalcElapsedTime(uint32_t startTime, uint32_t endTime);


/*!
 * @brief Helper function to decide if an interval has expired
 *
 * @detail Determination that an interval has expired needs to 
 *         include handling for rollover of the 32-bit timer, 
 *         plus should recover quickly if the time is missed or
 *         the timer synchronization is lost.
 *
 * @param currentTime - the current system timer value
 * @param previousTick  - the system timer value at the start of the interval
 * @param interval - the timer interval to be evaluated
 *
 * @return TRUE if interval has expired
 */
bool SYSTMR_HasIntervalExpired(uint32_t currentTime, uint32_t previousTime, uint32_t interval);

void SYSTMR_DelayMs(uint32_t delayMs);

#endif /* __SYS_TIMER_UTILS_H__ */
