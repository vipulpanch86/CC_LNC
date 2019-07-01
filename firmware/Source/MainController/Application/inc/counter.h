#ifndef __COUNTER_H_
#define __COUNTER_H_


/** @enum THICKNESS_SENSOR_STATE_E
  * @brief The state of the note thickness based on the thickness sensor values.
  */
typedef enum
{
    ThicknessUnknown = 0,   /* thickness state is not known */
    ThicknessWithinRange,   /* thickness in acceptable range */
    ThicknessUnderRange,    /* thickness above acceptable range */
    ThicknessOverRange,     /* thickness below acceptable range */
    ThicknessInValid,       /* invalid thickness value from sensor */
} THICKNESS_SENSOR_STATE_E;

/** @enum FEEDER_SENSOR_STATE_E
  * @brief The state of the feeder based on the feeder sensor values.
  */
typedef enum
{
    FeederUnknown = 0,     /* feeder state is not known */
    FeederNotEmpty,        /* feeder not empty */
    FeederEmpty,           /* feeder empty */
    FeederInValid,         /* invalid feeder value from sensor */
} FEEDER_SENSOR_STATE_E;

/** @enum STACKER_SENSOR_STATE_E
  * @brief The state of the stacker based on the stacker sensor values.
  */
typedef enum
{
    StackerUnknown = 0,     /* stacker state is not known */
    StackerNotEmpty,        /* stacker not empty */
    StackerEmpty,           /* stacker empty */
    StackerInValid,         /* invalid stacker value from sensor */
} STACKER_SENSOR_STATE_E;

/** @enum UV_SENSOR_STATE_E
  * @brief The state of the UV property based on the UV sensor values.
  */
typedef enum
{
    UVUnknown = 0,          /* UV state is not known */
    UVNotDetected,          /* UV property not detected */
    UVDetected,             /* UV property detected */
    UVInValid,              /* invalid UV value from sensor */
} UV_SENSOR_STATE_E;

/** @enum MG_SENSOR_STATE_E
  * @brief The state of the MG property based on the MG sensor values.
  */
typedef enum
{
    MGUnknown = 0,          /* MG state is not known */
    MGNotDetected,          /* MG property not detected */
    MGDetected,             /* MG property detected */
    MGInValid,              /* invalid MG value from sensor */
} MG_SENSOR_STATE_E;


/** @enum BILL_DETECTOR_STATE_E
  * @brief The state of the bill detector based on the thickness sensor values.
  */
typedef enum
{
    BillDetectorUnknown = 0,   /* bill detector state is not known */
    BillDetectorBlocked,       /* bill detector state is blocked  */
    BillDetectorNotBlocked,    /* bill detector state is not blocked */
    BillDetectorInValid,       /* invalid bill detector value from sensor */   
} BILL_DETECTOR_STATE_E;


/** @enum BILL_SCANNER_STATE_E
  * @brief The state of the Bill Scanner if occupied by a currency note.
  */
typedef enum
{
  BillScannerUnknown = 0,      /* Bill Scanner state is not known */
  BillScannerEmpty,            /* Bill Scanner state is Empty */
  BillScannerOccupied          /* Bill Scanner state is Occupied */
} BILL_SCANNER_STATE_E;

#endif /* __COUNTER_H_ */
