typedef enum
{
  StartTypeAuto,
  StartTypeManual
}START_TYPE_E;

typedef enum
{
  CountModeFree,
  CountModeCheck,
  CountModeBatch,
  CountModeSort,
}COUNT_MODE_E;

typedef enum
{
  CountAddDisable,
  CountAddEnable
}COUNT_ADD_E;


/* main application function of loose note counter */
void LooseNoteCounter(void)
{
  UI_MSG_T msg;
  
  /* Get the Counter Start Mode from Register */
  /* Get the Counter Mode of operation from Register */
  
  /* Get a message from UI */
  UI_GetMsg(&msg);
  
  switch (state)
  {
    case LNC_IDLE:
    {
      START_TYPE_E startType;
      /* TODO - get the Counter start type from register */
      /* Based on the type of counter start, selected by the user 
         Post a Command to the counter to start counting */
      
      if(((StartTypeAuto == startType) && (FeederNotEmpty == Counter.FeederState)) ||
        (UI_MSG_COUNT_START == msg))
      {
        
      }
      
    }
    break;
    
    case LNC_COUNTING:
    {
    }
    break;
  }
}
   