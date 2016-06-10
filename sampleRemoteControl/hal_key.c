/**************************************************************************************************
  Filename:       hal_key.c
  Revised:        $Date: 2010-09-15 19:02:45 -0700 (Wed, 15 Sep 2010) $
  Revision:       $Revision: 23815 $

  Description:    This file contains the interface to the HAL KEY Service.


  Copyright 2006-2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/
/*********************************************************************
 NOTE: If polling is used, the hal_driver task schedules the KeyRead()
       to occur every 100ms.  This should be long enough to naturally
       debounce the keys.  The KeyRead() function remembers the key
       state of the previous poll and will only return a non-zero
       value if the key state changes.

 NOTE: If interrupts are used, the KeyRead() function is scheduled
       25ms after the interrupt occurs by the ISR.  This delay is used
       for key debouncing.  The ISR disables any further Key interrupt
       until KeyRead() is executed.  KeyRead() will re-enable Key
       interrupts after executing.  Unlike polling, when interrupts
       are enabled, the previous key state is not remembered.  This
       means that KeyRead() will return the current state of the keys
       (not a change in state of the keys).

 NOTE: If interrupts are used, the KeyRead() fucntion is scheduled by
       the ISR.  Therefore, the joystick movements will only be detected
       during a pushbutton interrupt caused by S1 or the center joystick
       pushbutton.

 NOTE: When a switch like S1 is pushed, the S1 signal goes from a normally
       high state to a low state.  This transition is typically clean.  The
       duration of the low state is around 200ms.  When the signal returns
       to the high state, there is a high likelihood of signal bounce, which
       causes a unwanted interrupts.  Normally, we would set the interrupt
       edge to falling edge to generate an interrupt when S1 is pushed, but
       because of the signal bounce, it is better to set the edge to rising
       edge to generate an interrupt when S1 is released.  The debounce logic
       can then filter out the signal bounce.  The result is that we typically
       get only 1 interrupt per button push.  This mechanism is not totally
       foolproof because occasionally, signal bound occurs during the falling
       edge as well.  A similar mechanism is used to handle the joystick
       pushbutton on the DB.  For the EB, we do not have independent control
       of the interrupt edge for the S1 and center joystick pushbutton.  As
       a result, only one or the other pushbuttons work reasonably well with
       interrupts.  The default is the make the S1 switch on the EB work more
       reliably.

*********************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"
#include "hal_board.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "osal.h"

#if (defined HAL_KEY) && (HAL_KEY == TRUE)

/**************************************************************************************************
 *                                              MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/


/**************************************************************************************************
 *                                            TYPEDEFS
 **************************************************************************************************/


/**************************************************************************************************
 *                                        GLOBAL VARIABLES
 **************************************************************************************************/
static uint8 halKeySavedKeys;     /* used to store previous key state in polling mode */
static halKeyCBack_t pHalKeyProcessFunction;
static uint8 HalKeyConfigured;
bool Hal_KeyIntEnable;            /* interrupt enable/disable flag */

/**************************************************************************************************
 *                                        FUNCTIONS - Local
 **************************************************************************************************/
void halProcessKeyInterrupt(void);



/**************************************************************************************************
 *                                        FUNCTIONS - API
 **************************************************************************************************/


/**************************************************************************************************
 * @fn      HalKeyInit
 *
 * @brief   Initilize Key Service
 *
 * @param   none
 *
 * @return  None
 **************************************************************************************************/
void HalKeyInit( void )
{
  /* Initialize previous key to 0 */
  halKeySavedKeys = 0;

  //HAL_KEY_SW_6_SEL &= ~(HAL_KEY_SW_6_BIT);    /* Set pin function to GPIO */
  //HAL_KEY_SW_6_DIR &= ~(HAL_KEY_SW_6_BIT);    /* Set pin direction to Input */

  //HAL_KEY_JOY_MOVE_SEL &= ~(HAL_KEY_JOY_MOVE_BIT); /* Set pin function to GPIO */
  //HAL_KEY_JOY_MOVE_DIR &= ~(HAL_KEY_JOY_MOVE_BIT); /* Set pin direction to Input */

  HAL_KEY_SW_1_SEL &= ~(HAL_KEY_SW_1_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_1_DIR &= ~(HAL_KEY_SW_1_BIT);    /* Set pin direction to Input */

  HAL_KEY_SW_2_SEL &= ~(HAL_KEY_SW_2_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_2_DIR &= ~(HAL_KEY_SW_2_BIT);    /* Set pin direction to Input */

  HAL_KEY_SW_3_SEL &= ~(HAL_KEY_SW_3_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_3_DIR &= ~(HAL_KEY_SW_3_BIT);    /* Set pin direction to Input */

  HAL_KEY_SW_4_SEL &= ~(HAL_KEY_SW_4_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_4_DIR &= ~(HAL_KEY_SW_4_BIT);    /* Set pin direction to Input */

  HAL_KEY_SW_5_SEL &= ~(HAL_KEY_SW_5_BIT);    /* Set pin function to GPIO */
  HAL_KEY_SW_5_DIR &= ~(HAL_KEY_SW_5_BIT);    /* Set pin direction to Input */
  

  /* Initialize callback function */
  pHalKeyProcessFunction  = NULL;

  /* Start with key is not configured */
  HalKeyConfigured = FALSE;
}


/**************************************************************************************************
 * @fn      HalKeyConfig
 *
 * @brief   Configure the Key serivce
 *
 * @param   interruptEnable - TRUE/FALSE, enable/disable interrupt
 *          cback - pointer to the CallBack function
 *
 * @return  None
 **************************************************************************************************/
void HalKeyConfig (bool interruptEnable, halKeyCBack_t cback)
{
  /* Enable/Disable Interrupt or */
  Hal_KeyIntEnable = interruptEnable;

  /* Register the callback fucntion */
  pHalKeyProcessFunction = cback;

  /* Determine if interrupt is enable or not */
  if (Hal_KeyIntEnable)
  {
    PICTL |= (HAL_KEY_SW_1_3_5_EDGEBIT | HAL_KEY_SW_2_4_EDGEBIT);   /* Set the edge bit to set falling edge to give int */
    
    HAL_KEY_SW_1_ICTL |= HAL_KEY_SW_1_ICTLBIT; /* enable interrupt generation at port */
    HAL_KEY_SW_1_IEN |= HAL_KEY_SW_1_IENBIT;   /* enable CPU interrupt */
    HAL_KEY_SW_1_PXIFG = ~(HAL_KEY_SW_1_BIT);  /* Clear any pending interrupt */

    HAL_KEY_SW_2_ICTL |= HAL_KEY_SW_2_ICTLBIT; /* enable interrupt generation at port */
    HAL_KEY_SW_2_IEN |= HAL_KEY_SW_2_IENBIT;   /* enable CPU interrupt */
    HAL_KEY_SW_2_PXIFG = ~(HAL_KEY_SW_2_BIT);  /* Clear any pending interrupt */

    HAL_KEY_SW_3_ICTL |= HAL_KEY_SW_3_ICTLBIT; /* enable interrupt generation at port */
    HAL_KEY_SW_3_IEN |= HAL_KEY_SW_3_IENBIT;   /* enable CPU interrupt */
    HAL_KEY_SW_3_PXIFG = (~(HAL_KEY_SW_3_BIT));  /* Clear any pending interrupt */

    HAL_KEY_SW_4_ICTL |= HAL_KEY_SW_4_ICTLBIT; /* enable interrupt generation at port */
    HAL_KEY_SW_4_IEN |= HAL_KEY_SW_4_IENBIT;   /* enable CPU interrupt */
    HAL_KEY_SW_4_PXIFG = ~(HAL_KEY_SW_4_BIT);  /* Clear any pending interrupt */

    HAL_KEY_SW_5_ICTL |= HAL_KEY_SW_5_ICTLBIT; /* enable interrupt generation at port */
    HAL_KEY_SW_5_IEN |= HAL_KEY_SW_5_IENBIT;   /* enable CPU interrupt */
    HAL_KEY_SW_5_PXIFG = (~(HAL_KEY_SW_5_BIT)); 
    
    

    /* Do this only after the hal_key is configured - to work with sleep stuff */
    if (HalKeyConfigured == TRUE)
    {
      osal_stop_timerEx(Hal_TaskID, HAL_KEY_EVENT);  /* Cancel polling if active */
    }
  }
  else    /* Interrupts NOT enabled */
  {
    HAL_KEY_SW_1_ICTL &= ~(HAL_KEY_SW_1_ICTLBIT); /* don't generate interrupt */
    HAL_KEY_SW_1_IEN &= ~(HAL_KEY_SW_1_IENBIT);   /* Clear interrupt enable bit */

    HAL_KEY_SW_2_ICTL &= ~(HAL_KEY_SW_2_ICTLBIT); /* don't generate interrupt */
    HAL_KEY_SW_2_IEN &= ~(HAL_KEY_SW_2_IENBIT);   /* Clear interrupt enable bit */

    HAL_KEY_SW_3_ICTL &= ~(HAL_KEY_SW_3_ICTLBIT); /* don't generate interrupt */
    HAL_KEY_SW_3_IEN &= ~(HAL_KEY_SW_3_IENBIT);   /* Clear interrupt enable bit */
    
    HAL_KEY_SW_4_ICTL &= ~(HAL_KEY_SW_4_ICTLBIT); /* don't generate interrupt */
    HAL_KEY_SW_4_IEN &= ~(HAL_KEY_SW_4_IENBIT);   /* Clear interrupt enable bit */

    HAL_KEY_SW_5_ICTL &= ~(HAL_KEY_SW_5_ICTLBIT); /* don't generate interrupt */
    HAL_KEY_SW_5_IEN &= ~(HAL_KEY_SW_5_IENBIT);   /* Clear interrupt enable bit */
   
    osal_set_event(Hal_TaskID, HAL_KEY_EVENT);
  }

  /* Key now is configured */
  HalKeyConfigured = TRUE;
}


/**************************************************************************************************
 * @fn      HalKeyRead
 *
 * @brief   Read the current value of a key
 *
 * @param   None
 *
 * @return  keys - current keys status
 **************************************************************************************************/
uint8 HalKeyRead ( void )
{
  uint8 keys = 0;

  if(!(HAL_KEY_SW_1_PORT & HAL_KEY_SW_1_BIT)){
    keys |= HAL_KEY_SW_1;
  }
  
  if(!(HAL_KEY_SW_2_PORT & HAL_KEY_SW_2_BIT)){
    keys |= HAL_KEY_SW_2;
  }
  
  if(!(HAL_KEY_SW_3_PORT & HAL_KEY_SW_3_BIT)){
    keys |= HAL_KEY_SW_3;
  }
  
  if(!(HAL_KEY_SW_4_PORT & HAL_KEY_SW_4_BIT)){
    keys |= HAL_KEY_SW_4;
  }
  
  if(!(HAL_KEY_SW_5_PORT & HAL_KEY_SW_5_BIT)){
    keys |= HAL_KEY_SW_5;
  }

  return keys;
}


/**************************************************************************************************
 * @fn      HalKeyPoll
 *
 * @brief   Called by hal_driver to poll the keys
 *
 * @param   None
 *
 * @return  None
 **************************************************************************************************/
void HalKeyPoll (void)
{
  uint8 keys = 0;
  uint8 notify = 0;

  if (!(HAL_KEY_SW_1_PORT & HAL_KEY_SW_1_BIT))    /* Key is active low */
  {
    keys |= HAL_KEY_SW_1;
  }
  
  if (!(HAL_KEY_SW_2_PORT & HAL_KEY_SW_2_BIT))    /* Key is active low */
  {
    keys |= HAL_KEY_SW_2;
  }
  
  if (!(HAL_KEY_SW_3_PORT & HAL_KEY_SW_3_BIT))    /* Key is active low */
  {
    keys |= HAL_KEY_SW_3;
  }
  
  if (!(HAL_KEY_SW_4_PORT & HAL_KEY_SW_4_BIT))    /* Key is active low */
  {
    keys |= HAL_KEY_SW_4;
  }
  
  if (!(HAL_KEY_SW_5_PORT & HAL_KEY_SW_5_BIT))    /* Key is active low */
  {
    keys |= HAL_KEY_SW_5;
  }
  
  (void) keys;
  
  /* If interrupts are not enabled, previous key status and current key status
   * are compared to find out if a key has changed status.
   */
  if (!Hal_KeyIntEnable)
  {
    if (keys == halKeySavedKeys)
    {
      /* Exit - since no keys have changed */
      return;
    }
    else{
      notify = 1;
    }
   
  }
  else
  {
    /* Key interrupt handled here */
    if(keys)
      notify = 1;
  }
   /* Store the current keys for comparation next time */
    halKeySavedKeys = keys;
  /* Invoke Callback if new keys were depressed */
  if (notify && (pHalKeyProcessFunction))
  {
    (pHalKeyProcessFunction) (keys, HAL_KEY_STATE_NORMAL);
  }
}

/**************************************************************************************************
 * @fn      halProcessKeyInterrupt
 *
 * @brief   Checks to see if it's a valid key interrupt, saves interrupt driven key states for
 *          processing by HalKeyRead(), and debounces keys by scheduling HalKeyRead() 25ms later.
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void halProcessKeyInterrupt (void)
{
  bool valid=FALSE;

  if( HAL_KEY_SW_1_PXIFG & HAL_KEY_SW_1_BIT) /* Interrupt Flag has been set by SW1 */
  {
    HAL_KEY_SW_1_PXIFG = ~(HAL_KEY_SW_1_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  if (HAL_KEY_SW_2_PXIFG & HAL_KEY_SW_2_BIT)  /* Interrupt Flag has been set by SW2 */
  {
    HAL_KEY_SW_2_PXIFG = ~(HAL_KEY_SW_2_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  if (HAL_KEY_SW_3_PXIFG & HAL_KEY_SW_3_BIT)  /* Interrupt Flag has been set by SW3 */
  {
    HAL_KEY_SW_3_PXIFG = (~(HAL_KEY_SW_3_BIT)); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  if (HAL_KEY_SW_4_PXIFG & HAL_KEY_SW_4_BIT)  /* Interrupt Flag has been set by SW2 */
  {
    HAL_KEY_SW_4_PXIFG = ~(HAL_KEY_SW_4_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }
  if (HAL_KEY_SW_5_PXIFG & HAL_KEY_SW_5_BIT)  /* Interrupt Flag has been set by SW3 */
  {
    HAL_KEY_SW_5_PXIFG = (uint8)(~(HAL_KEY_SW_5_BIT)); /* Clear Interrupt Flag */
    valid = TRUE;
  }

  if (valid)
  {
    osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
  }
}

/**************************************************************************************************
 * @fn      HalKeyEnterSleep
 *
 * @brief  - Get called to enter sleep mode
 *
 * @param
 *
 * @return
 **************************************************************************************************/
void HalKeyEnterSleep ( void )
{
}

/**************************************************************************************************
 * @fn      HalKeyExitSleep
 *
 * @brief   - Get called when sleep is over
 *
 * @param
 *
 * @return  - return saved keys
 **************************************************************************************************/
uint8 HalKeyExitSleep ( void )
{
  /* Wake up and read keys */
  return ( HalKeyRead () );
}

/***************************************************************************************************
 *                                    INTERRUPT SERVICE ROUTINE
 ***************************************************************************************************/


#else


void HalKeyInit(void){}
void HalKeyConfig(bool interruptEnable, halKeyCBack_t cback){}
uint8 HalKeyRead(void){ return 0;}
void HalKeyPoll(void){}

#endif /* HAL_KEY */



/**************************************************************************************************
 * @fn      halKeyPort0Isr
 *
 * @brief   Port0 ISR
 *
 * @param
 *
 * @return
 **************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort0Isr, P0INT_VECTOR )
{
  HAL_ENTER_ISR();

  
  if ((HAL_KEY_SW_1_PXIFG & HAL_KEY_SW_1_BIT) || (HAL_KEY_SW_3_PXIFG & HAL_KEY_SW_3_BIT) || (HAL_KEY_SW_5_PXIFG & HAL_KEY_SW_5_BIT))
  { 
    halProcessKeyInterrupt();
  }
  //HAL_KEY_SW_1_PXIFG = 0;
  //HAL_KEY_SW_3_PXIFG = 0;
  //HAL_KEY_SW_5_PXIFG = 0;
  /*
  Clear the CPU interrupt flag for Port_0
  PxIFG has to be cleared before PxIF
  */
 
  P0IF = 0;
  CLEAR_SLEEP_MODE();
  HAL_EXIT_ISR();
}

/**************************************************************************************************
* @fn      halKeyPort1Isr
*
* @brief   Port1 ISR
**************************************************************************************************/
HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
    HAL_ENTER_ISR();
    if ((HAL_KEY_SW_2_PXIFG & HAL_KEY_SW_2_BIT) || (HAL_KEY_SW_4_PXIFG & HAL_KEY_SW_4_BIT))
    {   
        halProcessKeyInterrupt();
    }
    /*
    Clear the CPU interrupt flag for Port_1
    PxIFG has to be cleared before PxIF
    */
    //HAL_KEY_SW_2_PXIFG = 0;
    //HAL_KEY_SW_4_PXIFG = 0;
   
    P1IF = 0;
    CLEAR_SLEEP_MODE();
    HAL_EXIT_ISR();
}


/**************************************************************************************************
**************************************************************************************************/



