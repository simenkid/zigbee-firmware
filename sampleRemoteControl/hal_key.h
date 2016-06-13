/**************************************************************************************************
  Filename:       hal_key.h
  Revised:        $Date: 2007-07-06 10:42:24 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Description:    This file contains the interface to the KEY Service.


  Copyright 2005-2012 Texas Instruments Incorporated. All rights reserved.

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

#ifndef HAL_KEY_H
#define HAL_KEY_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 *                                             INCLUDES
 **************************************************************************************************/
#include "hal_board.h"
  
/**************************************************************************************************
 * MACROS
 **************************************************************************************************/

/**************************************************************************************************
 *                                            CONSTANTS
 **************************************************************************************************/

/* Interrupt option - Enable or disable */
#define HAL_KEY_INTERRUPT_DISABLE    0x00
#define HAL_KEY_INTERRUPT_ENABLE     0x01

/* Key state - shift or nornal */
#define HAL_KEY_STATE_NORMAL          0x00
#define HAL_KEY_STATE_SHIFT           0x01

#define HAL_KEY_SW_1 0x01  // Joystick up
#define HAL_KEY_SW_2 0x02  // Joystick right
#define HAL_KEY_SW_5 0x04  // Joystick center
#define HAL_KEY_SW_4 0x08  // Joystick left
#define HAL_KEY_SW_3 0x10  // Joystick down

#define HAL_KEY_SW_6 0x20  // Button S1 if available
#define HAL_KEY_SW_7 0x40  // Button S2 if available

/* Joystick */
#define HAL_KEY_UP     0x01  // Joystick up
#define HAL_KEY_RIGHT  0x02  // Joystick right
#define HAL_KEY_CENTER 0x04  // Joystick center
#define HAL_KEY_LEFT   0x08  // Joystick left
#define HAL_KEY_DOWN   0x10  // Joystick down

/* Buttons */  
#define HAL_PUSH_BUTTON_RIGHT  0x01  // Button right
#define HAL_PUSH_BUTTON_LEFT   0x02  // Button left
#define HAL_PUSH_BUTTON_SELECT 0x04  // Button select
#define HAL_KEY_BUTTON_UP      0x40  // Button up
#define HAL_KEY_BUTTON_DOWN    0x80  // Button down

  
#define HAL_KEY_RISING_EDGE   0
#define HAL_KEY_FALLING_EDGE  1

#define HAL_KEY_DEBOUNCE_VALUE  25

/* CPU port interrupt */
#define HAL_KEY_CPU_PORT_0_IF P0IF
#define HAL_KEY_CPU_PORT_2_IF P2IF

//Push Botton UP P0.1
#define HAL_KEY_SW_1_PORT   P0
#define HAL_KEY_SW_1_BIT    BV(1)
#define HAL_KEY_SW_1_SEL    P0SEL
#define HAL_KEY_SW_1_DIR    P0DIR


//Push Botton RIGHT P1.2
#define HAL_KEY_SW_2_PORT   P1
#define HAL_KEY_SW_2_BIT    BV(2)
#define HAL_KEY_SW_2_SEL    P1SEL
#define HAL_KEY_SW_2_DIR    P1DIR


//Push Botton DOWN P0.4
#define HAL_KEY_SW_3_PORT   P0
#define HAL_KEY_SW_3_BIT    BV(4)
#define HAL_KEY_SW_3_SEL    P0SEL
#define HAL_KEY_SW_3_DIR    P0DIR


//Push Botton LEFT P1.1
#define HAL_KEY_SW_4_PORT   P1
#define HAL_KEY_SW_4_BIT    BV(1)
#define HAL_KEY_SW_4_SEL    P1SEL
#define HAL_KEY_SW_4_DIR    P1DIR


//Push Botton SELECT P0.5
#define HAL_KEY_SW_5_PORT   P0
#define HAL_KEY_SW_5_BIT    BV(5)
#define HAL_KEY_SW_5_SEL    P0SEL
#define HAL_KEY_SW_5_DIR    P0DIR



#define HAL_KEY_SW_1_IEN      IEN1  /* CPU interrupt mask register */
#define HAL_KEY_SW_1_ICTL     P0IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_1_ICTLBIT  BV(1) /* P0IEN - P0.1 enable/disable bit */
#define HAL_KEY_SW_1_IENBIT   BV(5) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_1_PXIFG    P0IFG /* Interrupt flag at source */

#define HAL_KEY_SW_2_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_2_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_2_ICTLBIT  BV(2) /* P0IEN - P0.4 enable/disable bit */
#define HAL_KEY_SW_2_IENBIT   BV(4) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_2_PXIFG    P1IFG /* Interrupt flag at source */

#define HAL_KEY_SW_3_IEN      IEN1  /* CPU interrupt mask register */
#define HAL_KEY_SW_3_ICTL     P0IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_3_ICTLBIT  BV(4) /* P0IEN - P0.5 enable/disable bit */
#define HAL_KEY_SW_3_IENBIT   BV(5) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_3_PXIFG    P0IFG /* Interrupt flag at source */

#define HAL_KEY_SW_4_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_SW_4_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_4_ICTLBIT  BV(1) /* P0IEN - P1.1 enable/disable bit */
#define HAL_KEY_SW_4_IENBIT   BV(4) /* Mask bit for all of Port_1 */
#define HAL_KEY_SW_4_PXIFG    P1IFG /* Interrupt flag at source */

#define HAL_KEY_SW_5_IEN      IEN1  /* CPU interrupt mask register */
#define HAL_KEY_SW_5_ICTL     P1IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_5_ICTLBIT  BV(5) /* P0IEN - P1.2 enable/disable bit */
#define HAL_KEY_SW_5_IENBIT   BV(5) /* Mask bit for all of Port_1 */
#define HAL_KEY_SW_5_PXIFG    P0IFG /* Interrupt flag at source */

/* SW_6 is at P0.1 */
#define HAL_KEY_SW_6_PORT   P0
#define HAL_KEY_SW_6_BIT    BV(1)
#define HAL_KEY_SW_6_SEL    P0SEL
#define HAL_KEY_SW_6_DIR    P0DIR

/* edge interrupt */
#define HAL_KEY_SW_6_EDGEBIT  BV(0)
#define HAL_KEY_SW_6_EDGE     HAL_KEY_FALLING_EDGE


/* SW_6 interrupts */
#define HAL_KEY_SW_6_IEN      IEN1  /* CPU interrupt mask register */
#define HAL_KEY_SW_6_IENBIT   BV(5) /* Mask bit for all of Port_0 */
#define HAL_KEY_SW_6_ICTL     P0IEN /* Port Interrupt Control register */
#define HAL_KEY_SW_6_ICTLBIT  BV(1) /* P0IEN - P0.1 enable/disable bit */
#define HAL_KEY_SW_6_PXIFG    P0IFG /* Interrupt flag at source */

/* Joy stick move at P2.0 */
#define HAL_KEY_JOY_MOVE_PORT   P2
#define HAL_KEY_JOY_MOVE_BIT    BV(0)
#define HAL_KEY_JOY_MOVE_SEL    P2SEL
#define HAL_KEY_JOY_MOVE_DIR    P2DIR

/* edge interrupt */
#define HAL_KEY_JOY_MOVE_EDGEBIT  BV(3)
#define HAL_KEY_JOY_MOVE_EDGE     HAL_KEY_FALLING_EDGE

/* Joy move interrupts */
#define HAL_KEY_JOY_MOVE_IEN      IEN2  /* CPU interrupt mask register */
#define HAL_KEY_JOY_MOVE_IENBIT   BV(1) /* Mask bit for all of Port_2 */
#define HAL_KEY_JOY_MOVE_ICTL     P2IEN /* Port Interrupt Control register */
#define HAL_KEY_JOY_MOVE_ICTLBIT  BV(0) /* P2IENL - P2.0<->P2.3 enable/disable bit */
#define HAL_KEY_JOY_MOVE_PXIFG    P2IFG /* Interrupt flag at source */

#define HAL_KEY_JOY_CHN   HAL_ADC_CHANNEL_6

#define HAL_KEY_SW_1_3_5_EDGEBIT        BV(0)
#define HAL_KEY_SW_2_4_EDGEBIT          BV(1)  
  
  

/**************************************************************************************************
 * TYPEDEFS
 **************************************************************************************************/
typedef void (*halKeyCBack_t) (uint8 keys, uint8 state);

/**************************************************************************************************
 *                                             GLOBAL VARIABLES
 **************************************************************************************************/
extern bool Hal_KeyIntEnable;

/**************************************************************************************************
 *                                             FUNCTIONS - API
 **************************************************************************************************/

/*
 * Initialize the Key Service
 */
extern void HalKeyInit( void );

/*
 * Configure the Key Service
 */
extern void HalKeyConfig( bool interruptEnable, const halKeyCBack_t cback);

/*
 * Read the Key status
 */
extern uint8 HalKeyRead( void);

/*
 * Enter sleep mode, store important values
 */
extern void HalKeyEnterSleep ( void );

/*
 * Exit sleep mode, retore values
 */
extern uint8 HalKeyExitSleep ( void );

/*
 * This is for internal used by hal_driver
 */
extern void HalKeyPoll ( void );

/*
 * This is for internal used by hal_sleep
 */
extern bool HalKeyPressed( void );

void halProcessKeyInterrupt(void);

extern uint8 hal_key_keys(void);                                           

extern uint8 hal_key_int_keys(void);

/**************************************************************************************************
**************************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif
