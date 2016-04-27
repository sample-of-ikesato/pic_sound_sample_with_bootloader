/********************************************************************
 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *******************************************************************/

/** INCLUDES *******************************************************/
#include "system.h"
#include "system_config.h"

#include "app_device_cdc_basic.h"
#include "app_led_usb_status.h"

#include "usb.h"
#include "usb_device.h"
#include "usb_device_cdc.h"


/**
 * シリアル・ポートの Ready を待つ
 * timer1 を使ってオーバフローならば 0 を返す
 * Ready になれば 1 を返す
 */
int WaitToReadySerial(void)
{
  PIR1bits.TMR1IF = 0;
  TMR1 = 0;
  PIR1bits.TMR1IF = 0;
  while (PIR1bits.TMR1IF==0) {
    if (mUSBUSARTIsTxTrfReady())
      return 1;
    CDCTxService();
  }
  return 0;
}

//void PutsString(const char *str)
//{
//  if (!WaitToReadySerial()) return;
//  strcpypgm2ram(USB_In_Buffer, (const far rom char*)str);
//  putsUSBUSART(USB_In_Buffer);
//  if (!WaitToReadySerial()) return;
//}

void PutsStringCPtr(char *str)
{
  if (!WaitToReadySerial()) return;
  putsUSBUSART(str);
  if (!WaitToReadySerial()) return;
}




unsigned char debug_flg = 0;
char *dubug_str = "hoge\r\n";

#define T0CNT (65536-46875)
void interrupt_func(void)
{
  if (INTCONbits.TMR0IF == 1) {
    TMR0 = T0CNT;
    INTCONbits.TMR0IF = 0;
    debug_flg = 1;
    if (PORTBbits.RB6) {
      PORTA = 0;
      PORTB = 0;
      PORTC = 0;
    } else {
      PORTA = 0xFFFF;
      PORTB = 0xFFFF;
      PORTC = 0xFFFF;
    }
  }
}

void init(void)
{
  TRISA = 0;
  TRISB = 0;
  TRISC = 0;
  PORTA = 0;
  PORTB = 0xFFFF;
  PORTC = 0;

  // timer
  // USB Bootloader では 48MHz で動作
  //
  // 1[s] を作るには
  // 48MHz/4 = 12MHz
  // 1/12MHz = 0.083 [usec]
  // 1[s]/0.083[usec] = 12 * 10**6
  // prescaler を 1:256 なら
  //   12 * 10**6 / 256 = 46875
  //
  //
  // 48MHz/4 * x = 8kHz としたい
  //   x = 12*1000/8 = 1500
  //   prescaler を 1:4 とすると 1500/4 = 375
  //
  T0CONbits.T08BIT = 0;     // 16bit timer
  T0CONbits.T0PS = 0b111;   // prescaler 1:256
  T0CONbits.T0CS = 0;
  T0CONbits.PSA = 0;        // use prescaler
  T0CONbits.TMR0ON = 1;
  TMR0 = T0CNT;
  INTCON2bits.TMR0IP = 1;
  INTCONbits.TMR0IE = 1;
  INTCONbits.TMR0IF = 0;
  //RCONbits.IPEN = 1;
  INTCONbits.GIEH = 1;
  INTCONbits.GIEL = 1;


  // timer1
  T1CONbits.TMR1CS = 0;    // 内部クロック (FOSC/4)
  T1CONbits.T1CKPS = 0b11; // prescaler 1:8
  T1CONbits.RD16 = 1;      // 16bit
  T1CONbits.TMR1ON = 1;
  //PIE1bits.TMR1IE = 1;
  PIR1bits.TMR1IF = 0;
}

/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
MAIN_RETURN main(void)
{
    SYSTEM_Initialize(SYSTEM_STATE_USB_START);

    USBDeviceInit();
    USBDeviceAttach();

    init();


    while(1)
    {
        SYSTEM_Tasks();

        #if defined(USB_POLLING)
            // Interrupt or polling method.  If using polling, must call
            // this function periodically.  This function will take care
            // of processing and responding to SETUP transactions
            // (such as during the enumeration process when you first
            // plug in).  USB hosts require that USB devices should accept
            // and process SETUP packets in a timely fashion.  Therefore,
            // when using polling, this function should be called
            // regularly (such as once every 1.8ms or faster** [see
            // inline code comments in usb_device.c for explanation when
            // "or faster" applies])  In most cases, the USBDeviceTasks()
            // function does not take very long to execute (ex: <100
            // instruction cycles) before it returns.
            USBDeviceTasks();
        #endif


        /* If the USB device isn't configured yet, we can't really do anything
         * else since we don't have a host to talk to.  So jump back to the
         * top of the while loop. */
        if( USBGetDeviceState() < CONFIGURED_STATE )
        {
            /* Jump back to the top of the while loop. */
            continue;
        }

        /* If we are currently suspended, then we need to see if we need to
         * issue a remote wakeup.  In either case, we shouldn't process any
         * keyboard commands since we aren't currently communicating to the host
         * thus just continue back to the start of the while loop. */
        if( USBIsDeviceSuspended()== true )
        {
            /* Jump back to the top of the while loop. */
            continue;
        }

        //Application specific tasks
        APP_DeviceCDCBasicDemoTasks();
        if (debug_flg) {
          debug_flg = 0;
          if (WaitToReadySerial()) {
            PutsStringCPtr("hello world\r\n");
            PutsStringCPtr(dubug_str);
          }
        }

    }//end while
}//end main


/*******************************************************************
 * Function:        bool USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, uint16_t size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  uint16_t size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
bool USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, uint16_t size)
{
    switch( (int) event )
    {
        case EVENT_TRANSFER:
            break;

        case EVENT_SOF:
            /* We are using the SOF as a timer to time the LED indicator.  Call
             * the LED update function here. */
            APP_LEDUpdateUSBStatus();
            break;

        case EVENT_SUSPEND:
            /* Update the LED status for the suspend event. */
            APP_LEDUpdateUSBStatus();

            //Call the hardware platform specific handler for suspend events for
            //possible further action (like optionally going reconfiguring the application
            //for lower power states and going to sleep during the suspend event).  This
            //would normally be done in USB compliant bus powered applications, although
            //no further processing is needed for purely self powered applications that
            //don't consume power from the host.
            SYSTEM_Initialize(SYSTEM_STATE_USB_SUSPEND);
            break;

        case EVENT_RESUME:
            /* Update the LED status for the resume event. */
            APP_LEDUpdateUSBStatus();

            //Call the hardware platform specific resume from suspend handler (ex: to
            //restore I/O pins to higher power states if they were changed during the 
            //preceding SYSTEM_Initialize(SYSTEM_STATE_USB_SUSPEND) call at the start
            //of the suspend condition.
            SYSTEM_Initialize(SYSTEM_STATE_USB_RESUME);
            break;

        case EVENT_CONFIGURED:
            /* When the device is configured, we can (re)initialize the 
             * demo code. */
            APP_DeviceCDCBasicDemoInitialize();
            break;

        case EVENT_SET_DESCRIPTOR:
            break;

        case EVENT_EP0_REQUEST:
            /* We have received a non-standard USB request.  The HID driver
             * needs to check to see if the request was for it. */
            USBCheckCDCRequest();
            break;

        case EVENT_BUS_ERROR:
            break;

        case EVENT_TRANSFER_TERMINATED:
            break;

        default:
            break;
    }
    return true;
}

/*******************************************************************************
 End of File
*/
