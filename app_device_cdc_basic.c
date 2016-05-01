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

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "usb.h"
#include "app_led_usb_status.h"
#include "app_device_cdc_basic.h"
#include "usb_config.h"
#include "queue.h"

/** VARIABLES ******************************************************/

static bool buttonPressed;
static char buttonMessage[] = "Button pressed.\r\n";
static uint8_t readBuffer[CDC_DATA_OUT_EP_SIZE];
static uint8_t writeBuffer[CDC_DATA_IN_EP_SIZE];

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




unsigned short gcounter = 0;
Queue queue;
//static unsigned char queue_buffer[64];
unsigned char queue_buffer[32];
//int hangry = 1;
int hangry = 0;
int eat = 0;
int playing = 0;
int waiting_data = 0;
int debug_flag = 0;
Queue cmd_queue;
//unsigned char cmd_queue_buffer[sizeof(queue_buffer)+2];
unsigned char cmd_queue_buffer[CDC_DATA_OUT_EP_SIZE];


//#define T0CNT (65536-375)
#define T0CNT (65536-375+117)
void interrupt_func(void)
{
  if (INTCONbits.TMR0IF == 1) {
    gcounter++;
    TMR0 = T0CNT;
    INTCONbits.TMR0IF = 0;
    eat = 1;
    // 1/(440*2) = n*1/8000
    // n = 8000/880 = 9.09
    if (gcounter > 9) {
      gcounter = 0;
      PORTBbits.RB7 = !PORTBbits.RB7;
      debug_flag = !debug_flag;
      if (debug_flag) {
        CCPR1L = 63;
        CCP1CONbits.DC1B = 0b11;
      } else {
        CCPR1L = 0;
        CCP1CONbits.DC1B = 0;
      }
    //  PORTA = 0;
    //  PORTB = 0;
    //  PORTC = 0;
    //  CCPR1L  = 0x00;
    //} else {
    //  PORTA = 0xFFFF;
    //  PORTB = 0xFFFF;
    //  PORTC = 0xFFFF;
    //  //CCPR1L  = 63;
    //  CCPR1L  = 1;
    }
  }
}

void init(void)
{
  TRISA = 0;
  TRISB = 0;
  TRISC = 0;
  PORTA = 0;
  PORTB = 0;
  PORTC = 0;

  // timer
  // USB Bootloader では 48MHz で動作
  //
  // 8kHz を作るには
  //   48MHz/4 * x = 8kHz としたい
  //   x = (48/4)*1000/8 = 1500
  //   prescaler を 1:4 とすると 1500/4 = 375
  //
  T0CONbits.T08BIT = 0;     // 16bit timer
  T0CONbits.T0PS = 0b001;   // prescaler 1:4
  T0CONbits.T0CS = 0;
  T0CONbits.PSA = 0;        // use prescaler
  T0CONbits.TMR0ON = 1;
  TMR0 = T0CNT;
  INTCON2bits.TMR0IP = 1;
  INTCONbits.TMR0IE = 1;
  INTCONbits.TMR0IF = 0;
  INTCONbits.GIEH = 1;
  INTCONbits.GIEL = 1;


  // timer1
  T1CONbits.TMR1CS = 0;    // 内部クロック (FOSC/4)
  T1CONbits.T1CKPS = 0b11; // prescaler 1:8
  T1CONbits.RD16 = 1;      // 16bit
  T1CONbits.TMR1ON = 1;
  PIR1bits.TMR1IF = 0;


  // PWM settings
  CCP1CONbits.CCP1M = 0b1100; // P1A、P1C をアクティブ High、P1B、P1D をアクティブ High
  CCP1CONbits.DC1B  = 0b11;   // デューティ サイクル値の最下位 2 ビット
  CCP1CONbits.P1M   = 0b00;   // シングル出力
  PSTRCONbits.STRA = 1;
  PSTRCONbits.STRB = 1;
  PSTRCONbits.STRC = 1;
  PSTRCONbits.STRD = 1;
  PSTRCONbits.STRSYNC = 1;


  // 8ビットのデーティー幅とする場合は PR2 が 0x3F となる
  // 16MHz の場合
  //   16MHz/4 = 4MHz
  //   4MHz / (0x3F+1) = 4000kHz/64 = 62.5kHz
  // 48HMz の場合
  //   48MHz/4 = 12MHz
  //   12MHz / (0x3F+1) = 12000kHz/64 = 187.5kHz
  //CCPR1L  = 0x3F;              // デューティ値
  CCPR1L  = 0x00;
  PR2     = 0x3F;            // PWM周期 187.5kHz @48MHz

  TMR2 = 0;
  T2CONbits.T2CKPS = 0b00;  // prescaler 1:1
  T2CONbits.T2OUTPS = 0;    // postscaler 1:1
  T2CONbits.TMR2ON = 1;     // Timer ON

  // queue
  queue_init(&queue, queue_buffer, sizeof(queue_buffer));
  queue_init(&cmd_queue, cmd_queue_buffer, sizeof(cmd_queue_buffer));
}

int is_ready_cmd(Queue *q)
{
  int qsize = queue_size(q);
  if (qsize < 2)
    return 0;
  return (qsize >= queue_peek(q, 1) + 2);
}


/*********************************************************************
* Function: void APP_DeviceCDCBasicDemoInitialize(void);
*
* Overview: Initializes the demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCDCBasicDemoInitialize()
{
    CDCInitEP();

    line_coding.bCharFormat = 0;
    line_coding.bDataBits = 8;
    line_coding.bParityType = 0;
    line_coding.dwDTERate = 9600;

    buttonPressed = false;
}

/*********************************************************************
* Function: void APP_DeviceCDCBasicDemoTasks(void);
*
* Overview: Keeps the demo running.
*
* PreCondition: The demo should have been initialized and started via
*   the APP_DeviceCDCBasicDemoInitialize() and APP_DeviceCDCBasicDemoStart() demos
*   respectively.
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceCDCBasicDemoTasks()
{
    //if (eat && (queue_size(&queue) > 0)) {
    //  unsigned char raw;
    //  eat = 0;
    //  if (queue_dequeue(&queue, &raw, 1) == 1) {
    //    CCPR1L = (raw >> 2) & 0x3F;
    //    CCP1CONbits.DC1B = (raw & 0x3);
    //  }
    //}
    /* If the user has pressed the button associated with this demo, then we
     * are going to send a "Button Pressed" message to the terminal.
     */
    if(BUTTON_IsPressed(BUTTON_DEVICE_CDC_BASIC_DEMO) == true)
    {
        /* Make sure that we only send the message once per button press and
         * not continuously as the button is held.
         */
        if(buttonPressed == false)
        {
            /* Make sure that the CDC driver is ready for a transmission.
             */
            if(mUSBUSARTIsTxTrfReady() == true)
            {
                putrsUSBUSART(buttonMessage);
                buttonPressed = true;
            }
        }
    }
    else
    {
        /* If the button is released, we can then allow a new message to be
         * sent the next time the button is pressed.
         */
        buttonPressed = false;
    }

    /* Check to see if there is a transmission in progress, if there isn't, then
     * we can see about performing an echo response to data received.
     */
    if( USBUSARTIsTxTrfReady() == true)
    {
      uint8_t numBytesRead;
      //numBytesRead = getsUSBUSART(readBuffer, sizeof(readBuffer));
      numBytesRead = getsUSBUSART(readBuffer, sizeof(cmd_queue_buffer) - queue_size(&cmd_queue));
      queue_enqueue(&cmd_queue, readBuffer, numBytesRead);
      if (is_ready_cmd(&cmd_queue)) {
        unsigned char cmd = queue_peek(&cmd_queue, 0);
        unsigned char size = queue_peek(&cmd_queue, 1);
        queue_dequeue(&cmd_queue, NULL, 2);
        switch(cmd) {
        case 1: // 演奏開始
          playing = 1;
          waiting_data = 0;
          break;
        case 3: // データ転送
          queue_enqueue_from_queue(&queue, &cmd_queue);
          if (size == 0)
            playing = 0;
          waiting_data = 0;
          break;
        }
        queue_dequeue(&cmd_queue, NULL, size);
      }

      if (playing & waiting_data == 0) {
        if (hangry) {
          hangry = 0;
          waiting_data = 1;
          writeBuffer[0] = 2;
          writeBuffer[1] = 1;
          writeBuffer[2] = sizeof(queue_buffer) - queue_size(&queue);
          putUSBUSART(writeBuffer, 3);
        } else if ((size_t)queue_size(&queue) <= (sizeof(queue_buffer)>>1)) {
          hangry = 1;
        }
      }
    }

    CDCTxService();
}