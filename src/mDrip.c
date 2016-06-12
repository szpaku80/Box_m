
/***********************************************************************************
  Filename:     MMCommander.c

  Description:  Medtronic-enabled USB interface

  Version:      1
  Revision:     86
  Date:         Dec 24th, 2014

***********************************************************************************/

/***********************************************************************************
* INCLUDE
*/
#include "crc_4b6b.h"
#include "init.h"
#include "constants.h"
#include "interrupts.h"
#include "medtronicRF.h"
#include "smartRecovery.h"
#include "hal_board.h"
#include "hal_uart.h"
#include "usb_uart.h"
#include "bleComms.h"
#include "configuration.h"
#include "deviceSearch.h"
#include "timingController.h"
#include "dataProcessing.h"

/******************************************************************************
* GLOBAL VARIABLES
*/

char   __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
char   __xdata uartTxBuffer[SIZE_OF_UART_TX_BUFFER];
int    __xdata uartTxLength;
int    __xdata uartTxIndex;
int    __xdata uartRxIndex;

char   __xdata bleRxBuffer[SIZE_OF_UART_RX_BUFFER];
char   __xdata bleTxBuffer[SIZE_OF_UART_TX_BUFFER];
int    __xdata bleTxLength;
int    __xdata bleTxIndex;
int    __xdata bleRxIndex;

char   __xdata hm10Detected;
char   __xdata lastMinilinkID[3]= {0x00, 0x00, 0x00};
char   __xdata lastMinilinkSeqNum = 0x80;
unsigned int __xdata lastTimeStamp;
char   __xdata rfFrequencyMode;

unsigned long __xdata sgv, lastSgv;
unsigned long __xdata raw;//, lastRaw;
unsigned long __xdata lastraw[9];
unsigned long __xdata rawSgv;
unsigned int  __xdata bgReading;
unsigned char __xdata adjValue;
float  __xdata isig;
float  __xdata calFactor;
char   __xdata lastCalMoment[2];
char   __xdata nextCalMoment[2];
char   __xdata mySentryFlag;
char   __xdata minilinkFlag;
char   __xdata bleSentFlag;
char   __xdata sendFlag;

unsigned int   __xdata timingTable     [3][8][2];
unsigned char  __xdata missesTable     [3][8];
         int   __xdata fiveMinAdjTable [3][8];
         char  __xdata timingTableCorrect [3];
         
unsigned int __xdata timeCounter;
unsigned int __xdata timeCounterOn;
unsigned int __xdata timeCounterOff;
unsigned int __xdata fiveMinuteCounter;
unsigned int __xdata rfOnTimer;
unsigned int __xdata txTimer;
unsigned int __xdata glucometerTimer;

unsigned int  __xdata historySgv         [16];
unsigned int  __xdata historyRawSgv      [16];
unsigned int  __xdata historyRaw         [16];
unsigned char __xdata historySgvValid    [16];
unsigned char __xdata historyRawSgvValid [16];
unsigned char __xdata historyRawValid    [16];

/* Empty configuration */
char   __xdata minilinkID1[3]   = {0x00, 0x00, 0x00};
char   __xdata minilinkID2[3]   = {0x00, 0x00, 0x00};
char   __xdata minilinkID3[3]   = {0x00, 0x00, 0x00};
char   __xdata pumpID[3]        = {0x00, 0x00, 0x00};
char   __xdata glucometerID1[3] = {0x00, 0x00, 0x00};
char   __xdata glucometerID2[3] = {0x00, 0x00, 0x00};

/******************************************************************************
* MAIN FUNCTION
*/

 int main(void)
{
  char dataPacket[256];
  char repPacket[3];
  char dataErr;
  unsigned int dataLength;
  char i;
  char repeatedMessage;
  unsigned int j,k;

    /* Configure system */
    initGlobals();
    configureIO();
    configureOsc();
    crc16Init();
    configureMedtronicRFMode();
    enablePushButtonInt();
    hm10Detected = 0;
    if (_MDRIP_ENABLED_ == 1) {
      configureUART();
      uart0StartRxForIsr();
      for (j=0; j<15; j++) for (k=0; k<32567; k++);
      hm10Detected = initHM10();
    }
    
    // Flash coding for HM10 detection
    for (j=0; j<5; j++) for (k=0; k<32567; k++);
    P1_1 = 1;
    for (j=0; j<5; j++) for (k=0; k<32567; k++);
    P1_1 = 0;
    for (j=0; j<5; j++) for (k=0; k<32567; k++);
    P1_1 = 1;
    for (j=0; j<5; j++) for (k=0; k<32567; k++);
    P1_1 = 0;
    if (hm10Detected == 0) {
      for (j=0; j<5; j++) for (k=0; k<32567; k++);
      P1_1 = 1;
      for (j=0; j<5; j++) for (k=0; k<32567; k++);
      P1_1 = 0;
    }
    
    if (hm10Detected == 0) {
      halUartInit(HAL_UART_BAUDRATE_57600, 0);
    } else {
      initTimingTable();
      enableTimerInt();
    }
    
    /* Reception loop */
    while (1) {
      dataErr = receiveMedtronicMessage(dataPacket, &dataLength);

      if (_SMART_RECOVERY_ == 1) {
	smartRecovery(dataPacket,&dataLength,&dataErr);
      }

      if (dataLength > 0) {
        repeatedMessage = 0;
        if ( (_REPEATED_COMMAND_ENABLED_ == 1) && 
            (dataErr == ((uartTxBuffer[0]>>7) & 0x01)) && 
            (dataLength == (uartTxLength-2))
           ) {
          repeatedMessage = 1;
          for (i=0; i<dataLength; i++) {
            if (dataPacket[i] != uartTxBuffer[i+2]) {
              repeatedMessage = 0;
              break;
            }
          }
        }

	if (hm10Detected == 1) {
          respondToDeviceSearch(dataPacket,dataLength,dataErr);
	  processMessage(dataPacket,dataLength,dataErr);
        } else {
          if (repeatedMessage == 1) {
            repPacket[0] = 0x04;
            halUartWrite((uint8 const *)repPacket,1);
            usbUartProcess();
            usbReceiveData();
          } else {
            if (dataErr == 0) {
              uartTxBuffer[0] = 0x02;
            } else {
              uartTxBuffer[0] = 0x82;
            }
            uartTxBuffer[1] = dataLength;
            for (i=0; i<dataLength; i++) uartTxBuffer[i+2] = dataPacket[i];
            uartTxLength = dataLength+2;
            for (i=0; i<uartTxLength; i=i+48) {
              if (uartTxLength-i > 48) {
                halUartWrite((uint8 const *)&uartTxBuffer[i],48);
                usbUartProcess();
                usbReceiveData();
              } else {
                halUartWrite((uint8 const *)&uartTxBuffer[i],uartTxLength-i);
              }
            }
          }
        }
        
      }
    }    
}

