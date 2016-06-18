//#ifndef _GLOBALS_H_
//#define _GLOBALS_H_

#include "hal_types.h"
#include "constants.h"

extern char   __xdata uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
extern char   __xdata uartTxBuffer[SIZE_OF_UART_RX_BUFFER];
extern int    __xdata uartTxLength;
extern int    __xdata uartTxIndex;
extern int    __xdata uartRxIndex;
extern char   __xdata bleRxBuffer[SIZE_OF_UART_RX_BUFFER];
extern char   __xdata bleTxBuffer[SIZE_OF_UART_TX_BUFFER];
extern int    __xdata bleTxLength;
extern int    __xdata bleTxIndex;
extern int    __xdata bleRxIndex;
extern char   __xdata txFilterEnabled;
extern int    __xdata txCalcCRC;
extern int    __xdata txCalcCRC16;
extern char   __xdata txLength;
extern int    __xdata txTimes;
extern char   __xdata hm10Detected;
extern char   __xdata lastMinilinkID[3];
extern char   __xdata lastMinilinkSeqNum;
extern unsigned int __xdata lastTimeStamp;
extern char   __xdata rfFrequencyMode;
extern char   __xdata minilinkID1[3];
extern char   __xdata minilinkID2[3];
extern char   __xdata minilinkID3[3];
extern char   __xdata pumpID[3];
extern char   __xdata glucometerID1[3];
extern char   __xdata glucometerID2[3];
extern unsigned int __xdata timeCounter;
extern unsigned int __xdata timeCounterOn;
extern unsigned int __xdata timeCounterOff;
extern unsigned int __xdata rfOnTimer;
extern unsigned int __xdata txTimer;
extern unsigned int __xdata glucometerTimer;
extern unsigned int  __xdata bleCommsWatchdogTimer;
extern char         __xdata rfState[2];
extern char   __xdata mySentryFlag;
extern char   __xdata minilinkFlag;
extern char   __xdata bleSentFlag;
extern char   __xdata sendFlag;
extern unsigned long __xdata sgv, lastSgv;
extern unsigned long __xdata raw;
extern unsigned long __xdata rawSgv;
extern unsigned int  __xdata bgReading;
extern unsigned char __xdata adjValue;
extern unsigned char __xdata warmUp;
extern unsigned long __xdata lastraw[9];
extern float  __xdata isig;
extern float  __xdata calFactor;
extern char   __xdata lastCalMoment[2];
extern char   __xdata nextCalMoment[2];
extern unsigned int   __xdata timingTable [3][8][2];
extern unsigned char  __xdata missesTable     [3][8];
extern          int   __xdata fiveMinAdjTable [3][8];
extern          char  __xdata timingTableCorrect [3];
extern unsigned int  __xdata historySgv         [16];
extern unsigned int  __xdata historyRawSgv      [16];
extern unsigned int  __xdata historyRaw         [16];
extern unsigned char __xdata historySgvValid    [16];
extern unsigned char __xdata historyRawSgvValid [16];
extern unsigned char __xdata historyRawValid    [16];


//#endif
