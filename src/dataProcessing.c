#include "ioCCxx10_bitdef.h"
#include "ioCC1110.h"
#include "bleComms.h"
#include "constants.h"
#include "globals.h"
#include "interrupts.h"
#include "usb_uart.h"
#include "configuration.h"
#include "timingController.h"
#include "dataProcessing.h"

void processMessage (char *message, unsigned int length, char dataErr)
{
  static unsigned char lastSeqNum;
  unsigned long localSgv;
  
  // First check if the message has a correct CRC
  if (dataErr == 0) {
    
    // Second check if it's a minilink message
    if (((message[0] == 0xAA) || (message[0] == 0xAB)) &&
         (length == 34)) {
      // Third, check if it came from our transmitter
      if ( ((message[2] == minilinkID1[0]) &&
            (message[3] == minilinkID1[1]) &&
            (message[4] == minilinkID1[2])) ||
           ((message[2] == minilinkID2[0]) &&
            (message[3] == minilinkID2[1]) &&
            (message[4] == minilinkID2[2])) ||
           ((message[2] == minilinkID3[0]) &&
            (message[3] == minilinkID3[1]) &&
            (message[4] == minilinkID3[2])) ) {
        updateTimingTable(message[8],lastMinilinkSeqNum,lastTimeStamp);
        if ((lastMinilinkSeqNum & 0xF0) != (message[8] & 0xF0)) {
          lastMinilinkSeqNum = message[8];
          lastMinilinkID[0]  = message[2];
          lastMinilinkID[1]  = message[3];
          lastMinilinkID[2]  = message[4];
          raw        = (message[ 9] & 0x0FF)*256  + (message[10] & 0x0FF);
          lastraw[0] = (message[11] & 0x0FF)*256  + (message[12] & 0x0FF);
          adjValue   = message[7];
          isig = getISIGfromRAW (raw, adjValue);
          
          minilinkFlag = 1;
          if (message[8] & 0x01) {
            timeCounter = getTimeForNumSeq ( message[8] );
            if (timeCounter == 0) timeCounter = (0*60 + 5) * 10;
            rfOnTimer = (0*60 + 3) * 10;
          } else {
            timeCounter = 0;
            rfOnTimer  = getTimeForNumSeq ( message[8] | 0x01 );
            if (rfOnTimer == 0) rfOnTimer = (0*60 + 20) * 10;
            rfOnTimer += (0*60 + 3) * 10;
          }
          timeCounterOff = 0;
          if (fiveMinuteCounter >= ((2*60 + 30) * 10)) 
            fiveMinuteCounter = ((5*60 + 0) * 10) ;
        }
        lastMinilinkSeqNum = message[8];
      }
    }
    
    // MySentry Message
    else if (message[0] == 0xA2) {
      if ( (message[1] == pumpID[0]) &&
           (message[2] == pumpID[1]) &&
           (message[3] == pumpID[2]) &&
           (message[4] == 0x04)      &&
           (message[5] != lastSeqNum &&
           mySentryFlag == 0) ) {
        lastSeqNum = message[5];
        
        localSgv  = message[14] << 1;
        localSgv += message[29]&0x01;
        
        nextCalMoment[0] = message[25];
        nextCalMoment[1] = message[26];
        
        if (localSgv >= 30) {
          sgv      = localSgv;
          lastSgv  =  message[15]<<1;
          lastSgv += (message[29]>>1)&0x01;
          mySentryFlag = 1;
        }
      }
    }
    
    // Glucometer Reading
    if ((message[0] == 0xA5) && (length == 7)) {
      if ((((message[1] == glucometerID1[0]) &&
            (message[2] == glucometerID1[1]) &&
            (message[3] == glucometerID1[2])) ||
           ((message[1] == glucometerID2[0]) &&
            (message[2] == glucometerID2[1]) &&
            (message[3] == glucometerID2[2])))  &&
           (glucometerTimer == 0) ) {
        glucometerTimer = ((0*60 + 20) * 10);
        bgReading = (message[4] & 0x01)*256 + message[5];
        composeGlucometerMessage();
      }       
    }
    
    if ((minilinkFlag == 1) && (mySentryFlag == 1)) {
      rfOnTimer = 0;
    }
  }
}

void updateCalFactor (void)
{
  float currentFactor;
  static unsigned int calUpdateTimer;
  static char calibrated;
  static int lastSgv;
  int sgvDelta;
  
  if ((minilinkFlag == 1) && (mySentryFlag == 1) &&
      (sgv >= 30)) {
    
    currentFactor = (float)(sgv) / isig;
    // This should be done with a 15 min counter
    if ((nextCalMoment[0] != lastCalMoment[0]) ||
        (nextCalMoment[1] != lastCalMoment[1])) {
      lastCalMoment[0] = nextCalMoment[0];
      lastCalMoment[1] = nextCalMoment[1];
      calUpdateTimer = 3;
      calibrated = 0;
    }
    
    if (calUpdateTimer > 0) {
      calUpdateTimer--;
    } else {
      sgvDelta = sgv - lastSgv;
      if (calibrated == 0) {
        calFactor = currentFactor;
        if ((sgvDelta > -10) && (sgvDelta < 10)) {
          calibrated = 1;
        }
      }
    }
    
    lastSgv = sgv;
  }
}

float getISIGfromRAW (long raw, char adjValue)
{
  float isig;
  
  isig = (float)raw / 160.72 - ((float)raw * 0.00058); 
  isig += ((float)(adjValue & 0x0FF) * (float) raw * (6.25/1000000.0 + 
                      (1.5/1000000.0 * (float) raw / 65536.0))); 
          
  return(isig);
}

void processInfo (void)
{
  updateCalFactor();
  updateHistoryData();
  composeInfoUpdateMessage();
  
  mySentryFlag = 0;
  minilinkFlag = 0;
}

void updateHistoryData (void)
{
  char i;
  
  // Add most recent data
  for (i=15; i>0; i--) {
    historySgv      [i] = historySgv      [i-1];
    historySgvValid [i] = historySgvValid [i-1];
  }
  if (mySentryFlag == 1) {
    historySgv      [0] = sgv;
    historySgvValid [0] = 1;
    if (historySgvValid [1] == 0) {
      historySgv      [1] = lastSgv;
      historySgvValid [1] = 1;
    }
  } else {
    historySgv      [0] = 0;
    historySgvValid [0] = 0;
  }
  
  for (i=15; i>0; i--) {
    historyRawSgv      [i] = historyRawSgv      [i-1];
    historyRawSgvValid [i] = historyRawSgvValid [i-1];
  }
  if (minilinkFlag == 1) {
    rawSgv = (unsigned long) (isig * calFactor);
    historyRawSgv      [0] = rawSgv;
    historyRaw         [0] = raw;
    historyRawSgvValid [0] = (calFactor > 0.0) ? 1 : 0;
    historyRawValid    [0] = 1;
    if (historyRawValid [1] == 0) {
      rawSgv = (unsigned long) (getISIGfromRAW (lastraw[0], adjValue) * calFactor);
      historyRawSgv      [1] = rawSgv;
      historyRawSgvValid [1] = (calFactor > 0.0) ? 1 : 0;
      historyRaw         [1] = lastraw[0];
      historyRawValid    [1] = 1;
      rawSgv = (unsigned long) (isig * calFactor);
    }
  } else {
    historyRawSgv      [0] = 0;
    historyRaw         [0] = 0;
    historyRawSgvValid [0] = 0;
    historyRawValid    [0] = 0;
  }
    
}

void composeInfoUpdateMessage (void)
{
  unsigned int i;
  unsigned int timeOffset;
  unsigned char id;
  
  i=0;
  
  if      ((lastMinilinkID[0] == minilinkID1[0]) &&
           (lastMinilinkID[1] == minilinkID1[1]) &&
           (lastMinilinkID[2] == minilinkID1[2])) id = 0;
  else if ((lastMinilinkID[0] == minilinkID2[0]) &&
           (lastMinilinkID[1] == minilinkID2[1]) &&
           (lastMinilinkID[2] == minilinkID2[2])) id = 1;
  else if ((lastMinilinkID[0] == minilinkID3[0]) &&
           (lastMinilinkID[1] == minilinkID3[1]) &&
           (lastMinilinkID[2] == minilinkID3[2])) id = 2;
  else id = 3;
  
  timeOffset = 0;
  if (id < 3) {
    if (fiveMinAdjTable[id][(lastMinilinkSeqNum & 0x0F0)>>4] > 0) {
      timeOffset = timeCounter + fiveMinAdjTable[id][(lastMinilinkSeqNum & 0x0F0)>>4];
    } 
  }
  
  if ((mySentryFlag == 1) || (minilinkFlag == 1)) {

    bleTxBuffer[i++] = 0x00; // Message length
    bleTxBuffer[i++] = 0x01; // Message type -> 0x01 = Info update
    bleTxBuffer[i++] = mySentryFlag*2 + minilinkFlag; // Flags
    bleTxBuffer[i++] = (timeOffset >> 8) & 0x00FF; // Time offset to be applied
    bleTxBuffer[i++] = (timeOffset     ) & 0x00FF; // Time offset to be applied
    if (mySentryFlag == 1) {
      bleTxBuffer[i++] = (sgv >> 8) & 0x00FF;
      bleTxBuffer[i++] =  sgv       & 0x00FF;
    } else {
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = 0x00;
    }
    
    if (minilinkFlag == 1) {
      bleTxBuffer[i++] = (rawSgv >> 8) & 0x00FF;
      bleTxBuffer[i++] =  rawSgv       & 0x00FF;
      bleTxBuffer[i++] = (raw    >> 8) & 0x00FF;
      bleTxBuffer[i++] =  raw          & 0x00FF;
    } else {
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = 0x00;
    }
  
    bleTxBuffer[i++] = 0xFF;
    
    bleTxBuffer[0] = i-1;
        
    bleTxLength = i;
  
    uart0StartTxForIsr();
  }
}

void composeGlucometerMessage (void)
{
  unsigned int i;
  
  i=0;
   
  bleTxBuffer[i++] = 0x00; // Message length
  bleTxBuffer[i++] = 0x02; // Message type -> 0x02 = Glucometer reading
  
  bleTxBuffer[i++] = (bgReading >> 8) & 0x00FF;
  bleTxBuffer[i++] =  bgReading       & 0x00FF;
  
  bleTxBuffer[0] = i-1;
  
  bleTxLength = i;
  
  uart0StartTxForIsr();
}
