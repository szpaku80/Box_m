#include "ioCCxx10_bitdef.h"
#include "ioCC1110.h"
#include "bleComms.h"
#include "constants.h"
#include "globals.h"
#include "interrupts.h"
#include "usb_uart.h"
#include "configuration.h"
#include "timingController.h"
#include "init.h"

char initHM10 (void)
{
  unsigned int idx,i,j;
  unsigned int timeout;
  
  idx = 0;
  bleTxBuffer[idx++] = 'A';
  bleTxBuffer[idx++] = 'T';
  bleTxBuffer[idx++] = '+';
  bleTxBuffer[idx++] = 'N';
  bleTxBuffer[idx++] = 'A';
  bleTxBuffer[idx++] = 'M';
  bleTxBuffer[idx++] = 'E';
  bleTxBuffer[idx++] = 'm';
  bleTxBuffer[idx++] = 'D';
  bleTxBuffer[idx++] = 'r';
  bleTxBuffer[idx++] = 'i';
  bleTxBuffer[idx++] = 'p';
  
  bleRxIndex = 0;  
  timeout = 3;
  while ((bleRxIndex == 0) &&
          (timeout != 0)) {
    bleTxLength = 2;
    uart0StartTxForIsr();
    for (j=0; j<5; j++) for (i=0; i<32567; i++);
    timeout--;
  }
  if (timeout == 0) return (0);
  
  bleRxIndex = 0;  
  timeout = 3;
  while ((bleRxIndex == 0) &&
         (timeout != 0)) {
    bleTxLength = idx;
    uart0StartTxForIsr();
    for (j=0; j<5; j++) for (i=0; i<32567; i++);
    timeout--;
  }
  if (timeout == 0) return (0);
   
  bleRxIndex = 0;
  return(1);
}

void receiveBLEMessage (char *message, unsigned int messageLen)
{
  unsigned int temp;
  unsigned char i,j,k;

  if (hm10Detected) {
    switch(message[0]) {
    case 0x00:
      i=0;
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = message[0];
      bleTxBuffer[i++] = 'm';
      bleTxBuffer[i++] = 'D';
      bleTxBuffer[i++] = 'r';
      bleTxBuffer[i++] = 'i';
      bleTxBuffer[i++] = 'p';
      bleTxBuffer[i++] = ' ';
      bleTxBuffer[i++] = 'v';
      bleTxBuffer[i++] = '1';
      bleTxBuffer[i++] = '.';
      bleTxBuffer[i++] = '0';
      bleTxBuffer[i++] = '1';
      
      bleTxBuffer[0] = i-1;
      bleTxLength = i;
      uart0StartTxForIsr();
      break;
  
    case 0x10:
    case 0x11:
    case 0x12:
      i = 0;
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = message[0];
      for (j=0;j<8;j++) {
        for (k=0;k<2;k++) {
            bleTxBuffer[i++] = (timingTable[message[0]&0x03][j][k] >> 8) & 0x0FF;
            bleTxBuffer[i++] = (timingTable[message[0]&0x03][j][k]     ) & 0x0FF;
        }
      }
      bleTxBuffer[0] =i-1;
      bleTxLength = i;
      uart0StartTxForIsr();
      break;
    
    case 0x20:
    case 0x21:
    case 0x22:
      if (messageLen == 33) {
        i = 1;
        for (j=0;j<8;j++) {
          for (k=0;k<2;k++) {
              timingTable[message[0]&0x03][j][k] = (message[i] << 8) | 
                                                  (message[i+1]) ;
              i += 2;
          }
        }      
      }
      break;
    
    case 0x13:
      i = 0;
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = message[0];
      bleTxBuffer[i++] = (sgv >> 8) & 0x0FF;
      bleTxBuffer[i++] = (sgv     ) & 0x0FF;
      bleTxBuffer[i++] = (raw >> 8) & 0x0FF;
      bleTxBuffer[i++] = (raw     ) & 0x0FF;
      bleTxBuffer[i++] = (timeCounter >> 8) & 0x0FF;
      bleTxBuffer[i++] = (timeCounter     ) & 0x0FF;
      bleTxBuffer[i++] = (rfOnTimer   >> 8) & 0x0FF;
      bleTxBuffer[i++] = (rfOnTimer       ) & 0x0FF;
      bleTxBuffer[i++] = (fiveMinuteCounter >> 8) & 0x0FF;
      bleTxBuffer[i++] = (fiveMinuteCounter     ) & 0x0FF;
      bleTxBuffer[i++] = (txTimer     >> 8) & 0x0FF;
      bleTxBuffer[i++] = (txTimer         ) & 0x0FF;
      temp = (unsigned int)(calFactor * 256);
      bleTxBuffer[i++] = (temp >> 8) & 0x0FF;
      bleTxBuffer[i++] = (temp     ) & 0x0FF;
      bleTxBuffer[i++] = lastMinilinkSeqNum;
      bleTxBuffer[i++] = (mySentryFlag << 3) |
                        (minilinkFlag << 2) |
                        (bleSentFlag  << 1) |
                        (sendFlag) ;
      bleTxBuffer[0] = i-1;
      bleTxLength = i;
      uart0StartTxForIsr();
      break;
    
    case 0x14:
      i = 0;
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = message[0];
      bleTxBuffer[i++] = rfFrequencyMode;
      bleTxBuffer[i++] = minilinkID1[0];
      bleTxBuffer[i++] = minilinkID1[1];
      bleTxBuffer[i++] = minilinkID1[2];
      bleTxBuffer[i++] = minilinkID2[0];
      bleTxBuffer[i++] = minilinkID2[1];
      bleTxBuffer[i++] = minilinkID2[2];
      bleTxBuffer[i++] = minilinkID3[0];
      bleTxBuffer[i++] = minilinkID3[1];
      bleTxBuffer[i++] = minilinkID3[2];
      bleTxBuffer[i++] = pumpID[0];
      bleTxBuffer[i++] = pumpID[1];
      bleTxBuffer[i++] = pumpID[2];
      bleTxBuffer[i++] = glucometerID1[0];
      bleTxBuffer[i++] = glucometerID1[1];
      bleTxBuffer[i++] = glucometerID1[2];
      bleTxBuffer[i++] = glucometerID2[0];
      bleTxBuffer[i++] = glucometerID2[1];
      bleTxBuffer[i++] = glucometerID2[2];
      bleTxBuffer[0] = i-1;
      bleTxLength = i;
      uart0StartTxForIsr();
      break;
    case 0x24:
      if (messageLen == 20) {
        i = 1;
        if (message[1] != rfFrequencyMode) configureMedtronicRFMode();
        rfFrequencyMode = message[i++];
        minilinkID1[0] = message[i++];
        minilinkID1[1] = message[i++];
        minilinkID1[2] = message[i++];
        minilinkID2[0] = message[i++];
        minilinkID2[1] = message[i++];
        minilinkID2[2] = message[i++];
        minilinkID3[0] = message[i++];
        minilinkID3[1] = message[i++];
        minilinkID3[2] = message[i++];
        pumpID[0] = message[i++];
        pumpID[1] = message[i++];
        pumpID[2] = message[i++];
        glucometerID1[0] = message[i++];
        glucometerID1[1] = message[i++];
        glucometerID1[2] = message[i++];
        glucometerID2[0] = message[i++];
        glucometerID2[1] = message[i++];
        glucometerID2[2] = message[i++];    
      }
      break;
 
    case 0x15:
      i = 0;
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = message[0];
      bleTxBuffer[i++] = lastMinilinkSeqNum;
      for (j=0;j<16;j++) {
        bleTxBuffer[i++] = ((historySgv[j] >> 8) & 0x01) |
                            ((historySgvValid[j] & 0x01) << 7) ;
        bleTxBuffer[i++] = (historySgv[j] & 0x0FF);
      }
      bleTxBuffer[0] = i-1;
      bleTxLength = i;
      uart0StartTxForIsr();
      break;
      
    case 0x16:
      i = 0;
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = message[0];
      bleTxBuffer[i++] = lastMinilinkSeqNum;
      for (j=0;j<16;j++) {
        bleTxBuffer[i++] = ((historyRawSgv[j] >> 8) & 0x001) |
                            ((historyRawSgvValid[j] & 0x01) << 7) ;
        bleTxBuffer[i++] = (historyRawSgv[j] & 0x0FF);
      }
      bleTxBuffer[0] = i-1;
      bleTxLength = i;
      uart0StartTxForIsr();
      break;
      
    case 0x17:
      i = 0;
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = message[0];
      bleTxBuffer[i++] = lastMinilinkSeqNum;
      for (j=0;j<16;j++) {
        bleTxBuffer[i++] = ((historyRaw[j] >> 8) & 0x07F) |
                            ((historyRawValid[j] & 0x01) << 7) ;
        bleTxBuffer[i++] = (historyRaw[j] & 0x0FF);
      }
      bleTxBuffer[0] = i-1;
      bleTxLength = i;
      uart0StartTxForIsr();
      break;
    
    case 0x18:
    case 0x19:
    case 0x1A:
      i = 0;
      bleTxBuffer[i++] = 0x00;
      bleTxBuffer[i++] = message[0];
      bleTxBuffer[i++] = timingTableCorrect[(message[0]&0x0F)-8];
      for (j=0;j<8;j++) {
        bleTxBuffer[i++] = (fiveMinAdjTable[(message[0]&0x0F)-8][j] >> 8) & 0x0FF;
        bleTxBuffer[i++] = (fiveMinAdjTable[(message[0]&0x0F)-8][j]     ) & 0x0FF;
      }
      bleTxBuffer[0] = i-1;
      bleTxLength = i;
      uart0StartTxForIsr();
      break;

    case 0x2B:
      if (messageLen == 2) {
        rfOnTimer = (0*60 + (message[1] & 0x0FF))*10;
      }
      break;
      
    default:
      break;
    }
  }
}
