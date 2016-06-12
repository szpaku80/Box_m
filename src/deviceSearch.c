#include "constants.h"
#include "globals.h"
#include "crc_4b6b.h"
#include "medtronicRF.h"

void respondToDeviceSearch (char *message, unsigned int length, char dataErr)
{

  // First check if the message has a correct CRC
  if (dataErr == 0) { // && txFilterEnabled == 1) {
    // MySentry Message
    if ( (message[0] == 0xA2) &&
         (message[1] == pumpID[0]) &&
         (message[2] == pumpID[1]) &&
         (message[3] == pumpID[2]) ) {
           switch(message[4]) {
              
           case 0x09:
           case 0x0A:
           case 0x08:
           case 0x04:
           case 0x0B:
             uartTxBuffer[0]  = 0xA2;
             uartTxBuffer[1]  = message[1];
             uartTxBuffer[2]  = message[2];
             uartTxBuffer[3]  = message[3];
             uartTxBuffer[4]  = 0x06;
             uartTxBuffer[5]  = message[5] & 0x7F;
             uartTxBuffer[6]  = message[1];
             uartTxBuffer[7]  = message[2];
             uartTxBuffer[8]  = message[3];
             uartTxBuffer[9]  = 0x00;
             uartTxBuffer[10] = message[4];
             switch (message[4]){
               case 0x09: uartTxBuffer[11] = 0x0A; break; 
               case 0x0A: uartTxBuffer[11] = 0x08; break;
               case 0x08: uartTxBuffer[11] = 0x0B; break;
               case 0x0B: uartTxBuffer[11] = 0x04; break;
               default:   uartTxBuffer[11] = 0x00; break; //txFilterEnabled = 0; break;
             }
             uartTxBuffer[12] = 0x00;
             uartTxBuffer[13] = 0x00;
             uartTxBuffer[14] = crc8(uartTxBuffer,14);
             uartTxLength = 15;
             sendMedtronicMessage(uartTxBuffer,uartTxLength,1);
             
           default:
             break;
           }
      
    }
  }
}