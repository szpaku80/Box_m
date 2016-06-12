#include "globals.h"
#include "ioCCxx10_bitdef.h"
#include "ioCC1110.h"
#include "crc_4b6b.h"
#include "interrupts.h"
#include "medtronicRF.h"
#include "usb_uart.h"
#include "hal_uart.h"
#include "txFilter.h"
#include "bleComms.h"
#include "timingController.h"

#pragma vector = P1INT_VECTOR
__interrupt void PORT1_ISR(void)
{
  int i,j;
  int modeChange;
  char txString[2];
  
  // Clear Port 1 Interrupt Flag
  P1IFG = 0;
  IRCON2 &= 0xF7;
   
  modeChange = 1;
  for (j=0; j<0x20; j++) {
    for (i=0; i<0xFFFF; i++) asm("nop");
    if (P1_2 == 1) {
      modeChange = 0;
      j=0x20;
    }
  }
  
  if (modeChange == 1) {
    if (txFilterEnabled == 1) {
      txFilterEnabled = 0;
      P1_1 = 1;
      txString[0] = 0x13;
    } else {
      txFilterEnabled = 1;
      P1_1 = 0;
      txString[0] = 0x03;
    }
    if (hm10Detected == 0) 
      halUartWrite((uint8 const *)txString,1);
  }

  // Clear Port 1 Interrupt Flag
  P1IFG = 0;  
  IRCON2 &= ~0x04;
}

void enablePushButtonInt (void)
{
  // Initialize Filter to enabled.
  P1_0 = 0;
  txFilterEnabled = 1;
  
  // Clear any pending Port 1 Interrupt Flag (IRCON2.P1IF = 0)
  P1IFG = 0;
  IRCON2 &= 0xF7;
  
  // Enable interrupt on falling edges
  PICTL |= 0x02;
  
  // Enable global interrupt (IEN0.EA = 1) and Port 1 Interrupt (IEN2.P1IE = 1)
  P1IEN = 0x04;
  EA = 1; IEN2 |= IEN2_P1IE;  
}

#pragma vector = T1_VECTOR
__interrupt void TIMER1_ISR(void)
{
  static char rfState[2];
  unsigned int glucometerCheckSum, i;
  char periodicRx;
  
  // Clear Timer 1 Interrupt Flag
  P1IFG = 0;  
  IRCON &= ~0x02;
      
  if (hm10Detected == 1) {
    // Increase time counter
    if (timeCounter < ((20*60 + 0) * 10)) timeCounter++;
    if (rfOnTimer > 0) rfOnTimer--;
    if (glucometerTimer > 0) glucometerTimer--;
    if (timeCounterOff <= timeCounter) recalculateTiming();
    
    // Decrease 5 minute counter and manage txTimer
    /*if (fiveMinuteCounter == 0) {
      fiveMinuteCounter = ((5*60 + 0) * 10);
      txTimer = 0;
    } else {
      fiveMinuteCounter--;
      if (txTimer < ((1*60 + 0) * 10)) txTimer++;
    }
    if (txTimer == ((0*60 + 2) * 10)) sendFlag = 1;*/
     
    // Check if RF should be on or off
    glucometerCheckSum = 0;
    for (i=0; i<3; i++) glucometerCheckSum += glucometerID1[i];
    for (i=0; i<3; i++) glucometerCheckSum += glucometerID2[i];
    if (glucometerCheckSum > 0) {
      if ((timeCounter % 10) == 0) periodicRx = 1;
      else periodicRx = 0;
    } else periodicRx = 0;
    rfState[1] = rfState[0];
    if (rfOnTimer > 0) {
      rfState[0] = 1;
      P1_1 = 1;
    } else if (periodicRx == 1) {
      rfState[0] = 1;
    } else if (timeCounter >= (20*60 + 0)*10) {
      rfState[0] = 1;
      P1_1 = 1;
    } else if ((timeCounter >= timeCounterOn) && 
               (timeCounter <= timeCounterOff)) {
      if ((minilinkFlag == 1) && (mySentryFlag == 1) &&
          (getTimeForNumSeq ( lastMinilinkSeqNum | 0x01 ) > 0)) {
        rfState[0] = 0;
        P1_1 = 0;
      } else {      
        rfState[0] = 1;
        P1_1 = 1;
      }
    } else {
      rfState[0] = 0;
      P1_1 = 0;
    }
    
    if ((rfState[0] == 1) && (rfState[1] == 0)) {
      RFST = RFST_SRX;
    } else if ((rfState[0] == 0) && (rfState[1] == 1)) {
      RFST = RFST_SIDLE;
      if ((minilinkFlag == 1) || (mySentryFlag == 1)) {
        sendFlag = 1;
      }
    }
    
  } else {
    // Increase time counter
    timeCounter++;
  
    if (timeCounter >= 0x0005D00) {
      // Re-calibrate;
      RFST = RFST_SIDLE;
      RFST = RFST_SRX;
      timeCounter = 0;
    }
  }  
}

void stopTimerInt (void)
{
  // Stop Timer 1
  T1CTL &= 0xFC;
  // Reset Timer 1 Counter
  T1CNTL = 0x00;
  // Disable Timer 1 interrupt
  IEN1 &= ~0x02;
}

void resetTimerCounter (void)
{
  timeCounter = 0;
  T1CNTL = 0x00;
}

void enableTimerInt (void)
{
  if (hm10Detected == 0) {
    // Stop Timer 1
    T1CTL = 0x0C;

    // Set Timer 1 timeout value
    T1CC0H = 0xFF;
    T1CC0L = 0xFF;
  
    // Reset Timer 1 Counter
    T1CNTL = 0x00;
    timeCounter = 0;
 
    // Set Timer 1 mode 
    T1CCTL0 = 0x44; 
 
    // Clear any pending Timer 1 Interrupt Flag
    IRCON &= ~0x02;
  
    // Enable global interrupt (IEN0.EA = 1) and Timer 1 Interrupt (IEN1.1 = 1)
    EA = 1; IEN1 |= 0x02;  
  
    // Start Timer 1
    T1CTL = 0x0E;
  } else {
    // Stop Timer 1
    T1CTL = 0x08;

    // Set Timer 1 timeout value = Every 100 ms
    T1CC0H = 0x02;
    T1CC0L = 0x49; // 0x4A
    
    // Reset Timer 1 Counter
    T1CNTL = 0x00;
    //timeCounter = 0;
 
    // Set Timer 1 mode 
    T1CCTL0 = 0x44; 
 
    // Clear any pending Timer 1 Interrupt Flag
    IRCON &= ~0x02;
  
    // Enable global interrupt (IEN0.EA = 1) and Timer 1 Interrupt (IEN1.1 = 1)
    EA = 1; IEN1 |= 0x02;  
  
    // Start Timer 1
    T1CTL = 0x0E;
  }
}

#pragma vector = URX0_VECTOR
__interrupt void UART0_RX_ISR(void)
{ 
  char notFinishedYet, i, bufDif;
  
  // Clear UART0 RX Interrupt Flag (TCON.URX0IF = 0)
  URX0IF = 0;
  
  // Read UART0 RX buffer
  bleRxBuffer[bleRxIndex] = U0DBUF;
 
  //receiveBLEMessage (bleRxBuffer, 1);
  
  if (bleRxIndex < SIZE_OF_UART_RX_BUFFER-1) {
    bleRxIndex++;
  }

  notFinishedYet = 1;
  while (notFinishedYet) {
    if ((bleRxIndex > 0) && (bleRxIndex > (bleRxBuffer[0] & 0x00FF))) {
      receiveBLEMessage (&bleRxBuffer[1], ((unsigned int)bleRxBuffer[0] & 0x00FF));
      bufDif = bleRxIndex-(bleRxBuffer[0]+1);
      for (i=0;i<bufDif;i++) {
        bleRxBuffer[i] = bleRxBuffer[bleRxBuffer[0]+1+i];
      }
      bleRxIndex -= bleRxBuffer[0]+1;
    }
    if ((bleRxIndex == 0) || (bleRxIndex <= (bleRxBuffer[0] & 0x00FF))) {
      notFinishedYet = 0;
    }
  }
  
}

#pragma vector = UTX0_VECTOR
__interrupt void UART0_TX_ISR(void)
{
  // Clear UART0 TX Interrupt Flag (IRCON2.UTX0IF = 0)
  UTX0IF = 0;

  // If no UART byte left to transmit, stop this UART TX session
  if (bleTxIndex >= bleTxLength)
  {
    // Note:
    // In order to start another UART TX session the application just needs
    // to prepare the source buffer, and simply send the very first byte.
    bleTxIndex = 0; 
    IEN2 &= ~IEN2_UTX0IE; return;
  } else {
    // Send next UART byte
    U0DBUF = bleTxBuffer[bleTxIndex++];
  }
}

void uart0StartRxForIsr(void)
{

  // Initialize the UART RX buffer index
  bleRxIndex = 0;

  // Clear any pending UART RX Interrupt Flag (TCON.URXxIF = 0, UxCSR.RX_BYTE = 0)
  URX0IF = 0; U0CSR &= ~U0CSR_RX_BYTE;

  // Enable UART RX (UxCSR.RE = 1)
  U0CSR |= U0CSR_RE;

  // Enable global Interrupt (IEN0.EA = 1) and UART RX Interrupt (IEN0.URXxIE = 1)
  EA = 1; URX0IE = 1;
}

void uart0StartTxForIsr(void)
{

  // Initialize the UART TX buffer indexes.
  bleTxIndex = 0;

  // Clear any pending UART TX Interrupt Flag (IRCON2.UTXxIF = 0, UxCSR.TX_BYTE = 0)
  UTX0IF = 0; U0CSR &= ~U0CSR_TX_BYTE;

  // Send very first UART byte
  U0DBUF = bleTxBuffer[bleTxIndex++];

  // Enable global interrupt (IEN0.EA = 1) and UART TX Interrupt (IEN2.UTXxIE = 1)
  EA = 1; IEN2 |= IEN2_UTX0IE;
}