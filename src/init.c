#include "hal_types.h"
#include "hal_defs.h"
#include "hal_cc8051.h"
#include "ioCCxx10_bitdef.h"
#include "ioCC1110.h"
#include "constants.h"
#include "globals.h"
#include "configuration.h"

/***********************************************************************************
* LOCAL FUNCTIONS
*/
void configureIO (void)
{
    /* Set function of pin to general purpose I/O */
    P1SEL &= BIT0;
    P1SEL &= BIT1;

    /* Write value to pin. Note that this is a direct bit access.
     * Also note that the value is set before the pin is configured
     * as an output, as in some cases the value of the output needs
     * to be defined before the output is driven (e.g. to avoid
     * conflicts if the signal is shared with an other device)
     */
    P1_0 = 0;
    P1_1 = 0;

    /* Change direction to output */
    P1DIR |= BIT0;
    P1DIR |= BIT1;
}

void configureOsc (void)
{
  SLEEP &= ~OSC_PD_BIT;     // powering down all oscillators
  while(!XOSC_STABLE);      // waiting until the oscillator is stable
  asm("NOP");
  CLKCON &= 0xC0;
  CLKCON &= ~MAIN_OSC_BITS; // starting the Crystal Oscillator
  CLKCON |= 0x28;
  SLEEP |= OSC_PD_BIT;      // powering down the unused oscillator
}

void configureMedtronicRFMode (void)
{
  char currentRFST;
  
  currentRFST = RFST;
  
  RFST = RFST_SIDLE;
  
  SYNC1 = 0xFF; SYNC0 = 0x00;
  PKTLEN = 0xFF;
  PKTCTRL1 = 0x00; PKTCTRL0 = 0x00;
  ADDR = 0x00;
  CHANNR = 0x00;
  FSCTRL1 = 0x06; FSCTRL0 = 0x00;
  if (rfFrequencyMode == 0 ) {
    FREQ2 = 0x24;  
    FREQ1 = 0x2E; 
    FREQ0 = 0x38; 
  } else {
    FREQ2 = 0x26;
    FREQ1 = 0x30;
    FREQ0 = 0x00;
  }
  MDMCFG4 = 0x59; 
  MDMCFG3 = 0x66; 
  MDMCFG2 = 0x33; 
  MDMCFG1 = 0x62; 
  MDMCFG0 = 0x1A; 
  DEVIATN = 0x13;
  MCSM2 = 0x07; MCSM1 = 0x30; MCSM0 = 0x18;
  FOCCFG = 0x17;
  BSCFG = 0x6C;
  AGCCTRL2 = 0x03; AGCCTRL1 = 0x40; AGCCTRL0 = 0x91;
  FREND1 = 0x56; FREND0 = 0x12;
  FSCAL3 = 0xE9; FSCAL2 = 0x2A; FSCAL1 = 0x00; FSCAL0 = 0x1F;
  TEST2 = 0x88; TEST1 = 0x31; TEST0 = 0x09;
  PA_TABLE7 = 0x00; PA_TABLE6 = 0x00; PA_TABLE5 = 0x00; PA_TABLE4 = 0x00;
  PA_TABLE3 = 0x00; PA_TABLE2 = 0x52; PA_TABLE1 = 0x00; PA_TABLE0 = 0x00;

  RFTXRXIE = 0;
  
  RFST = currentRFST;
}

void configureUART (void)
{

  /***************************************************************************
   * Setup I/O ports
   *
   * Port and pins used by USART0 operating in UART-mode are
   * RX     : P0_2
   * TX     : P0_3
   * CT/CTS : P0_4
   * RT/RTS : P0_5
   *
   * These pins can be set to function as peripheral I/O to be be used by UART0.
   * The TX pin on the transmitter must be connected to the RX pin on the receiver.
   * If enabling hardware flow control (U0UCR.FLOW = 1) the CT/CTS (Clear-To-Send)
   * on the transmitter must be connected to the RS/RTS (Ready-To-Send) pin on the
   * receiver.
   */

  // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
  // To avoid potential I/O conflict with USART1:
  // configure USART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1)
  PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U1CFG;

  // Configure relevant Port P0 pins for peripheral function:
  // P0SEL.SELP0_2/3/4/5 = 1 => RX = P0_2, TX = P0_3, CT = P0_4, RT = P0_5
  P0SEL |= BIT5 | BIT4 | BIT3 | BIT2;

  /***************************************************************************
   * Configure UART
   *
   * The system clock source used is the HS XOSC at 48 MHz speed.
   */

  // Initialise bitrate = 9600 bbps (U0BAUD.BAUD_M = 163, U0GCR.BAUD_E = 7)
  U0BAUD = UART_BAUD_M;
  U0GCR = (U0GCR&~U0GCR_BAUD_E) | UART_BAUD_E;

  // Initialise UART protocol (start/stop bit, data bits, parity, etc.):

  // USART mode = UART (U0CSR.MODE = 1)
  U0CSR |= U0CSR_MODE;

  // Start bit level = low => Idle level = high  (U0UCR.START = 0)
  U0UCR &= ~U0UCR_START;

  // Stop bit level = high (U0UCR.STOP = 1)
  U0UCR |= U0UCR_STOP;

  // Number of stop bits = 1 (U0UCR.SPB = 0)
  U0UCR &= ~U0UCR_SPB;

  // Parity = disabled (U0UCR.PARITY = 0)
  U0UCR &= ~U0UCR_PARITY;

  // 9-bit data enable = 8 bits transfer (U0UCR.BIT9 = 0)
  U0UCR &= ~U0UCR_BIT9;

  // Level of bit 9 = 0 (U0UCR.D9 = 0), used when U0UCR.BIT9 = 1
  // Level of bit 9 = 1 (U0UCR.D9 = 1), used when U0UCR.BIT9 = 1
  // Parity = Even (U0UCR.D9 = 0), used when U0UCR.PARITY = 1
  // Parity = Odd (U0UCR.D9 = 1), used when U0UCR.PARITY = 1
  U0UCR &= ~U0UCR_D9;

  // Flow control = disabled (U0UCR.FLOW = 0)
  U0UCR &= ~U0UCR_FLOW;

  // Bit order = LSB first (U0GCR.ORDER = 0)
  U0GCR &= ~U0GCR_ORDER;
}

void initGlobals (void)
{
  int i = 0;
  
  for (i=0; i<SIZE_OF_UART_RX_BUFFER; i++) {
    uartRxBuffer[i] = 0x00;
    uartTxBuffer[i] = 0x00;
    bleRxBuffer[i]  = 0x00;
    bleTxBuffer[i]  = 0x00;
  }
  uartTxLength = 0;
  uartTxIndex  = 0;
  uartRxIndex  = 0;
  bleTxLength  = 0;
  bleTxIndex   = 0;
  bleRxIndex   = 0;
  timeCounter  = ((20*60 + 0) * 10);
  timeCounterOn  = 0;
  timeCounterOff = 0;
  mySentryFlag = 0;
  rfFrequencyMode = _USA_FREQUENCY_MODE_;
 
  for (i=0; i<16; i++) {
    historySgv         [i] = 0;
    historyRawSgv      [i] = 0;
    historyRaw         [i] = 0;
    historySgvValid    [i] = 0;
    historyRawSgvValid [i] = 0;
    historyRawValid    [i] = 0;
  }
}