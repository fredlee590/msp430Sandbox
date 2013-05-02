
#include "msp430.h"
#define ORIGINAL_STR "READY TO UART\n"

/*
this main demonstrates UART in several different ways. Use to develop functions
using UART module to replace bit banging.
*/

volatile unsigned int i = 0;     // general index for looping through strings to print
char* strToPrint = ORIGINAL_STR; // original assignment

void exampleUARTSetup( void );
unsigned char myStrLen( char* );
void transmitChar( unsigned char );

// main loop
int main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  
  exampleUARTSetup();
}

// set up UART to use 1 MHz digital clock
// set up to display ACLK and SMCLK
// enter low power mode and rely on interrupts
void exampleUARTSetup( void )
{
  // set up clocks (DCO)
  DCOCTL = CALDCO_1MHZ; // configure DCO clock DCO and MOD
  BCSCTL1 = CALBC1_1MHZ; // configure DCO clock RESL
  
  P1DIR = 0x03;                             // All P1.x outputs
  P1OUT = 0x03;                             // All P1.x reset
  
  P2SEL = 0x03;                             // enable pins to output clocks
  P2DIR = 0x03;                             // outputs
  
  P3SEL = 0x30;                             // P3.4,5 = USCI_A0 TXD/RXD
  P3DIR = 0xFF;                             // All P3.x outputs
  P3OUT = 0;                                // All P3.x reset

  // configure UCSI module for UART mode
  UCA0CTL1 |= UCSWRST;
  UCA0CTL1 |= UCSSEL_2;                     // BRCLK = SMCLK
  UCA0BR0 = 104;                            // 1MHz/9600 = 104.166
  UCA0BR1 = 0x00;                           //
  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  IE2 |= UCA0TXIE | UCA0RXIE;               // Enable USCI_A0 TX interrupt

  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, interrupts enabled
}

// lighter weight strlen than strlen
unsigned char myStrLen(char* str)
{
  unsigned char i = 0;
  while(*(str + i++) != '\0');
  return i - 1;
}

// USCI A0/B0 Transmit ISR
// first print default message once and quit
// one character at a time
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
  UCA0TXBUF = strToPrint[i++];
  if(i > myStrLen(strToPrint))
  {
    IE2 &= ~UCA0TXIE;
  }
}

// USCI A0/B0 Receive ISR
// takes input character and sends back different strings through UART
// reprints original
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  switch(UCA0RXBUF)
  {
  case 'a':
    strToPrint = "option A\n";
    break;
  case 'b':
    strToPrint = "option B\n";
    break;
  case 'c':
    strToPrint = "option C\n";
    break;
  default:
    strToPrint = "unknown option\n";
    break;
  }
  
  unsigned char len = myStrLen(strToPrint);
  for(i = 0; i < len; i++)
  {
    transmitChar(strToPrint[i]);
  }
  
  strToPrint = ORIGINAL_STR;
  i = 0;
  IE2 |= UCA0TXIE;
}

void transmitChar(unsigned char charToTransmit)
{
  while (!(IFG2&UCA0TXIFG));
  UCA0TXBUF = strToPrint[i];
}