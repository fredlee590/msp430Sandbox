/*
main.c - uartTest
This main demonstrates how to communicate with a Python script using the Python
serial library from the Namaste 3 board

Turns on LEDs and sends back data depending on options sent by Python script
*/

#include "msp430.h"

#define DBG0    BIT0    /* P1.0 - general debug */

#define XIN     BIT6    /* P2.6 - crystal oscillator in */
#define XOUT    BIT7    /* P2.7 - crystal oscillator out */

#define UARTTX  BIT4    /* P3.4 - UART Tx pin */
#define UARTRX  BIT5    /* P3.5 - UART Rx pin */
#define LED     BIT6    /* P3.6 - LED VCC */

/*
this main demonstrates UART in several different ways. Use to develop functions
using UART module to replace bit banging.
*/

void UARTSetup( void );
void transmitChar( unsigned char );

// main loop
int main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  
  UARTSetup();
}

// set up UART to use 1 MHz digital clock
// set up to display ACLK and SMCLK
// enter low power mode and rely on interrupts
void UARTSetup( void )
{
  // set up clocks (DCO)
  DCOCTL = CALDCO_1MHZ; // configure DCO clock DCO and MOD
  BCSCTL1 = CALBC1_1MHZ; // configure DCO clock RESL
  BCSCTL2 = SELS; /* SMCLK = ACLK */
  BCSCTL3 = LFXT1S_0 | XCAP_1; /* use 32768 Hz XT1 with 6pF effective load cap */
  
  P1DIR = DBG0;
  P1OUT = DBG0;
  
  P2DIR = 0;
  P2SEL = XIN | XOUT;
  
  P3DIR = LED;
  P3SEL = UARTTX | UARTRX;                  // P3.4,5 = USCI_A0 TXD/RXD
  P3OUT = 0;
  
  P1REN = (unsigned char)(~DBG0);
  P2REN = (unsigned char)(~(XIN | XOUT));
  P3REN = (unsigned char)(~(LED | UARTTX | UARTRX));

  // configure UCSI module for UART mode
  UCA0CTL1 |= UCSWRST;
  UCA0CTL1 |= UCSSEL_2;                     // BRCLK = SMCLK = ACLK = 32kHz crystal
  UCA0BR0 = 3;                              // 32kHz/9600 = 3
  UCA0BR1 = 0x00;
  UCA0MCTL = UCBRS1 | UCBRS0;               // Modulation UCBRSx = 3
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  IE2 |= UCA0TXIE;                          // Enable USCI_A0 TX interrupt

  __bis_SR_register(LPM3_bits + GIE);       // Enter LPM3, interrupts enabled
}

// USCI A0/B0 Transmit ISR
// Spam a certain character through UART
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
  UCA0TXBUF = 'a';
}