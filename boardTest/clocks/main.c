
#include "msp430.h"
#include "stdbool.h"

#define SMCLKO  BIT4    /* P1.4 - output sub master clock SMCLK */
#define XIN     BIT6    /* P2.6 - XIN for crystal */
#define XOUT    BIT7    /* P2.7 - XOUT for crystal */
#define LED     BIT6    /* P3.6 - LED output */

int main( void )
{
  
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;

  // non digital I/O
  P1SEL = SMCLKO;
  P2SEL = XIN | XOUT;
  
  // digital I/O
  P3DIR = LED;
  P3OUT = 0;
  
  // Pull up/down resistors
  P1REN = (unsigned char)(~(SMCLKO));
  P2REN = (unsigned char)(~(XIN | XOUT));
  P3REN = (unsigned char)(~LED);
  
  /* *** setup clocks *** 
  * 
  * XT1 = 32768 Hz (external oscillator)
  * DCOCLK = 1 MHz, sourced from XT1
  * 
  * MCLK = DCOCLK
  * SMCLK = DCOCLK
  * ACLK = XT1
  */
  DCOCTL = CALDCO_1MHZ;
  BCSCTL1 = XT2OFF | CALBC1_1MHZ;
  BCSCTL2 = 0;        /* = 0: SMCLK = DCOCLK || = SELS: SMCLK = ACLK*/
  BCSCTL3 = LFXT1S_0 | XCAP_1; /* use 32768 Hz VLO with 6pF effective load cap */
  
  /* wait until there are no osc. faults */
  do {
    /* clear osc. fault flags */
    BCSCTL3 &= ~(XT2OF | LFXT1OF);
    IFG1 &= ~OFIFG;
    __delay_cycles(50);             /* wait 50 us */
  } while (IFG1 & OFIFG);             /* test oscillator fault flag */
  
  P3OUT |= LED;
}
