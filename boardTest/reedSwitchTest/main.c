
#include "msp430.h"
#include "stdbool.h"

#define SMCLK   BIT4    /* P1.4 - smclk out */

#define SENSEIN BIT1    /* P2.1 - sensor in */
#define PCCOM   BIT2    /* P2.2 - Vprog */
#define XIN     BIT6    /* P2.6 - XIN for crystal */
#define XOUT    BIT7    /* P2.7 - XOUT for crystal */

#define LED     BIT6    /* P3.6 - LED circuit */

int main( void )
{
  
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  
  // non digital I/O
  P1SEL = SMCLK;
  P1DIR = SMCLK;
  
  P2SEL = XIN | XOUT;
  
  // LED set up
  P3DIR = LED;
  P3OUT = 0;
  
  // Pull up/down resistors
  P1REN = (unsigned char)(~SMCLK);
  P2REN = (unsigned char)(~(PCCOM | XIN | XOUT | SENSEIN));
  P3REN = (unsigned char)(~(LED));
  
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
  BCSCTL2 = SELS;        /* SMCLK = ACLK */
  BCSCTL3 = LFXT1S_0 | XCAP_1; /* use 32768 Hz VLO with 6pF effective load cap */
  
  /* wait until there are no osc. faults */
  do {
    /* clear osc. fault flags */
    BCSCTL3 &= ~(XT2OF | LFXT1OF);
    IFG1 &= ~OFIFG;
    __delay_cycles(50);             /* wait 50 us */
  } while (IFG1 & OFIFG);             /* test oscillator fault flag */
  
  while(1)
  {
    if(P2IN & SENSEIN)
    {
      P3OUT &= ~LED;
    }
    else
    {
      P3OUT |= LED;
    }
  }
}
