
#include "msp430.h"

#define SENSEMODE_TIMER_PERIOD      20000   /* 15 sec, assuming 1365 Hz clock (ACLK/8) */
#define UARTWAITMODE_TIMER_PERIOD   267     /* 200 ms, assuming 1365 Hz clock (ACLK/8) */
#define UARTMODE_TIMER_PERIOD       37      /* 1/300 sec, assuming 10922 Hz clk (ACLK) */
#define UARTDONEMODE_TIMER_PERIOD   1333   /* 1 sec, assuming 1365 Hz clock (ACLK/8) */

int main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;

  /* *** setup clocks *** 
  * 
  * XT1 = 10922 Hz (internal VLO)
  * DCOCLK = 1 MHz, sourced from XT1
  * 
  * MCLK = DCOCLK
  * SMCLK = XT1
  * ACLK = XT1
  */
  DCOCTL = CALDCO_1MHZ;
  BCSCTL1 = XT2OFF | CALBC1_1MHZ;
  BCSCTL2 = SELS;
  BCSCTL3 = LFXT1S_2; /* use 10922 Hz VLO with 1pF effective load cap */
  
  // set up digital output (to use oscilloscope)
  P2DIR = 0x03;
  P2OUT = 0x00;
  
  // set up timing interrupt  
  TACCTL0 = CCIE;                       /* enable interrupt */
  // SENSEMODE - 15 s
  //TACCR0 = SENSEMODE_TIMER_PERIOD - 1; /* set interrupt threshold */
  //TACTL = TASSEL_1 | ID_3 | TACLR;    /* use ACLK/8 */
  // UARTWAITMODE - 200 ms
  TACCR0 = UARTWAITMODE_TIMER_PERIOD - 1; /* set interrupt threshold */
  TACTL = TASSEL_1 | ID_3 | TACLR;    /* use ACLK/8 */
  // UARTMODE - 1/300 s
  //TACCR0 = UARTMODE_TIMER_PERIOD - 1; /* set interrupt threshold */
  //TACTL = TASSEL_1 | TACLR;    /* use ACLK */
  // UARTDONEMODE - 1 s
  //TACCR0 = UARTDONEMODE_TIMER_PERIOD - 1; /* set interrupt threshold */
  //TACTL = TASSEL_1 | ID_3 | TACLR;    /* use ACLK/8 */
  
  TACTL |= MC_1;                          /* start time in up mode */
  
  __bis_SR_register(GIE);
  
  while(1)
  {
    P2OUT |= 0x02;
    P2OUT &= ~0x02;
  }
}

#pragma vector=TIMERA0_VECTOR
__interrupt void TA_ISR(void)
{
  TACTL |= TACLR;
  P2OUT ^= 0x01;
}
