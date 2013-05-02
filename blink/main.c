
#include "msp430.h"
#define FLASH_SPEED 3000

void blink( void );

int main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;

  blink();
}

void blink( void )
{
  volatile unsigned int i;
  
  /*
  // set up clocks (1MHz DCO)
  DCOCTL = CALDCO_1MHZ; // configure DCO clock DCO and MOD
  BCSCTL1 = CALBC1_1MHZ; // configure DCO clock RESL
  
  // set up clocks (VLO)
  DCOCTL = CALDCO_1MHZ; // not using DCO clock for VLO test, so arbitrary
  BCSCTL1 = XT2OFF;     // select low power options
  BCSCTL2 = SELM_3 | SELS; // master clock = VLO;
                                    // small clock = VLO too;
                                    // master divider = 1;
  BCSCTL3 = LFXT1S_2 | XCAP_1;      // low frequency clock is VLO;
                                    // capacitance set to 6 pf (not sure why)
  */
  
  // set up clocks (internal 32kHz crystal)
  /*
  BCSCTL1 = XT2OFF;
  BCSCTL2 = 0;
  BCSCTL3 = LFXT1S_0;
  */
  
  // production setting
  DCOCTL = CALDCO_1MHZ;
  BCSCTL1 = XT2OFF | CALBC1_1MHZ;
  BCSCTL2 = SELS;
  BCSCTL3 = LFXT1S_2; /* use ~11kHz VLO*/
  
  // set up flashing lights
  P1DIR = 0x03;
  P1OUT = 0x01;
  
  // set up layout verification. if p4.5 is where it ought to be, then it's 
  // correct
  P4DIR = 0x10;
  P4OUT = 0x10;
  
  // let's look at clocks
  P2DIR |= 0x03;
  P2SEL |= 0x03;
  
  // main loop
  while(1)
  {
    P1OUT ^= 0x03; // visual (LED) confirmation of program presence
    P4OUT ^= 0x10; // trying to identify layout of Amber board
    for(i = 0; i < FLASH_SPEED; i++); // regulate speed of flashing
  }  
}