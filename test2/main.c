
#include "io430.h"

int main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;
  volatile int i;

  P1DIR = 0x01;
  
  while(1)
  {
    P1OUT ^= 0x01;
    for(i = 0; i < 20000; i++);
  }
}
