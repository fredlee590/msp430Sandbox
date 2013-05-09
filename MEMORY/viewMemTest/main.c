
#include "msp430.h"

#define DBG0            BIT0
#define DBG1            BIT1
#define UARTRX          BIT4
#define UARTTX          BIT5
#define ONE_DELAY       5000

void UARTSetup(void);
void UARTSleep(void);
void transmitChar(char charToTransmit);
void write_Seg(char* ptr, char value);
char read_Seg(char* ptr, unsigned int index);
void myDelay(unsigned char units);

int main( void )
{
  unsigned int i;
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;

  /* set inputs and outputs */
  P1DIR = DBG0 | DBG1; /* Two LEDs on AMBER are outputs */
  P3DIR = UARTTX;      /* UART TX is an output */
  
  /* set I/O type */
  P3SEL = UARTTX | UARTRX;
  
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
  BCSCTL2 = 0;
  BCSCTL3 = LFXT1S_2; /* use 10922 Hz VLO with 1pF effective load cap */
  
  UARTSetup();
  
  __enable_interrupt();   /* enable global interrupts */
  
  P1OUT |= DBG0;
  
  for(i = 0x8000; i < 0xFFFF; i++)
  {
    P1OUT ^= DBG1;
    myDelay(2);
    transmitChar(read_Seg((char*)0x8000, i));
  }
  
}

// configure USCI module for UART mode
void UARTSetup(void)
{
  UCA0CTL1 |= UCSWRST;
  UCA0CTL1 |= UCSSEL_2;                     // BRCLK = SMCLK
  UCA0BR0 = 104;                            // 1MHz/9600 = 104.166
  UCA0BR1 = 0x00;                           //
  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}

// software resets USCI module (thus rendering it inert)
void UARTSleep(void)
{
  UCA0CTL1 |= UCSWRST;
}

// transmit a single char with the USCI_A module
void transmitChar(char charToTransmit)
{
  while (!(IFG2&UCA0TXIFG));
  UCA0TXBUF = charToTransmit;
}

void write_Seg(char* ptr, char value)
{
  char *Flash_ptr;
  unsigned int i;
  
  Flash_ptr = ptr;               // Initialize Flash pointer D
  /* test */
  FCTL3 = FWKEY;                            // Clear Lock bit
  FCTL1 = FWKEY + ERASE + EEI;              // Set Erase bit, allow interrupts                           
  // Dummy write to erase Flash seg(s)
  *Flash_ptr = 0;
  
  FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation
  for (i = 0; i < 64; i++)
  {
    *Flash_ptr++ = value;                   // Write value to flash
  }
  /* test also */
  FCTL1 = FWKEY;                            // Clear WRT bit
  FCTL3 = FWKEY + LOCK;                     // Set LOCK bit

}

char read_Seg(char* ptr, unsigned int index)
{
  char *Flash_ptr;                          // Flash pointer

  Flash_ptr = ptr;               // Initialize Flash pointer
  return *(Flash_ptr + index);  
}

// debugging delay. usually blinking LEDs
void myDelay(unsigned char units)
{
  volatile unsigned int i;
  unsigned int delayTime = units * ONE_DELAY;
  for(i = 0; i < delayTime; i++);
}