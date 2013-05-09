#include <msp430.h>
#include <stdbool.h>

/* pin definitions */
#define DBG0    BIT0    /* P1.0 - debug pin 0 */
#define DBG1    BIT1    /* P1.1 - debug pin 1 */
#define SENSEIN BIT0    /* P2.0 - sensor input voltage signal */
#define PCCOMM  BIT2    /* P2.2 - high indicates that the serial communications cable is plugged in */
#define UARTRX  BIT4    /* P3.4 - UART RX Pin */
#define UARTTX  BIT5    /* P3.5 - UART TX Pin */

/* mode values */
#define IDLEMODE        0       /* wait for communication with PC to get timestamp */
#define UARTWAITMODE    1       /* cable plugged in, but waiting to start communicating with PC */
#define UARTMODE        2       /* communicating with PC */
#define UARTDONEMODE    3       /* done communicating with PC, but cable is still plugged in */
#define SENSEMODE       4       /* periodically sample sensor and record timestamps of events */

/* communications constants */
#define ACK_VALUE       '!'

/* timing constants */
#define SENSEMODE_TIMER_PERIOD      20480   /* 15 sec, assuming 1365 Hz clock (ACLK/8) */
#define UARTWAITMODE_TIMER_PERIOD   273     /* 200 ms, assuming 1365 Hz clock (ACLK/8) */
#define UARTMODE_TIMER_PERIOD       37      /* 1/300 sec, assuming 10922 Hz clk (ACLK) */
#define UARTDONEMODE_TIMER_PERIOD   1366    /* 1 sec, assuming 1365 Hz clock (ACLK/8) */
#define UARTWAIT_PCCOMM_HIGH_CNT    2       /* transition from UARTWAIT to UART mode when PCCOMM is high for 2 cycles (400 ms) */
#define UART_PCCOMM_LOW_CNT         30      /* transition from UART mode to UARTDONE mode when PCCOMM is low for 30 baud cycles (100 ms) */
#define UARTDONE_PCCOMM_LOW_CNT     2       /* transition from UARTDONE to SENSE mode when PCCOMM is low for 2 cycles (2 seconds) */

/* buffer and memory sizes */
#define TIMESTAMP_BYTES         4           /* 31-bit UNIX timestamp (integer seconds from epoch) */
#define TIMESTAMP_MASK          0x7FFFFFFF  /* 31-bit UNIX timestamp */
#define TIMESTAMP_BUFF_SIZE     8
#define TIMESTAMP_STOR_SIZE     128     /* must be 128 if using 4-byte timestamps (needs to use 1 segment = 512 bytes) */

/* macros */
#define PCCOMMIntrOn()  do{P2IFG &= ~(PCCOMM); P2IE |= PCCOMM;}while(0)     /* turn on PC comm. interrupt */
#define PCCOMMIntrOff() do{P2IE &= ~(PCCOMM); P2IFG &= ~(PCCOMM);}while(0)  /* turn off PC comm. interrupt */
#define getNumTimestamps() (unsigned short)5 /* returns the total number of stored timestamps (mocked) */

/* debugging */
#define ONE_DELAY 5000

/* function prototypes */
__interrupt void P2_ISR(void); // freddyChange: Use P2 for specialized tasks since P2 is accessible on AMBER
__interrupt void TA_ISR(void);
__interrupt void USCI0RX_ISR(void); // freddyChange: Use UART RX interrupt
void timerASetup(unsigned char op_mode);
void uartWaitModeStart(void); // freddyChange: config USCI_A module instead
void uartModeStart(void); // freddyChange: config USCI_A module instead
void uartModeStop(void); // freddyChange: config USCI_A module instead
void startIdleSenseMode(void); // freddyChange: config USCI_A module instead
void send16bit(unsigned short val); // freddyChange: Use USCI_A module instead
void send32bit(unsigned long val); // freddyChange: Use USCI_A module instead
unsigned char myStrLen(char* str); // freddyChange: custom strlen for space
void transmitChar(char charToTransmit); // freddyChange: transmit 1 char w/ USCI_A module
void transmitString(char* strToTransmit); // freddyChange: transmit > 1 char w/ USCI_A module
void UARTSetup(void); // freddyChange: configure USCI_A module for UART
void UARTSleep(void); // freddyChange: turn off UART on USCI_A module
 
 void myDelay(unsigned char units);// freddyChange: debugging function
/* shared variables */

/* time variable */
static unsigned long curTimestamp;      /* current system timestamp in seconds from epoch (UNIX timestamp) */
 
/* mode and state */
volatile static unsigned char mode;     /* system mode */
static unsigned char pcCommStableCnt;   /* number of seconds that PCCOMM is stable */

/* UART communications */ // freddyChange: removed a bunch of stuff
static bool recvingTimestamp;           /* true when we are receiving the timestamp (after quit) */ // keep

/* mainloop */
void main(void) {

  WDTCTL = WDTPW + WDTHOLD;   // Stop WDT
  __disable_interrupt();      // disable global interrupts during initialization

  // freddyChange: Take a look at this
  /* *** initialize all pins ***
   * 
   * This is done in one step to optimize code space. The following settings are used:
   *
   * Defaults:
   *  - digital I/O
   *  - input
   *  - pull-down resistor enabled
   *
   * Non-digital I/Os: 
   *  P1: ()
   *  P2: ()
   *  P3: (UARTTX | UARTRX)
   * Outputs: 
   *  P1: (DBG0 | DGB1)
   *  P2: ()
   *  P3: (UARTTX)
   * Pull-ups:
   *  P1: ()
   *  P2: ()
   *  P3: ()
   * Disabled pull-up/downs: 
   *  P1: (UARTOUT | ADCOUT)
   *  P2: (XIN | XOUT)
   *  P3: ()
   */
  /* set inputs and outputs */
  P1DIR = DBG0 | DBG1; /* debugging leds on AMBER */
  P3DIR = UARTTX;      /* UART TX is an output */

  /* use pulldowns */
  P1OUT = 0;
  P2OUT = 0;

  /* enable/disable pull-downs */
  P2REN = PCCOMM;

  /* set I/O type */
  P3SEL = UARTTX | UARTRX;

  /* initialize interrupt pins */
  P2IES &= ~(PCCOMM);                 /* respond to rising edge of PCCOMM */

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

  /* wait until there are no osc. faults */
  do {
    /* clear osc. fault flags */
    BCSCTL3 &= ~(XT2OF | LFXT1OF);
    IFG1 &= ~OFIFG;
    __delay_cycles(50);             /* wait 50 us */
  } while (IFG1 & OFIFG);             /* test oscillator fault flag */

  
  /* *** initialize shared variables and mode *** */
  curTimestamp = 0;
  startIdleSenseMode();   /* initially enter IDLE mode */

  __enable_interrupt();   /* enable global interrupts */

  while (true) {                  /* mainloop */
      if (mode == IDLEMODE) {
          __low_power_mode_4();   /* turn off all clocks - just wait for cable to be plugged in*/
      } else {
          __low_power_mode_3();   /* enter low power mode - only ACLK is on */
      }
  }
}


#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR(void)
{
  P1OUT = DBG0;
  if (P2IFG & PCCOMM) {       /* serial cable state changed */
    PCCOMMIntrOff();
    uartWaitModeStart();    /* wait until cable is stable before starting UART mode */
    __low_power_mode_off_on_exit(); /* change power modes if transitioning out of IDLEMODE */
  }
}

#pragma vector=TIMERA0_VECTOR
__interrupt void TA_ISR(void) {

  switch(mode)
  {
  case UARTWAITMODE:
   
    if (P2IN & PCCOMM) {            /* PC comm pin is high (cable is still connected) */
      if (++pcCommStableCnt == UARTWAIT_PCCOMM_HIGH_CNT) {
                                /* cable is stable and connected, switch to UART mode */
        uartModeStart();        /* switch to UART mode */
      }
    } else {                        /* cable is disconnected */
      pcCommStableCnt = 0;        /* reset counter */
    }
    break;
  case UARTMODE:
   
    if (!(P2IN & PCCOMM)) {         /* PC comm pin is low (cable is disconnected) */
      if (++pcCommStableCnt == UART_PCCOMM_LOW_CNT) {
                                        /* cable has been disconnected, switch to UARTDONE mode */
        uartModeStop();         /* UART mode is now done */
        return;
      }
    } else {                        /* cable is still connected */
      pcCommStableCnt = 0;        /* reset counter */
    }
    break;
  case UARTDONEMODE:
   
    if (curTimestamp != 0) {    // update time
      curTimestamp += 1;      // increment by 1 second
    }
    if (!(P2IN & PCCOMM)) {         /* PC comm pin is low (cable is disconnected) */
      if (++pcCommStableCnt == UARTDONE_PCCOMM_LOW_CNT) {
                                        /* cable has been disconnected, switch to SENSE mode */
        startIdleSenseMode();   /* switch to IDLE or SENSE mode */
        __low_power_mode_off_on_exit(); /* change power modes if transitioning to IDLEMODE */
      }
    } else {                        /* cable is still connected */
      pcCommStableCnt = 0;        /* reset counter */
    }
    break;
  case SENSEMODE:
    if (curTimestamp != 0) {    // update time
      curTimestamp += 15;     // increment by 15 seconds
    }
    break;
  }
}

// USCI A0/B0 Receive ISR
// takes input character and sends back different strings through UART
// reprints original
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
  static unsigned short sendingIndex = 0;
  static unsigned long rcvTimestamp = 0;
  
  if(recvingTimestamp) {
    rcvTimestamp |= ((unsigned long)UCA0RXBUF) << (8 * sendingIndex);
    
    if(++sendingIndex == TIMESTAMP_BYTES)
    {
      curTimestamp = rcvTimestamp;    /* save timestamp */
      recvingTimestamp = false;
      uartModeStop();                 /* UART mode completed */
    }
  } else {
    switch(UCA0RXBUF) {

    // Quitting, send 1 byte ack after receiving new timestamp
    case 'q':
      sendingIndex = 0;
      recvingTimestamp = true;
      transmitChar(ACK_VALUE);
      break;

    // Resetting, send 1 byte ACK
    case 'r':
    /* NOTE: because the FLASH erase operation in clearTimestamps() is so long,
    * serial timer interrupts will be missed, and will then be handled
    * as soon as the function returns, which will mess up the regular
    * timing of the serial timer interrupts.
    * Therefore, we must turn off the timer interrupt during the erase
    * operation, yet keep the timer running to ensure that the
    * timing is correct for the serial timer interrupts.
    */
      transmitChar(ACK_VALUE);
      break;

    // Initializing sending, send number of timestamps (2 bytes)
    case 'd':
      send16bit(getNumTimestamps());
      break;

    // Asks for next timestamp (4 bytes)
    case 'e':
      send32bit((unsigned long)curTimestamp);
      break;

    // Ignore all other inputs
    default:
      break;
    }
  }
}

/* *** Helper functions *** */

// Sets up timer A with different settings depending on the mode
void timerASetup(unsigned char op_mode) {
  CCTL0 = 0;                              /* disable interrupt */
  TACTL = 0;                              /* disable timer */

  if (op_mode == UARTWAITMODE) {
      CCR0 = UARTWAITMODE_TIMER_PERIOD - 1;
      CCTL0 = CCIE;                       /* enable interrupt */
      TACTL = TASSEL_1 | ID_3 | TACLR;    /* use ACLK/8 */
  } else if (op_mode == UARTMODE) {
      CCR0 = UARTMODE_TIMER_PERIOD - 1;
      CCTL0 = CCIE;                       /* enable interrupt */
      TACTL = TASSEL_1 | ID_0 | TACLR;    /* use ACLK/1 */
  } else if (op_mode == UARTDONEMODE) {
      CCR0 = UARTDONEMODE_TIMER_PERIOD - 1;
      CCTL0 = CCIE;                       /* enable interrupt */
      TACTL = TASSEL_1 | ID_3 | TACLR;    /* use ACLK/8 */
  } else { /* SENSEMODE */
      CCR0 = SENSEMODE_TIMER_PERIOD - 1;
      CCTL0 = CCIE;                       /* enable interrupt */
      TACTL = TASSEL_1 | ID_3 | TACLR;    /* use ACLK/8 */
  }
  TACTL |= MC_1;                          /* start time in up mode */
}

// Start UART wait mode (wait until cable is stable and then start UART mode)
// freddyChange: USCI (UART) module off
void uartWaitModeStart(void) {
   mode = UARTWAITMODE;
   //curTimestamp = 0;       // will stop keeping time, so throw out the timestamp
   pcCommStableCnt = 0;
   timerASetup(mode);      // will generate periodic interrupts
}

// Start UART mode
// freddyChange: turn USCI (UART) module on. actions triggered by RX interrupts
void uartModeStart(void) {
   mode = UARTMODE;
   recvingTimestamp = false;
   pcCommStableCnt = 0;
   UARTSetup();
   CCTL0 = 0;              /* disable timer interrupt */
   TACTL = 0;              /* disable timer */
}

// Wake up every 1 second to keep time
// freddyChange: turn USCI (UART) module off. done with uart
void uartModeStop(void) {
    mode = UARTDONEMODE;
    pcCommStableCnt = 0;
    UARTSleep();
    timerASetup(mode);      // will generate periodic interrupts
}

// Transition to either SENSEMODE or IDLEMODE
void startIdleSenseMode(void) {
   //P1OUT = 0x01;
   if (curTimestamp == 0) {    /* don't go to SENSE mode, just go into IDLE */
     mode = IDLEMODE;
     CCTL0 = 0;              /* disable timer interrupt */
     TACTL = 0;              /* disable timer */
     PCCOMMIntrOn();
   } else {                    /* go into SENSE mode */
     mode = SENSEMODE;
     PCCOMMIntrOn();
     timerASetup(mode);      // will generate periodic interrupts
   }
}

// sends 16-bit value low-order byte first
void send16bit(unsigned short val) {
    unsigned char i;
    for (i = 0; i < 2; i++) {
        transmitChar((char)val);            /* send low byte of val */
        val >>= 8;                      /* shift to next byte */
    }
}

// sends 32-bit value low-order byte first
void send32bit(unsigned long val) {
    unsigned char i;
    for (i = 0; i < 4; i++) {
        transmitChar((char)val);            /* send low byte of val */
        val >>= 8;                      /* shift to next byte */
    }
}

// custom function to determine a string's length. isolated function to avoid including strings.h
unsigned char myStrLen(char* str)
{
  unsigned char i = 0;
  while(*(str + i++) != '\0');
  return i - 1;
}

// transmit a single char with the USCI_A module
void transmitChar(char charToTransmit)
{
  while (!(IFG2&UCA0TXIFG));
  UCA0TXBUF = charToTransmit;
}

// transmit multiple character word with the USCI_A module
void transmitString(char* strToTransmit)
{
  unsigned int i;
  unsigned char len = myStrLen(strToTransmit);
  for(i = 0; i < len; i++)
  {
    transmitChar(*(strToTransmit + i));
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
  IE2 |= UCA0RXIE;               // Enable USCI_A0 TX interrupt
}

// software resets USCI module (thus rendering it inert)
void UARTSleep(void)
{
  UCA0CTL1 = UCSWRST;
}

void myDelay(unsigned char units)
{
  volatile unsigned int i;
  unsigned int delayTime = units * ONE_DELAY;
  for(i = 0; i < delayTime; i++);
}