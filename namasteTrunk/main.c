#include <msp430.h>
#include <stdbool.h>

/* pin definitions */
#define DBG0    BIT0    /* P1.0 - debug pin 0 */
#define DBG1    BIT1    /* P1.1 - debug pin 1 */
#define SENSEIN BIT0    /* P2.0 - sensor input voltage signal */
#define PCCOMM  BIT2    /* P2.2 - high indicates that the serial communications cable is plugged in */
#define UARTTX  BIT4    /* P3.4 - UART TX Pin */
#define UARTRX  BIT5    /* P3.5 - UART RX Pin */

/* mode values */
#define IDLEMODE        0       /* wait for communication with PC to get timestamp */
#define UARTWAITMODE    1       /* cable plugged in, but waiting to start communicating with PC */
#define UARTMODE        2       /* communicating with PC */
#define UARTDONEMODE    3       /* done communicating with PC, but cable is still plugged in */
#define SENSEMODE       4       /* periodically sample sensor and record timestamps of events */

/* mat state values */
#define MAT_OPEN        1
#define MAT_CLOSED      0
#define MAT_UNDEF       0xFF    /* undefined mat state */
#define MAT_STATE_SHIFT 31      /* shift the mat state 31 bits to the left to get it to the high bit of 32-bit timestamp */

/* communications constants */
#define ACK_VALUE       '!'

/* timing constants */
// freddyChange: These timing values correspond to the use of the VLO oscillator for prototyping
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
#define getNumTimestamps()  (((unsigned short)timeStorIndex) + ((unsigned short)timeBufferIndex))   /* returns the total number of stored timestamps */

/* debugging */
#define ONE_DELAY 5000

/* function prototypes */
/* interrupts */
__interrupt void P2_ISR(void);
__interrupt void TA_ISR(void);
__interrupt void USCI0RX_ISR(void);

/* timer setups */
void timerASetup(unsigned char op_mode);

/* UART functions */
void uartWaitModeStart(void);
void uartModeStart(void);
void uartModeStop(void);
void startIdleSenseMode(void);
void send16bit(unsigned short val);
void send32bit(unsigned long val);
void transmitChar(char charToTransmit);
void UARTSetup(void);
void UARTSleep(void);

/* Flash memory / data storage functions */
void recordEvent(unsigned char matState);
void clearTimestamps(void);
unsigned long getTimestamp(unsigned short timestampIndex);
 
/* shared variables */
/* time variables */
static unsigned long curTimestamp;      /* current system timestamp in seconds from epoch (UNIX timestamp) */
static unsigned char timeBufferIndex;   /* current index into timestampBuffer */
static unsigned long timestampBuffer[TIMESTAMP_BUFF_SIZE]; /* buffer holding timestamps of all events */
static unsigned char timeStorIndex;     /* current index into timestampStorage */
#pragma location="FLASH_TIMESTAMP_STORAGE"
const unsigned long timestampStorage[TIMESTAMP_STOR_SIZE]; /* segment of flash to hold saved timestamps */
 
/* mode and state */
volatile static unsigned char mode;     /* system mode */
static unsigned char pcCommStableCnt;   /* number of seconds that PCCOMM is stable */
static unsigned char prevMatState;      /* Previous state of mat */

/* UART communications */
static bool recvingTimestamp;           /* true when we are receiving the timestamp (after quit) */

/* mainloop */
void main(void) {

  WDTCTL = WDTPW + WDTHOLD;   // Stop WDT
  __disable_interrupt();      // disable global interrupts during initialization

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
   *  P3: ()
   * Pull-ups:
   *  P1: ()
   *  P2: ()
   *  P3: ()
   * Disabled pull-up/downs: 
   *  P1: (DBG0 | DBG1)
   *  P2: (PCCOM | SENSEIN)
   *  P3: (UARTTX | UARTRX)
   */
  /* set inputs and outputs */
  P1DIR = DBG0 | DBG1; /* debugging leds are outputs. all others are inputs / don't cares */

  /* use pulldowns */
  P1OUT = 0;
  P2OUT = 0;
  P3OUT = 0;

  /* enable/disable pull-downs */
  P1REN = (unsigned char)(~(DBG0 | DBG1));
  P2REN = (unsigned char)(~(PCCOMM | SENSEIN));
  P3REN = (unsigned char)(~(UARTTX | UARTRX));

  /* set I/O type */
  P3SEL = (UARTTX | UARTRX);

  /* initialize interrupt pins */
  P2IES &= ~(PCCOMM);                 /* respond to rising edge of PCCOMM */

  // freddyChange: only here to run on AMBER Board independently
  /* *** setup clocks *** 
   * 
   * XT1 = 10922 Hz (internal VLO)
   * DCOCLK = 1 MHz, sourced from XT1
   * 
   * MCLK = DCOCLK
   * SMCLK = DCOCLK
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
  
  /* *** setup FLASH controller *** */
  FCTL2 = FWKEY + FSSEL0 + FN1;       /* MCLK/3 for Flash Timing Generator */

  /* *** initialize shared variables and mode *** */
  curTimestamp = 0;
  clearTimestamps();
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
  P1OUT ^= DBG0;
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
    if (prevMatState != MAT_OPEN && (P2IN & SENSEIN)) {
      recordEvent(MAT_OPEN);
    } else if(prevMatState != MAT_CLOSED && !(P2IN & SENSEIN)) {
      recordEvent(MAT_CLOSED);
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
      CCTL0 = 0;              /* disable serial timer interrupts */
      clearTimestamps();
      CCTL0 = CCIE;           /* re-enable serial timer interrupts */
      transmitChar(ACK_VALUE);
      break;

    // Initializing sending, send number of timestamps (2 bytes)
    case 'd':
      send16bit(getNumTimestamps());
      sendingIndex = 0;
      break;

    // Asks for next timestamp (4 bytes)
    case 'e':
      if (sendingIndex < getNumTimestamps()) {
        send32bit(getTimestamp(sendingIndex++));
      }
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
void uartWaitModeStart(void) {
   mode = UARTWAITMODE;
   curTimestamp = 0;       // will stop keeping time, so throw out the timestamp
   pcCommStableCnt = 0;
   timerASetup(mode);      // will generate periodic interrupts
}

// Start UART mode
void uartModeStart(void) {
   mode = UARTMODE;
   recvingTimestamp = false;
   pcCommStableCnt = 0;
   UARTSetup();
   CCTL0 = 0;              /* disable timer interrupt */
   TACTL = 0;              /* disable timer */
}

// Wake up every 1 second to keep time
void uartModeStop(void) {
    mode = UARTDONEMODE;
    pcCommStableCnt = 0;
    UARTSleep();
    timerASetup(mode);      // will generate periodic interrupts
}

// Transition to either SENSEMODE or IDLEMODE
void startIdleSenseMode(void) {
   if (curTimestamp == 0) {    /* don't go to SENSE mode, just go into IDLE */
     mode = IDLEMODE;
     CCTL0 = 0;              /* disable timer interrupt */
     TACTL = 0;              /* disable timer */
     PCCOMMIntrOn();
   } else {                    /* go into SENSE mode */
     mode = SENSEMODE;
     prevMatState = MAT_UNDEF;
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

// transmit a single char with the USCI_A module
void transmitChar(char charToTransmit)
{
  while (!(IFG2&UCA0TXIFG));
  UCA0TXBUF = charToTransmit;
}

// configure USCI module for UART mode
// freddyChange: consider turning UCA0RXIE on and off (power considerations?)
void UARTSetup(void)
{
  UCA0CTL1 |= UCSWRST;
  UCA0CTL1 |= UCSSEL_2;                     // BRCLK = SMCLK = MCLK = 1MHz
  UCA0BR0 = 104;                            // 1MHz/9600 = 104.166
  UCA0BR1 = 0x00;
  UCA0MCTL = UCBRS0;                        // Modulation UCBRSx = 1
  UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
  IE2 |= UCA0RXIE;                          // Enable USCI_A0 TX interrupt
}

// software resets USCI module (thus rendering it inert)
// freddyChange: consider turning UCA0RXIE on and off (power considerations?)
void UARTSleep(void)
{
  UCA0CTL1 = UCSWRST;
}

// record event and timestamp in flash memory
void recordEvent(unsigned char matState) {
  /* timestamp buffer in RAM is full and there is space in the timestamp storage in FLASH */
  if ((timeBufferIndex == TIMESTAMP_BUFF_SIZE) && 
    ((timeStorIndex + TIMESTAMP_BUFF_SIZE) <= TIMESTAMP_STOR_SIZE))
  {
    /* copy RAM buffer to FLASH storage */
    unsigned long * timestampStoragePtr = (unsigned long *)timestampStorage;
    FCTL1 = FWKEY | WRT;                      // Set WRT bit (for write operations)
    FCTL3 = FWKEY | LOCKA;                    // Clear LOCK bit
    unsigned char i;
    for (i = 0; i < TIMESTAMP_BUFF_SIZE; i++) {
      timestampStoragePtr[timeStorIndex++] = timestampBuffer[i];
    }
    FCTL1 = FWKEY;                              // Clear WRT bit
    FCTL3 = FWKEY | LOCKA | LOCK;               // Set LOCK bit

    timeBufferIndex = 0;                /* RAM buffer is now empty */
  }

  /* store new timestamp into local RAM buffer */
  if (timeBufferIndex < TIMESTAMP_BUFF_SIZE) {
    /* store 31-bit timestamp with the matState in the high bit */
    timestampBuffer[timeBufferIndex++] = (((unsigned long)matState) << MAT_STATE_SHIFT) | (curTimestamp & TIMESTAMP_MASK);
    prevMatState = matState;
  }
}

// clear all timestamps, and prepares to record more
void clearTimestamps(void) {
  timeBufferIndex = 0;
  timeStorIndex = 0;

  /* erase timestamp storage (in FLASH) */
  unsigned long * timestampStoragePtr = (unsigned long *)timestampStorage;
  FCTL1 = FWKEY | ERASE;                      // Set ERASE bit
  FCTL3 = FWKEY | LOCKA;                      // Clear LOCK bit
  *timestampStoragePtr = 0;                   // Dummy write to erase Flash segment
  FCTL1 = FWKEY;                              // Clear ERASE bit
  FCTL3 = FWKEY | LOCKA | LOCK;               // Set LOCK bit
}

// retrieve timestamp from flash memory
unsigned long getTimestamp(unsigned short timestampIndex) {
  if (timestampIndex < timeStorIndex) {   /* pull value from FLASH storage */
    return timestampStorage[timestampIndex];
  } else if ((timestampIndex - timeStorIndex) < timeBufferIndex) { /* pull from RAM buffer */
    return timestampBuffer[timestampIndex - timeStorIndex];
  } else {    /* index out of range */
    return 0;
  }
}
