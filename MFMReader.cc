// FDD MFM reader

// for TI LM4F120XL dev board
// sends MFM data to FloppyReader.cpp


#include "inc\lm4f120h5qr.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"


// High density floppy can generate transitions every < 1us, taking 2 bits to send timing interval
// so BaudRate should be > 2 * 10 / 8  = 2.5 MBaud
const int BaudRate = 3000000;

/* special pins to avoid
PB0 3.6V max
PB1 3.6V max

PD4 USB0DM 3.6V max
PD5 USB0DP 3.6V max
*/

// Port A:
const int U0Rx = 1 << 0;  // PA0
const int U0Tx = 1 << 1;  // PA1

// Port C
const int U1Rx = 1 << 4;  // PC4
const int U1Tx = 1 << 5;  // PC5


// Port D: Inputs
const int Index      = 1 << 0;  // PD0
const int Track00    = 1 << 1;  // PD1
const int ReadData   = 1 << 2;  // PD2
const int DiskChange = 1 << 3;  // PD3

// Port E: Outputs
const int Direction = 1 << 1;  // PE1   Keep normally Hi = StepOut
const int Step 		= 1 << 2;  // PE2   Keep normally Hi  10ms after Direction change
const int Side0 	= 1 << 3;  // PE3
const int HiDensity = 1 << 4;  // PE4   300 vs. 360 RPM
const int MotorOff  = 1 << 5;  // PE5

// Port F:
const int SW2 = 1 << 0;
const int RED = 1 << 1;  // Index
const int BLU = 1 << 2;  // Track00
const int GRN = 1 << 3;  // DiskChange
const int SW1 = 1 << 4;


const unsigned int SysClock = 80 * 1000 * 1000; // max allowed  by MINSYSDIV using PLL

void initCpuClock(unsigned int xtal = SYSCTL_XTAL_16MHZ) {
	// power up main oscillator & PLL; continue to use PIOSC
    SYSCTL_RCC_R = SYSCTL_RCC_R & ~(SYSCTL_RCC_OSCSRC_M | SYSCTL_RCC_XTAL_M | SYSCTL_RCC_PWRDN | SYSCTL_RCC_USESYSDIV) | SYSCTL_RCC_BYPASS | SYSCTL_RCC_OSCSRC_INT | xtal;
    SYSCTL_RCC2_R = SYSCTL_RCC2_R & ~(SYSCTL_RCC2_OSCSRC2_M | SYSCTL_RCC2_PWRDN2) | SYSCTL_RCC2_USERCC2 | SYSCTL_RCC2_BYPASS2 | SYSCTL_RCC2_OSCSRC2_IO;

    SYSCTL_MISC_R = SYSCTL_INT_PLL_LOCK; // Clear the PLL lock interrupt
    SYSCTL_RCC_R &= ~SYSCTL_RCC_MOSCDIS;  // enable main osc
    SYSCTL_RCC2_R = SYSCTL_RCC2_R & ~(SYSCTL_RCC2_SYSDIV2_M | SYSCTL_RCC2_OSCSRC2_M | SYSCTL_RCC2_SYSDIV2LSB) | SYSCTL_SYSDIV_2_5;
    for (int delay = 64; delay--;);
    while (!(SYSCTL_RIS_R & SYSCTL_INT_PLL_LOCK)); // wait for PLL lock
    SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2; // switch to PLL clock
}


void initUART0(void) {
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;

    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // default
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOA;  // enable Port A

	UART0_CTL_R &= ~UART_CTL_UARTEN;

	int baud = BaudRate;
	if (baud * 16 > SysClock) {
		UART0_CTL_R |= UART_CTL_HSE; // UART clock = 8 * BaudRate
		baud /= 2;
	} else UART0_CTL_R &= ~UART_CTL_HSE; // UART clock = 16 * BaudRate
	int baudX64 = (SysClock * 8 / baud + 1) / 2;
	UART0_IBRD_R = baudX64 / 64;
	UART0_FBRD_R = baudX64 % 64;
	UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
	UART0_CC_R = 0;  // SysClock default
	UART0_CTL_R |= UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN;

    GPIO_PORTA_AFSEL_R |= U0Rx | U0Tx;
    GPIO_PORTA_DR8R_R = U0Tx; // 8 mA drive strength
	GPIO_PORTA_DEN_R = U0Rx | U0Tx;
    GPIO_PORTA_PCTL_R |= 0x11;  // pins 0 and 1 default
    GPIO_PORTA_AMSEL_R &= ~(U0Rx | U0Tx);
}


void initUART1(void) {
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART1;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC;

    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOC; // enable Port C

	UART1_CTL_R &= ~UART_CTL_UARTEN;

	int baud = BaudRate;
	if (baud * 16 > SysClock) {
		UART1_CTL_R |= UART_CTL_HSE;
		baud /= 2;
	} else UART1_CTL_R &= ~UART_CTL_HSE;
	int baudX64 = (SysClock * 8 / baud + 1) / 2;
	UART1_IBRD_R = baudX64 / 64;
	UART1_FBRD_R = baudX64 % 64;
	UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; //  FIFO doesn't help long string of 0s or 1s much
	UART1_CC_R = 0;  // SysClock default
	UART1_CTL_R |= UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN;

    GPIO_PORTC_AFSEL_R |= U1Rx | U1Tx;
    GPIO_PORTC_DR8R_R = U1Tx;  // 8 mA drive strength
	GPIO_PORTC_DEN_R = U1Rx | U1Tx;
    GPIO_PORTC_PCTL_R |= 0x220000; // pins 4 and 5
    GPIO_PORTC_AMSEL_R &= ~(U1Rx | U1Tx);
}

inline bool rxRdy(void) {
  return !(UART1_FR_R & UART_FR_RXFE);  // Rx FIFO not empty
}

inline unsigned char rxChar(void) {
	return UART1_DR_R;
}

inline void txWait(void) {
  while (UART1_FR_R & UART_FR_TXFF);  // wait for Tx FIFO room
}

inline void sendByte(unsigned char c) {
  UART1_DR_R = c;
}

void sendStr(char* str) {
	while (*str) {
	   txWait();
 	   sendByte(*str++);
	}
}


void initGPIO() {
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF; // Port enable (clock gating)
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF; // ??

	GPIO_PORTD_DIR_R = 0;    // inputs
	GPIO_PORTD_PUR_R = 0x0F; // 20K pull-ups
	GPIO_PORTD_DEN_R = 0x0F; // digital enable

	GPIO_PORTE_DIR_R = 0x3E; // outputs
	GPIO_PORTE_ODR_R = 0x3E; // open drain
	GPIO_PORTE_PUR_R = 0x3E; // for disconnected checking
	GPIO_PORTE_DR2R_R = 0;
	GPIO_PORTE_DR8R_R = 0x3E;// drive strength 8mA (5V / 330 ohms = 15 mA  -> < 1V out for TTL)
	GPIO_PORTE_DATA_R= 0x3E; // normally Hi
	GPIO_PORTE_DEN_R = 0x3E; // digital enable

	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // for PF0 NMI
	GPIO_PORTF_CR_R  = 0xFF; // commit NMI
	GPIO_PORTF_DIR_R = 0x0E; // LED outputs
	GPIO_PORTF_ODR_R = 0x0E; // open drain
	GPIO_PORTF_PUR_R = 0x1F; // 20K pull-ups
	GPIO_PORTF_DEN_R = 0x1F; // digital enable
}


void delay_us(int us) {
  unsigned int endCount = WTIMER3_TAV_R + us * (SysClock / 1000000) + 10;  // incl. overhead
  while ((int)(WTIMER3_TAV_R - endCount) < 0);
}

void delay_ms(int ms) {  // up to 26.8 secs
  unsigned int endCount = WTIMER3_TAV_R + ms * (SysClock / 1000) + 10;  // incl. overhead
  while ((int)(WTIMER3_TAV_R - endCount) < 0);
}

void step(bool out = false) {
  if (!out) {
	GPIO_PORTE_DATA_BITS_R[Direction] = 0; // step in
	delay_ms(10);
  }

  GPIO_PORTE_DATA_BITS_R[Step] = 0;
  delay_us(1);
  GPIO_PORTE_DATA_BITS_R[Step] = Step;
  delay_ms(10);
  GPIO_PORTE_DATA_BITS_R[Direction] = Direction;  // keep Hi vs. total current limit
}


unsigned int motorTimeout;

void motor(bool on = true) {
  GPIO_PORTE_DATA_BITS_R[MotorOff] = on ? 0 : MotorOff;
}


void initTimer(void) {
  SYSCTL_RCGCWTIMER_R = SYSCTL_RCGCWTIMER_R3;

  GPIO_PORTD_AFSEL_R |= ReadData;
  GPIO_PORTD_PCTL_R  |= GPIO_PCTL_PD2_WT3CCP0; // ReadData to WTIMER3

  WTIMER3_CTL_R &= ~(TIMER_CTL_TBEN | TIMER_CTL_TAEN);
  WTIMER3_CFG_R = 4;  // 32 bit mode
  WTIMER3_TAMR_R = TIMER_TAMR_TACDIR | TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP; // count up, edge-time mode
  WTIMER3_CTL_R |= TIMER_CTL_TAEVENT_NEG;
  WTIMER3_TAILR_R = 0xFFFFFFFF; // default
  WTIMER3_CTL_R |= TIMER_CTL_TAEN;

  // timer TAV_R should be counting; TAR_R captures edge times
}

const int DiscRPM = 300; // min - sonme are 360 RPM

const int RevClocks = SysClock / (DiscRPM / 60) * 1015 / 1000;  // includes +1.5% max speed error
const int PastIndex = RevClocks * 12 / 16; // time to look for necxt index hole

const int SlowBitClocks = 2 * SysClock / 1000000;  // 2, 3, 4 X 2 us
const int HistogramSize = SlowBitClocks * 6;  // max
int histogram[HistogramSize]; // 960


// MFM bit time interval thresholds
// can set using 'A'djust command
int BitClocks = SlowBitClocks; // 300 RPM  2/3/4 us

int bitTime1p5 = BitClocks * 3 / 2;
int bitTime2p5 = BitClocks * 5 / 2;
int bitTime3p5 = BitClocks * 7 / 2;
int bitTime4p5 = BitClocks * 9 / 2;

void readTrack(int side = 0, int density = 0) {
	GPIO_PORTE_DATA_BITS_R[HiDensity] = density ? 0 : HiDensity;  // also 2 steps for 48 tpi
	GPIO_PORTE_DATA_BITS_R[Side0] = side ? 0 : Side0;  // Lo = Side1 select

	if (GPIO_PORTE_DATA_BITS_R[MotorOff]) {
	  motor();
	  delay_ms(500); // wait until up to speed
	}
	motorTimeout = WTIMER3_TAV_R + SysClock; // 1 second

	unsigned int indexTimeout = WTIMER3_TAV_R + 3 * RevClocks;
	while (!(GPIO_PORTD_DATA_BITS_R[Index])) { //wait for no index hole
	  if ((int)(WTIMER3_TAV_R - indexTimeout) > 0) {
		sendByte('H');  // status: stuck at index Hole
		return;
	  }
	}

	while (GPIO_PORTD_DATA_BITS_R[Index]) { //wait for index hole
	  if ((int)(WTIMER3_TAV_R - indexTimeout) > 0) {
		sendByte('I'); // status: no Index -- WHY??
		return;
	  }
	}

	unsigned int revPastIndex = WTIMER3_TAV_R + PastIndex;
	unsigned int noDataEnd = WTIMER3_TAV_R + 2 * RevClocks;
	unsigned int prevEdgeTime = WTIMER3_TAR_R;

	int mfmBits = 1;
	int mfmPos = 1;  // clock then data
	int runtTime = 0;

	while (1) {
	  while (!(WTIMER3_RIS_R & TIMER_RIS_CAERIS))  { // wait for neg edge capture vs. interrupt?
		if ((int)(WTIMER3_TAV_R - revPastIndex) > 0) {
	   	  if (!GPIO_PORTD_DATA_BITS_R[Index]) // index hole at end of track
	   		  return; // done with track
          if ((int)(WTIMER3_TAV_R - noDataEnd) > 0) { // prevent stuck if no data
        	sendByte('D'); // no Data
		    return;
          }
		}
	  }

	  int bitInterval = WTIMER3_TAR_R - prevEdgeTime; // captured count between neg edges
	  prevEdgeTime = WTIMER3_TAR_R;
      WTIMER3_ICR_R |= TIMER_ICR_CAECINT; // clear status

      bitInterval += runtTime;

	  if (bitInterval >= HistogramSize)
		continue;

	  histogram[bitInterval]++; // diagnostic

	  if (bitInterval < bitTime1p5) {
		runtTime = bitInterval;
		continue;
	  }
	  runtTime = 0;

	  int bitCount;
	  if (bitInterval < bitTime2p5)
		bitCount = 2;
	  else if (bitInterval < bitTime3p5)
		bitCount = 3;
	  else if (bitInterval < bitTime4p5)
		bitCount = 4;
	  else continue; // not in synch

	  mfmBits <<= bitCount;
	  mfmBits |= 1;

  	  if ((mfmPos += bitCount) >= 8) {
		  mfmPos -= 8;
		sendByte((mfmBits >> mfmPos) & 0xFF);
	  }
	}
}


int main(void) {
	initCpuClock();
	initGPIO();
    initUART1();
    initTimer();

    while (rxRdy())
      volatile int flush = rxChar();

	while (1) {
	  GPIO_PORTF_DATA_R = (GPIO_PORTD_DATA_BITS_R[Index | Track00] << 1 | GPIO_PORTD_DATA_BITS_R[DiskChange]) ^ 0xE; // update status LEDs

	  if (rxRdy()) switch(rxChar() & 0x5F) { // command character received
	     case 'A' :  // Adjust bit timing
	    	 delay_ms(16); // Rx FIFO should have all 4 values -- better a check header
	    	 unsigned char* p = (unsigned char*)&bitTime1p5;
	    	 for (int count = 4 * sizeof(int); count--;)
	    		*p++ = UART1_DR_R;
	    	 break;

	     case 'B' : step(true); break; // step Back
	     case 'S' : step(); break; // Step: twice on high density drives
	     case 'Z' : // retract to track Zero
	    	 int maxSteps = 80; // for hi density drives
	    	 while (GPIO_PORTD_DATA_BITS_R[Track00] && maxSteps-- >= 0) step(true); // to track Zero
	    	 break;

	     case 'R' : readTrack(); break;
	     case 'T' : readTrack(1); break; // read side 1
	     case 'D' : readTrack(0, 1); break; // HiDensity

	     case 'H' : // sned Histogram data
	    	 histogram[0] = 0xBE66A7ED; // marker
	    	 p = (unsigned char*)&histogram;
	    	 for (int i = sizeof(histogram); i--;) {
	    	   txWait();
	    	   sendByte(*p);
	    	   *p++ = 0;
	         }
	         break;

	     case 'I' : sendStr("FDD MFM reader " __DATE__ " " __TIME__ "\n"); break; // Identify

	     case 'M' : // Motor on
	    	 motorTimeout = WTIMER3_TAV_R + SysClock; // 1 second
	         motor();
	         break;

	     case 'Q' : sendByte(GPIO_PORTD_DATA_BITS_R[Index | Track00 | DiskChange]); break; // status
	  }

	  if (!GPIO_PORTE_DATA_BITS_R[MotorOff] && (int)(WTIMER3_TAV_R - motorTimeout) > 0) { // timeout
		  motor(false);
		  GPIO_PORTE_DATA_BITS_R[Side0 | HiDensity] = 0xFF; // Hi to keep pin currents low
	  }

	  if (!GPIO_PORTF_DATA_BITS_R[SW1]) { // SW1 pressed
		readTrack();
		GPIO_PORTF_DATA_BITS_R[GRN] = GRN;
	    while (!GPIO_PORTF_DATA_BITS_R[SW1]);
	  }

	  if (!GPIO_PORTF_DATA_BITS_R[SW2]) { // SW2 pressed
		 step();
		 int longPress = 500;
		 while (!GPIO_PORTF_DATA_BITS_R[SW2]) { // still pressed
            delay_ms(1);
			if (!longPress--)
			  while (GPIO_PORTD_DATA_BITS_R[Track00]) step(true);
		 }
	  }
	}
}
