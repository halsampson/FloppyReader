// FDD MFM reader

// for TI LM4F120XL dev board
// send MFM data to FloppyReader.cpp

#include "inc\lm4f120h5qr.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"

const int BaudRate = 921600;

/*
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


const int SysClock = 80 * 1000 * 1000; // max allowed  by MINSYSDIV using PLL

void initUART0(void) {
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;
	SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;

    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // default
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOA;  // enable Port A

	UART0_CTL_R &= ~UART_CTL_UARTEN;

	int baud = BaudRate;
	if (baud * 16 > SysClock) {
		UART0_CTL_R |= UART_CTL_HSE;
		baud /= 2;
	} else UART0_CTL_R &= ~UART_CTL_HSE;
	int baudX64 = (SysClock * 8 / baud + 1) / 2;
	UART0_IBRD_R = baudX64 / 64;
	UART0_FBRD_R = baudX64 % 64;
	UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
	UART0_CC_R = 0;  // SysClock default
	UART0_CTL_R |= UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN; //  | UART_CTL_HSE for / 8

    GPIO_PORTA_AFSEL_R |= U0Rx | U0Tx;
	GPIO_PORTA_DEN_R = U0Rx | U0Tx;
    GPIO_PORTA_PCTL_R |= 0x11;  // pins 0 and 1 default
    GPIO_PORTA_AMSEL_R &= ~(U0Rx | U0Tx);

    // if (UART0_FR_R & UART_FR_RXFE) recv(UAR0_DR_R & 0xFF);
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
	UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;
	UART1_CC_R = 0;  // SysClock default
	UART1_CTL_R |= UART_CTL_RXE | UART_CTL_TXE | UART_CTL_UARTEN; //  | UART_CTL_HSE for / 8

    GPIO_PORTC_AFSEL_R |= U1Rx | U1Tx;
	GPIO_PORTC_DEN_R = U1Rx | U1Tx;
    GPIO_PORTC_PCTL_R |= 0x220000; // pins 4 and 5
    GPIO_PORTC_AMSEL_R &= ~(U1Rx | U1Tx);

    // if (UART1_FR_R & UART_FR_RXFE) recv(UART1_DR_R & 0xFF);
}

inline void sendByte(unsigned char c) {
	UART1_DR_R = c;
}

void sendStr(char* str) {
	while (*str) {
	   int timeout = 8 * SysClock * 10 / BaudRate;
	   while (!(UART1_RIS_R & UART_RIS_TXRIS) && timeout-- > 0); // room in Tx FIFO
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
  long endCount = WTIMER3_TAV_R + us * (SysClock / 1000000) + 10;  // incl.overhead
  while ((long)WTIMER3_TAV_R - endCount < 0);
}

void delay_ms(int ms) {
  long endCount = WTIMER3_TAV_R + ms * (SysClock / 1000) + 10;  // incl.overhead
  while ((long)WTIMER3_TAV_R - endCount < 0);
}

void step(bool out = false) {
  if (!out) {
	GPIO_PORTE_DATA_BITS_R[Direction] = 0; // step in
	delay_ms(10);
  }

  GPIO_PORTE_DATA_BITS_R[Step] = 0;
  delay_us(1);
  GPIO_PORTE_DATA_BITS_R[Step] = Step;
  delay_ms(3);
  GPIO_PORTE_DATA_BITS_R[Direction] = Direction;  // keep Hi vs. total current limit
}


int motorTimeout;

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

const int TrackClocks = SysClock / (360 / 60);  // 360 RPM
const int TrackEnd = TrackClocks * 15 / 16;

const int BitClocks = SysClock / 1000000 * 5 / 3;  // 1.67 us
const int HistogramSize = BitClocks * 9;  // 4.5 slow bit times max
int histogram[HistogramSize];

int bitTime2p5 = 333 + 13;
int bitTime3p5 = 467 - 17;

void readTrack(int side = 0, int density = 0) {
	GPIO_PORTE_DATA_BITS_R[HiDensity] = density ? 0 : HiDensity;  // also 2 steps for 48 tpi
	GPIO_PORTE_DATA_BITS_R[Side0] = side ? 0 : Side0;

	if (GPIO_PORTE_DATA_BITS_R[MotorOff]) {
	  motor();
	  delay_ms(500); // wait until up to speed
	}
	motorTimeout = (int)WTIMER3_TAV_R + SysClock;

	while (GPIO_PORTD_DATA_BITS_R[Index]); //wait for index hole

	int prevEdgeTime = (int)WTIMER3_TAR_R;
	int start = WTIMER3_TAV_R;
	int trackEnd = start + TrackEnd;
	int noDataEnd = start + SysClock / 2; // 500 ms timeout

	int mfmBits = 1;
	int mfmPos = 1;  // clock then data

	while (1) {
	  while (!(WTIMER3_RIS_R & TIMER_RIS_CAERIS))  { // wait for neg edge capture vs. interrupt?
		if ((int)WTIMER3_TAV_R - trackEnd > 0) {
	   	  if (!GPIO_PORTD_DATA_BITS_R[Index]
		  || (int)WTIMER3_TAV_R - noDataEnd > 0 // prevent stuck when no data
		  ) {
		    GPIO_PORTE_DATA_BITS_R[Side0 | HiDensity] = 0xFF;  // keep current low
		    return;
	   	  }
		}
	  }

	  int bitInterval = (int)WTIMER3_TAR_R - prevEdgeTime; // captured count between neg edges
	  prevEdgeTime = (int)WTIMER3_TAR_R;
      WTIMER3_ICR_R |= TIMER_ICR_CAECINT; // clear status

	  if (bitInterval < HistogramSize) {
		 histogram[bitInterval]++; // diagnostic
		 int bitCount;
		 if (bitInterval < bitTime2p5)
		   bitCount = 2;
		 else if (bitInterval < bitTime3p5)
			bitCount = 3;
		 else bitCount = 4;

		 mfmBits <<= bitCount;
		 mfmBits |= 1;

		 if ((mfmPos += bitCount) >= 8) {
		   mfmPos -= 8;
		   sendByte((mfmBits >> mfmPos) & 0xFF);
		 }
	  } else { // out of synch
		  mfmPos = 1;
		  mfmBits = 1;
	  }
	}
}


int main(void) {
    SysCtlClockSet(SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ | SYSCTL_USE_PLL | SYSCTL_SYSDIV_2_5);  // 400 MHz PLL / 2 / 2.5 = 80 MHz
      // MINSYSDIV limits any higher speed divisor (but can push PLL Hz using lower XTAL setting -- vs. USB)
	initGPIO();
    initUART1();
    initTimer();

    while (!(UART1_FR_R & UART_FR_RXFE))
    	UART1_DR_R = UART1_DR_R; // flush

	while (1) {
	  GPIO_PORTF_DATA_R = (GPIO_PORTD_DATA_BITS_R[Index | Track00] << 1 | GPIO_PORTD_DATA_BITS_R[DiskChange]) ^ 0xE;

	  if (!(UART1_FR_R & UART_FR_RXFE)) switch(UART1_DR_R & 0x5F) {
	     case 'B' : step(true); step(true); break; // Back
	     case 'S' : step(); step(); break;
	     case 'Q' : step(); break;  // 96 tpi only
	     case 'Z' : while (GPIO_PORTD_DATA_BITS_R[Track00]) step(true); break;  // to track Zero

	     case 'R' : readTrack(); break;
	     case 'T' : readTrack(1); break; // side 1
	     case 'H' :
	    	 unsigned char* p = (unsigned char*)&histogram;
	    	 for (int i = 0; i < sizeof(histogram); i++) {
	    	   int timeout = 8 * SysClock * 10 / BaudRate;
	    	   while (!(UART1_RIS_R & UART_RIS_TXRIS) && timeout-- > 0); // wait for 8 bytes room in Tx FIFO
	    	   sendByte(*p);
	    	   *p++ = 0;
	         }
	         break;

	     case 'A' :  // adjust timing
	    	 delay_ms(16); // Rx FIFO should have values
	    	 p = (unsigned char*)&bitTime2p5;
	    	 for (int count = 8; count--;)
	    		*p++ = UART1_DR_R;
	    	 break;

	     case 'I' : sendStr("FDD MFM reader " __DATE__ " " __TIME__ "\n"); break;

	     case 'D' : readTrack(0, 1); break; // HiDensity
	  }

	  if (!GPIO_PORTE_DATA_BITS_R[MotorOff] && (WTIMER3_TAV_R - motorTimeout) > 0) motor(false);

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
