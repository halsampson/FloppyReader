// Floppy.cpp: Interprets MFM data sent from FDD MFMReader on LMF4120 MCU

#include <Windows.h>
#include <stdio.h>
#include <conio.h>
#include <math.h>


void showError(void) {
  DWORD errCode = GetLastError();
  LPVOID lpMsgBuf;
  FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
    NULL, errCode, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPTSTR)&lpMsgBuf,  0, NULL);
  printf(": %s\n", (char*)lpMsgBuf);
  LocalFree(lpMsgBuf);
}

HANDLE hCom;

// TODO: compare with PuTTY code which somehow sets 3M baud OK
//    perhaps also to bridge drivers directly???

HANDLE openSerial(const char* portName) {
  char portDev[16] = "\\\\.\\";
  strcat_s(portDev, sizeof(portDev), portName);

  hCom = CreateFile(portDev, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL); // better OVERLAPPED
  if (hCom == INVALID_HANDLE_VALUE) return NULL;

#if 1 // configure far end bridge COM port - only for bridges - could check endpoint capabilites??
  DCB dcb = { 0 };
  dcb.DCBlength = sizeof(DCB);
  dcb.BaudRate = 3000000;
  // PL2303HX:  Divisor = 12 * 1000 * 1000 * 32 / baud --> 12M, 6M, 3M, 2457600, 1228800, 921600, ... baud
  // FTDI 3 MHz / (n + 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875; where n is an integer between 2 and 16384)
  // Note: special cases n = 0 -> 3 MBaud; n = 1 -> 2 MBaud; Sub-integer divisors between 0 and 2 not allowed.
  // CP2102: Use configuration utility to set aliases: 2M, 3M, and higher supported
  dcb.ByteSize = 8;
  dcb.StopBits = 1;
  dcb.fBinary = TRUE; // no EOF check
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;

  dcb.XonChar = 17;
  dcb.XoffChar = 19; // error if same as XonChar
#endif  

  if (!SetCommState(hCom, &dcb)) {
    printf("Can't set baud - use PuTTY"); 
    showError();
  }

  if (!SetupComm(hCom, 16384, 16)) printf("Can't SetupComm"); // Set size of I/O buffers (max 16384 on Win7)

  // USB bulk packets arrive at 1 kHz rate
  COMMTIMEOUTS timeouts = { 0 };  // in ms

  const int DiscRPM = 300 - 5; // min
  timeouts.ReadIntervalTimeout = 50 + 2 * 60 * 1000 / DiscRPM + 100; // wait for index + 1 rev
  timeouts.ReadTotalTimeoutConstant = timeouts.ReadIntervalTimeout + 300;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  if (!SetCommTimeouts(hCom, &timeouts))  printf("Can't SetCommTimeouts");

  return hCom;
}

int rxRdy(void) { // returns # of characters in read buffer
  COMSTAT cs;
  DWORD commErrors;
  if (!ClearCommError(hCom, &commErrors, &cs)) return -1;
  if (commErrors)
    printf("\n\rCommErr %X\n", commErrors); // 8 = framing (wrong baud rate); 2 = overrurn; 1 = overflow
  return cs.cbInQue;
}

void sendCmd(char ch) {
  WriteFile(hCom, &ch, 1, NULL, NULL);
}


HWND consoleWnd;
HDC console;
int maxX, yOfs;

void consoleGraphInit() {
  char consoleWindowTitle[128];
  GetConsoleTitle(consoleWindowTitle, sizeof(consoleWindowTitle));
  consoleWnd = FindWindow(NULL, consoleWindowTitle);
  console = GetDC(consoleWnd);
  RECT rect;
  GetClientRect(consoleWnd, &rect);
  maxX = rect.right - rect.left;
  yOfs = rect.bottom - 1;
}

int x;

void plot(int y) {
  SetPixel(console, x, yOfs - y, RGB(0, 255, 0));
  if (++x >= maxX) x = 0;
}


UINT16 crc_ccitt[256]; // x**16 + x**12 + x**5 + 1 CRC16 table
const UINT16 CrcPreset = 0xFFFF; // preset to all 1's

void crc16_init() { // init crc16 table
  for (int i = 0; i < 256; i++) {
    UINT16 w = i << 8;
    for (int j = 0; j < 8; j++)
      w = (w << 1) ^ ((w & 0x8000) ? 0x1021 : 0);
    crc_ccitt[i] = w;
  }
}

inline UINT16 crc16(UINT16 crc, UINT8 b) { // calc crc16 for 1 byte
  return crc = (crc << 8) ^ crc_ccitt[((crc >> 8) & 0xFF) ^ b];
}

UINT16 crc16_block(UINT8* ptr, size_t len, UINT16 crc = CrcPreset) { // calc crc16 for block
  while (len--) crc = crc16(crc, *ptr++);
  return crc;
}


void identify() {
  sendCmd('I');
  char id[64] = { 0 };
  if (ReadFile(hCom, id, sizeof(id), NULL, NULL))
    printf("%s\n", id);
}

// 80 MHz bit timing clocks * 2, 3, or 4 
//const int BitClocks = 80 * 1;      // 1us    360 RPM high density
//const int BitClocks = 80 * 5 / 3;  // 1.67us 360 RPM normal density
const int BitClocks = 80 * 2;      // 2us    300 RPM normal density

int bitTime1p5 = BitClocks * 3 / 2;
int bitTime2p5 = BitClocks * 5 / 2;
int bitTime3p5 = BitClocks * 7 / 2;
int bitTime4p5 = BitClocks * 9 / 2;

void adjustTiming(void) {
  sendCmd('A');
  WriteFile(hCom, &bitTime1p5, 3 * sizeof(int), NULL, NULL);
}


UINT8 mfm[32768]; // clock/data/clock/data/... raw bits
DWORD mfmTrackBytes;

int mfmBytePos, mfmBitPos;

int getMfmBit() {
  if (--mfmBitPos < 0) {
    mfmBitPos = 7;
    ++mfmBytePos;
  }
  return (mfm[mfmBytePos] >> mfmBitPos) & 1;
}

struct {
  UINT8  synch[12];
  UINT32 IDAM; // ID Address Mark 0xA1A1A1FE
  UINT8  track;
  UINT8  side;
  UINT8  sector;
  UINT8  sectorSizeID;  // 128 << N
  UINT16 headerCRC;
  UINT8  n4E[22];  
  UINT8  n00[12];
  UINT32 DAM;  // Data Address Mark 0xA1A1A1FB
  UINT8  data[512]; // varies
  UINT16 dataCRC;
  UINT8  fill[54];  // 4E
} sector;

FILE* fDiscImage;

int decodeMFMtrack() {  // returns errorCount
  int errorCount = 0;

  mfmBytePos = 0; // could skip 130+ bytes header
  mfmBitPos = 0;

  int data = 0;
  int dataMFMbits = 0;
  int dataBits = 0;

  UINT8* pSector = NULL; //  sector.synch; // to look at pre-header synch
  sector.sectorSizeID = 0;  // assume small until overwritten

  ULONG64 mfmBits = 0;

  while (mfmBytePos < (int)mfmTrackBytes) {
    mfmBits <<= 1;
    int mfmBit = getMfmBit();
    mfmBits |= mfmBit;

    if (mfmBits == 0x4489448944895554) { // IDAM  A1 A1 A1 FE with missing clocks
      sector.IDAM = 0xFEA1A1A1;  // for header CRC calc
      pSector = &sector.track; // next byte
      dataMFMbits = 1; // current mfmBit is a data bit: clock | data | clock | data ...
      dataBits = -1;
    } else if (mfmBits == 0x4489448944895545) { // DAM  A1 A1 A1 FB with missing clocks
      sector.DAM = 0xFBA1A1A1;  // for header CRC calc
      pSector = &sector.data[0]; // next byte
      dataMFMbits = 1; // current mfmBit is a data bit: clock | data | clock | data ...
      dataBits = -1;
    }

    if (pSector && dataMFMbits++ & 1) {
      data <<= 1;
      data |= mfmBit;
      if (!(++dataBits % 8) && dataBits > 0)
        *pSector++ = data;
    }

    int sectorLen = min(128 << sector.sectorSizeID, 512);

    // could check n00s, header CRC

    if (pSector > &sector.data[sectorLen] + sizeof(sector.dataCRC)) {
      pSector = NULL;
      static int lastSide = -1, lastTrack = -1;
      if (sector.side != lastSide) printf("Side %d ", lastSide = sector.side);
      if (sector.track != lastTrack) printf("\nTrack %2d ", lastTrack = sector.track);      
      if (sector.sector == 1) printf("Len %3d Sectors %d ", sectorLen, 1);
      else  printf("%d ", sector.sector);

      UINT16 dCRC = crc16_block((UINT8*)&sector.DAM, sizeof(sector.DAM) + sectorLen);
      dCRC = _byteswap_ushort(dCRC);
      if (dCRC != sector.dataCRC) {
        if (sector.side > 1 || sector.track > 39 || sector.sector > 9 || sectorLen != 512) {
          printf("skipping intersector junk\n");
          continue;
        }
        printf("\tCRC %4X != %4X\n", dCRC, sector.dataCRC);
        ++errorCount;
      }

      if (fDiscImage) fwrite(sector.data, 1, sectorLen, fDiscImage);

      if (sector.sector == 9) return errorCount;  // don't scan through garbage (could check BPB)
    }
  }
  return errorCount;
}

int getTrackData(void) {
  mfmTrackBytes = 0;
  do {
    DWORD gotBytes = 0;
    if (!ReadFile(hCom, mfm + mfmTrackBytes, sizeof(mfm), &gotBytes, NULL)) break;
    mfmTrackBytes += gotBytes;
    Sleep(700);
  } while (rxRdy());

  if (mfmTrackBytes < 7000) printf("Got %d, status %c\n", mfmTrackBytes, mfmTrackBytes ? mfm[0] : '0');
  // TODO: if get 0 bytes then retry

  return decodeMFMtrack(); // error count
}

unsigned int histogram[80 * 2 * 6];

void plotHistogram() {
  sendCmd('H');
  DWORD gotBytes;
  if (!ReadFile(hCom, histogram, sizeof(histogram), &gotBytes, NULL)) return;

  if (histogram[0] != 0xBE66A7ED) { // marker
    printf("Synch err\n");
    return; // or scan and shift buffer
  }

  x = 0;
  for (DWORD x = 1; x < gotBytes / sizeof(int); x++) {
    if (x == bitTime1p5 || x == bitTime2p5 || x == bitTime3p5) { // time threshold markers
      int y = 64;
      while (y--) SetPixel(console, x, yOfs - y, RGB(255, 255, 0));
    }
    plot((int)(log((double)histogram[x] + 1) * 20));
  }
}

int main() {
  openSerial("COM10");
  identify();
  adjustTiming(); // TODO: dynamic based on histogram
  consoleGraphInit();
  crc16_init();

  char imgFileName[] = "floppy.A.img";
  
  while (1) {
    while (!_kbhit());
    char cmd = toupper(_getch());
    switch (cmd) { 
      case 'C': // clear screen
      case ' ':
        InvalidateRect(consoleWnd, NULL, true);
        system("CLS");
        break;

      case 'F' : // copy to .IMG File
        if (fopen_s(&fDiscImage, imgFileName, "wb")) {
          printf("Can't open %s\n", imgFileName);
          break;
        }
        ++imgFileName[7];
        sendCmd('Z');  Sleep(40 * 5); // seek to track 00
        for (int track = 0; track < 40; track++) {
          for (int side = 0; side <= 1; side++) {
            if (side) sendCmd('T'); else sendCmd('R');
            if (track == 0 && side == 0) Sleep(500); // motor spin up
            int errors = getTrackData();
            plotHistogram();
            if (_kbhit()) break;            
          }
          sendCmd('S'); // Step
          if (_kbhit()) break;
        }
        if (fDiscImage) {fclose(fDiscImage); fDiscImage = 0;}
        break;

      case 'H': plotHistogram(); break;
      case 'I': identify(); break;

      default: 
        sendCmd(cmd); 
        if (cmd == 'R' || cmd == 'T' || cmd == 'D') {
          Sleep(500); // motor spin up
          getTrackData();
          plotHistogram();
          sendCmd('S');
        }
        break;
    }
  }
}
