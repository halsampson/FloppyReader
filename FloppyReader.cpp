// Floppy.cpp: Get data from FDD LMF4120 MCU

#include <Windows.h>
#include <stdio.h>
#include <conio.h>
#include <math.h>


void showError(void) {
  DWORD errCode = GetLastError();
  LPVOID lpMsgBuf;
  FormatMessage(
    FORMAT_MESSAGE_ALLOCATE_BUFFER |
    FORMAT_MESSAGE_FROM_SYSTEM |
    FORMAT_MESSAGE_IGNORE_INSERTS,
    NULL,
    errCode,
    MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
    (LPTSTR)&lpMsgBuf,
    0, NULL);

  printf(": %s\n", (char*)lpMsgBuf);
  LocalFree(lpMsgBuf);
}

HANDLE hCom;

HANDLE openSerial(const char* portName) {
  char portDev[16] = "\\\\.\\";
  strcat_s(portDev, sizeof(portDev), portName);

  hCom = CreateFile(portDev, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL); // better OVERLAPPED
  if (hCom == INVALID_HANDLE_VALUE) return NULL;

#if 1 // configure far end bridge COM port - only for bridges - could check endpoint capabilites??
  DCB dcb = { 0 };
  dcb.DCBlength = sizeof(DCB);
  dcb.BaudRate = 921600; // 2000000; //  115200;
  // PL2303HX:  Divisor = 12 * 1000 * 1000 * 32 / baud --> 12M, 6M, 3M, 2457600, 1228800, 921600, ... baud
  // FTDI 3 MHz / (n + 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875; where n is an integer between 2 and 16384)
  // Note: special cases n = 0 -> 3 MBaud; n = 1 -> 2 MBaud; Sub-integer divisors between 0 and 2 not allowed.
  dcb.ByteSize = 8;
  dcb.StopBits = 1;
  dcb.fBinary = TRUE; // no EOF check
#endif  

  if (!SetCommState(hCom, &dcb)) {
    printf("Can't set baud");
    showError();
  }

  if (!SetupComm(hCom, 16384, 16)) printf("Can't SetupComm"); // Set size of I/O buffers (max 16384 on Win7)

  // USB bulk packets arrive at 1 kHz rate
  COMMTIMEOUTS timeouts = { 0 };  // in ms
  timeouts.ReadIntervalTimeout = 500;
  timeouts.ReadTotalTimeoutConstant = 500;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  if (!SetCommTimeouts(hCom, &timeouts))  printf("Can't SetCommTimeouts");

  return hCom;
}

int rxRdy(void) {
  COMSTAT cs;
  DWORD commErrors;
  if (!ClearCommError(hCom, &commErrors, &cs)) return -1;
  if (commErrors)
    printf("\n\rCommErr %X\n", commErrors); // 8 = framing (wrong baud rate); 2 = overrurn; 1 = overflow
  return cs.cbInQue;
}

HWND consoleWnd;
HDC console;
int x, maxX, yOfs;

void consoleGraph() {
  char consoleWindowTitle[128];
  GetConsoleTitle(consoleWindowTitle, sizeof(consoleWindowTitle));
  consoleWnd = FindWindow(NULL, consoleWindowTitle);
  console = GetDC(consoleWnd);
  RECT rect;
  GetClientRect(consoleWnd, &rect);
  maxX = rect.right - rect.left;
  yOfs = rect.bottom - 1;
}

void plot(int y) {
  SetPixel(console, x, yOfs - y, RGB(0, 255, 0));
  if (++x >= maxX) x = 0;
}

void sendCmd(char ch) {
  WriteFile(hCom, &ch, 1, NULL, NULL);
}


// x**16+x**12+x**5+1, with the initial sum preset to all 1's

UINT16 crc_ccitt[256]; // crc16 table

void crc16_init() { // crc16 init table
  static bool inited = false;
  if (inited)
    return;

  for (int i = 0; i < 256; i++) {
    UINT16 w = i << 8;
    for (int j = 0; j < 8; j++)
      w = (w << 1) ^ ((w & 0x8000) ? 0x1021 : 0);
    crc_ccitt[i] = w;
  }
  inited = true;
}


inline UINT16 crc16(UINT16 crc, UINT8 b) { // calc crc16 for 1 byte
  crc = (crc << 8) ^ crc_ccitt[((crc >> 8) & 0xff) ^ b];
  return crc;
}

UINT16 crc16_block(UINT8* ptr, UINT16 len, UINT16 crc) { // calc crc16 for block
  while (len--)
    crc = crc16(crc, *ptr++);
  return crc;
}


UINT8 mfm[32768];
DWORD mfmBytes;

int bytePos, bitPos;

int getBit() {
  if (--bitPos < 0) {
    bitPos = 7;
    ++bytePos;
  }
  return (mfm[bytePos] >> bitPos) & 1;
}

UINT8 decoded[12000];

struct {
  UINT32 IDAM; // ID Address Mark 0xA1A1A1FE
  UINT8 track;
  UINT8 side;
  UINT8 sector;
  UINT8 sectorSizeID;  // 128 << N
  UINT16 headerCRC;
  UINT8 n4E[22];  
  UINT8 n00[12];
  UINT32 DAM;  // Data Address Mark 0xA1A1A1FB
  UINT8 data[512]; // varies
  UINT16 dataCRC;
  UINT8 fill[54];  // 4E
} sector;


void decodeMFM(int phaseOfs = 1) {
  bytePos = 0; // could skip 130+ bytes header
  bitPos = 0;

  int data = 0;
  int dataMFMbits = 0;
  int dataBits = 0;

  UINT8* pSector = NULL;
  sector.sectorSizeID = 0;  // assume small until overwritten

  ULONG64 mfmBits = 0;

  while (bytePos < mfmBytes) {
    mfmBits <<= 1;
    int mfmBit = getBit();
    mfmBits |= mfmBit;

    if (mfmBits == 0x4489448944895554) { //  A1 A1 A1 FE
      sector.IDAM = 0xFEA1A1A1;  // for header CRC calc
      // printf("\nIDAM %d ", bitPos);
      pSector = &sector.track; // next byte
      dataMFMbits = 1; // current mfmBit is a data bit: clock | data | clock | data ...
      dataBits = -1;
    }

    if (pSector && dataMFMbits++ & 1) {
      data <<= 1;
      data |= mfmBit;
      if (!(++dataBits % 8) && dataBits > 0)
        *pSector++ = data;
    }

    int sectorLen = 128 << sector.sectorSizeID;

    if (pSector > &sector.data[sectorLen] + sizeof(sector.dataCRC)) {
      pSector = NULL;
      printf("Side %d ", sector.side);
      printf("Track %2d ", sector.track);
      printf("Sector %d ", sector.sector); 
      if (sector.sector == 1) printf("Len %3d ", sectorLen);

      if (sector.DAM != 0xFBA1A1A1) printf("DAM! %8X ", sector.DAM);

      UINT16 dCRC = crc16_block((UINT8*)&sector.DAM, sizeof(sector.DAM) + sectorLen, 0xFFFF);
      dCRC = _byteswap_ushort(dCRC);
      if (dCRC != sector.dataCRC)
        printf("CRC! %4X %4X ", dCRC, sector.dataCRC);

      // if (datab >= ' ' && datab <= 'z')
      //  printf("%c", datab);
    }
  }
}

void getData(void) {
  Sleep(500 + 200); // spin up + index detect

  ReadFile(hCom, mfm, sizeof(mfm), &mfmBytes, NULL);
  // printf("\r%d ", mfmBytes);

  decodeMFM();
}

  
void plotHistogram() {
  InvalidateRect(consoleWnd, NULL, TRUE); Sleep(16);
  for (int line = 16; line--;) printf("\n");

  sendCmd('H');
  unsigned int hist[80 * 5 / 3 * 9 / 2];
  DWORD gotBytes;
  ReadFile(hCom, hist, sizeof(hist), &gotBytes, NULL);

  x = 0;
  for (DWORD x = 0; x < gotBytes / sizeof(int); x++)
    plot((int)(log(hist[x] + 1) * 20));
}


int main() {
  openSerial("COM10");
  consoleGraph();
  crc16_init();
  
  while (1) {
    while (!_kbhit());
    char cmd = toupper(_getch());
    switch (cmd) { 
      case 'H': plotHistogram(); break;
      case 'D': decodeMFM(0); break;

      default: 
        sendCmd(cmd); 
        if (cmd == 'R' || cmd == 'T' || cmd == 'D') {
          getData();
          sendCmd('S');
        }
        break;
    }
  }
}
