#ifdef _WIN32
#include <Windows.h>

HANDLE hndlCOM;
DCB dcbMasterInitState;
#else
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>

int USB;
termios tty, tty_old;
#endif

bool InitSerial() {
#ifdef _WIN32
	hndlCOM = CreateFile("\\\\.\\COM1",
		GENERIC_READ | GENERIC_WRITE,
		0,
		0,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		0);

	if (hndlCOM == INVALID_HANDLE_VALUE) {
		return false;
	}

	if (!PurgeComm(hndlCOM, PURGE_TXABORT | PURGE_RXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR)) {
		return false;
	}

	if (!GetCommState(hndlCOM, &dcbMasterInitState)) {
		return false;
	}

	DCB dcbMaster = dcbMasterInitState;

	dcbMaster.BaudRate = 115200;
	dcbMaster.Parity = NOPARITY;
	dcbMaster.ByteSize = 8;
	dcbMaster.StopBits = ONESTOPBIT;
	dcbMaster.fAbortOnError = TRUE;
	if (!SetCommState(hndlCOM, &dcbMaster)) {
		return false;
	}
	Sleep(60);
	return true;
#else
	USB = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	if (USB < 0) {
		return false;
	}

	memset(&tty, 0, sizeof(tty));

	if (tcgetattr(USB, &tty) != 0) {
		return false;
	}

	tty_old = tty;

	cfsetospeed(&tty, (speed_t)B115200);
	cfsetispeed(&tty, (speed_t)B115200);

	tty.c_cflag     &=  ~PARENB;            // Make 8n1
	tty.c_cflag     &=  ~CSTOPB;
	tty.c_cflag     &=  ~CSIZE;
	tty.c_cflag     |=  CS8;

	tty.c_cflag     &=  ~CRTSCTS;           // no flow control
	tty.c_cc[VMIN]   =  1;                  // read doesn't block
	tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
	tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines

	cfmakeraw(&tty);

	tcflush(USB, TCIFLUSH);
	if (tcsetattr(USB, TCSANOW, &tty) != 0) {
		return false;
	}
	usleep(60000);
	return true;
#endif
}

bool WriteData(const unsigned char* data, int length) {
#ifdef _WIN32
	DWORD written;
	if (!WriteFile(hndlCOM, data, length, &written, NULL)) {
		return false;
	}
	return length == written;
#else
	int written = write(USB, data, length);
	usleep(8000);
	return length == written;
#endif
}

bool ReadData(unsigned char* data, int length, int* nread) {
#ifdef _WIN32
	return ReadFile(hndlCOM, data, length, reinterpret_cast<LPDWORD>(nread), NULL) != 0;
#else
	*nread = read(USB, data, length);
	return true;
#endif
}

void CloseSerial() {
	WriteData(reinterpret_cast<const unsigned char*>("00F"), 3);
#ifdef _WIN32
	SetCommState(hndlCOM, &dcbMasterInitState);
	CloseHandle(hndlCOM);
#else
	tcsetattr(USB, TCSANOW, &tty_old);
	close(USB);
#endif
}
