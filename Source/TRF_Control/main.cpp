#include <iostream>
#include "TRF7970AEVM.h"

void print_buffer(unsigned char* data, int length) {
	for (int i = 0; i < length; ++i) {
		std::cout << data[i];
	}
	std::cout << std::endl;
}

int main() {
	unsigned char init1[] = "010A0003041001210000";
	unsigned char init2[] = "010C00030410002101020000";
	unsigned char init3[] = "0109000304F0000000";
	unsigned char init4[] = "0109000304F1FF0000";
	unsigned char cmd[] = "010C0003041802AA07020000";
	unsigned char recv[2100] = { 0 };
	int read;

	if (!InitSerial()) {
		std::cout << "Error initializing serial port" << std::endl;
		return -1;
	}

	//TRF initialization
	WriteData(init1, sizeof(init1) - 1);
	ReadData(recv, 2100, &read);
	print_buffer(recv, read);

	WriteData(init2, sizeof(init2) - 1);
	ReadData(recv, 2100, &read);
	print_buffer(recv, read);

	WriteData(init3, sizeof(init3) - 1);
	ReadData(recv, 2100, &read);
	print_buffer(recv, read);

	WriteData(init4, sizeof(init4) - 1);
	ReadData(recv, 2100, &read);
	print_buffer(recv, read);

	//Sending Actual Data
	for (int i = 0; i < 100; ++i) {
		WriteData(cmd, sizeof(cmd) - 1);
		ReadData(recv, 2100, &read);
		print_buffer(recv, read);
	}

	CloseSerial();

	//system("pause");
}
