#ifndef TRF7970AEVM_H
#define TRF7970AEVM_H

bool InitSerial();

bool WriteData(const unsigned char* data, int length);

bool ReadData(unsigned char* buffer, int length, int* read);

void CloseSerial();

#endif
