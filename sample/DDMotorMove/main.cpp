#include "RS232c.h"
#include <Windows.h>
#include <cstdint>
#include <iostream>

using namespace std;
#define READBUFFERSIZE 256

int main(int argc, char* argv[]) {
	RS232c mbed;
	char buf[256];
	LARGE_INTEGER freq, start, end;
	char mode = 'R';
	int rpm = 10;
	char command[256] = "";
	double time = 0;

	mbed.Connect("COM4", 115200, 8, NOPARITY, 0, 0, 0, 5000, 20000);
	if (!QueryPerformanceFrequency(&freq)) return 0;
	//ŠJŽn
	if (!QueryPerformanceCounter(&start)) return 0;
	snprintf(command, READBUFFERSIZE, "%c,%d,\r", mode, rpm);
	mbed.Send(command);
	memset(command, '\0', READBUFFERSIZE);
	for (int i = 0; i < 10; i++)
	{
		QueryPerformanceCounter(&end);
		time = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		while (time<((double)i+1))
		{
			QueryPerformanceCounter(&end);
			time = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		}
		if (mode == 'R') mode = 'L';
		else mode = 'R';
		snprintf(command, READBUFFERSIZE, "%c,%d,\r", mode, rpm);
		mbed.Send(command);
		memset(command, '\0', READBUFFERSIZE);
	}

	mbed.Send("$F\r");
	cout << "SERVO OFF" << endl;
	system("pause");
}