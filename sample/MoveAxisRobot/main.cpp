#include "RS232c.h"
#include <Windows.h>
#include <cstdint>
#include <iostream>
#include <cstring>

using namespace std;
#define READBUFFERSIZE 256

int main(int argc, char* argv[]) {
	RS232c axisrobot;
	char buf[256];
	LARGE_INTEGER freq, start, end;
	char modes[][10] = { "@SRVO", "@START" };
	char command[256] = "";
	double time = 0;

	axisrobot.Connect("COM6", 38400, 8, ODDPARITY, 0, 0, 0, 20000, 20000);
	if (!QueryPerformanceFrequency(&freq)) return 0;
	//ŠJŽn
	if (!QueryPerformanceCounter(&start)) return 0;
	snprintf(command, READBUFFERSIZE, "%s%d.1\r\n", modes[0], 1);
	axisrobot.Send(command);
	cout << "SERVO ON" << endl;
	memset(command, '\0', READBUFFERSIZE);
	while (time < 10)
	{
		QueryPerformanceCounter(&end);
		time = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
	}
	snprintf(command, READBUFFERSIZE, "%s%d.1\r\n", modes[0], 0);
	axisrobot.Send(command);
	memset(command, '\0', READBUFFERSIZE);
	cout << "SERVO OFF" << endl;
}