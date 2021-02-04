#include "RS232c.h"
#include <Windows.h>
#include <cstdint>
#include <iostream>
#include <cstring>
#include <ctime>

using namespace std;
#define READBUFFERSIZE 256

void wait_QueryPerformance(double finishtime, LARGE_INTEGER freq);

int main(int argc, char* argv[]) {
	RS232c axisrobot;
	char buf[256];
	char reply[256];
	LARGE_INTEGER freq, start, end;
	char modes[][10] = { "@SRVO", "@START", "@ORG" };
	char command[256] = "";
	double maintime = 0;

	if (!axisrobot.Connect("COM6", 38400, 8, ODDPARITY, 0, 0, 0, 20000, 20000)) {
		cout << "No connect" << endl;
		return 1;
	}
	if (!QueryPerformanceFrequency(&freq)) return 0;
	//ŠJŽn
	if (!QueryPerformanceCounter(&start)) return 0;
	snprintf(command, READBUFFERSIZE, "%s%d.1\r\n", modes[0], 1);
	axisrobot.Send(command);
	axisrobot.Read_CRLF(buf, 256);
	cout << "SERVO ON" << endl;
	wait_QueryPerformance(1, freq);
	axisrobot.Send("@?D18.1\r\n");
	axisrobot.Read_CRLF(reply, 256);
	axisrobot.Read_CRLF(buf, 256);

	snprintf(command, READBUFFERSIZE, "%s.1\r\n", modes[2]);
	axisrobot.Send(command);
	axisrobot.Read_CRLF(buf, 256);
	axisrobot.Read_CRLF(buf, 256);
	cout << "ORG START" << endl;
	memset(command, '\0', READBUFFERSIZE);
	wait_QueryPerformance(2.0, freq);

	
	
	srand(time(NULL));
	int pointno;
	//wait_QueryPerformance(0.1, freq);
	for (size_t i = 0; i < 5; i++)
	{
		pointno = rand()%15 + 1;
		snprintf(command, READBUFFERSIZE, "%s%d.1\r\n", modes[1], pointno);
		axisrobot.Send(command);
		axisrobot.Read_CRLF(buf, 256);
		axisrobot.Read_CRLF(buf, 256);
		while (true)
		{
			axisrobot.Send("@?D18.1\r\n");
			axisrobot.Read_CRLF(reply, 256);
			if (reply[0] == 'D' && reply[6] == '0') break;
			axisrobot.Read_CRLF(reply, 256);
			if (reply[0] == 'D' && reply[6] == '0') break;
		}
	}
	axisrobot.Send("@?D18.1\r\n");
	axisrobot.Read_CRLF(reply, 256);
	axisrobot.Read_CRLF(buf, 256);

	snprintf(command, READBUFFERSIZE, "%s%d.1\r\n", modes[0], 0);
	axisrobot.Send(command);
	memset(command, '\0', READBUFFERSIZE);
	cout << "SERVO OFF" << endl;
}

void wait_QueryPerformance(double finishtime, LARGE_INTEGER freq) {
	LARGE_INTEGER waitstart, waitstop;
	double waittime = 0;
	QueryPerformanceCounter(&waitstart);
	while (waittime < finishtime)
	{
		QueryPerformanceCounter(&waitstop);
		waittime = (double)(waitstop.QuadPart - waitstart.QuadPart) / freq.QuadPart;
	}
}