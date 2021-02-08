#include "RS232c.h"
#include <Windows.h>
#include <cstdint>
#include <iostream>
#include <cstring>
#include <ctime>

using namespace std;
#define READBUFFERSIZE 256

void wait_QueryPerformance(double finishtime, LARGE_INTEGER freq);
void Read_Reply_toEND(RS232c* robot);

int main(int argc, char* argv[]) {
	RS232c axisrobot;
	char buf[256];
	char reply[256];
	LARGE_INTEGER freq, start, end;
	char modes[][10] = { "@SRVO", "@START", "@ORG" };
	char command[256] = "";
	double maintime = 0;
	bool comresult;

	if (!axisrobot.Connect("COM6", 38400, 8, ODDPARITY, 0, 0, 0, 20000, 20000)) {
		cout << "No connect" << endl;
		return 1;
	}
	if (!QueryPerformanceFrequency(&freq)) return 0;
	//開始
	if (!QueryPerformanceCounter(&start)) return 0;
	snprintf(command, READBUFFERSIZE, "%s%d.1\r\n", modes[0], 1);
	axisrobot.Send(command);
	Read_Reply_toEND(&axisrobot);
	cout << "SERVO ON" << endl;
	wait_QueryPerformance(1, freq);
	axisrobot.Send("@?D18.1\r\n");
	Read_Reply_toEND(&axisrobot);

	snprintf(command, READBUFFERSIZE, "%s.1\r\n", modes[2]);
	axisrobot.Send(command);
	cout << "ORG START" << endl;
	memset(command, '\0', READBUFFERSIZE);
	Read_Reply_toEND(&axisrobot);
	cout << "ORG STOP" << endl;
	wait_QueryPerformance(2.0, freq);

	
	
	srand(time(NULL));
	int pointno = -1, pointno_old = -1;
	//wait_QueryPerformance(0.1, freq);
	//for (size_t i = 0; i < 5; i++)
	//{
	//	while (pointno == pointno_old) {
	//		pointno = rand() % 15 + 1;
	//	}
	//	snprintf(command, READBUFFERSIZE, "%s%d.1\r\n", modes[1], pointno);
	//	axisrobot.Send(command);
	//	/*axisrobot.Read_CRLF(buf, 256);
	//	axisrobot.Read_CRLF(buf, 256);
	//	while (true)
	//	{
	//		axisrobot.Send("@?D18.1\r\n");
	//		axisrobot.Read_CRLF(reply, 256);
	//		if (reply[0] == 'D' && reply[6] == '0') break;
	//		axisrobot.Read_CRLF(reply, 256);
	//		if (reply[0] == 'D' && reply[6] == '0') break;
	//	}*/
	//	Read_Reply_toEND(&axisrobot);
	//	pointno_old = pointno;
	//}

	srand(time(NULL));
	int axisspeed = 100;
	int axisposition = 200000;
	int initaxispos = 600;
	char controlcommand[READBUFFERSIZE];
	//位置と速度のランダム設定
	for (size_t i = 0; i < 11; i++)
	{
		if (initaxispos == 600) initaxispos = 0;
		else if (initaxispos == 0) initaxispos = 600;
		axisposition = (initaxispos + rand() % 100 + 1) * 100; //0~100 or 600~700
		axisspeed = (rand() % 10 + 1) * 10; //10~100で10刻み

		//コマンド送信
		snprintf(controlcommand, READBUFFERSIZE, "@S_17.1=%d\r\n", axisspeed);
		axisrobot.Send(controlcommand);
		memset(controlcommand, '\0', READBUFFERSIZE);
		Read_Reply_toEND(&axisrobot);
		snprintf(controlcommand, READBUFFERSIZE, "@START17#P%d.1\r\n", axisposition);
		axisrobot.Send(controlcommand);
		memset(controlcommand, '\0', READBUFFERSIZE);
		Read_Reply_toEND(&axisrobot);
	}
	

	//comresult = axisrobot.Send("@S_17.1=30\r\n");
	//Read_Reply_toEND(&axisrobot);
	//comresult = axisrobot.Send("@START17#P20000.1\r\n");
	//Read_Reply_toEND(&axisrobot);

	snprintf(command, READBUFFERSIZE, "%s%d.1\r\n", modes[0], 0);
	axisrobot.Send(command);
	Read_Reply_toEND(&axisrobot);
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

void Read_Reply_toEND(RS232c *robot) {
	char bufreply[256];
	while (true)
	{
		robot->Read_CRLF(bufreply, 256);
		if (bufreply[0] == 'E' || bufreply[0] == 'O') {
			break;
		}
	}
}