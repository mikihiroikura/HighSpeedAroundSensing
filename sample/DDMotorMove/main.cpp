#include "RS232c.h"
#include <Windows.h>
#include <cstdint>
#include <iostream>

using namespace std;

int main(int argc, char* argv[]) {
	RS232c ddmotor;
	char buf[256];
	LARGE_INTEGER freq, start, end;

	ddmotor.Connect("COM3", 38400, 8,NOPARITY, 0, 0, 0, 20000, 20000);
	if (!QueryPerformanceFrequency(&freq)) return 0;

	//ŠJŽn
	if (!QueryPerformanceCounter(&start)) return 0;
	//ddmotor“ü—Í
	ddmotor.Send("$J+10\r");
	if (!QueryPerformanceCounter(&end)) return 0;
	double logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
	while (logtime<10.0)
	{
		if (!QueryPerformanceCounter(&end)) return 0;
		logtime = (double)(end.QuadPart - start.QuadPart) / freq.QuadPart;
		cout << "Time: " << logtime << endl;
	}

	//’âŽ~
	ddmotor.Send("$Q\r");
	system("pause");
}