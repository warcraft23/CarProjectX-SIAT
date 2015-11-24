#include <stdio.h>


#include "car_auth.h"


int main(int argc,char ** argv) {
	//printf("MSG_LEN : %d",MSG_LEN);
	CarAuth car_auth(1);
	SYSTEMTIME sysTime = { 0 };
	sysTime = car_auth.getCurrentTimestamp();
	printf("NOW %d/%d/%d %d:%d:%d", sysTime.wYear,sysTime.wMonth,sysTime.wDay,sysTime.wHour,sysTime.wMinute,sysTime.wSecond);
	getchar();
	return 0;
}