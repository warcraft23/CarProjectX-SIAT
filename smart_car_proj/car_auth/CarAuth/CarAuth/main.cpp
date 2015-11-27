#include <iostream>
#include "car_auth.h"
using namespace std;

int main(int argc,char ** argv) {
	//printf("MSG_LEN : %d",MSG_LEN);
	CarAuth car_auth(1);
	Msg_Item item;
	char * data = (char *)&item;
	int i = 0;
	//数据包初始化
	for (; i < MSG_LEN;i++)
	{
		if (i < 40)
			data[i] = i;
		else
			data[i] = 0;
	}
	//填充数据包
	car_auth.packPkt(&item);
	Sleep(3000);
	item.index = 1;
	item.chksum = car_auth.getCRC32((char *)&item, MSG_LEN - sizeof(unsigned int));
	int ret=car_auth.checkMsg(item);
	if (ret == 1)
		cout << "VALID !" << endl;
	else if (ret == 0)
		cout << " INVALID! " << endl;

	
	return 0;
}
