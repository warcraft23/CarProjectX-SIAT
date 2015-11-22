//#include "stdafx.h"
#include "Aria.h"

#define OUTDOOR 1
#define UNTEST_NET 1
#define ENABLE_ECOM 0


#define ROBOT_NUM	10
#define ARG_GRP_LEN	4
#define MSG_QUEUE_LEN	10
#define SPORT 9999

int slaveGpsTest = false;

class Robot_Obj {
public:
	ArRobot myRobot;
	ArArgumentParser *myParser;
	ArRobotConnector *myRobotConnector;
	ArLaserConnector *myLaserConnector;
	ArSonarDevice *mySonar;

	Robot_Obj(int argc, char *argv[]);
	~Robot_Obj();
};

Robot_Obj::Robot_Obj(int argc, char **argv)
{
	ArLog::log(ArLog::Terse, "enter --- %s\n", __FUNCTION__);

	myParser = new ArArgumentParser(&argc, argv);
	myParser->loadDefaultArguments();
	myRobotConnector = new ArRobotConnector(myParser, &myRobot);
	mySonar = new ArSonarDevice();

	// Try connecting to the robot.
	if (!myRobotConnector->connectRobot(&myRobot)){
		// Error!
		ArLog::log(ArLog::Verbose, "Error, could not connect to robot.\n");
		myRobotConnector->logOptions();
		Aria::exit(1);
	}

	// add the sonar to the robot
	myRobot.addRangeDevice(mySonar);

	// turn on the motors, turn off amigobot sounds
	myRobot.comInt(ArCommands::ENABLE, 1);
	myRobot.comInt(ArCommands::SOUNDTOG, 0);

	myRobot.comInt(ArCommands::SETV, 1000);
	myRobot.comInt(ArCommands::SETA, 50);

	// Run the ArRobot processing/task cycle thread.
	myRobot.runAsync(true);

	myLaserConnector = new ArLaserConnector(myParser, &myRobot, myRobotConnector);

	// Parse command line arguments (there may be arguments specifying 
	// what lasers to try to connect to)
	if (!Aria::parseArgs()) {
		Aria::logOptions();
		Aria::exit(2);
	}

	// Try connecting to all lasers specified in the robot's parameter file
	// and in command line arguments
	if (!myLaserConnector->connectLasers()) {
		ArLog::log(ArLog::Verbose, "Error, could not connect to lasers.\n");
		Aria::logOptions();
		Aria::exit(3);
	}

	ArLog::log(ArLog::Terse, "exit --- %s\n", __FUNCTION__);
}

Robot_Obj::~Robot_Obj()
{
	ArLog::log(ArLog::Normal, "Robot exit ....\n");
	if (myRobot.isRunning()) {
		delete myParser;
		delete myRobotConnector;
		delete myLaserConnector;
		delete mySonar;
	}
}

enum { TYPE_GENERIC = 1, TYPE_SPECIAL = 2, TYPE_DIRECTION = 4,
	TYPE_ECOMPASS = 8};

struct Msg_Item {
	int 	index;
	char 	type;
	short 	key;
	int	 	val;
	union {
		struct color {
			short r;
			short g;
			short b;
		} color;
		struct array {
			int row;
			int col;
		} array;
		struct gps {
			double distance;
			int angle;
		}gps;
		struct ecompass {
			int angle;
		}ecompass;
		char buf[24];
	}u;
};

#define MSG_LEN (sizeof(struct Msg_Item))

class Robot_Msg {
public:
	struct Msg_Item myMsgQueue[MSG_QUEUE_LEN];
	int myMsgNumber;
	int myMsgQueueHead, myMsgQueueTail;
	bool myOverflowFlag;
	Robot_Msg();
	~Robot_Msg() { };
	int msgItemEnqueue(struct Msg_Item *item);
	struct Msg_Item *msgItemDequeue();
	int isFullItemQueue();
	int isEmptyItemQueue();
	int getMsgnum() { return myMsgNumber; }
};

Robot_Msg::Robot_Msg()
{
	myMsgNumber = 0;
	myMsgQueueHead = 0;
	myMsgQueueTail = 0;
	myOverflowFlag = false;
}

int Robot_Msg::msgItemEnqueue(struct Msg_Item *item)
{
	int ret = 0;

	if (!isFullItemQueue()) {
		myMsgQueue[myMsgQueueHead++] = *item;
		myMsgNumber++;
		if (MSG_QUEUE_LEN == myMsgQueueHead) {
			myMsgQueueHead = 0;
		}
	} else {
		ret = -1;
		ArLog::log(ArLog::Terse, "msgqueue is full.\n");
	}

	return ret;
}

struct Msg_Item* Robot_Msg::msgItemDequeue()
{
	struct Msg_Item *item = NULL;

	if (!isEmptyItemQueue()) {
		item = &myMsgQueue[myMsgQueueTail++];
		myMsgNumber--;
		if (MSG_QUEUE_LEN == myMsgQueueTail) {
			myMsgQueueTail = 0;
		}
	} else {
		ArLog::log(ArLog::Terse, "msgqueue is empty.\n");
	}

	return item;
}

int Robot_Msg::isFullItemQueue()
{
	return myMsgNumber == MSG_QUEUE_LEN ? 1 : 0;
}

int Robot_Msg::isEmptyItemQueue()
{
	return myMsgNumber == 0 ? 1 : 0;
}

void msg_test()
{
	int i = 0;
	Robot_Msg robot_msg;
	struct Msg_Item *item = NULL;

	for (; i < 11; i++) {
		robot_msg.myMsgQueue[i].key = i;
		if (robot_msg.msgItemEnqueue(&robot_msg.myMsgQueue[i]) < 0) {
			printf("queue is full.\n");
		}
	}

	i = 0;
	while (i++ < 13) {
		item = robot_msg.msgItemDequeue();
		printf("msg[%d] = %d\n", i++, item->key);
	}
}

class SmartCarCom
{
public:
	SmartCarCom(const std::string name);
	virtual ~SmartCarCom();

	void init();
	void init(DWORD BaudRate, BYTE Parity, BYTE ByteSize, BYTE StopBits);
	bool open();
	bool write(const char* pOutBuffer, DWORD size);
	bool read(char* pInBuffer, DWORD size);
	void close();
	static void test(std::string name);

private:
	HANDLE m_Com;
	DCB    m_ComConfig;
	std::string m_ComName;
};

SmartCarCom::SmartCarCom(std::string name) :
m_Com((HANDLE)-1), m_ComConfig(), m_ComName(name)
{
}

SmartCarCom::~SmartCarCom()
{ 
}

void SmartCarCom::init()
{
	//use the default com config
	SetupComm(m_Com, 100, 100); //input and output buffer size, 100 bytes

	COMMTIMEOUTS timeOuts;
	timeOuts.ReadIntervalTimeout = 100;
	timeOuts.ReadTotalTimeoutMultiplier = 400;
	timeOuts.ReadTotalTimeoutConstant = 100; //读取一次操作后就立即返回，而不管读取的是否是要求的字符

	timeOuts.WriteTotalTimeoutMultiplier = 500;
	timeOuts.WriteTotalTimeoutConstant = 100;
	SetCommTimeouts(m_Com, &timeOuts);
	
	GetCommState(m_Com, &m_ComConfig);
	m_ComConfig.BaudRate = CBR_115200;
	m_ComConfig.ByteSize = 8;
	m_ComConfig.Parity = NOPARITY;
	m_ComConfig.StopBits = ONESTOPBIT;
	SetCommState(m_Com, &m_ComConfig);

	PurgeComm(m_Com, PURGE_RXCLEAR | PURGE_TXCLEAR);

	ArLog::log(ArLog::Normal, "SmartCarCom: init COM %s",
		m_ComName.c_str());
}

void SmartCarCom::init(DWORD BaudRate, BYTE Parity, BYTE ByteSize, BYTE StopBits)
{
	SetupComm(m_Com, 100, 100); //input and output buffer size, 100 bytes

	COMMTIMEOUTS timeOuts;
	timeOuts.ReadIntervalTimeout = 100;
	timeOuts.ReadTotalTimeoutMultiplier = 0;
	timeOuts.ReadTotalTimeoutConstant = 100; //读取一次操作后就立即返回，而不管读取的是否是要求的字符

	timeOuts.WriteTotalTimeoutMultiplier = 500;
	timeOuts.WriteTotalTimeoutConstant = 100;
	SetCommTimeouts(m_Com, &timeOuts);

	GetCommState(m_Com, &m_ComConfig);
	m_ComConfig.BaudRate = BaudRate;
	m_ComConfig.ByteSize = ByteSize;
	m_ComConfig.Parity = Parity;
	m_ComConfig.StopBits = StopBits;
	SetCommState(m_Com, &m_ComConfig);

	PurgeComm(m_Com, PURGE_RXCLEAR | PURGE_TXCLEAR);
}

bool SmartCarCom::open()
{
	m_Com = CreateFile(m_ComName.c_str(), 
						GENERIC_READ | GENERIC_WRITE, // read and write access 
						0,							  // not share, must be NULL
						NULL,						  // security
						OPEN_EXISTING,				  // just open, not create
						0,							  // synchronization IO
						NULL						  // must be NULL
						);
	if (m_Com == (HANDLE)-1) {
		ArLog::log(ArLog::Terse, 
			"SmartCarCom: open COM port %s failed.", m_ComName.c_str());
		return false;
	}

	ArLog::log(ArLog::Normal, "SmartCarCom: open COM %s",
		m_ComName.c_str());

	return true;
}

//sync to write serial port
bool SmartCarCom::write(const char* pOutBuffer, DWORD size)
{
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	BOOL bWriteStat;

	ClearCommError(m_Com, &dwErrorFlags, &ComStat);
	bWriteStat = WriteFile(m_Com, pOutBuffer, size, &size, NULL);
	if (!bWriteStat) {
		ArLog::log(ArLog::Normal, "Failed to write COM port, %s\n",
			m_ComName);
		return false;
	}

	return true;
}

//sync to read serial port
bool SmartCarCom::read(char* pInBuffer, DWORD size)
{
	BOOL bReadStat;
	COMSTAT ComStat;
	DWORD dwErrorFlags;
	DWORD read_size = 0;
	DWORD retCnt = 0;
	DWORD temp = 0;

	if (size <= 0)
		return true;

	ClearCommError(m_Com, &dwErrorFlags, &ComStat);
	PurgeComm(m_Com, PURGE_RXCLEAR | PURGE_TXCLEAR);

	while (retCnt < size) {
		read_size = size - retCnt;
		bReadStat = ReadFile(m_Com, pInBuffer+retCnt, read_size, &temp, NULL);
		if (!bReadStat)
		{
			ArLog::log(ArLog::Terse, "Failed to read COM port, %s\n",
				m_ComName);
			return false;
		}
		//printf("%c: %c:%d %c:%d\n", *(pInBuffer+retCnt)[0], );
		retCnt += temp;
	}

	//PurgeComm(m_Com, PURGE_RXCLEAR | PURGE_TXCLEAR);

	return true;
}
void SmartCarCom::close()
{
	CloseHandle(m_Com);
}

void SmartCarCom::test(std::string name)
{
	SmartCarCom* com = new SmartCarCom(name);

	com->open();
	com->init();

	#pragma pack()
	struct data {
		char _rc;
		unsigned short _r;
		char _gc;
		unsigned short _g;
		char _bc;
		unsigned short _b;
		char _uc;
		unsigned short _u;
		char _enter;
		char _nextline;
	};

	struct data buf;
	
	int n = 100;
	while (1) {
		com->read((char*)&buf, sizeof(data));

		printf("read: %c:%d %c:%d %c:%d\n", 
			buf._rc, buf._r, buf._gc, buf._g, buf._bc, buf._b);
	}

	delete com;
}

#define PI                      3.14159265
#define EARTH_RADIUS            6378.137        //地球近似半径

class LogLatDistance {
public:
	LogLatDistance () {}
	~LogLatDistance () {}
	double radian(double d);
	double get_distance(double lat1, double lng1, double lat2, double lng2);
};

double LogLatDistance::radian(double d)
{
	return d * PI / 180.0;   //角度1? = π / 180
}

double LogLatDistance::get_distance(double lat1, double lng1, double lat2, double lng2)
{
	double radLat1 = radian(lat1);
	double radLat2 = radian(lat2);
	double a = radLat1 - radLat2;
	double b = radian(lng1) - radian(lng2);

	double dst = 2 * asin((sqrt(pow(sin(a / 2), 2) + cos(radLat1) * cos(radLat2) * pow(sin(b / 2), 2))));

	dst = dst * EARTH_RADIUS;
	dst = round(dst * 10000) / 10; /* meter unit */
	
	return dst;
}

class GPSPosition 
{
public:
	SmartCarCom *myGPSCom;
	double myLongitude, myLatidude;

	GPSPosition(std::string comPort);
	virtual ~GPSPosition() { }
	bool getGPSPosition(double *logitude, double *latitude);
	void testGPS();
};

GPSPosition::GPSPosition(std::string comPort)
{
	myGPSCom = new SmartCarCom(comPort.c_str());
	if (myGPSCom->open()) {
		myGPSCom->init(CBR_9600, NOPARITY, 8, ONESTOPBIT);
	} else {
		ArLog::log(ArLog::Terse, "GPSPosition: open COM %s failed.",
			comPort);
		return;
	}
}

#define GPS_BUF_SZ 100
bool GPSPosition::getGPSPosition(double *logitude, double *latitude)
{
	char buffer[GPS_BUF_SZ + 1] = { 0 };
	const char* buf = buffer;
	int A_index = 0, N_index = 0, E_index = 0;
	static double logitude_s = 0, latitude_s = 0;

	if (!myGPSCom->read(buffer, GPS_BUF_SZ)) {
		ArLog::log(ArLog::Terse, "GPSPosition: read COM port failed.");
		return false;
	}
	buffer[GPS_BUF_SZ] = '\0';
	while (*buf != '$')
		buf++;

	if (strncmp(buf, "$GNRMC", 6)) {
		return false;
	}

	buf += 6;
	std::string str_gps(buf, 0, 70);	

	A_index = str_gps.find_first_of('A');
	N_index = str_gps.find_first_of('N');
	E_index = str_gps.find_first_of('E');

	if (A_index > N_index || A_index > E_index || N_index > E_index) {
		ArLog::log(ArLog::Normal, 
			"A_i: %d, N_i: %d, E_i: %d; bad gps data: %s", 
		A_index, N_index, E_index, str_gps.c_str());
		return false;
	} else {
		ArLog::log(ArLog::Verbose, "gps: %s", str_gps.c_str());
	}

	std::string str_N_degree(str_gps, A_index + 2, 2);
	std::string str_N_minute(str_gps, A_index + 4, N_index - A_index - 5);
	std::string str_E_degree(str_gps, N_index + 2, 3);
	std::string str_E_minute(str_gps, N_index + 5, E_index - N_index - 6);
	
	myLatidude = atof(str_N_degree.c_str()) + atof(str_N_minute.c_str()) / 60;
	myLongitude = atof(str_E_degree.c_str()) + atof(str_E_minute.c_str()) / 60;;

	ArLog::log(ArLog::Verbose, "getGPSPosition: latitude=%f, logitude=%f",
		myLatidude, myLongitude);

	*latitude = myLatidude;
	*logitude = myLongitude;

	return true;
}

void GPSPosition::testGPS()
{
	double logitude, latitude;
	
	while (1) {
		getGPSPosition(&logitude, &latitude);
		ArUtil::sleep(1000);
	}
}

class ECompass
{
public:
	SmartCarCom *myECompassCom;
	int myMx, myMy, myMz;

	ECompass(std::string name);
	virtual ~ECompass() {}
	bool getAngle(int *angle);
	void testEcompass();
};

ECompass::ECompass(std::string comPort)
{
	myECompassCom = new SmartCarCom(comPort.c_str());
	if (myECompassCom->open()) {
		myECompassCom->init();
	} else {
		ArLog::log(ArLog::Terse, "ECompass: open COM %s failed.",
			comPort);
		return;
	}
}

#define ECOMPASS_BUF_SZ	36
#define ECOMPASS_WIGHT_SZ 18
#define WEIGHT_LEN	4

/* Z offset relative to X */
#define Z_OFFSET 6	

/* Y offset relative to Z */
#define Y_OFFSET 6
bool ECompass::getAngle(int *angle)
{
	char buffer[ECOMPASS_BUF_SZ + 1] = { 0 };
	const char* buf = buffer;
	int x_index, z_index, y_index;

	if (!myECompassCom->read(buffer, ECOMPASS_BUF_SZ)) {
		ArLog::log(ArLog::Terse, "ECompass: read COM port failed.");
		return false;
	}
	buffer[ECOMPASS_BUF_SZ] = '\0';
	while (*buf != 'X')
		buf++;

	std::string str_ecompass(buf, 0, ECOMPASS_WIGHT_SZ);	

	x_index = str_ecompass.find_first_of('X');
	z_index = str_ecompass.find_first_of('Z');
	y_index = str_ecompass.find_first_of('Y');

	if (z_index - x_index != Z_OFFSET || y_index - z_index != Y_OFFSET) {
		ArLog::log(ArLog::Normal, "bad ecompass data: %s",
			str_ecompass.c_str());
		return false;
	} else {
		ArLog::log(ArLog::Verbose, "ecompass: %s",
			str_ecompass.c_str());
	}

	std::string str_x(str_ecompass, x_index + 2, WEIGHT_LEN);
	std::string str_z(str_ecompass, z_index + 2, WEIGHT_LEN);
	std::string str_y(str_ecompass, y_index + 2, WEIGHT_LEN);

	myMx = std::atoi(str_x.c_str());
	if (buf[x_index + 1] == '-')
		myMx = -myMx;
	
	myMz = std::atoi(str_z.c_str());
	if (buf[z_index + 1] == '-')
		myMz = -myMz;
	
	myMy = std::atoi(str_y.c_str());
	if (buf[y_index + 1] == '-')
		myMy = -myMy;

	*angle= atan2(myMy, myMx) * (180 / PI) + 180;

	ArLog::log(ArLog::Verbose, 
			"Mx: %d, My: %d, Mz: %d, angle: %d",
			myMx, myMy, myMz, *angle);

	return true;
}


void ECompass::testEcompass()
{
	int angle;
	
	while (1) {
		if (getAngle(&angle)) {
			ArLog::log(ArLog::Normal, "cur angle: %d\n",
				angle);
		}

		ArUtil::sleep(100);
	}
}

#define MAX_SLAVE 16
class SendSlaveMsgThread : public ArASyncTask {
public:
	ArCondition myCondition;
	ArMutex myMutex;	
	ArSocket mySlaveSock[MAX_SLAVE];
	int mySlaveNr;
	Robot_Msg *mySubSlaveRobotMsg;
	struct Msg_Item *myMsgItem;	
	int condition_wait_flag;

	SendSlaveMsgThread();
	void setRobotMsg(Robot_Msg *robot_msg);	
	void msgEnqueue(struct Msg_Item *msg_item);
	void parseAndForwardMsg(struct Msg_Item *msg_item);	
	void *runThread(void *arg);	
};

SendSlaveMsgThread::SendSlaveMsgThread()
{
	memset(mySlaveSock, 0x00, sizeof(mySlaveSock));
	mySlaveNr = 0;
}

void SendSlaveMsgThread::msgEnqueue(struct Msg_Item *msg_item)
{
	if (mySubSlaveRobotMsg->isFullItemQueue()) {
		condition_wait_flag = 1;
		myCondition.wait();
	}		

	myMutex.lock();
	mySubSlaveRobotMsg->msgItemEnqueue(msg_item);	
	myMutex.unlock();
	
	myCondition.signal();
}

void SendSlaveMsgThread::parseAndForwardMsg(struct Msg_Item *msg_item)
{
	int i = 0;
	
#if 0
	if (msg_item->type == TYPE_DIRECTION) {	
		if (myOpState->leftright_flag) {
			msg_item->u.array.row = myCarMap->getMapRow();
			msg_item->u.array.col = myCarMap->getMapCol();
			ArLog::log(ArLog::Normal, 
				"%s, position, map[%2d][%2d] = %s", 
				myOpState->turn_left ? "Turn_left" : "Turn_right",
				msg_item->u.array.row, msg_item->u.array.col,
				myCarMap->map[msg_item->u.array.row][msg_item->u.array.col] ? 
				"White" : "Black");
		}
	}
#endif

	for (i = 0; i < mySlaveNr; i++) {
		if(mySlaveSock[i].write((void *)msg_item, MSG_LEN) == MSG_LEN)
			ArLog::log(ArLog::Verbose, 
				"Slave_Car: Send msg to another slave car %d", i);
		else {
			ArLog::log(ArLog::Terse, 
				"Slave_Car: Error sending msg to another slave car %d", i);
	  	}
	}
}

#define SAMPLE_DELAY_MS 200 
void *SendSlaveMsgThread::runThread(void *arg)
{
	ArLog::log(ArLog::Normal, "SendSlaveMsgThread ...\n");

	/* this thread loop for ever */
	for (;;) {
		/* handle special MSG: e.g. car direction */
		while (!mySubSlaveRobotMsg->isEmptyItemQueue()) {
			myMutex.lock();
			myMsgItem = mySubSlaveRobotMsg->msgItemDequeue();
			myMutex.unlock();

			if (condition_wait_flag) {
				condition_wait_flag = 0;
				myCondition.signal();
			}

			parseAndForwardMsg(myMsgItem);			
		}

		myCondition.wait();		
	}

	ArLog::log(ArLog::Terse, "SendSlaveMsgThread exit, oooh\n");

	return NULL;
}


enum {IDLE_GPS = 0, START_POINT, MIDDLE_POINT, END_POINT};

class RcvMasterMsgThread : public ArASyncTask {
public:
	ArCondition myCondition;
	ArMutex myMutex;
	Robot_Obj *myRobot_Obj;
	ArSocket  myMasterSock;
	Robot_Msg *myGeneralRobotMsg, *myDirectionRobotMsg;
	struct Msg_Item *myMsgItem;
	static int myCarSpeed;
	int condition_wait_flag;
	
#if ENABLE_ECOM
	ECompass *myECompass;
#endif
	ArMutex myMutexForStartGetGPSFlag;
	int myStartGetGPSFlag;
	double myLongitude, myLatidude;
	int delayGo;

	RcvMasterMsgThread();
	static int getCarSpeed();
#if ENABLE_ECOM
	void calibrateDir(struct Msg_Item *msg_item);
#endif
	void setRobotMsg(Robot_Obj *robot_obj, Robot_Msg *general_robot_msg);
	void msgEnqueue(struct Msg_Item *msg_item);
	void parseMsg(struct Msg_Item *msg_item);
	void *runThread(void *arg);	
};

RcvMasterMsgThread::RcvMasterMsgThread() 
{ 
	condition_wait_flag = 0;
#if 1
	myStartGetGPSFlag = IDLE_GPS;
	myLongitude = 0.0;
	myLatidude = 0.0;
	delayGo = 0;
#endif
}

int RcvMasterMsgThread::myCarSpeed = 0;
int RcvMasterMsgThread::getCarSpeed()
{ 
	return myCarSpeed; 
}

void RcvMasterMsgThread::setRobotMsg(Robot_Obj *robot_obj, 
	Robot_Msg *general_robot_msg)
{
	myRobot_Obj = robot_obj;
	myGeneralRobotMsg = general_robot_msg;		
}

#if ENABLE_ECOM
void RcvMasterMsgThread::calibrateDir(struct Msg_Item *msg_item)
{
	int angle, cur_angle = 0;

	angle = msg_item->u.ecompass.angle;
	myECompass->getAngle(&cur_angle);
	if (angle - cur_angle > 1) {
		/* turn left */
		myRobot_Obj->myRobot.comInt(ArCommands::DHEAD, cur_angle - angle);
		ArLog::log(ArLog::Normal, "calibrateDir left: angle=%d, cur_angle=%d",
			angle, cur_angle);
	} else if (cur_angle - angle > 1) {
		/* turn right */
		myRobot_Obj->myRobot.comInt(ArCommands::DHEAD, cur_angle - angle);
		ArLog::log(ArLog::Normal, "calibrateDir right: angle=%d, cur_angle=%d",
			angle, cur_angle);
	}	
}
#endif

void RcvMasterMsgThread::msgEnqueue(struct Msg_Item *msg_item)
{
	if (myGeneralRobotMsg->isFullItemQueue()) {
		condition_wait_flag = 1;
		myCondition.wait();
	}		

	myMutex.lock();
	myGeneralRobotMsg->msgItemEnqueue(msg_item);	
	myMutex.unlock();
	
	myCondition.signal();
}

#define DELAY_GO 4000
void RcvMasterMsgThread::parseMsg(struct Msg_Item *msg_item)
{
	ArLog::log(ArLog::Normal, "Slave parseMsg: idx=%d, type=%d, key=%d, val=%d",
		msg_item->index, msg_item->type, msg_item->key, msg_item->val);

	/* 
	* 1. parse the msg
	* 2. forfard msg to slave robot
	*/	
	if (msg_item->type == TYPE_GENERIC) {
		switch (msg_item->key) {
		case ArKeyHandler::UP:
		case ArKeyHandler::DOWN:
			#if OUTDOOR	
			myMutexForStartGetGPSFlag.lock();		
			if (myStartGetGPSFlag == IDLE_GPS) {
				/* 
				* slave needs to wait for DELAY_GO seconds, then begin to run.
				*/
				ArLog::log(ArLog::Normal, 
					"Slave Car RcvMasterMsgThread, sleep, for %dms",
						delayGo);
				ArUtil::sleep(delayGo);
				ArLog::log(ArLog::Normal, 
					"Slave Car RcvMasterMsgThread, wakeup, after %dms\n\n",
						delayGo);
				myStartGetGPSFlag = START_POINT; 
			}	
			myMutexForStartGetGPSFlag.unlock();
			#endif
			
			myRobot_Obj->myRobot.comInt(ArCommands::VEL, msg_item->val);
			myCarSpeed = abs(msg_item->val);
			break;
		#if 0
		case ArKeyHandler::LEFT:
		case ArKeyHandler::RIGHT:
			myRobot_Obj->myRobot.comInt(ArCommands::DHEAD, msg_item->val);
			break;
		#endif
		#if ENABLE_ECOM
		case 'c':
			calibrateDir(msg_item);
			break;
		#endif
		case 'p':
			myRobot_Obj->myRobot.comInt(ArCommands::ENABLE, msg_item->val);
			break;
		default:
			ArLog::log(ArLog::Terse, "Slave bad msg: idx=%d, type=%d, key=%d, val=%d",
				msg_item->index, msg_item->type, msg_item->key, msg_item->val);
		}
	}
}

void *RcvMasterMsgThread::runThread(void *arg)
{
	ArLog::log(ArLog::Normal, "RcvMasterMsgThread ...\n");

	for (;;) {
		while (!myGeneralRobotMsg->isEmptyItemQueue()) {
			myMutex.lock();
			myMsgItem = myGeneralRobotMsg->msgItemDequeue();
			myMutex.unlock();

			if (condition_wait_flag) {
				condition_wait_flag = 0;
				myCondition.signal();
			}		
			
			parseMsg(myMsgItem);			
		}

		myCondition.wait();
	}

	ArLog::log(ArLog::Normal, "RcvMasterMsgThread exit, oooh\n");

	return NULL;
}

class MonitorThread : public ArASyncTask {
public:
	ArCondition myCondition;
	ArMutex myMutex;
	Robot_Obj *myRobot_Obj;	
	Robot_Msg *myDirectionRobotMsg;
	struct Msg_Item *myMsgItem;
	int condition_wait_flag;
	GPSPosition *myGPSPosition;
#if ENABLE_ECOM
	ECompass *myEcompass;
#endif
	RcvMasterMsgThread *myRcvMasterMsgThread;
	LogLatDistance *myLogLatDistance;
	int sleepTime;

	MonitorThread();
	~MonitorThread(){}
	void setRobotMsg(Robot_Obj *robot_obj,
		Robot_Msg *direction_robot_msg);
	void msgEnqueue(struct Msg_Item *msg_item);
	bool gpsPositionMatch(double distance);
	void waitTurnDone(struct Msg_Item *msg_item);
	void adjustDir(int angle);	
	struct Msg_Item* getOneMsg();
	void getInitGPS();
	void getDist();
	void *runThread(void *arg);	
};

MonitorThread::MonitorThread() 
{ 
	condition_wait_flag = 0;
	myMsgItem = NULL;
	sleepTime = 3000;
}


void MonitorThread::setRobotMsg(Robot_Obj *robot_obj, Robot_Msg *direction_robot_msg)
{
	myRobot_Obj = robot_obj;
	myDirectionRobotMsg = direction_robot_msg;	
}

void MonitorThread::msgEnqueue(struct Msg_Item *msg_item)
{
	myMutex.lock();
	myDirectionRobotMsg->msgItemEnqueue(msg_item);	
	myMutex.unlock();
}

#define DISTANCE_DIFF (1.50000)
bool MonitorThread::gpsPositionMatch(double distance)
{
	int timeout = 3;
	static int i = 0;
	double cur_longitude, cur_latitude, cur_distance;

	#if !UNTEST_NET
	return 1;
	#endif

	if (distance < 0.20)
		return false;

	while (timeout--) {
		if (myGPSPosition->getGPSPosition(&cur_longitude, &cur_latitude)) {
			break;	
		}

		if (!timeout) {
			ArLog::log(ArLog::Terse, "gpsPositionMatch timeout");
			return false;
		}
	}

	if (myRcvMasterMsgThread->myLatidude < 0.2 || 
		myRcvMasterMsgThread->myLongitude < 0.2) {
		ArLog::log(ArLog::Normal, "%s: reference record wrong, myLatidude: %f, myLongitude: %f",
			__FUNCTION__,
			myRcvMasterMsgThread->myLatidude, myRcvMasterMsgThread->myLongitude);
		return false;
	}

	cur_distance = myLogLatDistance->get_distance(myRcvMasterMsgThread->myLatidude, 
		myRcvMasterMsgThread->myLongitude, cur_latitude, cur_longitude);
	if (abs(distance - cur_distance) < DISTANCE_DIFF || 
		cur_distance - distance > DISTANCE_DIFF) {
		/* reset myLatidude & myLongitude for next round calculation */
		myRcvMasterMsgThread->myLatidude = 0.0;
		myRcvMasterMsgThread->myLongitude = 0.0;
		ArLog::log(ArLog::Normal, "GPS Match, distance: %f, cur_distance: %f\n",
			distance, cur_distance);
		return true;
	} else {
		if (++i > 5) {
			ArLog::log(ArLog::Normal, "GPS not Match, distance: %f, cur_distance: %f\n",
				distance, cur_distance);
			i = 0;
		}
		return false;
	}
}

void MonitorThread::waitTurnDone(struct Msg_Item *msg_item)
{
	double old_angular, cur_angular;

	old_angular = myRobot_Obj->myRobot.getTh();
	myRobot_Obj->myRobot.comInt(ArCommands::DHEAD, msg_item->val);
	cur_angular = myRobot_Obj->myRobot.getTh();
	while (abs(abs(cur_angular) - abs(old_angular)) < abs(msg_item->val) - 2) {
		ArUtil::sleep(100);
		cur_angular = myRobot_Obj->myRobot.getTh();	
	}

	ArLog::log(ArLog::Normal, "old_angular: %f, cur_angular: %f",
		old_angular, cur_angular);
}

#if ENABLE_ECOM
void MonitorThread::adjustDir(int angle)
{
	int cur_angle = 0;

	myEcompass->getAngle(&cur_angle);
	if (angle - cur_angle > 1) {
		/* turn left */
		myRobot_Obj->myRobot.comInt(ArCommands::DHEAD, angle - cur_angle);
		ArLog::log(ArLog::Normal, "adjustDir left: angle=%d, cur_angle=%d",
			angle, cur_angle);
	} else if (cur_angle - angle > 1) {
		/* turn right */
		myRobot_Obj->myRobot.comInt(ArCommands::DHEAD, angle - cur_angle);
		ArLog::log(ArLog::Normal, "adjustDir right: angle=%d, cur_angle=%d",
			angle, cur_angle);
	}	
}
#endif
struct Msg_Item* MonitorThread::getOneMsg()
{
	struct Msg_Item *msg_item = NULL;
	
	if (!myDirectionRobotMsg->isEmptyItemQueue()) {
		myMutex.lock();
		msg_item = myDirectionRobotMsg->msgItemDequeue();
		myMutex.unlock();
#if OUTDOOR
		ArLog::log(ArLog::Verbose, "Slave getOneMsg: idx=%d, type=%d, key=%d, val=%d, "
			"distance=%d, angle=%d\n",
		msg_item->index, msg_item->type, msg_item->key, msg_item->val,
		msg_item->u.gps.distance, msg_item->u.gps.angle);
#else		
		ArLog::log(ArLog::Normal, "Slave getOneMsg: type=%d, key=%d, val=%d "
			"row=%d, col=%d\n",
		msg_item->type, msg_item->key, msg_item->val,
		msg_item->u.array.row, msg_item->u.array.col);
#endif
	}	

	return msg_item;
}

void MonitorThread::getDist()
{
	double log, lat, dst;

	if(myGPSPosition->getGPSPosition(&log,&lat)) {
		if (myRcvMasterMsgThread->myLongitude > 0.2 && 
			myRcvMasterMsgThread->myLatidude > 0.2) {
			dst = myLogLatDistance->get_distance(myRcvMasterMsgThread->myLatidude,
				myRcvMasterMsgThread->myLongitude, lat, log);
			ArLog::log(ArLog::Normal, "(%f, %f) -> (%f, %f) = %f",
				myRcvMasterMsgThread->myLongitude, myRcvMasterMsgThread->myLatidude,
				log, lat, dst);
		}
	}	
}

#define GPS_SAMPLES 50
void MonitorThread::getInitGPS()
{
	double gps[GPS_SAMPLES][2] = {0};
	double logitude = 0.0, latitude = 0.0;
	int i = 0, valid_cnt = 0;

	while (i++ < GPS_SAMPLES) {
		if(myGPSPosition->getGPSPosition(&gps[i][0], &gps[i][1])) {
			valid_cnt++;
			logitude += gps[i][0];
			latitude += gps[i][1];
		}
		ArUtil::sleep(1000);
	}

	myRcvMasterMsgThread->myLongitude = logitude / valid_cnt;
	myRcvMasterMsgThread->myLatidude = latitude / valid_cnt;

	ArLog::log(ArLog::Normal, "getInitGPS done...\n"
		"myLongitude=%f, myLatidude=%f\n\n",
		myRcvMasterMsgThread->myLongitude, myRcvMasterMsgThread->myLatidude);
}


/* once car moves, it needs to wait for 3s and begin get GPS data */
#define GPS_DELAY_PERIOD 3000
void *MonitorThread::runThread(void *arg)
{
	double distance = 0.10;
	int angle = 0;
	static int i = 0;

	ArLog::log(ArLog::Normal, "MonitorThread ...\n");
	if (slaveGpsTest) {
		double logitude = 0.0, latitude = 0.0;
		double tmp_logitude = 0.0, tmp_latitude = 0.0, distance = 0.0;

		while (1) {
			myGPSPosition->getGPSPosition(&logitude, &latitude);
			if (tmp_logitude > 0.2 && tmp_latitude > 0.2) {
				distance = myLogLatDistance->get_distance(tmp_latitude,
					tmp_logitude, latitude, logitude);
				ArLog::log(ArLog::Normal, "lat:lng (%f, %f) -> (%f, %f), dis=%f",
					tmp_latitude, tmp_logitude, latitude, logitude, distance);
				
			}	
			
			tmp_logitude = logitude;
			tmp_latitude = latitude;
			
			ArUtil::sleep(sleepTime);
		}
	}else {
		/* thread run periodicly */
		for (;;) {			
			/* get MSG sent by Master thread */
			if (!myMsgItem) {	
				myMsgItem = getOneMsg();
				if (myMsgItem) {
					distance = myMsgItem->u.gps.distance;			
					angle = myMsgItem->u.gps.angle;

					ArLog::log(ArLog::Normal, "getOneMsg:\n"
						"idx=%d, type=%d, key=%d, val=%d, distance=%f, angle=%d\n",
						myMsgItem->index, myMsgItem->type, myMsgItem->key, myMsgItem->val,
						myMsgItem->u.gps.distance, myMsgItem->u.gps.angle);
				}
			}

			/* get GPS data as reference position */
			if (myRcvMasterMsgThread->myStartGetGPSFlag == START_POINT) {
				ArUtil::sleep(sleepTime);
				
				myRcvMasterMsgThread->myMutexForStartGetGPSFlag.lock();
				#if UNTEST_NET
				myGPSPosition->getGPSPosition(&myRcvMasterMsgThread->myLongitude, 
					&myRcvMasterMsgThread->myLatidude);	
				#endif
				myRcvMasterMsgThread->myStartGetGPSFlag = MIDDLE_POINT;
				myRcvMasterMsgThread->myMutexForStartGetGPSFlag.unlock();
			}	
	
			/* judge the turning position and turning */
			if (myRcvMasterMsgThread->myStartGetGPSFlag == MIDDLE_POINT
				&& myMsgItem && gpsPositionMatch(distance)) {

				ArLog::log(ArLog::Normal, "Slave Car turn begin...");
				myRcvMasterMsgThread->myMutexForStartGetGPSFlag.lock();	
				myRcvMasterMsgThread->myStartGetGPSFlag = END_POINT;
				
				waitTurnDone(myMsgItem);

				ArLog::log(ArLog::Normal, "idx=%d, type=%d, key=%d, val=%d, distance=%f, angle=%d",
					myMsgItem->index, myMsgItem->type, myMsgItem->key, myMsgItem->val,
					myMsgItem->u.gps.distance, myMsgItem->u.gps.angle);
				
				//ArUtil::sleep(50);
				//adjustDir(angle);
				myMsgItem = NULL;

				myRcvMasterMsgThread->myStartGetGPSFlag = START_POINT;
				myRcvMasterMsgThread->myMutexForStartGetGPSFlag.unlock();
				ArLog::log(ArLog::Normal, "Slave Car turn over...\n\n");
			}
			ArUtil::sleep(100);

			/* for debug */
			#if UNTEST_NET
			if (++i > 5 && myRcvMasterMsgThread->myStartGetGPSFlag == MIDDLE_POINT) {
				getDist();
				i = 0;
			}
			#endif
		}
	}

	ArLog::log(ArLog::Normal, "MonitorThread exit, oooh\n");

	return NULL;
}

#define LEFT_RIGHT 	1
#define TURN_LEFT 	1
#define TURN_RIGHT 	1
class Operation_State {
public:
	ArMutex myMutex;
	bool leftright_flag;
	bool turn_left;
	bool turn_right;

	Operation_State(bool lr_flag = 0, bool t_left = 0, bool t_right = 0) {
		leftright_flag = lr_flag;
		turn_left = t_left;
		turn_right = t_right;
	}	
	virtual ~Operation_State() { }

	void updateOpState(bool lr_flag, bool t_left, bool t_right);	
};

void Operation_State::updateOpState(bool lr_flag, bool t_left, bool t_right)
{
	myMutex.lock();
	leftright_flag = lr_flag;
	turn_left = t_left;
	turn_right = t_right;
	myMutex.unlock();
}

#define MAP_DEFAULT_X 20
#define MAP_DEFAULT_Y MAP_DEFAULT_X
#define WHITE 1
#define BLACK 0
class CarMap 
{
public:
	int map[MAP_DEFAULT_X][MAP_DEFAULT_Y];	
	int map_row, map_col; /* initial map Row & Col */
	int row, col;
	bool row_increase, col_increase;
	bool row_axis, col_axis;
	int cur_color;

	CarMap(int m_row, int m_col, int r, int c, 
		bool r_axis, bool r_increase, 
		bool c_axis, bool c_increase);
	virtual ~CarMap() { }	
	int getMapRow() { return row;}
	int getMapCol() { return col;}	
	void incMapRow(); 
	void decMapRow(); 
	void incMapCol(); 
	void decMapCol(); 
	void dumpPosition();
};

void CarMap::incMapRow() 
{
	if (row >= map_row - 1) {
		ArLog::log(ArLog::Terse, "row=%d beyond ++++ ", row);
		return;
	}
	
	row++;
}

void CarMap::decMapRow() 
{ 
	if (row <= 0) {
		ArLog::log(ArLog::Terse, "row=%d beyond ++++ ", row);
		return;
	}
	
	row--;
}

void CarMap::incMapCol() 
{
	if (col >= map_col - 1) {
		ArLog::log(ArLog::Terse, "col=%d beyond ---- ", col);
		return;
	}
	
	col++;
}

void CarMap::decMapCol() 
{ 
	if (col <= 0) {
		ArLog::log(ArLog::Terse, "col=%d beyond ---- ", col);
		return;
	}
	
	col--;
}

CarMap::CarMap(int m_row, int m_col, int r, int c, 
		bool r_axis, bool r_increase, 
		bool c_axis, bool c_increase)
{
	int i, j, val;
	
	map_row = m_row;
	map_col = m_col;
	row = r;
	col = c;
	row_axis = r_axis;
	row_increase = r_increase;
	col_axis = c_axis;
	col_increase = c_increase;	
	
	if (map_row > MAP_DEFAULT_X || map_col > MAP_DEFAULT_Y) {
		ArLog::log(ArLog::Terse, "map_row = %d, map_col = %d, invalid",
			map_row, map_col);
	}

	/* construct map in form of array[row][col] */
	for (i = 0; i < map_row; i++) {
		if (i % 2 == 0) {
			val = WHITE; /* white color */
			for (j = 0; j < map_col; j++) {
				map[i][j] = val;
				printf("%4d", map[i][j]);
				val = !val;
			}
		} else {
			val = BLACK; /* black color */
			for (j = 0; j < map_col; j++) {
				map[i][j] = val;
				printf("%4d", map[i][j]);
				val = !val;
			}
		}
		printf("\n");
	}

	cur_color = map[row][col];
	ArLog::log(ArLog::Normal, "Current Color: %d\n",
		cur_color);
}

void CarMap::dumpPosition()
{
	char *pdir = NULL;
	
	if (row_axis) {
		if (row_increase) {
			pdir = "-->";
		} else {
			pdir = "<--";
		}
	} else if (col_axis) {
		if (col_increase) {
			pdir = "-->";
		} else {
			pdir = "<--";
		}
	}				
	ArLog::log(ArLog::Terse, 
		"map[%d][%d] = %s, cur_color = %s, "
		"axis: %c %s\n",			
		row, col, map[row][col] ? "White" : "Black", 
		cur_color ? "White" : "Black", 
		row_axis ? 'Y' : 'X', pdir);
}


class ColorToCoord : public ArASyncTask
{
public:	
	ArCondition myCondition;
	ArMutex myMytex;
	Robot_Obj *myRobot_Obj;
	SmartCarCom *mySmartCarCom;
	CarMap *myCarMap;
	Operation_State *myOp_state;
	Robot_Msg *myDirectionRobotMsg;
	struct Msg_Item *myMsgItem;
	bool wait_for_leftright_done;	

	struct ColorSensor {
		unsigned int r;
		unsigned int g;
		unsigned int b;

		ColorSensor() : r(0), g(0), b(0) {}
		ColorSensor(unsigned int _r, unsigned int _g, unsigned int _b) :
			r(_r), g(_g), b(_b) {}
	};

	ColorSensor SENSOR_COLOR_WHITE;
	ColorSensor SENSOR_COLOR_BLACK;

	bool WHITE_BLANCE_EN;
	const int WHITE_BLANCE_DIFF;
	const float WHITE_BLANCE_FACTORG;
	const float WHITE_BLANCE_FACTORB;		

	ColorToCoord(std::string comPort);
	virtual ~ColorToCoord() { }
	void setMapAndState(CarMap *carMap, Operation_State *op_state);
	void msgEnqueue(struct Msg_Item *msg_item);
	struct Msg_Item *getOneMsg();
	bool getColor(ColorSensor& color);
	bool getBlackWhite();	
	void checkWB();
	void adjustDir(struct Msg_Item *msg_item);
	void handleLeftRight(int turn_left, int turn_right);
	void *runThread(void *arg);
};

ColorToCoord::ColorToCoord(std::string comPort) :
SENSOR_COLOR_WHITE(700, 700, 700),
SENSOR_COLOR_BLACK(1500, 1500, 1500),
WHITE_BLANCE_EN(true),
WHITE_BLANCE_DIFF(50),
WHITE_BLANCE_FACTORG(1.017866),
WHITE_BLANCE_FACTORB(1.533923)
{
	wait_for_leftright_done = 0;
	myMsgItem = NULL;
	
	mySmartCarCom = new SmartCarCom(comPort.c_str());
	if (mySmartCarCom->open()) {
		mySmartCarCom->init();
	} else {
		ArLog::log(ArLog::Terse, 
			"ColorToCoord: open COM %s failed.", comPort);
		return;
	}	
}

void ColorToCoord::setMapAndState(CarMap *carMap, 
	Operation_State *op_state)
{
	myCarMap = carMap;
	myOp_state = op_state;
}

void ColorToCoord::msgEnqueue(struct Msg_Item *msg_item)
{
	myMutex.lock();
	myDirectionRobotMsg->msgItemEnqueue(msg_item);	
	myMutex.unlock();
}

struct Msg_Item* ColorToCoord::getOneMsg()
{
	struct Msg_Item *msg_item = NULL;
	
	if (!myDirectionRobotMsg->isEmptyItemQueue()) {
		myMutex.lock();
		msg_item = myDirectionRobotMsg->msgItemDequeue();
		myMutex.unlock();
		
		ArLog::log(ArLog::Normal, "Slave getOneMsg: type=%d, key=%d, val=%d "
			"row=%d, col=%d\n",
		msg_item->type, msg_item->key, msg_item->val,
		msg_item->u.array.row, msg_item->u.array.col);
	}	

	return msg_item;
}

const unsigned int MAX_SENSOR_DATA_SIZE = 20;
bool ColorToCoord::getColor(ColorSensor& color)
{
	char buffer[MAX_SENSOR_DATA_SIZE * 3 + 1];
	const char* buf = buffer;

	if (!mySmartCarCom->read(buffer, MAX_SENSOR_DATA_SIZE * 3)) {
		ArLog::log(ArLog::Terse, 
			"ColorToCoord: read COM port failed.");
		return false;
	}
	buffer[MAX_SENSOR_DATA_SIZE * 3] = '\0';
	while (*buf != 'R')
		buf++;

	std::string str_value(buf);

	int r_index = str_value.find_first_of('R');
	int g_index = str_value.find_first_of('G');
	int b_index = str_value.find_first_of('B');
	int u_index = str_value.find_first_of('U');
	std::string str_r(str_value, r_index + 1, g_index - r_index - 1);
	std::string str_g(str_value, g_index + 1, b_index - g_index - 1);
	std::string str_b(str_value, b_index + 1, u_index - b_index - 1);

	color.r = std::atoi(str_r.c_str());
	color.g = std::atoi(str_g.c_str());
	color.b = std::atoi(str_b.c_str());

	//std::cout << "r(" << color.r << ")" << "g(" << color.g << ")" << "b(" << color.b << ")" << std::endl;

	return true;
}

bool ColorToCoord::getBlackWhite()
{
	ColorSensor color;
	int i = 0, white_cnt = 0, black_cnt = 0;

	for (i = 0; i < 3; i++) {
		if (!getColor(color)) {
			ArLog::log(ArLog::Terse, "Failed to get color\n");
			return false;
		}
#if 0
			if (WHITE_BLANCE_EN) {
				color.g = (unsigned int)((float)color.r * WHITE_BLANCE_FACTORG);
				color.b = (unsigned int)((float)color.b * WHITE_BLANCE_FACTORB);
			}
		
			//std::cout << "BLANCE: r(" << color.r << ")";
			//std::cout << "BLANCE: g(" << color.g << ")";
			//std::cout << "BLANCE: b(" << color.b << ")" << std::endl;
		
			if ((abs((int)color.r - (int)color.g) < WHITE_BLANCE_DIFF) &&
				(abs((int)color.r - (int)color.b) < WHITE_BLANCE_DIFF) &&
				(abs((int)color.g - (int)color.b) < WHITE_BLANCE_DIFF))
				return true;
		
			return false;
#else
			if ((abs((int)color.r + (int)color.g + (int)color.b)) < 3000)
				white_cnt++;
			else 
				black_cnt++;
#endif
		}

	if (white_cnt > black_cnt)
		return true;
	else 
		return false;
}

void ColorToCoord::checkWB()
{
	while (1) {
		if (getBlackWhite())
			ArLog::log(ArLog::Normal, "White + + + +");
		else
			ArLog::log(ArLog::Normal, "Black - -");			

		ArUtil::sleep(50);
	}
}

void ColorToCoord::adjustDir(struct Msg_Item *msg_item)
{
	double angular1, angular2;
	
	ArLog::log(ArLog::Normal, "Slave adjustDir: type=%d, key=%d, val=%d",
		msg_item->type, msg_item->key, msg_item->val);

	switch (msg_item->key) {
	case ArKeyHandler::LEFT:
	case ArKeyHandler::RIGHT:
		angular1 = myRobot_Obj->myRobot.getTh();
		myRobot_Obj->myRobot.comInt(ArCommands::DHEAD, msg_item->val);
		angular2 = myRobot_Obj->myRobot.getTh();
		while (abs(abs(angular2) - abs(angular1)) <= 89) {
			ArUtil::sleep(50);
			angular2 = myRobot_Obj->myRobot.getTh();	
		}
		break;
	default:
		ArLog::log(ArLog::Normal, "unknow Special MSG key: %d\n",
			msg_item->key);
	}
}

void ColorToCoord::handleLeftRight(int turn_left, int turn_right)
{
	wait_for_leftright_done = 1;
	
	myMutex.lock();
	if (myCarMap->col_axis == true) {
		myCarMap->col_axis = false;
		myCarMap->row_axis = true;
		if (myCarMap->col_increase) {
			if (turn_left) {
				if (myCarMap->row >= myCarMap->map_row - 1) {
					ArLog::log(ArLog::Normal, "++++ row beyond, row = %d",
						myCarMap->row);
					myCarMap->col_axis = !myCarMap->col_axis;
					myCarMap->row_axis = !myCarMap->row_axis;
					goto out;
				}
				
				myCarMap->incMapRow();
				myCarMap->row_increase = true;
			} else if (turn_right) {
				if (myCarMap->row <= 0) {
					ArLog::log(ArLog::Normal, "++++ row beyond, row = %d",
						myCarMap->row);
					myCarMap->col_axis = !myCarMap->col_axis;
					myCarMap->row_axis = !myCarMap->row_axis;
					goto out;
				}

				myCarMap->decMapRow();
				myCarMap->row_increase = false;
			}
		} else if (!myCarMap->col_increase) {
			if (turn_left) {
				if (myCarMap->row <= 0) {
					ArLog::log(ArLog::Normal, "++++ row beyond, row = %d",
						myCarMap->row);
					myCarMap->col_axis = !myCarMap->col_axis;
					myCarMap->row_axis = !myCarMap->row_axis;
					goto out;
				}
				
				myCarMap->decMapRow();
				myCarMap->row_increase = false;
			} else if (turn_right) {
				if (myCarMap->row >= myCarMap->map_row - 1) {
					ArLog::log(ArLog::Normal, "++++ row beyond, row = %d",
						myCarMap->row);
					myCarMap->col_axis = !myCarMap->col_axis;
					myCarMap->row_axis = !myCarMap->row_axis;
					goto out;
				}
				
				myCarMap->incMapRow();
				myCarMap->row_increase = true;
			}
		}
	} else if (myCarMap->row_axis == true) {
		myCarMap->row_axis = false;
		myCarMap->col_axis = true;
		if (myCarMap->row_increase) {
			if (turn_left) {
				if (myCarMap->col <= 0) {
					ArLog::log(ArLog::Normal, "---- col beyond, col = %d",
						myCarMap->col);
					myCarMap->row_axis = !myCarMap->row_axis;
					myCarMap->col_axis = !myCarMap->col_axis;
					goto out;
				}
				
				myCarMap->decMapCol();
				myCarMap->col_increase = false;
			} else if (turn_right) {
				if (myCarMap->col >= myCarMap->map_col - 1) {
					ArLog::log(ArLog::Normal, "---- col beyond, col = %d",
						myCarMap->col);
					myCarMap->row_axis = !myCarMap->row_axis;
					myCarMap->col_axis = !myCarMap->col_axis;
					goto out;
				}
				
				myCarMap->incMapCol();
				myCarMap->col_increase = true;
			}
		} else if (!myCarMap->row_increase) {
			if (turn_left) {
				if (myCarMap->col >= myCarMap->map_col - 1) {
					ArLog::log(ArLog::Normal, "---- col beyond, col = %d",
						myCarMap->col);
					myCarMap->row_axis = !myCarMap->row_axis;
					myCarMap->col_axis = !myCarMap->col_axis;
					goto out;
				}
			
				myCarMap->incMapCol();
				myCarMap->col_increase = true;
			} else if (turn_right) {
				if (myCarMap->col <= 0) {
					ArLog::log(ArLog::Normal, "---- col beyond, col = %d",
						myCarMap->col);
					myCarMap->row_axis = !myCarMap->row_axis;
					myCarMap->col_axis = !myCarMap->col_axis;
					goto out;
				}
				
				myCarMap->decMapCol();
				myCarMap->col_increase = false;
			}
		}
	}

out:
	myMutex.unlock();
	wait_for_leftright_done = 0;
	myCondition.signal();
}

#define MOVE_OFFSET 240
static int g_distanceOffset = 240; //240mm
void *ColorToCoord::runThread(void *argv)
{
	int color, row = -1, col = -1;
	char *pdir = NULL;	
	bool wait_for_turn_done = 0;
	
	for (;;) {
		color = getBlackWhite();
		#if 0
		if (color != myCarMap->cur_color) {
			ArLog::log(ArLog::Terse, 
				"old_color: %d, cur_color: %d\n", myCarMap->cur_color, color);
			myCarMap->cur_color = color;
		}
		#else
		if (!myMsgItem) {
			row = -1;
			col = -1;
			myMsgItem = getOneMsg();
			if (myMsgItem) {
				row = myMsgItem->u.array.row;
				col = myMsgItem->u.array.col;
				if (myMsgItem->key == ArKeyHandler::LEFT) {
					myOp_state->turn_left = LEFT_RIGHT;					
				} else if (myMsgItem->key == ArKeyHandler::RIGHT) {
					myOp_state->turn_right = TURN_RIGHT;					
				}
			}
		}
		
		if (color != myCarMap->cur_color) {
			myCarMap->cur_color = color;
			#if 0
			printf("m_row=%d, m_col=%d, row=%d, col=%d, cur_color=%d, map_color = %d\n",
				myCarMap->getMapRow(), myCarMap->getMapCol(), row, col,
				color, myCarMap->map[row][col]);
			#endif

			if (wait_for_turn_done) {			
				handleLeftRight(myOp_state->turn_left, myOp_state->turn_right);
				myOp_state->updateOpState(!LEFT_RIGHT, !TURN_LEFT, !TURN_RIGHT);
				myMsgItem = NULL;
				wait_for_turn_done = 0;
				ArLog::log(ArLog::Normal, "Turn end: ");
				myCarMap->dumpPosition();
			} else {				
				if (myCarMap->col_axis == true) {
					if (myCarMap->col_increase) {
						myCarMap->incMapCol();
					} else {
						myCarMap->decMapCol();
					}
				} else if (myCarMap->row_axis == true) {
					if (myCarMap->row_increase) {
						myCarMap->incMapRow();
					} else {
						myCarMap->decMapRow();
					}
				}

				if (row == myCarMap->getMapRow() && col == myCarMap->getMapCol()
					&& myCarMap->cur_color == myCarMap->map[row][col]) {
					wait_for_turn_done = 1;	
					ArLog::log(ArLog::Normal, "Turn begin: ");
					if (myMsgItem) {
						int secs = 12000; /* default delay for turning */
						int carSpeed = RcvMasterMsgThread::getCarSpeed();
						if (carSpeed !=0) {
							secs = g_distanceOffset / carSpeed * 1000;
						} else {
							ArLog::log(ArLog::Terse, 
								"CarSpeed = 0, using default delay 12secs");
						}
						
						ArUtil::sleep(secs);
						adjustDir(myMsgItem);
					}
				} 

				myCarMap->dumpPosition();
			}
		}
		#endif

		ArUtil::sleep(1000);
	}
	return NULL;
}

bool check_encrypt_info(struct Msg_Item *msg_item)
{
	ArLog::log(ArLog::Terse, 
		"Check Packet: idx=%d ----", msg_item->index); 

	return true;
}


int main(int argc, char** argv)
{
	int i = 0, hasClient = 0;
	const char *server = NULL;
	int sport = -1, slaveServerPort = -1;
	const char *strToServer = "slave";
	Robot_Obj *robot_obj = NULL;
	RcvMasterMsgThread rcvMasterMsgThread;
	MonitorThread monitorThread;
	ArSocket *masterSock = &rcvMasterMsgThread.myMasterSock;
	Robot_Msg genericRobot_Msg, specialRobot_Msg, sendSubSlaveRobot_Msg;
	Operation_State operation_State;
	CarMap *carMap;	
	ColorToCoord *colorToCoord;
	struct Msg_Item msg_item;
	SendSlaveMsgThread sendSubSlaveMsgThread;
	ArSocket serverSock, tempSock;
	int len = 0, clientNum = 1;
	char buff[10] = {0};
	
#if 1
	GPSPosition *gpsPosition;
	ECompass *eCompass;
	LogLatDistance logLatDistance;
#endif

	Aria::init();	

#if 1
	while (i < argc) {
		ArLog::log(ArLog::Terse, "argvp[%d] = %s\n", i, argv[i]);
		/*
		* create a robot according to the cmdline arguments format
		* {"-rh"  "127.0.0.1" "-rrtp" "8101" "-server" "127.0.0.1"  "-sport" "9999"} 
		* or { "-rp" "COM4" "-rb" "9600" "-server" "172.20.86.52" "-sport" "9999"} 
		* or {"-sensor" "5" "5" "0" "0" "1" "1" "0" "0" "COM2"}
		* represent a robot.
		*/
		if (!strcmp(argv[i], "-rh")) {
			robot_obj = new Robot_Obj(ARG_GRP_LEN, &argv[i]);			
			ArLog::log(ArLog::Normal,
				"connect via network, ip: %s, port: %s",
				argv[i + 1], argv[i + 3]);
			i += ARG_GRP_LEN;
			continue;
		}

		if (!strcmp(argv[i], "-rp")) {
			robot_obj = new Robot_Obj(ARG_GRP_LEN, &argv[i]);		
			ArLog::log(ArLog::Normal,
				"connect via serial, port: %s, baud: %s",
				argv[i + 1], argv[i + 3]);
			i += ARG_GRP_LEN;
			continue;
		}

		if (!strcmp(argv[i], "-server")) {
			server = argv[i + 1];	
			sport = atoi(argv[i + 3]);
			ArLog::log(ArLog::Normal, 
				"connect... server: %s, sport = %d", server, sport);
			i += 4;
			continue;
		}

		if (!strcmp(argv[i], "-hasClient") && 
			!strcmp(argv[i + 1], "-slaveServerPort")) {
			hasClient = 1;
			slaveServerPort = atoi(argv[i + 2]);
			i += 3;
			ArLog::log(ArLog::Normal, 
				"Slave_Car hasClient, slaveServerPort = %d\n",
				slaveServerPort);
			continue;
		}

		if (!strcmp(argv[i], "-disOffset")) {
			g_distanceOffset = atoi(argv[i + 1]);
			ArLog::log(ArLog::Normal, 
				"g_distanceOffset = %dms", g_distanceOffset);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-sensor")) {
			int m_row, m_col, r, c; 
			bool r_axis, r_increase, c_axis, c_increase;
			m_row = atoi(argv[i + 1]);
			m_col = atoi(argv[i + 2]);
			r = atoi(argv[i + 3]);
			c = atoi(argv[i + 4]);
			r_axis = atoi(argv[i + 5]);
			r_increase = atoi(argv[i + 6]);
			c_axis = atoi(argv[i + 7]);
			c_increase = atoi(argv[i + 8]);
			carMap = new CarMap(m_row, m_col, r, c,
				r_axis, r_increase, c_axis, c_increase);
			colorToCoord = new ColorToCoord(argv[i + 9]);
			colorToCoord->setMapAndState(carMap, &operation_State);
			colorToCoord->myDirectionRobotMsg = &specialRobot_Msg;
			colorToCoord->myRobot_Obj = robot_obj;
			ArLog::log(ArLog::Normal,
				"map[%d][%d]\n"
				"pos: (%d, %d)\n"
				"r_axis: %d, r_inc: %d\n"
				"c_axis: %d, c_inc: %d\n"
				"Com Port: %s\n",
				 m_row, m_col, r, c, 
				 r_axis, r_increase, c_axis, c_increase, argv[i + 9]);
			i += 10;
			colorToCoord->runAsync();			
			continue;
		}

		if (!strcmp(argv[i], "-st")) {			
			ColorToCoord colorToCoord(argv[i + 1]);
			colorToCoord.checkWB();	
			
			ArLog::log(ArLog::Normal,
				"Sensor Com Port = %s", argv[i + 1]);
			i += 2;
			continue;
		}
		
		if (!strcmp(argv[i], "-slaveGpsTest")) {
			slaveGpsTest = atoi(argv[i + 1]);
			ArLog::log(ArLog::Normal,
				"slaveGpsTest = %d", slaveGpsTest);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-sleepTime")) {
			monitorThread.sleepTime = atoi(argv[i + 1]);
			ArLog::log(ArLog::Normal,
				"MonitorThread.sleepTime = %d", monitorThread.sleepTime);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-delayGo")) {
			rcvMasterMsgThread.delayGo = atoi(argv[i + 1]);
			ArLog::log(ArLog::Normal,
				"RcvMasterMsgThread.delayGo = %d",
				rcvMasterMsgThread.delayGo);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-gps")) {
			gpsPosition = new GPSPosition(argv[i + 1]);
			ArLog::log(ArLog::Normal,
				"GPSPosition Com port = %s", argv[i + 1]);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-gpstest")) {
			gpsPosition = new GPSPosition(argv[i + 1]);
			#if 1
			gpsPosition->testGPS();
			#endif
			ArLog::log(ArLog::Normal,
				"GPSPosition Com port = %s", argv[i + 1]);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-ecompass")) {
			eCompass = new ECompass(argv[i + 1]);			
			ArLog::log(ArLog::Normal,
				"ECompass Com port = %s", argv[i + 1]);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-ecomtest")) {
			eCompass = new ECompass(argv[i + 1]);
			#if 1
			eCompass->testEcompass();
			#endif
			ArLog::log(ArLog::Normal,
				"ECompass Com port = %s", argv[i + 1]);
			i += 2;
			continue;
		}
		
		i += 1;
	}

	/* connect server logic begin */
	if (!server || sport < 0)
		ArLog::log(ArLog::Normal, "server or sport invalid\n");

	if (masterSock->connect(server, sport, ArSocket::TCP))
		ArLog::log(ArLog::Normal,
			"Slave_Car: Connected to Master_Car at %s TCP port %d\n",
			server, sport);
	else {
		ArLog::log(ArLog::Terse,
			"Slave_Car: Error Connected to Master_Car at %s TCP port %d.\n",
			server, sport);
		return(-1);
	}

	if (masterSock->write(strToServer, strlen(strToServer)) == strlen(strToServer))
		ArLog::log(ArLog::Normal,
			"Slave_Car: Send handshake to the server.");
	else {
		ArLog::log(ArLog::Terse,
			"Slave_Car: Error sending handshake string to the server.");
		Aria::exit(-1);
	}
	/* connect server logic end */


	/* wait subslave car connecting begin */
	if (hasClient) {
		if (slaveServerPort < 0) {
			ArLog::log(ArLog::Normal, 
				"Slave_Car slaveServerPort =%d, invalid\n",
				slaveServerPort);
			return -1;
		}
			
		if (serverSock.open(slaveServerPort, ArSocket::TCP)) {
			ArLog::log(ArLog::Normal, 
				"Slave_Car: Opened the server port %d\n", slaveServerPort);
		} else {
			ArLog::log(ArLog::Normal, 
				"Slave_Car: Failed to open the server port: %s.",
				serverSock.getErrorStr().c_str());
			Aria::exit(-1);    
			return(-1);
		}

		/* just Control and Slave, two client-server connection */
		clientNum = 1; //default wait only one client to connect
		for(i = 0; Aria::getRunning() && i < clientNum; ++i){
			/* Wait for a client to connect to us */
			ArLog::log(ArLog::Terse, 
				"Slave_Car: Waiting for a client to connect. Press CTRL-C to exit.");
			if (serverSock.accept(&tempSock))
				ArLog::log(ArLog::Terse, 
					"Slave_Car: Client %d has connected.", i);
			else
				ArLog::log(ArLog::Terse, 
					"Slave_Car: Error in accepting a connection from the client: %s.",
					serverSock.getErrorStr().c_str());

			len = tempSock.read(buff, sizeof(buff));	
			
			// If the amount read is 0 or less, its an error condition.
			if (len > 0) {	
				buff[len]='\0';
				if(!strcmp(buff, "slave")) {
					sendSubSlaveMsgThread.mySlaveSock[sendSubSlaveMsgThread.mySlaveNr] = tempSock;
					ArLog::log(ArLog::Terse, "Slave_Car: Client %d is: %s.", 
						sendSubSlaveMsgThread.mySlaveNr, buff);
					sendSubSlaveMsgThread.mySlaveNr++;
				}
			} else {
				ArLog::log(ArLog::Terse, 
					"Slave_Car: Error in handshake from the client.");
				Aria::exit(-1);		
			}		
		}	

		sendSubSlaveMsgThread.mySubSlaveRobotMsg = &sendSubSlaveRobot_Msg;
		sendSubSlaveMsgThread.runAsync();
	}
	/* wait subslave car connecting end */
#endif

	rcvMasterMsgThread.setRobotMsg(robot_obj, &genericRobot_Msg);

#if OUTDOOR	
#if ENABLE_ECOM
	rcvMasterMsgThread.myECompass = eCompass;
#endif
#endif

	rcvMasterMsgThread.runAsync();
	
#if OUTDOOR	
#if UNTEST_NET
	monitorThread.myGPSPosition = gpsPosition;
#if ENABLE_ECOM
	monitorThread.myEcompass = eCompass;
#endif // ENABLE_ECOM
#endif	//UNTEST_NET
	monitorThread.setRobotMsg(robot_obj, &specialRobot_Msg);
	monitorThread.myLogLatDistance = &logLatDistance;
	monitorThread.myRcvMasterMsgThread = &rcvMasterMsgThread;
	monitorThread.runAsync();	
#endif	

	for (;;) {
		if (masterSock->read((void *)&msg_item, MSG_LEN) == MSG_LEN) {

			/* check encrypt information */
			if (!check_encrypt_info(&msg_item)) {
				ArLog::log(ArLog::Terse, 
					"check encrypt information fail");
				continue;
			}
			
			if (msg_item.type == TYPE_DIRECTION) {
			#if OUTDOOR
				monitorThread.msgEnqueue(&msg_item);	
			#else				
				colorToCoord->msgEnqueue(&msg_item);
			#endif
			} else {
				rcvMasterMsgThread.msgEnqueue(&msg_item);
			}

			if (hasClient) {				
				sendSubSlaveMsgThread.msgEnqueue(&msg_item);				
			}
					
			ArLog::log(ArLog::Verbose, "slave rcv: idx=%d, type=%d, key=%d, val=%d",
				msg_item.index, msg_item.type, msg_item.key, msg_item.val);
		}
		ArUtil::sleep(50);
	}
	
	ArLog::log(ArLog::Normal, "Slave_Car exit ...");
	Aria::exit(0);
}
