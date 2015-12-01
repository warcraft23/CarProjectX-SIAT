//#include "Windows.h"
#pragma comment(lib,"CarAuthDll.lib")
#include <math.h>
#include <string>
#include "Aria.h"
#include "car_auth.h"

#define OUTDOOR 1
/************************************************************************/
/* UNTEST_NET 1为室外有GPS，0为室内无GPS                                     */
/************************************************************************/
#define UNTEST_NET 1
#define ENABLE_ECOM 0

#define CHECK_COLOR 0

#define ROBOT_NUM	10
#define ARG_GRP_LEN	4
#define MSG_QUEUE_LEN	10
#define SPORT 9999

int masterGpsTest = 0;
CarAuth car_auth = CarAuth(1);

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
		ArLog::log(ArLog::Terse, 
			"Error, could not connect to robot.\n");
		myRobotConnector->logOptions();
		Aria::exit(1);
	}

	// add the sonar to the robot
	myRobot.addRangeDevice(mySonar);

	// turn on the motors, turn off amigobot sounds
	myRobot.comInt(ArCommands::ENABLE, 1);
	myRobot.comInt(ArCommands::SOUNDTOG, 0);

	myRobot.comInt(ArCommands::MOVE, ~0);
	myRobot.comInt(ArCommands::VEL, 0);
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
		ArLog::log(ArLog::Terse, 
			"Error, could not connect to lasers.\n");
		Aria::logOptions();
		Aria::exit(3);
	}
	ArLog::log(ArLog::Terse, "exit --- %s\n", __FUNCTION__);
}

Robot_Obj::~Robot_Obj()
{
	if (myRobot.isRunning()) {
		delete myParser;
		delete myRobotConnector;
		delete myLaserConnector;
		delete mySonar;
	}
}
/*
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
*/
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
		ArLog::log(ArLog::Terse, 
			"map_row = %d, map_col = %d, invalid", map_row, map_col);
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
	ArLog::log(ArLog::Normal, "Current Color: %d\n", cur_color);
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

	ArLog::log(ArLog::Normal, "SmartCarCom: init COM %s, baudrate %d", 
		m_ComName.c_str(), BaudRate);
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
		ArLog::log(ArLog::Terse, "SmartCarCom: open COM port %s failed.",
			m_ComName.c_str());
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
		ArLog::log(ArLog::Terse, "Failed to write COM port, %s\n",
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
		if (!bReadStat) {
			printf("Failed to read COM port, %s\n", m_ComName);
			return false;
		}
		//printf("%c: %c:%d %c:%d\n", *(pInBuffer+retCnt)[0], );
		retCnt += temp;
	}
	
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

class ColorToCoord : public ArASyncTask
{
public:	
	ArCondition myCondition;
	ArMutex myMytex;
	SmartCarCom *mySmartCarCom;
	CarMap *myCarMap;
	Operation_State *myOp_state;
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
	bool getColor(ColorSensor& color);
	bool getBlackWhite();	
	void checkWB();
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
					printf("++++ row beyond, row = %d\n", myCarMap->row);
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
bool wait_tunr_done = 0;
void *ColorToCoord::runThread(void *argv)
{
	int color = WHITE;
	char *pdir = NULL;
	
	for (;;) {		
		#if CHECK_COLOR
		checkWB();
		#else
		color = getBlackWhite();
		if (color != myCarMap->cur_color && !wait_tunr_done) {
			myCarMap->cur_color = color;
			if (myOp_state->leftright_flag) {
				ArLog::log(ArLog::Normal, "Turn begin: ");
				myCarMap->dumpPosition();
				handleLeftRight(myOp_state->turn_left, myOp_state->turn_right);
				myOp_state->updateOpState(!LEFT_RIGHT, !TURN_LEFT, !TURN_RIGHT);				
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

				myCarMap->dumpPosition();
			}			
		}
		#endif

		ArUtil::sleep(1000);
	}
	return NULL;
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
	return d * PI / 180.0;  
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
	myLongitude = atof(str_E_degree.c_str()) + atof(str_E_minute.c_str()) / 60;

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
		ArLog::log(ArLog::Normal, "ecompass: %s",
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

	ArLog::log(ArLog::Normal, "Mx: %d, My: %d, Mz: %d, angle: %d",
			myMx, myMy, myMz, *angle);

	return true;
}

void ECompass::testEcompass()
{
	int angle;
	
	while (1) {
		getAngle(&angle);
		ArUtil::sleep(500);
	}
}

#define MAX_SLAVE 16
class SendSlaveMsgThread : public ArASyncTask {
public:
	ArCondition myCondition;
	ArMutex myMutex;
	Robot_Obj *myRobot_Obj;
	ArSocket mySlaveSock[MAX_SLAVE];
	int mySlaveNr;
	Robot_Msg *mySlaveRobotMsg;
	Operation_State *myOpState;
	CarMap *myCarMap;
	struct Msg_Item *myMsgItem;	
	int condition_wait_flag;

	SendSlaveMsgThread();
	void setRobotMsg(Robot_Msg *robot_msg);
	void setMapAndState(CarMap *carMap, Operation_State *op_state);
	void msgEnqueue(struct Msg_Item *msg_item);
	void parseAndForwardMsg(struct Msg_Item *msg_item);	
	void *runThread(void *arg);	
};

SendSlaveMsgThread::SendSlaveMsgThread()
{
	memset(mySlaveSock, 0x00, sizeof(mySlaveSock));
	mySlaveNr = 0;
}

void SendSlaveMsgThread::setMapAndState(CarMap *carMap, 
	Operation_State *op_state)
{
	myCarMap = carMap;
	myOpState = op_state;
}

void SendSlaveMsgThread::msgEnqueue(struct Msg_Item *msg_item)
{
	if (mySlaveRobotMsg->isFullItemQueue()) {
		condition_wait_flag = 1;
		myCondition.wait();
	}		

	myMutex.lock();
	mySlaveRobotMsg->msgItemEnqueue(msg_item);	
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

	/************************************************************************/
	/* 对数据包进行打包，填充其中的签名和公玥，时间戳和crc等                 */
	/************************************************************************/
#if CARAUTH
	car_auth.packPkt(msg_item);
#endif
	for (i = 0; i < mySlaveNr; i++) {
		if(mySlaveSock[i].write((void *)msg_item, MSG_LEN) == MSG_LEN)
			ArLog::log(ArLog::Verbose, 
				"Slave_Car: Send msg to slave car %d", i);
		else {
			ArLog::log(ArLog::Terse, 
				"Slave_Car: Error sending msg to slave car %d", i);
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
		while (!mySlaveRobotMsg->isEmptyItemQueue()) {
			myMutex.lock();
			myMsgItem = mySlaveRobotMsg->msgItemDequeue();
			myMutex.unlock();

			if (condition_wait_flag) {
				condition_wait_flag = 0;
				myCondition.signal();
			}

			parseAndForwardMsg(myMsgItem);			
		}

		myCondition.wait();		
	}

	ArLog::log(ArLog::Normal, "RcvControlMsgThread exit, oooh\n");

	return NULL;
}

enum {IDLE_GPS = 0, START_POINT, MIDDLE_POINT, END_POINT};

class RcvControlMsgThread : public ArASyncTask {
public:
	ArCondition myCondition;
	ArMutex myMutex;
	Robot_Obj *myRobot_Obj;
	ArSocket myControlSock;
	SendSlaveMsgThread *mySendSlaveMsgThread;
	Robot_Msg *myRobotMsg;
	struct Msg_Item *myMsgItem;
	Operation_State *myOp_state;
	int condition_wait_flag;
	/* control whether deliver msg to master car and slave car */
	bool routeToMaster, routeToSlave; 

	GPSPosition *myGPSposition;
	#if ENABLE_ECOM
	ECompass *myECompass;
	#endif
	LogLatDistance *myLogLatDistance;
	ArMutex myMutexForStartGetGPSFlag;
	int myStartGetGPSFlag;
	double myLongitude, myLatidude;

	RcvControlMsgThread();
	void setRobotMsg(Robot_Obj *robot_obj, 
		Robot_Msg *robot_msg, Operation_State *op_state);
	void waitTurnDone(struct Msg_Item *msg_item);
	int getDist(struct Msg_Item *msg_item);
	#if ENABLE_ECOM
	void getEcom(struct Msg_Item *msg_item);
	#endif
	void msgEnqueueSlavequeue(struct Msg_Item *msg_item);
	void msgEnqueue(struct Msg_Item *msg_item);
	int	parseSpecialKey(struct Msg_Item *msg_item);
	void parseMsg(struct Msg_Item *msg_item);	
	void *runThread(void *arg);	
};

RcvControlMsgThread::RcvControlMsgThread()
{
	 condition_wait_flag = 0;
	 routeToMaster = true;
	 routeToSlave = true;
	 myStartGetGPSFlag = IDLE_GPS;
}

void RcvControlMsgThread::setRobotMsg(Robot_Obj *robot_obj, 
	Robot_Msg *robot_msg, Operation_State *op_state)
{
	myRobot_Obj = robot_obj;
	myRobotMsg = robot_msg;
	myOp_state = op_state;		
}

void RcvControlMsgThread::waitTurnDone(struct Msg_Item *msg_item)
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

int RcvControlMsgThread::getDist(struct Msg_Item *msg_item)
{
	int timeout = 10;
	double longitude, latitude;

	while (timeout-- > 0) {
		if (myGPSposition->getGPSPosition(&longitude, 
			&latitude)) {
			ArLog::log(ArLog::Verbose, "getGPS longitude=%f, latitude=%f",
				longitude, latitude);
			break;
		} 

		if (timeout == 0) {
			ArLog::log(ArLog::Normal, "getGPS timeout ...");
			while(1);
		}
	}

	if (myLongitude < 0.2 || myLatidude < 0.2) {
		ArLog::log(ArLog::Terse, 
			"wrong reference parameter, myLongitude=%f, myLatidude=%f",
			myLongitude, myLatidude);

		msg_item->u.gps.distance = 0;

		return false;
	}

	msg_item->u.gps.distance = 
		myLogLatDistance->get_distance(myLatidude, myLongitude, latitude, longitude);

	myLongitude = 0.0;
	myLatidude = 0.0;

	return true;
}

#if ENABLE_ECOM
void RcvControlMsgThread::getEcom(struct Msg_Item *msg_item)
{	
	int timeout = 10;
	
	while (timeout-- > 0) {
		if (myECompass->getAngle(&msg_item->u.gps.angle)) {
			ArLog::log(ArLog::Normal, "getEcompass angle=%d",
				msg_item->u.gps.angle);
			break;
		}

		if (timeout == 0) {
			ArLog::log(ArLog::Normal, "getEcom timeout ...");
			while(1);
		}
	}
}
#endif
void RcvControlMsgThread::msgEnqueueSlavequeue(struct Msg_Item *msg_item)
{
	if (routeToSlave) {	
		mySendSlaveMsgThread->myMutex.lock();
		mySendSlaveMsgThread->msgEnqueue(msg_item);
		mySendSlaveMsgThread->myMutex.unlock();
		mySendSlaveMsgThread->myCondition.signal();	
	}
}

void RcvControlMsgThread::msgEnqueue(struct Msg_Item *msg_item)
{
	if (myRobotMsg->isFullItemQueue()) {
		condition_wait_flag = 1;
		myCondition.wait();
	}		

	myMutex.lock();
	myRobotMsg->msgItemEnqueue(msg_item);	
	myMutex.unlock();
	
	myCondition.signal();
}

/*
* ret = 0: indicate special key
* ret = -1: generic key, need to forward the msg
*/
int RcvControlMsgThread::parseSpecialKey(struct Msg_Item *msg_item)
{
	int ret = -1;

	if (msg_item->type != TYPE_SPECIAL)
		return ret;

	switch (msg_item->key) {
		case 'm':			
			routeToMaster = msg_item->val;
			ret = 0;
			break;
		case 's':
			routeToSlave = msg_item->val;
			ret = 0;
			break;
		default:
			ArLog::log(ArLog::Terse, "unknown special key: %d\n", 
				msg_item->key);
	}

	ArLog::log(ArLog::Terse, 
		"Master parseSpecialKey: idx=%d, key=%d, routeToMaster=%d, routeToSlave=%d",
		msg_item->index, msg_item->key, routeToMaster, routeToSlave);	
	

	return ret;
}

void RcvControlMsgThread::parseMsg(struct Msg_Item *msg_item)
{
	double angular1, angular2;

	ArLog::log(ArLog::Verbose, "Master parseMsg: idx=%d, type=%d, key=%d, val=%d",
		msg_item->index, msg_item->type, msg_item->key, msg_item->val);	

	/* 
	* 1. parse the msg	
	* 2. send command to master robot
	* 3. forfard msg to slave robot
	*/		
	if (routeToMaster && 
		(msg_item->type & (TYPE_GENERIC | TYPE_DIRECTION))) {
		switch (msg_item->key) {
		case ArKeyHandler::UP:
		case ArKeyHandler::DOWN:
			#if OUTDOOR	
			ArLog::log(ArLog::Normal, "Master parseMsg: idx=%d, type=%d, key=%d, val=%d",
								msg_item->index, msg_item->type, msg_item->key, msg_item->val); 
			myMutexForStartGetGPSFlag.lock();		
			if (myStartGetGPSFlag == IDLE_GPS) {				
				myStartGetGPSFlag = START_POINT;				
			}			
			myMutexForStartGetGPSFlag.unlock();
			myRobot_Obj->myRobot.comInt(ArCommands::VEL, msg_item->val);
			#else
			myOp_state->updateOpState(!LEFT_RIGHT, !TURN_LEFT, !TURN_RIGHT);
			myRobot_Obj->myRobot.comInt(ArCommands::VEL, msg_item->val);
			#endif
			msgEnqueueSlavequeue(msg_item);
			break;
		case ArKeyHandler::LEFT:
			#if OUTDOOR	
			myMutexForStartGetGPSFlag.lock();
			if (myStartGetGPSFlag == MIDDLE_POINT) {				
				myStartGetGPSFlag = END_POINT;				
			} else {
				ArLog::log(ArLog::Terse, 
					"Begin status wrong, myStartGetGPSFlag = %d, should be MIDDLE_POINT\n"
					"drop MSG: idx=%d, type=%d, key=%d, val=%d, distance=%f, angle=%d\n\n",
					myStartGetGPSFlag,
					msg_item->index, msg_item->type, msg_item->key, 
					msg_item->val, msg_item->u.gps.distance, msg_item->u.gps.angle);
					myMutexForStartGetGPSFlag.unlock();
				return;
			}

			ArLog::log(ArLog::Normal, "1----- left\n");
			#if UNTEST_NET
			getDist(msg_item);
			#endif
			msgEnqueueSlavequeue(msg_item);
			ArLog::log(ArLog::Normal, "2----- left\n");
			waitTurnDone(msg_item);
			#if UNTEST_NET
			//getEcom(msg_item);
			#endif
			ArLog::log(ArLog::Normal, 
				"2 --- idx=%d, type=%d, key=%d, val=%d, distance=%f, angle=%d\n\n",
				msg_item->index, msg_item->type, msg_item->key, 
				msg_item->val, msg_item->u.gps.distance, msg_item->u.gps.angle); 

			if (myStartGetGPSFlag == END_POINT) {				
				myStartGetGPSFlag = START_POINT;				
			} else {
				ArLog::log(ArLog::Terse, 
					"End status wrong, myStartGetGPSFlag = %d, should be START_POINT",
					myStartGetGPSFlag);
			}
			myMutexForStartGetGPSFlag.unlock();
			#else			
			if (!myOp_state->leftright_flag) {
				wait_tunr_done = 1;
				angular1 = myRobot_Obj->myRobot.getTh();	
				//myOp_state->updateOpState(LEFT_RIGHT, TURN_LEFT, !TURN_RIGHT);
				myRobot_Obj->myRobot.comInt(ArCommands::DHEAD, msg_item->val);
				while (abs(abs(angular2) - abs(angular1)) <= 89) {
					ArUtil::sleep(50);
					angular2 = myRobot_Obj->myRobot.getTh();	
				}
				myOp_state->updateOpState(LEFT_RIGHT, TURN_LEFT, !TURN_RIGHT);
				wait_tunr_done = 0;
			}
			msgEnqueueSlavequeue(msg_item);
			#endif
			break;
		case ArKeyHandler::RIGHT:
			#if OUTDOOR
			myMutexForStartGetGPSFlag.lock();
			if (myStartGetGPSFlag == MIDDLE_POINT) {				
				myStartGetGPSFlag = END_POINT;				
			} else {
				ArLog::log(ArLog::Terse, 
					"Begin status wrong, myStartGetGPSFlag = %d, should be MIDDLE_POINT\n"
					"idx=%d, type=%d, key=%d, val=%d, distance=%f, angle=%d\n\n",
					myStartGetGPSFlag,
					msg_item->index, msg_item->type, msg_item->key, 
					msg_item->val, msg_item->u.gps.distance, msg_item->u.gps.angle);				

				myMutexForStartGetGPSFlag.unlock();
				return ;
			}
			
			ArLog::log(ArLog::Normal, "1----- right\n");
			#if UNTEST_NET
			getDist(msg_item);
			#endif
			msgEnqueueSlavequeue(msg_item);
			ArLog::log(ArLog::Normal, "2----- right\n");
			waitTurnDone(msg_item);
			#if UNTEST_NET
			//getEcom(msg_item);
			#endif
			ArLog::log(ArLog::Normal, 
				"2 --- idx=%d, type=%d, key=%d, val=%d, distance=%f, angle=%d\n\n",
				msg_item->index, msg_item->type, msg_item->key, 
				msg_item->val, msg_item->u.gps.distance, msg_item->u.gps.angle); 

			if (myStartGetGPSFlag == END_POINT) {				
				myStartGetGPSFlag = START_POINT;				
			} else {
				ArLog::log(ArLog::Terse, 
					"End status wrong, myStartGetGPSFlag = %d, should be START_POINT",
					myStartGetGPSFlag);
			}
			myMutexForStartGetGPSFlag.unlock();
			#else
			if (!myOp_state->leftright_flag) {
				wait_tunr_done = 1;
				angular1 = myRobot_Obj->myRobot.getTh();
				//myOp_state->updateOpState(LEFT_RIGHT, !TURN_LEFT, TURN_RIGHT);
				myRobot_Obj->myRobot.comInt(ArCommands::DHEAD, msg_item->val);
				angular2 = myRobot_Obj->myRobot.getTh();
				while (abs(abs(angular2) - abs(angular1)) <= 89) {
					ArUtil::sleep(100);
					angular2 = myRobot_Obj->myRobot.getTh();	
				}
				myOp_state->updateOpState(LEFT_RIGHT, !TURN_LEFT, TURN_RIGHT);
				wait_tunr_done = 0;
			}
			msgEnqueueSlavequeue(msg_item);
			#endif
			break;
		#if ENABLE_ECOM
		case 'c':
			myECompass->getAngle(&msg_item->u.ecompass.angle);
			msgEnqueueSlavequeue(msg_item);
			break;
		#endif
		case 'p':
			//myRobot_Obj->myRobot.comInt(ArCommands::ENABLE, msg_item->val);
			myRobot_Obj->myRobot.comInt(ArCommands::VEL, msg_item->val);
			msgEnqueueSlavequeue(msg_item);
			ArLog::log(ArLog::Normal, "Master parseMsg: idx=%d, type=%d, key=%d, val=%d",
					msg_item->index, msg_item->type, msg_item->key, msg_item->val); 
			break;
		default:
			ArLog::log(ArLog::Normal, "Master bad msg: idx=%d, type=%d, key=%d, val=%d",
				msg_item->index, msg_item->type, msg_item->key, msg_item->val);
		}
	}
	
#if 0	
	msgEnqueueSlavequeue(msg_item);
#endif


}

void *RcvControlMsgThread::runThread(void *arg)
{
	ArLog::log(ArLog::Normal, "RcvControlMsgThread ...\n");

	for (;;) {
		while (!myRobotMsg->isEmptyItemQueue()) {
			myMutex.lock();
			myMsgItem = myRobotMsg->msgItemDequeue();
			myMutex.unlock();

			if (condition_wait_flag) {
				condition_wait_flag = 0;
				myCondition.signal();
			}

			parseMsg(myMsgItem);			
		}

		myCondition.wait();
	}

	ArLog::log(ArLog::Normal, "RcvControlMsgThread exit, oooh\n");

	return NULL;
}

class GPSThread : public ArASyncTask {
public:
	RcvControlMsgThread *myRcvControlMsgThread;
	GPSPosition *myGPSposition;
	LogLatDistance *myLogLatDistance;		
	int sleepTime;
	
	GPSThread();
	~GPSThread() {}
	void getInitGPS();
	void getDist();
	void *runThread(void *arg);
};

GPSThread::GPSThread()
{
	sleepTime = 3000;
}

void GPSThread::getDist()
{
	double log, lat, dst;

	if(myGPSposition->getGPSPosition(&log,&lat)) {
		if (myRcvControlMsgThread->myLongitude > 0.2 && 
			myRcvControlMsgThread->myLatidude > 0.2) {
			dst = myLogLatDistance->get_distance(myRcvControlMsgThread->myLatidude,
				myRcvControlMsgThread->myLongitude, lat, log);
			ArLog::log(ArLog::Normal, "(%f, %f) -> (%f, %f) = %f",
				myRcvControlMsgThread->myLongitude, myRcvControlMsgThread->myLatidude,
				log, lat, dst);
		}
	}	
}

#define GPS_SAMPLES 50
void GPSThread::getInitGPS()
{
	double gps[GPS_SAMPLES][2] = {0};
	double logitude = 0.0, latitude = 0.0;
	int i = 0, valid_cnt = 0;

	while (i++ < GPS_SAMPLES) {
		if(myGPSposition->getGPSPosition(&gps[i][0], &gps[i][1])) {
			valid_cnt++;
			logitude += gps[i][0];
			latitude += gps[i][1];
		}
		ArUtil::sleep(1000);
	}

	myRcvControlMsgThread->myLongitude = logitude / valid_cnt;
	myRcvControlMsgThread->myLatidude = latitude / valid_cnt;

	ArLog::log(ArLog::Normal, "getInitGPS done...\n"
		"myLongitude=%f, myLatidude=%f\n\n",
		myRcvControlMsgThread->myLongitude, myRcvControlMsgThread->myLatidude);
}

void *GPSThread::runThread(void *argv)
{
	double logitude = 0.0, latitude = 0.0;
	static int i = 0;

	ArLog::log(ArLog::Normal, "GPSThread ...\n");

	if (masterGpsTest) {
		double tmp_logitude = 0.0, tmp_latitude = 0.0, distance = 0.0;

		while (1) {
			myGPSposition->getGPSPosition(&logitude, &latitude);
			if (tmp_logitude > 0.2 && tmp_latitude > 0.2) {
				distance = myLogLatDistance->get_distance(tmp_latitude, tmp_logitude, latitude, logitude);
				ArLog::log(ArLog::Normal, "lat:lng (%f, %f) -> (%f, %f), dis=%f",
					tmp_latitude, tmp_logitude, latitude, logitude, distance);				
			}	
			
			tmp_logitude = logitude;
			tmp_latitude = latitude;
			
			ArUtil::sleep(sleepTime);
		}
	} else {
		while (1) {
			if (myRcvControlMsgThread->myStartGetGPSFlag == START_POINT) {
				/*
				* start getPGS data after master car runs for sleepTime(default 3s),
				* as GPS's sampling trends to be stable after a duration.
				*/
				ArUtil::sleep(sleepTime);

				myRcvControlMsgThread->myMutexForStartGetGPSFlag.lock();
				#if UNTEST_NET
				myGPSposition->getGPSPosition(&myRcvControlMsgThread->myLongitude, 
					&myRcvControlMsgThread->myLatidude);			
				#endif
				myRcvControlMsgThread->myStartGetGPSFlag = MIDDLE_POINT;
				myRcvControlMsgThread->myMutexForStartGetGPSFlag.unlock();
				ArLog::log(ArLog::Normal, "Attention: you can turn now...\n");
			} else {
				ArUtil::sleep(100);	

				/* for debug */
				#if UNTEST_NET
				if (++i > 5 && myRcvControlMsgThread->myStartGetGPSFlag == MIDDLE_POINT) {
					getDist();
					i = 0;
				}
				#endif
			}
		}		
	}
}

bool check_encrypt_info(struct Msg_Item *msg_item)
{
	ArLog::log(ArLog::Terse, 
		"Check Packet: idx=%d ----", msg_item->index); 
	bool res = true;
	/************************************************************************/
	/* 对数据包进行验证确定其是否合法                                                        */
	/************************************************************************/
#if CARAUTH
	if (car_auth.checkMsg(*msg_item)==1)
	{
		res = true;
	} 
	else
	{
		res = false;
	}
	
#endif
	return res;
}

int main(int argc, char **argv)
{	
	Robot_Obj *robot_obj = NULL;
	Robot_Msg robot_msg, slaveRobotMsg;
 	ArSocket serverSock, tempSock;
	RcvControlMsgThread rcvControlMsgThread;
	SendSlaveMsgThread sendSlaveMsgThread;
	Operation_State operation_State;
	CarMap *carMap;	
	ColorToCoord *colorToCoord;
	struct Msg_Item msg_item;
#if 1
	GPSPosition *gpsPosition;
	ECompass *eCompass;
	GPSThread gpsThread;
	LogLatDistance logLatDistance;
#endif
	int i = 0, sport = -1, len = 0, clientNum = 1;
	char buff[10] = {0};

	Aria::init();

	while (i < argc) {
		ArLog::log(ArLog::Verbose, "argvp[%d] = %s\n", i, argv[i]);
		/*
		* create a robot according to the cmdline arguments format
		* {"-rh" "127.0.0.1" "-rrtp" "8101" "-sport" "9999"  "-rn" "2"} 
		* or { "-rp" "COM4" "-rb" "9600" "-sport" "9999"   "-rn" "2"} represent a robot.
		* 
		* indoor:
		* -rp COM5 -rb 9600 -sport 9999 -rn 2 -sensor 5 5 0 0 0 0 1 1 COM6
		* outdoor:
		* -rp COM5 -rb 9600 -sport 9999 -rn 2 -gps COM6 -ecompass COM4 -sleepTime 2000 -masterGpsTest 1
		*/
		if (!strcmp(argv[i], "-rh")) {
			robot_obj = new Robot_Obj(ARG_GRP_LEN, &argv[i]);			
			ArLog::log(ArLog::Normal,
				"Connect Robot via IP: %s, Port: %s",
				argv[i + 1], argv[i + 3]);
			i += ARG_GRP_LEN;
			continue;
		}

		if (!strcmp(argv[i], "-rp")) {
			robot_obj = new Robot_Obj(ARG_GRP_LEN, &argv[i]);			
			ArLog::log(ArLog::Terse,
				"Connect Robot via Com: %s, Baud: %s",
				argv[i + 1], argv[i + 3]);
				i += ARG_GRP_LEN;
			continue;
		}
		
		if (!strcmp(argv[i], "-sport")) {
			sport = atoi(argv[i + 1]);
			ArLog::log(ArLog::Normal,
				"Server Listen Pport = %d", sport);
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

		if (!strcmp(argv[i], "-rn")) {			
			clientNum = atoi(argv[i + 1]);
			ArLog::log(ArLog::Normal,
				"Robot Number = %d", clientNum);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-gps")) {
			gpsPosition = new GPSPosition(argv[i + 1]);
			ArLog::log(ArLog::Normal,
				"GPSPosition Com port = %d", argv[i + 1]);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-sleepTime")) {
			gpsThread.sleepTime = atoi(argv[i + 1]);
			ArLog::log(ArLog::Normal,
				"GPSThread.sleepTime = %d", gpsThread.sleepTime);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-masterGpsTest")) {
			masterGpsTest = atoi(argv[i + 1]);
			ArLog::log(ArLog::Normal,
				"masterGpsTest = %d", masterGpsTest);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-gpstest")) {
			gpsPosition = new GPSPosition(argv[i + 1]);
			#if 1
			gpsPosition->testGPS();
			#endif
			ArLog::log(ArLog::Normal,
				"GPSPosition Com port = %d", argv[i + 1]);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-ecompass")) {
			eCompass = new ECompass(argv[i + 1]);
			ArLog::log(ArLog::Normal,
				"ECompass Com port = %d", argv[i + 1]);
			i += 2;
			continue;
		}

		if (!strcmp(argv[i], "-ecomtest")) {
			eCompass = new ECompass(argv[i + 1]);
			#if 1
			eCompass->testEcompass();
			#endif
			ArLog::log(ArLog::Normal,
				"ECompass Com port = %d", argv[i + 1]);
			i += 2;
			continue;
		}
		
		i++;
	}

	if (sport == -1) {	
		sport = SPORT;
		ArLog::log(ArLog::Terse,
			"using default port [9999]\n");
	}

  	// Open the server socket
  	if (serverSock.open(sport, ArSocket::TCP)) {
		ArLog::log(ArLog::Normal, 
			"Master_Car: Opened the server port %d\n", sport);
	} else {
		ArLog::log(ArLog::Normal, 
			"Master_Car: Failed to open the server port: %s.",
			serverSock.getErrorStr().c_str());
		Aria::exit(-1);    
		return(-1);
	}
#if 1
	/* just Control and Slave, two client-server connection */
	for(i = 0; Aria::getRunning() && i < clientNum; ++i){
		/* Wait for a client to connect to us */
		ArLog::log(ArLog::Terse, 
			"Master_Car: Waiting for a client to connect. Press CTRL-C to exit.");
		if (serverSock.accept(&tempSock))
			ArLog::log(ArLog::Terse, 
				"Master_Car: Client %d has connected.", i);
		else
			ArLog::log(ArLog::Terse, 
				"Master_Car: Error in accepting a connection from the client: %s.",
				serverSock.getErrorStr().c_str());

		len = tempSock.read(buff, sizeof(buff));	
		
		// If the amount read is 0 or less, its an error condition.
		if (len > 0) {	
			buff[len]='\0';
			if (!strcmp(buff, "control")) {
				rcvControlMsgThread.myControlSock = tempSock;
				ArLog::log(ArLog::Terse, "Master_Car: Client is: %s.", buff);
			} else if(!strcmp(buff, "slave")) {
				sendSlaveMsgThread.mySlaveSock[sendSlaveMsgThread.mySlaveNr] = tempSock;
				ArLog::log(ArLog::Terse, "Master_Car: Client %d is: %s.", 
					sendSlaveMsgThread.mySlaveNr, buff);
				sendSlaveMsgThread.mySlaveNr++;
			}
		} else {
			ArLog::log(ArLog::Terse, 
				"Master_Car: Error in handshake from the client.");
			Aria::exit(-1);		
		}		
	}	
#endif

	/* bring up thread for recieving MSG from Main_Control */
	rcvControlMsgThread.setRobotMsg(robot_obj, 
		&robot_msg, &operation_State);

#if OUTDOOR
#if UNTEST_NET
	rcvControlMsgThread.myGPSposition = gpsPosition;
	#if ENABLE_ECOM
	rcvControlMsgThread.myECompass = eCompass;
	#endif
	rcvControlMsgThread.myLogLatDistance = &logLatDistance;
#endif
#endif
	rcvControlMsgThread.mySendSlaveMsgThread = &sendSlaveMsgThread;
	rcvControlMsgThread.runAsync();

#if OUTDOOR
#if UNTEST_NET
	gpsThread.myGPSposition = gpsPosition;
#endif
	gpsThread.myRcvControlMsgThread = &rcvControlMsgThread;
	gpsThread.myLogLatDistance = &logLatDistance;
	gpsThread.runAsync();
#endif

	/* bring up thread for handling MSG delivered to Slave_Car */
	sendSlaveMsgThread.mySlaveRobotMsg = &slaveRobotMsg;
#if !OUTDOOR
	sendSlaveMsgThread.setMapAndState(carMap, &operation_State);	
#endif
	sendSlaveMsgThread.runAsync();

	for (;;) {
		if (rcvControlMsgThread.myControlSock.read((void *)&msg_item, 
			MSG_LEN) == MSG_LEN) {
			/* check encrypt information */
			if(!check_encrypt_info(&msg_item)) {
				ArLog::log(ArLog::Terse, 
					"check encrypt information fail");
				continue;
			}
			
			/* special control key doesn't need to deliver */
			if (!rcvControlMsgThread.parseSpecialKey(&msg_item))
				continue;
			
			rcvControlMsgThread.msgEnqueue(&msg_item);			
		} else {
			ArLog::log(ArLog::Verbose, "rcv main control msg fail");
		}
	}
	
	ArLog::log(ArLog::Normal, "Master_car exit ...");
	Aria::exit(0);
}
