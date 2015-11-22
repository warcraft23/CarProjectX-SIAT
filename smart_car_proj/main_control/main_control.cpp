//#include "stdafx.h"
#include "Aria.h"

#define ROBOT_NUM	10
#define ARG_GRP_LEN	4
#define MSG_QUEUE_LEN	10
#define SPORT 9999


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
}

Robot_Obj::~Robot_Obj()
{
	printf("Robot exit ....\n\n");
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
	int		index;
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

class Counter {
public:
	int index;
	Counter() { index = 0; }
	~Counter() { }
	int getCounter();
};

int Counter::getCounter()
{
	int i = 0;

	i = index;
	
	if (++index > 0xffff0000) {
		index = 0;
	}

	return i;
}

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

	//printf("%s: %d\n", __FUNCTION__, myMsgQueueHead);

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
	}
	else {
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
	//struct Msg_Item *item = NULL;

	for (; i < 11; i++) {
		robot_msg.myMsgQueue[i].key = i;
		if (robot_msg.msgItemEnqueue(&robot_msg.myMsgQueue[i]) < 0) {
			printf("queue is full.\n");
		}
	}

	i = 0;
	while (i++ < 13) {
		//item = robot_msg.msgItemDequeue();
		//printf("msg[%d] = %d\n", i++, item->key);
	}
}

class MsgConsumeThread : public ArASyncTask {
public:
	ArCondition myCondition;
	ArMutex myMutex;
	ArSocket *myServerSock;
	Robot_Msg *myRobotMsg;

	void *runThread(void *arg);
	void sendMsg(struct Msg_Item *msg_item);
};

void MsgConsumeThread::sendMsg(struct Msg_Item *msg_item)
{
	if(myServerSock->write((void *)msg_item, MSG_LEN) == MSG_LEN) {
		ArLog::log(ArLog::Verbose, 
			"Main_Control: Said control to the server.");
	} else {
		ArLog::log(ArLog::Terse, 
			"Main_Control: Error sending msg to the server.");		
	}
}

void *MsgConsumeThread::runThread(void * arg)
{
	struct Msg_Item *msg_item = NULL;
	int key = 0;
	
	for (;;) {
		while (!myRobotMsg->isEmptyItemQueue()) {
			myMutex.lock();
			msg_item = myRobotMsg->msgItemDequeue();
			myMutex.unlock();

			ArLog::log(ArLog::Normal, 
				"Control msg: idx=%d, type=%d, key=%d, val=%d",
					msg_item->index, msg_item->type,
					msg_item->key, msg_item->val);

			key = msg_item->key;
			switch (key) {
			case ArKeyHandler::UP :
			case ArKeyHandler::DOWN :
			case ArKeyHandler::LEFT :
			case ArKeyHandler::RIGHT :
			case 'm':
			case 's':
			case 'p':
				/* encrypt logic here */
				/* to do */
				
				sendMsg(msg_item);				
				break;
			default:
				ArLog::log(ArLog::Terse, 
					"Control drop msg: type=%d, key=%d, val=%d, ",
					msg_item->index, msg_item->type, 
					msg_item->key, msg_item->val);
			}
		}

		myCondition.wait();
	}

	ArLog::log(ArLog::Terse, "exit runThread ...\n");

	return NULL;
}

class InputHandler
{
public:
	MsgConsumeThread *myMsgConsumeThread;
	Robot_Msg *myRobotMsg;
	Msg_Item msg_item;
	Counter myCounter;	
	int mySpeed_step;
	InputHandler(ArKeyHandler *keyHandler);
	virtual ~InputHandler(void) {};
	void resetParameter() {myVel = 0;}
	void up(void);
	void down(void);
	void left(void);
	void right(void);
	void keyM(void); // control msg to Master car
	void keyS(void); // control msg to Slave car
	void keyP(void); // control motors
	void keyC(void);
	void queueMsg(struct Msg_Item *msg_item);
	
protected:
	ArKeyHandler *myKeyHandler;
	int myMaxVel = 2000;
	int myMinVel = -2000;
	int myVel;		
	bool myKeyM_Switch;
	bool myKeyS_Switch;
	bool myActiveMotor;

	ArFunctorC<InputHandler> myUpCB;
	ArFunctorC<InputHandler> myDownCB;
	ArFunctorC<InputHandler> myLeftCB;
	ArFunctorC<InputHandler> myRightCB;
	ArFunctorC<InputHandler> myKeyM_CB; // switch for control master car
	ArFunctorC<InputHandler> myKeyS_CB; // switch for control slave car
	ArFunctorC<InputHandler> myKeyP_CB; // switch for control motors
	ArFunctorC<InputHandler> myKeyC_CB; // calibrate slave car's direction
	
};

InputHandler::InputHandler(ArKeyHandler *keyHandler) :
myKeyHandler(keyHandler),
/* Initialize functor objects with pointers to our handler methods: */
myUpCB(this, &InputHandler::up),
myDownCB(this, &InputHandler::down),
myLeftCB(this, &InputHandler::left),
myRightCB(this, &InputHandler::right),
myKeyM_CB(this, &InputHandler::keyM),
myKeyS_CB(this, &InputHandler::keyS),
myKeyP_CB(this, &InputHandler::keyP),
myKeyC_CB(this, &InputHandler::keyC)
{
	myVel = 0;
	myMaxVel = 2000;
	myMinVel = -2000;
	mySpeed_step = 20;
	myKeyM_Switch = 1;
	myKeyS_Switch = 1;
	myActiveMotor = 1;
	memset((void *)&msg_item, 0x00, sizeof(msg_item));	
	myKeyHandler->addKeyHandler(ArKeyHandler::UP, &myUpCB);
	myKeyHandler->addKeyHandler(ArKeyHandler::DOWN, &myDownCB);
	myKeyHandler->addKeyHandler(ArKeyHandler::LEFT, &myLeftCB);
	myKeyHandler->addKeyHandler(ArKeyHandler::RIGHT, &myRightCB);
	myKeyHandler->addKeyHandler('m', &myKeyM_CB);
	myKeyHandler->addKeyHandler('s', &myKeyS_CB);
	myKeyHandler->addKeyHandler('p', &myKeyP_CB);
	myKeyHandler->addKeyHandler('c', &myKeyC_CB);
}

#define CHECK_COLOR 0

#if CHECK_COLOR
#define ROTATE_STEP 5
#else
#define ROTATE_STEP 90
#endif
void InputHandler::up(void)
{
	ArLog::log(ArLog::Verbose,"up...\n");
	if (myVel >= myMaxVel) {
		ArLog::log(ArLog::Terse,
			"driver forward: overspeed %d mm/s\n", myVel);
		return;
	}
	myVel += mySpeed_step;
	msg_item.index = myCounter.getCounter();
	msg_item.type = TYPE_GENERIC;
	msg_item.val = myVel; 
	msg_item.key = ArKeyHandler::UP;
	queueMsg(&msg_item);
	memset((void *)&msg_item, 0x00, sizeof(msg_item));
}

void InputHandler::down(void)
{
	ArLog::log(ArLog::Verbose,"down...\n");
	if (myVel <= myMinVel){
		ArLog::log(ArLog::Terse,
			"driver backward: overspeed %d mm/s\n", myVel);
		return;
	}
	myVel -= mySpeed_step;
	msg_item.index = myCounter.getCounter();
	msg_item.type = TYPE_GENERIC;
	msg_item.val = myVel;
	msg_item.key = ArKeyHandler::DOWN;
	queueMsg(&msg_item);
	memset((void *)&msg_item, 0x00, sizeof(msg_item));
}

void InputHandler::left(void)
{
	ArLog::log(ArLog::Verbose,"left...\n");
	msg_item.index = myCounter.getCounter();
	msg_item.type = TYPE_DIRECTION;
	msg_item.val = ROTATE_STEP;
	msg_item.key = ArKeyHandler::LEFT;
	queueMsg(&msg_item);
	memset((void *)&msg_item, 0x00, sizeof(msg_item));
}

void InputHandler::right(void)
{
	ArLog::log(ArLog::Verbose,"right...\n");
	msg_item.index = myCounter.getCounter();
	msg_item.type = TYPE_DIRECTION;
	msg_item.val = -ROTATE_STEP;
	msg_item.key = ArKeyHandler::RIGHT;
	queueMsg(&msg_item);
	memset((void *)&msg_item, 0x00, sizeof(msg_item));
}

void InputHandler::keyM()
{
	ArLog::log(ArLog::Verbose,"key m...\n");
	msg_item.index = myCounter.getCounter();
	myKeyM_Switch = !myKeyM_Switch;
	msg_item.type = TYPE_SPECIAL;
	msg_item.val = myKeyM_Switch;
	msg_item.key = 'm';
	queueMsg(&msg_item);
	memset((void *)&msg_item, 0x00, sizeof(msg_item));
}

void InputHandler::keyS()
{
	ArLog::log(ArLog::Verbose,"key s...\n");
	msg_item.index = myCounter.getCounter();
	myKeyS_Switch = !myKeyS_Switch;
	msg_item.type = TYPE_SPECIAL;
	msg_item.val = myKeyS_Switch;
	msg_item.key = 's';
	queueMsg(&msg_item);
	memset((void *)&msg_item, 0x00, sizeof(msg_item));
}

void InputHandler::keyP()
{
	ArLog::log(ArLog::Verbose,"key s...\n");
	myActiveMotor = !myActiveMotor;
	msg_item.index = myCounter.getCounter();
	msg_item.type = TYPE_GENERIC;
	msg_item.val = myActiveMotor;
	msg_item.key = 'p';
	queueMsg(&msg_item);
	memset((void *)&msg_item, 0x00, sizeof(msg_item));
	resetParameter(); // reset velocity to 0
}

void InputHandler::keyC()
{
	ArLog::log(ArLog::Verbose,"key c...\n");
	msg_item.index = myCounter.getCounter();
	msg_item.type = TYPE_GENERIC;
	msg_item.val = 0;
	msg_item.key = 'c';
	queueMsg(&msg_item);
	memset((void *)&msg_item, 0x00, sizeof(msg_item));
}

void InputHandler::queueMsg(struct Msg_Item *msg_item)
{
	myMsgConsumeThread->myMutex.lock();
	myRobotMsg->msgItemEnqueue(msg_item);
	myMsgConsumeThread->myMutex.unlock();
	
	myMsgConsumeThread->myCondition.signal();
}

int main(int argc, char** argv)
{
	int i = 0;
	const char *server = NULL;
	int sport;
	const char *strToServer = "control";	
	MsgConsumeThread msgConsumeThread;
	ArSocket controlSock;
	Robot_Msg robot_msg;
	struct Msg_Item msg_item;
	
	Aria::init();	
	ArKeyHandler keyHandler;
	Aria::setKeyHandler(&keyHandler);
	InputHandler inputHandler(&keyHandler);

	inputHandler.myMsgConsumeThread = &msgConsumeThread;
	inputHandler.myRobotMsg = &robot_msg;

	msgConsumeThread.myRobotMsg = &robot_msg;
#if 1
	while (i < argc) {
		ArLog::log(ArLog::Terse, "argvp[%d] = %s\n", i, argv[i]);
		/*
		* create a robot according to the cmdline arguments format
		* {"-rh", "172.20.86.52", "-sport", "9999",} represent a robot.
		*/
		if (!strcmp(argv[i], "-rh")) {
			server = argv[i + 1];												
			i += 2;
			continue;
		}
		if (!strcmp(argv[i], "-sport")) {
			sport = atoi(argv[i + 1]);
			printf("connect... port = %d\n", sport);
			i += 2;
			continue;
		}
		if (!strcmp(argv[i], "-speed_step")) {
			inputHandler.mySpeed_step = atoi(argv[i + 1]);
			printf("init Speed = %dmm\n", inputHandler.mySpeed_step);
			i += 2;
			continue;
		}
		
		i += 1;
	}

	if (controlSock.connect(server, sport, ArSocket::TCP))
		ArLog::log(ArLog::Normal, 
			"Main_Control: Connected to server at %s TCP port %d\n",
			server, sport);
	else {
		ArLog::log(ArLog::Terse,
			"Main_Control: Error Connected to server at %s TCP port %d.\n",
			server, sport);
		return(-1);
	}

	if (controlSock.write(strToServer, strlen(strToServer)) == strlen(strToServer))
		ArLog::log(ArLog::Normal, 
			"Main_Control: Send handshake to the server.");
	else {
		ArLog::log(ArLog::Terse, 
			"Main_Control: Error sending handshake string to the server.");
		Aria::exit(-1);
  	}
#endif

	msgConsumeThread.myServerSock = &controlSock;

	msgConsumeThread.runAsync();

	while (1) {
		keyHandler.checkKeys();
		ArUtil::sleep(100);
	}

	Aria::exit(0);

}