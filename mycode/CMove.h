
#include "CRoute.h"
#include "CEventQueue.h"
#include <ctime>
#include "CBaumerCamera.h"
#include "CRobotCommunication.h"


using namespace std;

#ifndef CMOVE_H_
#define CMOVE_H_

class CMove
{
private:
	//CRoute DrivingValue;
	CEventQueue &m_evtQueue;
	sockaddr_in server;
	SOCKET out;
	double robotposX = 0, robotposY = 0;
	


public:
	CMove(CEventQueue & eventQueue);
	virtual ~CMove();
	void Testklasse(CRoute &m_route);
	void run(CRoute &m_route, CBaumerCamera & camera);
	bool initiateArm();

	CRobotCommunication m_RobotComInteface;




};

#endif /* CMOVE_H_ */
