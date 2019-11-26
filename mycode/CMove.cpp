
#include "stdafx.h"
#include "CMove.h"
#include "CRoute.h"
#include <sstream>




using namespace cv;
using namespace std;

CMove::CMove(CEventQueue & eventQueueRef):m_evtQueue(eventQueueRef)
{
	m_RobotComInteface.initSocket(4556, (char*)"192.168.43.30");
}

CMove::~CMove()
{
	// TODO Auto-generated destructor stub
}

bool CMove::initiateArm()
{


	////////////////////////////////////////////////////////////
	// CONNECT TO THE SERVER
	////////////////////////////////////////////////////////////

	m_RobotComInteface.initArm((char*)"C105;0X");

	m_evtQueue.addEvent(EV_BallTracking);
	
	return true;
}


void CMove::Testklasse(CRoute &m_route)
{
	//todo: ankit
	//call methods of arm and platform to move the robot to its final position
	
	cout << "x speed passed to cmove: " << m_route.m_move[0] << endl;
	cout << "y speed passed to cmove: " << m_route.m_move[1] << endl;
	cout << "angle passed to cmove: " << m_route.m_move[2] << endl;
	cout << "sector passed to cmove: " << m_route.m_move[3] << endl;
	cout << "ballX passed to cmove: " << m_route.m_move[4] << endl;
	cout << "ballY passed to cmove: " << m_route.m_move[5] << endl;



}
void CMove::run(CRoute &m_route, CBaumerCamera & camera)
{
	char* movingCmd = NULL;
	stringstream converter;
	cout << "x speed passed to cmove: " << m_route.m_move[0] << endl;
	cout << "y speed passed to cmove: " << m_route.m_move[1] << endl;
	cout << "angle passed to cmove: " << m_route.m_move[2] << endl;
	cout << "sector passed to cmove: " << m_route.m_move[3] << endl;
	cout << "ballX passed to cmove: " << m_route.m_move[4] << endl;
	cout << "ballY passed to cmove: " << m_route.m_move[5] << endl;

	converter<< "C1;" <<m_route.m_move[0] << "XC2;" <<m_route.m_move[1]<<"XC"<< m_route.m_move[3]<<";0X";

	


	string message = converter.str();
	cout << " moving command" << message << endl;

	movingCmd = (char*)message.c_str();

	// OpenCV Matrices
	//Mat cameraFeed;				//Matrix to store each frame of the webcam feed

	//VideoCapture capture;

	//capture.open(0);

	//waitKey(100);
	//
	//while (1)
	//{
	//
	//
	//	capture.read(cameraFeed);
	//	imshow("Original ", cameraFeed);
	
	//robotMovement(m_route);
	//	auto key = cv::waitKey(1);
	//	if (key == 'q') break;
	//	
	//}



	while (true)
	{
		camera.AquisitionPolling(robotposX,robotposY);
		
		

		if (m_RobotComInteface.functionMove(robotposX, robotposY, movingCmd, m_route.m_move[4], m_route.m_move[5]))
		{
			//break;
		}
		
		// implementation with drive by key- leave this while loop by pressing q

	}


	

}


