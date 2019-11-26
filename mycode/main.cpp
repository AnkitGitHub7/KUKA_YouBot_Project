
// 8.8.18

// Header-Dateien
#include "stdafx.h"
#include "CRoute.h"
#include "CObjectDetection.h"
#include "CMove.h"
#include "CController.h"
#include "CEventQueue.h"
#include <iostream>
#include <stdlib.h>
#include <string>
#include <list>
#include "CBaumerCamera.h"

//#include <template>

using namespace std;





int main (void)
{
	CEventQueue eventQueue;
	CController m_controller(eventQueue);
	CMove m_move(eventQueue);
	CObjectDetection m_objectDetection(eventQueue);
	CRoute m_route(eventQueue);
	CBaumerCamera m_baumer;
	double x1, y1;
	


	
	
	//first initialization
	eventQueue.addEvent(EV_CatchMode);

	while (1) //enum Event{EV_CatchMode, EV_BallTracking, EV_BallEndPos, EV_SendSpeed, EV_BackHome};
	{
		switch (eventQueue.nextEvent())
		{
		case(EV_CatchMode):
			cout << "case catch mode" << endl;
			m_objectDetection.initilization_Kinect();
			m_baumer.initBaumerCamera();
			m_baumer.startAquisitonPolling();
			//cout << "run calibration"; // This is just for calibration 
			//m_baumer.runCameraCalibration(m_move.m_RobotComInteface); // This is just for calibration 
			m_baumer.loadCameraCalibration("CameraCoefficients");

			//connect the kommunication to the cameras
			//move the arm to its initial catching position
			if (m_move.initiateArm())
				cout << " catch position is set " << endl;
			else {
				cout << " socket connection error " << endl;
				exit(0);
			}
			break;
		case(EV_BallTracking):
			cout << "case ball tracking" << endl;
			
			// track the ball and calculate its final predicted endposition

			//if ball is within range
			m_objectDetection.runKinect();
			//m_objectDetection.Testklasse();
			//if ball is out of range
			//eventQueue.addEvent(EV_BallOutOfRange);
			break;
		case(EV_BallEndPos):
			cout << "case ball end position" << endl;
			m_route.run(m_objectDetection);
			break;

		case(EV_SendSpeed):
			cout << "case send speed" << endl;
			//handle the speed and angle of arm movement
			m_move.run(m_route,m_baumer);
			system("pause");
			return 0;
			eventQueue.addEvent(EV_BackHome);
			break;
		
		case (EV_BackHome):
			//after catching, go back to home position
			m_route.backHome(m_objectDetection);
			break;

		case(EV_BallOutOfRange):
			cout << "case ball out of range" << endl;
			
				return 0;
				//end of program
				break;
		default:
			cout << "default" << endl;
			return 0;
			break;
		}
		
	}

	// Close down Winsock
	m_baumer.stopAquisition();
	WSACleanup();
	system("pause");
	return 0;
}
