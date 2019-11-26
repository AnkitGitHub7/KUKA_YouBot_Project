#include "stdafx.h"
#include "CRoute.h"

CRoute::CRoute(CEventQueue & eventQueueRef) :m_evtQueue(eventQueueRef) //,Coordinate(eventQueueRef)
{
	m_vm = 1.0;
	m_bm = 1.0;

	m_seX = 0;
	m_seY = 0;
	m_vX = 0;
	m_vY = 0;

	m_move[0] = 0.0;	//speed in x direction
	m_move[1] = 0.0;	//speed in y direction
	m_move[2] = 0.0;	//angle
	m_move[3] = 105;	//sektor initial value 
	m_move[4] = 0.0;	//Ballposition X
	m_move[5] = 0.0;	//Ballposition Y

	m_BallCoordinate[0][1] = 0; //Robotposition Y
	m_BallCoordinate[1][0] = 0;//Robotposition X
	m_RobotCoordinate[0][1] = 0;//Robotposition Y
	m_RobotCoordinate[1][0] = 0;//Ballposition X

}
CRoute::~CRoute()
{
	
}

void CRoute::getPositions(CObjectDetection &objectDetection)
{
	m_RobotCoordinate[0][1] = objectDetection.m_coordinate[1]; //Robotposition Y
	m_RobotCoordinate[1][0] = objectDetection.m_coordinate[0]; //Robotposition x
	m_BallCoordinate[0][1] = objectDetection.m_coordinate[3]; //Ballposition Y
	m_BallCoordinate[1][0] = objectDetection.m_coordinate[2]; //Ballposition X
	m_move[4] = m_BallCoordinate[1][0]; //Ballposition X
	m_move[5] = m_BallCoordinate[0][1]; //Ballposition Y
}

void CRoute::calculateDistance()
{
	//set sign of speed in x direction
	if(m_BallCoordinate[1][0]>m_RobotCoordinate[1][0]
	)
	{
		m_seX = m_BallCoordinate[1][0]- m_RobotCoordinate[1][0]; //Ballx-Robotx
		m_move[0] = 1; //positive x direction
	}
	else
	{
		m_seX =m_RobotCoordinate[1][0]-m_BallCoordinate[1][0]; //Robotx-Ballx
		m_move[0] = -1; //negative x direction
	}

	//set sign of speed in y direction
	if(m_BallCoordinate[0][1]>m_RobotCoordinate[0][1])
	{
		m_seY = m_BallCoordinate[0][1]- m_RobotCoordinate[0][1]; //Bally-Roboty
		m_move[1] = 1; //positive y direction
	}
	else
	{
		m_seY = m_RobotCoordinate[0][1]- m_BallCoordinate[0][1]; //RobotY - Bally
		m_move[1] = -1; //negative y direction
	}

}

void CRoute::calculateSpeed()
{
	double maxSpeed; //for example start with 0.8
	maxSpeed = 0.7;

	if (m_seX > m_seY)
	{
		m_vX = maxSpeed;
		m_vY = maxSpeed *m_seY/m_seX;
	}
	else if (m_seY > m_seX)
	{
		m_vY = maxSpeed;
		m_vX = maxSpeed *m_seX/m_seY;
	}
	else //seX=seY
	{
		m_vX = maxSpeed;
		m_vY = maxSpeed;
	}
	//assign speeds to robot
	//store signs (+1 or -1)
	double signx = m_move[0];
	double signy = m_move[1];

	//set speeds  (dont drive in negative x direction)
	if (signx < 0)
	{
		m_move[0] = 0;
		m_move[1] = 0;
	}
	else
	{
		m_move[0] = signx * m_vX; //set speed in x direction
		m_move[1] = signy * m_vY; //set speed in y direction
	}

}


void CRoute::setAngle()
{
	//Calculate angle
	double angle = 0;
	double seX = m_BallCoordinate[1][0] - m_RobotCoordinate[1][0]; //can have positive or negative value
	double seY = m_BallCoordinate[0][1] - m_RobotCoordinate[0][1]; //can have positive or negative value

	if(seY == 0) angle = 0;
	if(seX == 0 && seY > 0) angle = 90;
	if(seX == 0 && seY < 0) angle = -90;
	else angle = atan(seY / seX) * 180 / PI;


	cout << "angle in calculation " << angle<<endl;
	if (-90 <= angle <= 90)
	{
		m_move[2] = angle;
	}
	else
	{
		// errorhandling if calculated angle out of range
		m_move[2] = 0;
	}
	
}
void CRoute::setSector()
{
	double sektor = 105;
	int angle =(int) m_move[2];
	cout << "angle in setSector" << angle << endl;

		//set positive x and positive y quarter
		if (m_move[0] >= 0 && m_move[1] >= 0)
		{
			if ((angle >= 0) && (angle < 3)) sektor = 141;
			else if ((angle >= 4) && (angle < 11)) sektor = 147;
			else if ((angle >= 11) && (angle < 17)) sektor = 148;
			else if ((angle >= 17) && (angle < 23)) sektor = 149;
			else if ((angle >= 23) && (angle < 28)) sektor = 150;
			else if ((angle >= 28) && (angle <= 45))sektor = 151;
			else if (angle > 45) sektor = 151;
		else cout << "error in CRoute-angle+x+y" << endl;
		}
		

		//set positive x and negative y quarter
		else if (m_move[0] >= 0 && m_move[1] < 0)
		{
		if( (angle <= 0  ) && (angle > -3)) sektor = 141;
		else if ((angle  <= - 4 ) && (angle > -11) )sektor = 142;
		else if ((angle <= -11) && (angle > -17) )sektor = 143;
		else if ((angle <= -17) && (angle > -23) )sektor = 144;
		else if ((angle <= -23) && (angle > -28) )sektor = 145;
		else if ((angle <= -28) && (angle >= -45))sektor = 146;	
		else if (angle < -45) sektor = 146;
		else cout << "error in CRoute-angle+x-y" << endl;
		}
		//set negative x and negarive y quarter
		else if (m_move[0] < 0 && m_move[1] <0)
		{
			if ((angle >= 0) && (angle < 3)) sektor = 141;
			else if ((angle >= 4) && (angle < 11)) sektor = 147;
			else if ((angle >= 11) && (angle < 17)) sektor = 148;
			else if ((angle >= 17) && (angle < 23)) sektor = 149;
			else if ((angle >= 23) && (angle < 28)) sektor = 150;
			else if ((angle >= 28) && (angle <= 45))sektor = 151;
			else if (angle > 45) sektor = 151;

			else cout << "error in CRoute-angle-x-y" << endl;
		}

		//set negative x and positive y quarter
		else if  (m_move[0] < 0 && m_move[1] >= 0)
		{
			if ((angle <= 0) && (angle > -3)) sektor = 141;
			else if ((angle <= -4) && (angle > -11))sektor = 142;
			else if ((angle <= -11) && (angle > -17))sektor = 143;
			else if ((angle <= -17) && (angle > -23))sektor = 144;
			else if ((angle <= -23) && (angle > -28))sektor = 145;
			else if ((angle <= -28) && (angle >= -45))sektor = 146;
			else if (angle < -45) sektor = 146;
			else cout << "error in CRoute-angle-x+y" << endl;
		}
		else cout << "error in CRoute::setSector" << endl;

		m_move[3] = sektor;

}

void CRoute::Testklasse(CObjectDetection &objectDetection)
{

	getPositions(objectDetection);
	calculateDistance();
	setAngle();
	setSector();
	calculateSpeed();

	cout << "testRX" << ":   " << m_RobotCoordinate[1][0] << endl;
	cout << "testRY" << ":   " << m_RobotCoordinate[0][1] << endl;
	cout << "testBX" << ":   " << m_BallCoordinate[1][0] << endl;
	cout << "testBY" << ":   " << m_BallCoordinate[0][1] << endl;
	cout << "test vx: " << m_move[0] << endl;
	cout << "test vy: " << m_move[1] << endl;
	cout << "test angle: " << m_move[2] << endl;
	cout << "test sector: " << m_move[3] << endl;
	cout << "test ballX: " << m_move[4] << endl;
	cout << "test ballY: " << m_move[5] << endl;

	m_evtQueue.addEvent(EV_SendSpeed);
}

void CRoute::run(CObjectDetection &objectDetection)
{
	getPositions(objectDetection);
	calculateDistance();
	setAngle();
	setSector();
	calculateSpeed();
	m_evtQueue.addEvent(EV_SendSpeed);
}

void CRoute::backHome(CObjectDetection &objectDetection)
{
	m_RobotCoordinate[0][1] = objectDetection.m_coordinate[3]; //Robotposition(Ballendposition) Y
	m_RobotCoordinate[1][0] = objectDetection.m_coordinate[2]; //Robotposition(Ballendposition) x
	m_BallCoordinate[0][1] = objectDetection.m_coordinate[1]; //Homeposition Y
	m_BallCoordinate[1][0] = objectDetection.m_coordinate[0]; //Homeposition X
	m_move[4] = objectDetection.m_coordinate[0]; //Homeposition X
	m_move[5] = objectDetection.m_coordinate[1]; //Homeposition Y

	calculateDistance();
	calculateSpeed();
	m_move[3] = 0; //Sector is set to 0
	m_evtQueue.addEvent(EV_SendSpeed);
}