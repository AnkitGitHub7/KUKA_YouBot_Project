/*
 * CRoute.h
 *
 *  Created on: 07.05.2018
 */

#include <string>
#include <iostream>
#include <stdlib.h>
#include <list>
#include <cmath>
using namespace std;

#define PI 3.14159265


#ifndef CROUTE_H_
#define CROUTE_H_

#include "CObjectDetection.h"
#include "CEventQueue.h"

class CRoute
{
private:

	double hase;

	friend class CMove;

	//CObjectDetection Coordinate;
	CEventQueue &m_evtQueue;

	double m_vm; // will be set in the constructor
	double m_bm; // will be set in the constructor
	double m_seX;
	double m_seY;
	double m_vX;
	double m_vY;

	// First Value X-Position, second  Y-Position
	double m_RobotCoordinate[2][2];
	double m_BallCoordinate[2][2];
	double m_HomeCoordinate[2][2];

protected:
	double m_move[6];

//TODO: Range of Plattform movement must be initialized by the Camera Team



public:
	CRoute(CEventQueue & eventQueue);
	~CRoute();
	void getPositions(CObjectDetection &objectDetection);
	void calculateDistance();
	void calculateSpeed();
	void setSector();
	void setAngle();
	void Testklasse(CObjectDetection &objectDetection);
	void run(CObjectDetection &objectDetection);
	void backHome(CObjectDetection &objectDetection);

};

#endif /* CROUTE_H_ */
