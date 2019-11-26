/*
 * CController.h
 *
 *  Created on: 10.05.2018
 */


#include "CObjectDetection.h"
#include "CMove.h"
#include "CRoute.h"
#ifndef CCONTROLLER_H_
#define CCONTROLLER_H_

class CController
{
private:
	CEventQueue & m_evtQueue;

//protected:
//	CObjectDetection* m_ObjectTracked;
//	CMove* m_MovePlattform;
//	CRoute* m_RouteCoordinate;
	
public:
	
	CController(CEventQueue & eventQueue);
	virtual ~CController();
	void run();

};

#endif /* CCONTROLLER_H_ */
