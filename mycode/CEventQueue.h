/*
 * CEventQueue.h
 *
 *  Created on: 10.05.2018
 */
#include <list>
#include <vector>
#ifndef CEVENTQUEUE_H_
#define CEVENTQUEUE_H_


 enum Event{EV_CatchMode, EV_BallTracking, EV_BallEndPos, EV_SendSpeed, EV_BackHome, EV_BallOutOfRange};

class CEventQueue
{




private:

	

std::list<Event> eventList;
public:
	CEventQueue();
	Event nextEvent();
	void addEvent(Event event);

	


};

#endif /* CEVENTQUEUE_H_ */
