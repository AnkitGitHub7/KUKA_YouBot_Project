
#include "stdafx.h"
#include "CEventQueue.h"
#include <exception>
#include <iostream>
CEventQueue::CEventQueue()
{
	// TODO Auto-generated constructor stub

}

Event CEventQueue::nextEvent()
{
	std::list<Event>::reverse_iterator rit = eventList.rbegin();
	Event e = *rit;
	if (eventList.size() != 0)
	{
		eventList.remove(*rit); //delete whatever is on rit
	}
	return e;
}

void CEventQueue::addEvent(Event event)
{
	eventList.push_back(event);	
}
