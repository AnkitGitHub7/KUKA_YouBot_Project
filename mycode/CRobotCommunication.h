#pragma once

#include <ctime>
#include <cmath>


#include <stdio.h>
#include <iostream>

#include <ctime>
#include <chrono>



#pragma comment (lib, "ws2_32.lib")
#define SERVER "127.0.0.1"  //ip address of udp server
#define BUFLEN 512  //Max length of buffer
#define PORT 8888   //The port on which to listen for incoming data

#pragma comment (lib, "ws2_32.lib")
using namespace std;



class CRobotCommunication
{



	struct sockaddr_in si_other;
	int s, slen = sizeof(si_other);
	char buf[BUFLEN];
	char message[BUFLEN];
	WSADATA wsa;
	chrono::milliseconds initTime;
	chrono::milliseconds timeout;
	bool wasStarted;
	bool stoppedflag;
	


	
public:
	CRobotCommunication();
	~CRobotCommunication();
	void initSocket(u_short port, char * ipadress);
	bool functionMove( const double &x, const double &y, char* cmd, double ballX, double ballY, chrono::milliseconds _timeout=1900ms);
	void initArm(char* message);
	bool startTimer(chrono::milliseconds _timeout);
	void functionMoveThreading(const double &x, const double &y);
	bool driveByKey(const double &x, const double &y) ;

	
	
};


