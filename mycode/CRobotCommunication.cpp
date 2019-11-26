#include "stdafx.h"
#include "CRobotCommunication.h"
#include "sharedMutex.h"
#include <conio.h>



using namespace std;
using namespace std::chrono;
CRobotCommunication::CRobotCommunication()
{
	wasStarted = false;
	stoppedflag = false;
}

void CRobotCommunication::initSocket(u_short port, char * ipadress)
{
	printf("\nInitialising Winsock...");
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Failed. Error Code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}
	printf("Initialised.\n");

	//create socket
	if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
	{
		printf("socket() failed with error code : %d", WSAGetLastError());
		exit(EXIT_FAILURE);
	}

	//setup address structure
	memset((char *)&si_other, 0, sizeof(si_other));
	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(port);
	//si_other.sin_addr.S_un.S_addr = inet_addr(ipadress);
	inet_pton(AF_INET, ipadress, &si_other.sin_addr);

}



bool CRobotCommunication::startTimer(chrono::milliseconds _timeout)
{

	if (!wasStarted)
	{
		timeout = _timeout;
		initTime = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
		wasStarted = true;
	}
	if((duration_cast<milliseconds>(system_clock::now().time_since_epoch() - initTime) > timeout))
	{
		cout << "timeout reached" << endl;
		return true;
	}
	return false;
}



void CRobotCommunication::functionMoveThreading(const double &x, const double &y)
{
        chrono::milliseconds threadstart = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
		chrono::milliseconds timeout;
		bool stop = false;

		// Write out to that socket
		double curretx = 0, currenty = 0;
		char buffer[100];
		sprintf_s(buffer, "function move thread: %i\n", GetCurrentThreadId);
		OutputDebugStringA(buffer);

		//string s(argv[1]);
		//string s = "C990;0XC1;0.8X";
		while(true)
		{ 
		
			timeout = duration_cast<milliseconds>(
				system_clock::now().time_since_epoch());

			if ((timeout - threadstart) > milliseconds(2000))
			{
				stop = false; //without timeout 
			}
				
			unique_lock<std::mutex> guard(MutexCam);
			curretx = x;
			currenty = y;
			OutputDebugString(L"in Mutex function move\n");
			guard.unlock();
			
			const char* message;
			int sendOk;
	

		if ((((curretx <= 47) && (curretx >= 27)) && ((currenty >= 227) && (currenty <= 247)))|| stop)
		{
			//cout << "area reached" << endl << endl;
			//s = "C102;0XC0;0XC999;X";
			message = (char *)"C0;0XC999;X";
			//send the message
			if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
			{
				printf("sendto() failed with error code : %d", WSAGetLastError());
			}
			//return 0;
			// s = "C1;0.8XC2;0.8X";

			sprintf_s(buffer, "function move thread: %d  %d\n", curretx,currenty);
			OutputDebugStringA(buffer);
			OutputDebugString(L"stopcommand\n");
			cout << "--------------stopcommand--------------x= "<< curretx<<"y= " << currenty <<"\n";
			this_thread::sleep_for(1s);
			
		}

		else {
			//s = "C0;0XC999;X";
			//cout << "area outside" << endl << endl;
			message = (char *)"C1;0.8XC105;0X";//C105;0X//

			OutputDebugString(L"transverse\n");
			if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
			{
				printf("sendto() failed with error code : %d", WSAGetLastError());
			}		
		}
		}
			
}

void CRobotCommunication::initArm(char* message)
{
	
	if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
	{
		printf("sendto() failed with error code : %d", WSAGetLastError());
	}

}





bool CRobotCommunication::functionMove(const double &x, const double &y, char* cmd, double ballX, double ballY, chrono::milliseconds _timeout)
{

	double curretx = x;
	double currenty = y;
	const char* message;
	int sendOk;
	double resXX, resYY;
	
	resXX = ((150 - ballX) / 0.36) + 103;
	resYY = (ballY / 0.36) + 42;

	//cout << "mapped coord are " << resXX << " and " << resYY << endl;
	


	
	if(!stoppedflag)
	{ 
  
		if (((((curretx <= resXX + 25) && (curretx >= resXX - 25)) && ((currenty >= resYY - 25) && (currenty <= resYY + 25))) || startTimer(_timeout)))
		{
		
			message = (char *)"C0;0XC999;X";
	
			//send the message
		
			if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
			{
				printf("sendto() failed with error code : %d", WSAGetLastError());
			}
		

			stoppedflag = true;
			cout << "--------------stopcommand--------------x= " << curretx << "y= " << currenty << "\n";
	
	
		}	
		else {
		
			//message = cmd;

			//message = (char *)"C105;0X";

			message = cmd;
			//cout << "ich fahre " << cmd  << endl;

			if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
			{
				printf("sendto() failed with error code : %d", WSAGetLastError());
			}
		

		}


	}

	else
	{
		
		return driveByKey(x, y); // if you just want to check the pixel values

	}

	return false;
}






bool CRobotCommunication::driveByKey( const double &x, const  double&y)
{
	char *message;
		
	bool exit = false;
		
			char c = cv::waitKey(1000/30);// Nur wenn auch eine Taste gedrückt ist
			switch (c)
			{
				
			case 's': 

				message = (char *)"C1;-0.4X";//C105;0XC1;0.8X//
				if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
				{
					printf("sendto() failed with error code : %d", WSAGetLastError());
				}

				cout << "x= " << x << " | y= " << y << endl;
				break;

			case 'w': 
				message = (char *) "C1;0.4X";//C105;0XC1;0.8X//
				if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
				{
					printf("sendto() failed with error code : %d", WSAGetLastError());
				}
				cout << "x= " << x << " | y= " << y << endl;
		
				break;

			case 'a':
				message = (char *) "C2;0.4X";//C105;0XC1;0.8X//
				if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
				{
					printf("sendto() failed with error code : %d", WSAGetLastError());
				}
				cout << "x= " << x << " | y= " << y << endl;

				break;

			case 'd':
				message = (char *)"C2;-0.4X";//C105;0XC1;0.8X//
				if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
				{
					printf("sendto() failed with error code : %d", WSAGetLastError());
				}
				cout << "x= " << x << " | y= " << y << endl;

				break;

			case 'c':
				message = (char *) "C105;0X";//C105;0XC1;0.8X//
				if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
				{
					printf("sendto() failed with error code : %d", WSAGetLastError());
				}
				cout << "x= " << x << " | y= " << y << endl;

				break;

			case 't':
				message = (char *) "C1;-0.8XC2;0.4XC135;0X";//C105;0XC1;0.8X//
				if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
				{
					printf("sendto() failed with error code : %d", WSAGetLastError());
				}
				cout << "x= " << x << " | y= " << y << endl;

				break;
			case 'r':
				message = (char *) "C105;0X";//C105;0XC1;0.8X//
				if (sendto(s, message, strlen(message), 0, (struct sockaddr *) &si_other, slen) == SOCKET_ERROR)
				{
					printf("sendto() failed with error code : %d", WSAGetLastError());
				}
				break;

			case 'q': return true;

			
			default: break;

			}
		
			//return; //getting current coordinate
		
		
	

}





CRobotCommunication::~CRobotCommunication()
{

	// Close the socket
	closesocket(s);

	
}


