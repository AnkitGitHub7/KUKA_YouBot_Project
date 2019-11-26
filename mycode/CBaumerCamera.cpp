#pragma once

#include "stdafx.h"
#include "CBaumerCamera.h"
#include <sstream>
#include <algorithm>
#include <Windows.h>
#include <stdio.h>
#include <tchar.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <queue>
#include "opencv2\core.hpp"
#include <chrono>
#include "sharedMutex.h"





#define ONECAMERA
#define WORK_WITH_SWITCH
#define WORK_2_ETHERNET_ADAPTERS
//#define BOOST 0

#ifdef BOOST 0
#include <boost/log/core/core.hpp>
#include <boost/log/common.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/core/null_deleter.hpp>
#include <boost/shared_ptr.hpp>

#include <boost\circular_buffer\base.hpp>



#endif // Boost

using namespace cv;
using namespace BGAPI2;
using namespace std;
using namespace std::chrono;

Mat cameraFeed;
int imageCounter=0;

int cam1_FrameID = 0;
void BGAPI2CALL BufferHandler(void * callBackOwner, Buffer * pBufferFilled);
void clickAndDrag_Rectangle(int event, int x, int y, int flags, void* param);
queue<cv::Mat> queueCam1;
queue<cv::Mat> queueCamOriginal;




cv::Size size(620,420);


#ifdef BOOST
using namespace boost::log;
#endif
CBaumerCamera::CBaumerCamera()
{
	


}

void CBaumerCamera::initBaumerCamera()
{

	coordinate[0] = 0;
	coordinate[1] = 0;
	initSystem();


	initDataStreamsAndBuffers();
	initializeCameraImagePolling();
	//registerCallbackFunctions();
	params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
	params->aprilTagQuadDecimate = 2.0;


}




CBaumerCamera::~CBaumerCamera()
{
}

int CBaumerCamera::initSystem()
{
	std::cout << "SYSTEM LIST" << std::endl;
	std::cout << "###########" << std::endl << std::endl;

	//COUNTING AVAILABLE SYSTEMS (TL producers)
	try
	{
		systemList = SystemList::GetInstance();
		systemList->Refresh();
		std::cout << "5.1.2.  Detected systems:  " << systemList->size() << std::endl;

		//SYSTEM DEVICE INFORMATION
		for (SystemList::iterator sysIterator = systemList->begin(); sysIterator != systemList->end(); sysIterator++)
		{
			std::cout << "  5.2.1.  System Name:     " << sysIterator->second->GetFileName() << std::endl;
			std::cout << "          System Type:     " << sysIterator->second->GetTLType() << std::endl;
			std::cout << "          System Version:  " << sysIterator->second->GetVersion() << std::endl;
			std::cout << "          System PathName: " << sysIterator->second->GetPathName() << std::endl << std::endl;
		}
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}


	//OPEN THE FIRST SYSTEM IN THE LIST WITH A CAMERA CONNECTED
	try
	{
		for (SystemList::iterator sysIterator = systemList->begin(); sysIterator != systemList->end(); sysIterator++)
		{
			std::cout << "SYSTEM" << std::endl;
			std::cout << "######" << std::endl << std::endl;

			try
			{
				sysIterator->second->Open();
				std::cout << "5.1.3.  Open next system:" << std::endl;
				std::cout << "  5.2.1.  System Name:     " << sysIterator->second->GetFileName() << std::endl;
				std::cout << "          System Type:     " << sysIterator->second->GetTLType() << std::endl;
				std::cout << "          System Version:  " << sysIterator->second->GetVersion() << std::endl;
				std::cout << "          System PathName: " << sysIterator->second->GetPathName() << std::endl << std::endl;
				sSystemID = sysIterator->first;
				std::cout << "        Opened system - NodeList Information:" << std::endl;
				std::cout << "          GenTL Version:   " << sysIterator->second->GetNode("GenTLVersionMajor")->GetValue() << "." << sysIterator->second->GetNode("GenTLVersionMinor")->GetValue() << std::endl << std::endl;

				std::cout << "INTERFACE LIST" << std::endl;
				std::cout << "##############" << std::endl << std::endl;

				try
				{
					interfaceList = sysIterator->second->GetInterfaces();
					//COUNT AVAILABLE INTERFACES
					interfaceList->Refresh(100); // timeout of 100 msec
					std::cout << "5.1.4.  Detected interfaces: " << interfaceList->size() << std::endl;

					//INTERFACE INFORMATION
					for (InterfaceList::iterator ifIterator = interfaceList->begin(); ifIterator != interfaceList->end(); ifIterator++)
					{
						std::cout << "  5.2.2.  Interface ID:      " << ifIterator->first << std::endl;
						std::cout << "          Interface Type:    " << ifIterator->second->GetTLType() << std::endl;
						std::cout << "          Interface Name:    " << ifIterator->second->GetDisplayName() << std::endl << std::endl;
					}
				}
				catch (BGAPI2::Exceptions::IException& ex)
				{
					returncode = 0 == returncode ? 1 : returncode;
					std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
					std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
					std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
				}


				std::cout << "INTERFACE" << std::endl;
				std::cout << "#########" << std::endl << std::endl;

				//OPEN THE NEXT INTERFACE IN THE LIST
				try
				{
					for (InterfaceList::iterator ifIterator = interfaceList->begin(); ifIterator != interfaceList->end(); ifIterator++)
					{
						try
						{
							std::cout << "5.1.5.  Open interface: " << std::endl;
							std::cout << "  5.2.2.  Interface ID:      " << ifIterator->first << std::endl;
							std::cout << "          Interface Type:    " << ifIterator->second->GetTLType() << std::endl;
							std::cout << "          Interface Name:    " << ifIterator->second->GetDisplayName() << std::endl;
							ifIterator->second->Open();
							//search for any camera is connetced to this interface
							deviceList = ifIterator->second->GetDevices();
							deviceList->Refresh(100);
							if (deviceList->size() == 0)
							{
								std::cout << "5.1.13.  Close interface (" << deviceList->size() << " cameras found). " << std::endl << std::endl;
								ifIterator->second->Close();
							}
							else
							{
								sInterfaceID = ifIterator->first;
								std::cout << "   " << std::endl;
								std::cout << "        Opened interface - NodeList Information:" << std::endl;
								if (ifIterator->second->GetTLType() == "GEV")
								{
									std::cout << "          GevInterfaceSubnetIPAddress: " << ifIterator->second->GetNode("GevInterfaceSubnetIPAddress")->GetValue() << std::endl;
									std::cout << "          GevInterfaceSubnetMask:      " << ifIterator->second->GetNode("GevInterfaceSubnetMask")->GetValue() << std::endl;
								}
								if (ifIterator->second->GetTLType() == "U3V")
								{
									//std::cout << "          NodeListCount:     " << ifIterator->second->GetNodeList()->GetNodeCount() << std::endl;
								}
								std::cout << "  " << std::endl;
								break;
							}
						}
						catch (BGAPI2::Exceptions::ResourceInUseException& ex)
						{
							returncode = 0 == returncode ? 1 : returncode;
							std::cout << " Interface " << ifIterator->first << " already opened." << std::endl;
							std::cout << " ResourceInUseException: " << ex.GetErrorDescription() << std::endl;
						}
					}
				}
				catch (BGAPI2::Exceptions::IException& ex)
				{
					returncode = 0 == returncode ? 1 : returncode;
					std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
					std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
					std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
				}


				//if a camera is connected to the system interface then leave the system loop
				if (sInterfaceID != "")
				{
					break;
				}
			}
			catch (BGAPI2::Exceptions::ResourceInUseException& ex)
			{
				returncode = 0 == returncode ? 1 : returncode;
				std::cout << " System " << sysIterator->first << " already opened." << std::endl;
				std::cout << " ResourceInUseException: " << ex.GetErrorDescription() << std::endl;
			}
		}
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}

	if (sSystemID == "")
	{
		std::cout << " No System found." << std::endl;
		std::cout << std::endl << "End" << std::endl << "Input any number to close the program:";
		int endKey = 0;
		std::cin >> endKey;
		BGAPI2::SystemList::ReleaseInstance();
		return returncode;
	}
	else
	{
		pSystem = (*systemList)[sSystemID];
	}


	if (sInterfaceID == "")
	{
		std::cout << " No camera found." << sInterfaceID << std::endl;
		std::cout << std::endl << "End" << std::endl << "Input any number to close the program:";
		int endKey = 0;
		std::cin >> endKey;
		pSystem->Close();
		BGAPI2::SystemList::ReleaseInstance();
		return returncode;
	}
	else
	{
		pInterface = (*interfaceList)[sInterfaceID];
	}


	std::cout << "DEVICE LIST" << std::endl;
	std::cout << "###########" << std::endl << std::endl;

	try
	{
		//COUNTING AVAILABLE CAMERAS
		deviceList = pInterface->GetDevices();
		deviceList->Refresh(100);
		std::cout << "5.1.6.  Detected devices:         " << deviceList->size() << std::endl;

		//DEVICE INFORMATION BEFORE OPENING
		for (DeviceList::iterator devIterator = deviceList->begin(); devIterator != deviceList->end(); devIterator++)
		{
			std::cout << "  5.2.3.  Device DeviceID:        " << devIterator->first << std::endl;
			std::cout << "          Device Model:           " << devIterator->second->GetModel() << std::endl;
			std::cout << "          Device Vendor:          " << devIterator->second->GetVendor() << std::endl;
			std::cout << "          Device TLType:          " << devIterator->second->GetTLType() << std::endl;
			std::cout << "          Device AccessStatus:    " << devIterator->second->GetAccessStatus() << std::endl;
			std::cout << "          Device SerialNumber:    " << devIterator->second->GetSerialNumber() << std::endl;
			std::cout << "          Device UserID:          " << devIterator->second->GetDisplayName() << std::endl << std::endl;
		}
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}


	std::cout << "DEVICE" << std::endl;
	std::cout << "######" << std::endl << std::endl;

	//OPEN THE FIRST CAMERA IN THE LIST
	try
	{
		for (DeviceList::iterator devIterator = deviceList->begin(); devIterator != deviceList->end(); devIterator++)
		{
			try
			{
				std::cout << "5.1.7.  Open first device:" << std::endl;
				std::cout << "          Device DeviceID:        " << devIterator->first << std::endl;
				std::cout << "          Device Model:           " << devIterator->second->GetModel() << std::endl;
				std::cout << "          Device Vendor:          " << devIterator->second->GetVendor() << std::endl;
				std::cout << "          Device TLType:          " << devIterator->second->GetTLType() << std::endl;
				std::cout << "          Device AccessStatus:    " << devIterator->second->GetAccessStatus() << std::endl;
				std::cout << "          Device SerialNumber:    " << devIterator->second->GetSerialNumber() << std::endl;
				std::cout << "          Device UserID:          " << devIterator->second->GetDisplayName() << std::endl << std::endl;
				devIterator->second->Open();
				sDeviceID = devIterator->first;
				std::cout << "        Opened device - RemoteNodeList Information:" << std::endl;
				std::cout << "          Device AccessStatus:    " << devIterator->second->GetAccessStatus() << std::endl;

				//SERIAL NUMBER
				if (devIterator->second->GetRemoteNodeList()->GetNodePresent("DeviceSerialNumber"))
					std::cout << "          DeviceSerialNumber:     " << devIterator->second->GetRemoteNode("DeviceSerialNumber")->GetValue() << std::endl;
				else if (devIterator->second->GetRemoteNodeList()->GetNodePresent("DeviceID"))
					std::cout << "          DeviceID (SN):          " << devIterator->second->GetRemoteNode("DeviceID")->GetValue() << std::endl;
				else
					std::cout << "          SerialNumber:           Not Available." << std::endl;

				//DISPLAY DEVICEMANUFACTURERINFO
				if (devIterator->second->GetRemoteNodeList()->GetNodePresent("DeviceManufacturerInfo"))
					std::cout << "          DeviceManufacturerInfo: " << devIterator->second->GetRemoteNode("DeviceManufacturerInfo")->GetValue() << std::endl;

				//DISPLAY DEVICEFIRMWAREVERSION OR DEVICEVERSION
				if (devIterator->second->GetRemoteNodeList()->GetNodePresent("DeviceFirmwareVersion"))
					std::cout << "          DeviceFirmwareVersion:  " << devIterator->second->GetRemoteNode("DeviceFirmwareVersion")->GetValue() << std::endl;
				else if (devIterator->second->GetRemoteNodeList()->GetNodePresent("DeviceVersion"))
					std::cout << "          DeviceVersion:          " << devIterator->second->GetRemoteNode("DeviceVersion")->GetValue() << std::endl;
				else
					std::cout << "          DeviceVersion:          Not Available." << std::endl;

				if (devIterator->second->GetTLType() == "GEV")
				{
					std::cout << "          GevCCP:                 " << devIterator->second->GetRemoteNode("GevCCP")->GetValue() << std::endl;
					std::cout << "          GevCurrentIPAddress:    " << devIterator->second->GetRemoteNode("GevCurrentIPAddress")->GetValue() << std::endl;
					std::cout << "          GevCurrentSubnetMask:   " << devIterator->second->GetRemoteNode("GevCurrentSubnetMask")->GetValue() << std::endl;
				}
				std::cout << "  " << std::endl;
				break;
			}
			catch (BGAPI2::Exceptions::ResourceInUseException& ex)
			{
				returncode = 0 == returncode ? 1 : returncode;
				std::cout << " Device  " << devIterator->first << " already opened." << std::endl;
				std::cout << " ResourceInUseException: " << ex.GetErrorDescription() << std::endl;
			}
			catch (BGAPI2::Exceptions::AccessDeniedException& ex)
			{
				returncode = 0 == returncode ? 1 : returncode;
				std::cout << " Device  " << devIterator->first << " already opened." << std::endl;
				std::cout << " AccessDeniedException " << ex.GetErrorDescription() << std::endl;
			}
		}
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}

	if (sDeviceID == "")
	{
		std::cout << " No Device found." << sDeviceID << std::endl;
		std::cout << std::endl << "End" << std::endl << "Input any number to close the program:";
		int endKey = 0;
		std::cin >> endKey;
		pInterface->Close();
		pSystem->Close();
		BGAPI2::SystemList::ReleaseInstance();
		return returncode;
	}
	else
	{
		pDevice = (*deviceList)[sDeviceID];
	}


	std::cout << "DEVICE PARAMETER SETUP" << std::endl;
	std::cout << "######################" << std::endl << std::endl;

	try
	{
		//SET TRIGGER MODE OFF (FreeRun)
		pDevice->GetRemoteNode(SFNC_PIXELFORMAT)->SetValue("BayerRG8");
		
		pDevice->GetRemoteNode("TriggerSource")->SetString("Software");
		pDevice->GetRemoteNode("TriggerMode")->SetString("On");
		std::cout << "         TriggerMode:             " << pDevice->GetRemoteNode("TriggerMode")->GetValue() << std::endl;
		std::cout << std::endl;

		pDevice->GetRemoteNode(SFNC_EXPOSURETIME)->SetDouble(13000);
		pDevice->GetRemoteNode(SFNC_GAIN)->SetDouble(2.3);

		std::cout<<"Gain:"<<pDevice->GetRemoteNode(SFNC_GAIN)->GetDouble()<<endl;
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}


	std::cout << "DATA STREAM LIST" << std::endl;
	std::cout << "################" << std::endl << std::endl;

	try
	{
		//COUNTING AVAILABLE DATASTREAMS
		datastreamList = pDevice->GetDataStreams();
		datastreamList->Refresh();
		std::cout << "5.1.8.  Detected datastreams:     " << datastreamList->size() << std::endl;

		//DATASTREAM INFORMATION BEFORE OPENING
		for (DataStreamList::iterator dstIterator = datastreamList->begin(); dstIterator != datastreamList->end(); dstIterator++)
		{
			std::cout << "  5.2.4.  DataStream ID:          " << dstIterator->first << std::endl << std::endl;
		}
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}


	std::cout << "DATA STREAM" << std::endl;
	std::cout << "###########" << std::endl << std::endl;

}

int CBaumerCamera::initInterface()
{		
	std::cout << "INTERFACE LIST" << std::endl;
	std::cout << "##############" << std::endl << std::endl;

	try
	{
		interfaceList = pSystem->GetInterfaces();
		//COUNT AVAILABLE INTERFACES
		interfaceList->Refresh(100); // timeout of 100 msec
		std::cout << "        Detected interfaces: " << interfaceList->size() << std::endl;

		//openinterfaces
		for (InterfaceList::iterator ifIterator = interfaceList->begin(); ifIterator != interfaceList->end(); ifIterator++)
		{
			#pragma region output
			std::cout << "        Open interface " << std::endl;
			std::cout << "          Interface ID:      " << ifIterator->first << std::endl;
			std::cout << "          Interface Type:    " << ifIterator->second->GetTLType() << std::endl;
			std::cout << "          Interface Name:    " << ifIterator->second->GetDisplayName() << std::endl << std::endl;
			#pragma endregion
			ifIterator->second->Open();
			//Global Discovery
			ifIterator->second->GetNode("GlobalDiscovery")->SetBool(true);
			std::cout << "          GlobalDiscovery:   " << ifIterator->second->GetNode("GlobalDiscovery")->GetBool() << std::endl;

		}
	}
	#pragma region catch
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}
	#pragma endregion

	std::cout << "INTERFACE" << std::endl;
	std::cout << "#########" << std::endl << std::endl;

	//OPEN THE NEXT INTERFACE IN THE LIST
	try
	{
		for (InterfaceList::iterator ifIterator = interfaceList->begin(); ifIterator != interfaceList->end(); ifIterator++)
		{
			try
			{
				//search for any camera is connetced to this interface
				deviceList = ifIterator->second->GetDevices();
				deviceList->Refresh(100);

				InterfaceList::iterator checkLastIterator = ifIterator;
				checkLastIterator++;
				std::cout << " |" << std::endl;
				std::cout << " +-- " << ifIterator->second->GetDisplayName();
				//std::cout << " (devices: " << deviceList->size() << ")";
				for (bo_uint64 j = 0; j <= ifIterator->second->GetNode("GevInterfaceSubnetSelector")->GetIntMax(); j++)
				{
					ifIterator->second->GetNode("GevInterfaceSubnetSelector")->SetInt(j);
					if (j < ifIterator->second->GetNode("GevInterfaceSubnetSelector")->GetIntMax())
						std::cout << " |";
				}
			

				if (deviceList->size() == 0)
				{
					std::cout << "         Close interface (" << deviceList->size() << " cameras found) " << std::endl << std::endl;
					ifIterator->second->Close();
				}
				else
				{
					sInterfaceID = ifIterator->first;
					#pragma region output

					std::cout << "   " << std::endl;
					std::cout << "        Opened interface - NodeList Information " << std::endl;
					#pragma endregion
					if (ifIterator->second->GetTLType() == "GEV")
					{
						bo_int64 iIpAddress = ifIterator->second->GetNode("GevInterfaceSubnetIPAddress")->GetInt();
						#pragma region output


						std::cout << "          GevInterfaceSubnetIPAddress: " << (iIpAddress >> 24) << "."
							<< ((iIpAddress & 0xffffff) >> 16) << "."
							<< ((iIpAddress & 0xffff) >> 8) << "."
							<< (iIpAddress & 0xff) << std::endl;
						#pragma endregion
						bo_int64 iSubnetMask = ifIterator->second->GetNode("GevInterfaceSubnetMask")->GetInt();

						#pragma region output


						std::cout << "          GevInterfaceSubnetMask:      " << (iSubnetMask >> 24) << "."
							<< ((iSubnetMask & 0xffffff) >> 16) << "."
							<< ((iSubnetMask & 0xffff) >> 8) << "."
							<< (iSubnetMask & 0xff) << std::endl;
						#pragma endregion

						bo_int64 iMACaddress = ifIterator->second->GetNode("GevInterfaceMACAddress")->GetInt();

						#pragma region output

									

						std::cout << "          GevInterfaceMACAddress:      " << std::hex << std::setfill('0') << std::setw(2)
							<< ((iMACaddress & 0xffffffffffffLL) >> 40) << ":" << std::setw(2)
							<< ((iMACaddress & 0xffffffffffLL) >> 32) << ":" << std::setw(2)
							<< ((iMACaddress & 0xffffffff) >> 24) << ":" << std::setw(2)
							<< ((iMACaddress & 0xffffff) >> 16) << ":" << std::setw(2)
							<< ((iMACaddress & 0xffff) >> 8) << ":" << std::setw(2)
							<< (iMACaddress & 0xff) << std::setfill(' ') << std::dec << std::endl;
						#pragma endregion
					}
					if (ifIterator->second->GetTLType() == "U3V")
					{
						//std::cout << "          NodeListCount:     " << ifIterator->second->GetNodeList()->GetNodeCount() << std::endl;    
					}
					std::cout << "  " << std::endl;
							
				}
			}
			catch (BGAPI2::Exceptions::ResourceInUseException& ex)
			{
				returncode = 0 == returncode ? 1 : returncode;
				std::cout << " Interface " << ifIterator->first << " already opened " << std::endl;
				std::cout << " ResourceInUseException: " << ex.GetErrorDescription() << std::endl;
			}
		}
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}



	


	if (sInterfaceID == "")
	{
		std::cout << " No camera found " << sInterfaceID << std::endl;
		std::cout << std::endl << "End" << std::endl << "Input any number to close the program:";
		int endKey = 0;
		std::cin >> endKey;
		pSystem->Close();
		BGAPI2::SystemList::ReleaseInstance();
		return returncode;
	}
	else
	{
		pInterface = (*interfaceList)[sInterfaceID];
	}	
}



int CBaumerCamera::initDataStreamsAndBuffers()
{
	//OPEN THE FIRST DATASTREAM IN THE LIST
	try
	{
		for (DataStreamList::iterator dstIterator = datastreamList->begin(); dstIterator != datastreamList->end(); dstIterator++)
		{
			std::cout << "5.1.9.  Open first datastream:" << std::endl;
			std::cout << "          DataStream ID:          " << dstIterator->first << std::endl << std::endl;
			dstIterator->second->Open();
			sDataStreamID = dstIterator->first;
			std::cout << "        Opened datastream - NodeList Information:" << std::endl;
			std::cout << "          StreamAnnounceBufferMinimum:  " << dstIterator->second->GetNode("StreamAnnounceBufferMinimum")->GetValue() << std::endl;
			if (dstIterator->second->GetTLType() == "GEV")
			{
				std::cout << "          StreamDriverModel:            " << dstIterator->second->GetNode("StreamDriverModel")->GetValue() << std::endl;
			}
			std::cout << "  " << std::endl;
			break;
		}
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}

	if (sDataStreamID == "")
	{
		std::cout << " No DataStream found." << sDataStreamID << std::endl;
		std::cout << std::endl << "End" << std::endl << "Input any number to close the program:";
		int endKey = 0;
		std::cin >> endKey;
		pDevice->Close();
		pInterface->Close();
		pSystem->Close();
		BGAPI2::SystemList::ReleaseInstance();
		return returncode;
	}
	else
	{
		pDataStream = (*datastreamList)[sDataStreamID];
	}


	std::cout << "BUFFER LIST" << std::endl;
	std::cout << "###########" << std::endl << std::endl;

	try
	{
		//BufferList
		bufferList = pDataStream->GetBufferList();

		// 4 buffers using internal buffer mode
		for (int i = 0; i<4; i++)
		{
			pBuffer = new BGAPI2::Buffer();
			bufferList->Add(pBuffer);
		}
		std::cout << "5.1.10.  Announced buffers:       " << bufferList->GetAnnouncedCount() << " using " << pBuffer->GetMemSize() * bufferList->GetAnnouncedCount() << " [bytes]" << std::endl;
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}

	try
	{
		for (BufferList::iterator bufIterator = bufferList->begin(); bufIterator != bufferList->end(); bufIterator++)
		{
			bufIterator->second->QueueBuffer();
		}
		std::cout << "5.1.11.  Queued buffers:          " << bufferList->GetQueuedCount() << std::endl;
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}
	std::cout << std::endl;

}



int CBaumerCamera::registerCallbackFunctions()
{

	//EVENTMODE IMAGE HANDLER
	//=======================

	std::cout << "REGISTER NEW BUFFER EVENT TO: EVENTMODE_EVENT_HANDLER" << std::endl;
	std::cout << "#####################################################" << std::endl << std::endl;
	//SET TRIGGER MODE OFF (FreeRun)
	pDevice->GetRemoteNode("TriggerMode")->SetString("On");
	std::cout << "         TriggerMode:             " << pDevice->GetRemoteNode("TriggerMode")->GetValue() << std::endl;

	try
	{
		pDataStream->RegisterNewBufferEvent(Events::EVENTMODE_EVENT_HANDLER);
		BGAPI2::Events::EventMode currentEventMode = pDataStream->GetEventMode();
		BGAPI2::String sCurrentEventMode = "";
		switch (currentEventMode)
		{
		case BGAPI2::Events::EVENTMODE_POLLING:
			sCurrentEventMode = "EVENTMODE_POLLING";
			break;
		case BGAPI2::Events::EVENTMODE_UNREGISTERED:
			sCurrentEventMode = "EVENTMODE_UNREGISTERED";
			break;
		case BGAPI2::Events::EVENTMODE_EVENT_HANDLER:
			sCurrentEventMode = "EVENTMODE_EVENT_HANDLER";
			break;
		default:
			sCurrentEventMode = "EVENTMODE_UNKNOWN";
		}
		std::cout << "         Register Event Mode:     " << sCurrentEventMode << std::endl << std::endl;
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}

	//REGISTER CALLBACK FUNCTION
	//============================

	std::cout << "REGISTER CALLBACK FUNCTION" << std::endl;
	std::cout << "##########################" << std::endl << std::endl;

	try
	{
		pDataStream->RegisterNewBufferEventHandler(pDataStream, (Events::NewBufferEventHandler) &BufferHandler);
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}
	return returncode;
}


int CBaumerCamera::initializeCameraImagePolling()
{

	try
	{
		pDataStream->RegisterNewBufferEvent(Events::EVENTMODE_POLLING);
		BGAPI2::Events::EventMode currentEventMode = pDataStream->GetEventMode();
		BGAPI2::String sCurrentEventMode = "";
		switch (currentEventMode)
		{
		case BGAPI2::Events::EVENTMODE_POLLING:
			sCurrentEventMode = "EVENTMODE_POLLING";
			break;
		case BGAPI2::Events::EVENTMODE_UNREGISTERED:
			sCurrentEventMode = "EVENTMODE_UNREGISTERED";
			break;
		case BGAPI2::Events::EVENTMODE_EVENT_HANDLER:
			sCurrentEventMode = "EVENTMODE_EVENT_HANDLER";
			break;
		default:
			sCurrentEventMode = "EVENTMODE_UNKNOWN";
		}
		std::cout << "         Register Event Mode:     " << sCurrentEventMode << std::endl << std::endl;
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}


	try
	{
		//SET TRIGGER MODE OFF (FreeRun)
		pDevice->GetRemoteNode("TriggerMode")->SetString("Off");
		std::cout << "         TriggerMode:             " << pDevice->GetRemoteNode("TriggerMode")->GetValue() << std::endl;
		std::cout << std::endl;



		pDevice->GetRemoteNode(SFNC_GEV_HEARTBEATTIMEOUT)->SetInt(10000);
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}
	
	return true;
}



int CBaumerCamera::startAquisition(double & x, double& y)
{
	int posX = -1, posY = -1;
	Mat imgThresholded;
	
	std::clock_t start;
	registerCallbackFunctions();

	pDevice->GetRemoteNode(SFNC_ACQUISITION_START)->Execute();
	char buffer[100];
	sprintf_s(buffer, "check it out: %i\n", GetCurrentThreadId);
	OutputDebugStringA(buffer);

	milliseconds aftertrigger = duration_cast<milliseconds>(
		system_clock::now().time_since_epoch());
	milliseconds beforeArea = duration_cast<milliseconds>(
		system_clock::now().time_since_epoch());

	while (true)
	{

		
		OutputDebugString(L"hit function startAquisition \n");
		try
		{
			
			std::cout << "current thread id"<< endl;

			OutputDebugString(L"before trigger\n");
			std::this_thread::sleep_for(30ms);
			pDevice->GetRemoteNode("TriggerSoftware")->Execute();

			OutputDebugString(L"after trigger\n");
		}
		catch (BGAPI2::Exceptions::IException& ex)
		{
			OutputDebugString(L"hit Exception \n");
			returncode = 0 == returncode ? 1 : returncode;
			std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
			std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
			std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
			break;
		}
			
		beforeArea = duration_cast<milliseconds>(
			system_clock::now().time_since_epoch());
		char buffer[100];
		sprintf_s(buffer, "time before tracking: %d\n", beforeArea);
		OutputDebugStringA(buffer);
			
			//display the current image in the window ----
			try
			{
			

				cvtColor(cameraFeed, imgThresholded, CV_BGR2HSV); // Convert captured frame from BGR to HSV
				inRange(imgThresholded, Scalar(b_min, g_min, r_min), Scalar(b_max, g_max, r_max), imgThresholded); // Threshold the image
				
																															 // Morphological opening (removes small objects from the foreground)
				cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				cv::dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

				// Morphological closing (removes small holes from the foreground)
				//dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
				//erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

				
				// Calculate the moments of the thresholded image
				Moments oMoments = moments(imgThresholded);
				double dM01 = oMoments.m01;
				double dM10 = oMoments.m10;
				double dArea = oMoments.m00;
			
				
				// If Area <= 10000, assume there are no objects in image and it's due to noise
				if (dArea > 5000) {
					// Calculate position of ball
					posX = dM10 / dArea;
					posY = dM01 / dArea;

					circle(cameraFeed, Point(posX, posY), 10, Scalar(0, 0, 255), -1);

					start = std::clock();
					std::cout << "[ x , y ] = [" << posX << " , " << posY << " ] " << start<< endl;
					char buffer[100];
					sprintf_s(buffer, "time difference: %i and %i \n", posX, posY);
					OutputDebugStringA(buffer);
					
				}


				aftertrigger = duration_cast<milliseconds>(
					system_clock::now().time_since_epoch());
				char buffer[100];
				sprintf_s(buffer, "time difference: %d\n", aftertrigger - beforeArea);
				OutputDebugStringA(buffer);
				if (MutexCam.try_lock())
				{
					if (posX>-1 && posY>-1)
					{
						OutputDebugString(L"in mutex Tracking\n");

						x = (double)posX;
						y = (double)posY;
					}
					MutexCam.unlock();
				}
				
				
				cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);// Create a window for display.
				cv::imshow("Original ", cameraFeed);
				cv::waitKey(1);

				
				
				//queueCamOriginal.pop();
				//cout << "#######popque" << queueCamOriginal.size() << endl;
			}
			catch (cv::Exception &e)
			{
				OutputDebugString(L"hit Exception \n");
				std::cout << e.msg << endl;
				
			}
			catch (std::exception &e)
			{
				OutputDebugString(L"hit Exception \n");
				std::cout << "other exception" << endl;


			}
			
	}

	return 1;

}




int CBaumerCamera::startThreaFunctionAquisitionPolling(double & x, double& y)
{
	int posX = -1, posY = -1;
	Mat imgThresholded;
	Mat imageCopy;
	std::clock_t start;

	char buffer[100];
	sprintf_s(buffer, "check it out: %i\n", GetCurrentThreadId);
	OutputDebugStringA(buffer);
	pDevice->GetRemoteNode(SFNC_ACQUISITION_START)->Execute();

	milliseconds aftertrigger = duration_cast<milliseconds>(
		system_clock::now().time_since_epoch());
	milliseconds beforeArea;

	while (true)
	{
		beforeArea = duration_cast<milliseconds>(
			system_clock::now().time_since_epoch());


		char buffer[100];
		sprintf_s(buffer, "time before tracking: %d\n", beforeArea);
		OutputDebugStringA(buffer);
		BGAPI2::Buffer * pBufferFilled = NULL;
		try
		{	
				pBufferFilled = pDataStream->GetFilledBuffer(1500); //timeout 1000 msec
				if (pBufferFilled == NULL)
				{
					std::cout << "Error: Buffer Timeout after 1000 msec" << std::endl;
				}
				else if (pBufferFilled->GetIsIncomplete() == true)
				{
					std::cout << "Error: Image is incomplete" << std::endl;
					// queue buffer again
					pBufferFilled->QueueBuffer();
				}
				else
				{
					// queue buffer again
					cv::Mat imOriginal = cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC1, (char *)((bo_uint64)(pBufferFilled->GetMemPtr()) + pBufferFilled->GetImageOffset()));
					cv::Mat imTransformBGR8 = cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC3); //memory allocation
					bo_uint64 imTransformBGR8_FrameID = pBufferFilled->GetFrameID();
					if (pBufferFilled->GetPixelFormat() == "BayerRG8")
					{	
						cv::cvtColor(imOriginal, imTransformBGR8, CV_BayerBG2BGR); //to BGR
						std::cout << " Image " << std::setw(5) << pBufferFilled->GetFrameID() << " transformed by cv::cvtColor() to 'BGR8'" << std::endl;
					}
					
					Mat reresizedImage;
					cv::Size size(640, 480);
					resize(imTransformBGR8, reresizedImage, size);
					cameraFeed = reresizedImage;
					pBufferFilled->QueueBuffer();
				}
			}
		catch (BGAPI2::Exceptions::IException& ex)
		{
			returncode = 0 == returncode ? 1 : returncode;
			std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
			std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
			std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
		}

		//display the current image in the window ----
		try
		{

			/*
			cvtColor(cameraFeed, imgThresholded, CV_BGR2HSV); // Convert captured frame from BGR to HSV
			inRange(imgThresholded, Scalar(b_min, g_min, r_min), Scalar(b_max, g_max, r_max), imgThresholded); // Threshold the image

																											   // Morphological opening (removes small objects from the foreground)
			cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			cv::dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

			// Morphological closing (removes small holes from the foreground)
			//dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			//erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


			// Calculate the moments of the thresholded image
			Moments oMoments = moments(imgThresholded);
			double dM01 = oMoments.m01;
			double dM10 = oMoments.m10;
			double dArea = oMoments.m00;


			// If Area <= 10000, assume there are no objects in image and it's due to noise
			if (dArea > 5000) {
				// Calculate position of ball
				posX = dM10 / dArea;
				posY = dM01 / dArea;


				start = std::clock();
				std::cout << "[ x , y ] = [" << posX << " , " << posY << " ] " << start << endl;

			}
			*/
			std::vector<int> ids;
			std::vector<std::vector<cv::Point2f> > corners;
			std::vector<std::vector<cv::Point2f> > rejectedPoints;

			cv::aruco::detectMarkers(cameraFeed, dictionary, corners, ids, params, rejectedPoints);
			cout << "Ids size: " << ids.size() << endl;

			if (ids.size() > 0) {

				std::cout << (int)corners[0][0].x << "|" << (int)corners[0][0].y << std::endl;
				posX = (int)corners[0][0].x;
				posY = (int)corners[0][0].y;



				if (MutexCam.try_lock())
				{
					if (posX > -1 && posY > -1)
					{
						OutputDebugString(L"in mutex Tracking\n");

						x = (double)posX;
						y = (double)posY;
					}
					MutexCam.unlock();
				}
			}


			//queueCamOriginal.pop();
			//cout << "#######popque" << queueCamOriginal.size() << endl;
		}
		catch (cv::Exception &e)
		{
			OutputDebugString(L"hit Exception \n");
			std::cout << e.msg << endl;

		}
		catch (std::exception &e)
		{
			OutputDebugString(L"hit Exception \n");
			std::cout << "other exception" << endl;

		}

	}

	return 1;

}


int CBaumerCamera::startAquisitonPolling()
{
		std::cout << "CAMERA START" << std::endl;
		std::cout << "############" << std::endl << std::endl;
		std::clock_t start;
		//START DataStream acquisition
		try
		{
			//pDataStream->StartAcquisition(20000);
			pDataStream->StartAcquisitionContinuous();
			std::cout << "5.1.12.  DataStream started " << std::endl;
		}
		catch (BGAPI2::Exceptions::IException& ex)
		{
			returncode = 0 == returncode ? 1 : returncode;
			std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
			std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
			std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
		}

		//START CAMERA
		try
		{
			std::cout << "5.1.12.  " << pDevice->GetModel() << " started " << std::endl;
			pDevice->GetRemoteNode("AcquisitionStart")->Execute();
		}
		catch (BGAPI2::Exceptions::IException& ex)
		{
			returncode = 0 == returncode ? 1 : returncode;
			std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
			std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
			std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
		}

		return returncode;
}


int CBaumerCamera::AquisitionPolling(double & x, double& y)
{
	
	int posX = -1, posY = -1;
	chrono::milliseconds start, end;

	start = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());
	BGAPI2::Buffer * pBufferFilled = NULL;
	try
	{
		Sleep(15);
		pBufferFilled = pDataStream->GetFilledBuffer(1500); //timeout 1000 msec
		if (pBufferFilled == NULL)
		{
			std::cout << "Error: Buffer Timeout after 1000 msec" << std::endl;
		}
		else if (pBufferFilled->GetIsIncomplete() == true)
		{
			//std::cout << "Error: Image is incomplete" << std::endl;
			// queue buffer again
			pBufferFilled->QueueBuffer();
		}
		else
		{

			
			// queue buffer again
			cv::Mat imOriginal = cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC1, (char *)((bo_uint64)(pBufferFilled->GetMemPtr()) + pBufferFilled->GetImageOffset()));
			cv::Mat imTransformBGR8 = cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC3); //memory allocation
			bo_uint64 imTransformBGR8_FrameID = pBufferFilled->GetFrameID();
			if (pBufferFilled->GetPixelFormat() == "BayerRG8")
			{
				cv::cvtColor(imOriginal, imTransformBGR8, CV_BayerBG2BGR); //to BGR
			    //std::cout << " Image " << std::setw(5) << pBufferFilled->GetFrameID() << " transformed by cv::cvtColor() to 'BGR8'" << std::endl;
			}

			Mat reresizedImage;
			cv::Size size(640, 480);
			resize(imTransformBGR8, reresizedImage, size);
			cameraFeed = reresizedImage;


			//display the current image in the window ----
			try
			{

				std::vector<int> ids;
				std::vector<std::vector<cv::Point2f> > corners;
				std::vector<std::vector<cv::Point2f> > rejectedPoints;

				cv::aruco::detectMarkers(cameraFeed, dictionary, corners, ids, params, rejectedPoints,cameraMatrix,distanceCoefficients);
				//cout << "Ids size: " << ids.size() << endl;

				if (ids.size() > 0)
				{

					//std::cout << (int)corners[0][0].x << "|" << (int)corners[0][0].y;;
					posX = (int)corners[0][0].x;
					posY = (int)corners[0][0].y;

					if (posX > -1 && posY > -1)
					{


						x = (double)posX;
						y = (double)posY;
					}
					cv::aruco::drawDetectedMarkers(cameraFeed, corners, ids);
				}
				cv::namedWindow("Original", cv::WINDOW_AUTOSIZE);// Create a window for display.
				cv::imshow("Original ", cameraFeed);
				cv::waitKey(1);

			}

			catch (cv::Exception &e)
			{
				OutputDebugString(L"hit Exception \n");
				std::cout << e.msg << endl;

			}
			catch (std::exception &e)
			{
				OutputDebugString(L"hit Exception \n");
				std::cout << "other exception" << endl;

			}
			end = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch());

			//std::cout << " -duration: " << (end - start).count() <<" ms"<<std::endl;

			pBufferFilled->QueueBuffer();
		}
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}

}




int CBaumerCamera::stopAquisition()
{
	cv::destroyAllWindows();


	//STOP CAMERA
	try
	{
		//SEARCH FOR 'AcquisitionAbort'
		if (pDevice->GetRemoteNodeList()->GetNodePresent("AcquisitionAbort"))
		{
			pDevice->GetRemoteNode("AcquisitionAbort")->Execute();
			std::cout << "5.1.12.  " << pDevice->GetModel() << " aborted " << std::endl;
		}

		pDevice->GetRemoteNode("AcquisitionStop")->Execute();
		std::cout << "5.1.12.  " << pDevice->GetModel() << " stopped " << std::endl;
		std::cout << std::endl;

		BGAPI2::String sExposureNodeName = "";
		if (pDevice->GetRemoteNodeList()->GetNodePresent("ExposureTime")) {
			sExposureNodeName = "ExposureTime";
		}
		else if (pDevice->GetRemoteNodeList()->GetNodePresent("ExposureTimeAbs")) {
			sExposureNodeName = "ExposureTimeAbs";
		}
		std::cout << "         ExposureTime:                   " << std::fixed << std::setprecision(0) << pDevice->GetRemoteNode(sExposureNodeName)->GetDouble() << " [" << pDevice->GetRemoteNode(sExposureNodeName)->GetUnit() << "]" << std::endl;
		if (pDevice->GetTLType() == "GEV")
		{
			if (pDevice->GetRemoteNodeList()->GetNodePresent("DeviceStreamChannelPacketSize"))
				std::cout << "         DeviceStreamChannelPacketSize:  " << pDevice->GetRemoteNode("DeviceStreamChannelPacketSize")->GetInt() << " [bytes]" << std::endl;
			else
				std::cout << "         GevSCPSPacketSize:              " << pDevice->GetRemoteNode("GevSCPSPacketSize")->GetInt() << " [bytes]" << std::endl;
			std::cout << "         GevSCPD (PacketDelay):          " << pDevice->GetRemoteNode("GevSCPD")->GetInt() << " [tics]" << std::endl;
		}
		std::cout << std::endl;
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}

	//STOP DataStream acquisition
	try
	{
		if (pDataStream->GetTLType() == "GEV")
		{
			//DataStream Statistic
			std::cout << "         DataStream Statistics " << std::endl;
			std::cout << "           DataBlockComplete:              " << pDataStream->GetNodeList()->GetNode("DataBlockComplete")->GetInt() << std::endl;
			std::cout << "           DataBlockInComplete:            " << pDataStream->GetNodeList()->GetNode("DataBlockInComplete")->GetInt() << std::endl;
			std::cout << "           DataBlockMissing:               " << pDataStream->GetNodeList()->GetNode("DataBlockMissing")->GetInt() << std::endl;
			std::cout << "           PacketResendRequestSingle:      " << pDataStream->GetNodeList()->GetNode("PacketResendRequestSingle")->GetInt() << std::endl;
			std::cout << "           PacketResendRequestRange:       " << pDataStream->GetNodeList()->GetNode("PacketResendRequestRange")->GetInt() << std::endl;
			std::cout << "           PacketResendReceive:            " << pDataStream->GetNodeList()->GetNode("PacketResendReceive")->GetInt() << std::endl;
			std::cout << "           DataBlockDroppedBufferUnderrun: " << pDataStream->GetNodeList()->GetNode("DataBlockDroppedBufferUnderrun")->GetInt() << std::endl;
			std::cout << "           Bitrate:                        " << pDataStream->GetNodeList()->GetNode("Bitrate")->GetDouble() << std::endl;
			std::cout << "           Throughput:                     " << pDataStream->GetNodeList()->GetNode("Throughput")->GetDouble() << std::endl;
			std::cout << std::endl;
		}
		if (pDataStream->GetTLType() == "U3V")
		{
			//DataStream Statistic
			std::cout << "         DataStream Statistics " << std::endl;
			std::cout << "           GoodFrames:            " << pDataStream->GetNodeList()->GetNode("GoodFrames")->GetInt() << std::endl;
			std::cout << "           CorruptedFrames:       " << pDataStream->GetNodeList()->GetNode("CorruptedFrames")->GetInt() << std::endl;
			std::cout << "           LostFrames:            " << pDataStream->GetNodeList()->GetNode("LostFrames")->GetInt() << std::endl;
			std::cout << std::endl;
		}
		//BufferList Information
		std::cout << "         BufferList Information " << std::endl;
		std::cout << "           DeliveredCount:        " << bufferList->GetDeliveredCount() << std::endl;
		std::cout << "           UnderrunCount:         " << bufferList->GetUnderrunCount() << std::endl;
		std::cout << std::endl;

		pDataStream->StopAcquisition();
		std::cout << "5.1.12.  DataStream stopped " << std::endl;
		bufferList->DiscardAllBuffers();
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}
	std::cout << std::endl;


	// RESET EVENT MODE TO UNREGISTERED
	//=============================
	try
	{
		pDataStream->UnregisterNewBufferEvent();
		pDataStream->RegisterNewBufferEvent(Events::EVENTMODE_UNREGISTERED);
		BGAPI2::Events::EventMode currentEventMode = pDataStream->GetEventMode();
		BGAPI2::String sCurrentEventMode = "";
		switch (currentEventMode)
		{
		case BGAPI2::Events::EVENTMODE_POLLING:
			sCurrentEventMode = "EVENTMODE_POLLING";
			break;
		case BGAPI2::Events::EVENTMODE_UNREGISTERED:
			sCurrentEventMode = "EVENTMODE_UNREGISTERED";
			break;
		case BGAPI2::Events::EVENTMODE_EVENT_HANDLER:
			sCurrentEventMode = "EVENTMODE_EVENT_HANDLER";
			break;
		default:
			sCurrentEventMode = "EVENTMODE_UNKNOWN";
		}
		std::cout << "         Unregister Event Mode:   " << sCurrentEventMode << std::endl << std::endl;
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}


	std::cout << "RELEASE" << std::endl;
	std::cout << "#######" << std::endl << std::endl;

	//Release buffers
	std::cout << "5.1.13.  Releasing the resources " << std::endl;
	try
	{
		while (bufferList->size() > 0)
		{
			pBuffer = bufferList->begin()->second;
			bufferList->RevokeBuffer(pBuffer);
			delete pBuffer;
		}
		std::cout << "         buffers after revoke:    " << bufferList->size() << std::endl;

		pDataStream->Close();
		pDevice->Close();
		pInterface->Close();
		pSystem->Close();
		BGAPI2::SystemList::ReleaseInstance();
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}

	std::cout << std::endl;
	std::cout << "End" << std::endl << std::endl;

	std::cout << "Input any number to close the program:";
	int endKey = 0;
	std::cin >> endKey;
	return returncode;
}



//CALLBACK FUNCTION
//==================
void BGAPI2CALL BufferHandler(void * callBackOwner, Buffer * pBufferFilled)
{

	//std::cout << "[callback of " << ((BGAPI2::DataStream *) callBackOwner)->GetParent()->GetModel() << "] ";  // device

	try
	{
		
		if (pBufferFilled == NULL)
		{
			std::cout << "Error: Buffer Timeout after 1000 msec" << std::endl;
		}
		else if (pBufferFilled->GetIsIncomplete() == true)
		{
			std::cout << "Error: Image is incomplete" << std::endl;
			// queue buffer again
			pBufferFilled->QueueBuffer();
		}
		else
		{
			std::cout << " Image " << std::setw(5) << pBufferFilled->GetFrameID() << " received in memory address " << pBufferFilled->GetMemPtr() << std::endl;
			// queue buffer again
			cv::Mat imOriginal =cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC1, (char *)((bo_uint64)(pBufferFilled->GetMemPtr()) + pBufferFilled->GetImageOffset()));
			cv::Mat imTransformBGR8 = cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC3); //memory allocation
		
		

			bo_uint64 imTransformBGR8_FrameID = pBufferFilled->GetFrameID();

			if (pBufferFilled->GetPixelFormat() == "BayerRG8")
			{
				//cv::cvtColor(*imOriginal, imTransformBGR8, CV_BayerRG2RGB);
				//Baumer: RGrgrg  >>  OpenCV: rgrgrg
				//        gbgbgb              gBGbgb
				cv::cvtColor(imOriginal, imTransformBGR8, CV_BayerBG2BGR); //to BGR
				




				std::cout << " Image " << std::setw(5) << pBufferFilled->GetFrameID() << " transformed by cv::cvtColor() to 'BGR8'" << std::endl;
			}
			else if (pBufferFilled->GetPixelFormat() == "BayerGB8")
			{
				
				//cv::cvtColor(*imOriginal, *imTransformBGR8, CV_BayerGB2RGB);
				//Baumer: GBgbgb  >>  OpenCV: gbgbgb
				//        rgrgrg              rGRgrg 
				cv::cvtColor(imOriginal, imTransformBGR8, CV_BayerGR2BGR); //to BGR
				std::cout << " Image " << std::setw(5) << pBufferFilled->GetFrameID() << " transformed by cv::cvtColor() to 'BGR8'" << std::endl;
			}
			else if (pBufferFilled->GetPixelFormat() == "BGR8Packed")
			{


				std::cout << "Image BGR8\n";

			}

		
	
			OutputDebugString(L"Image captured \n");
			Mat reresizedImage;
			cv::Size size(640, 480);
			resize(imTransformBGR8, reresizedImage, size);
			cameraFeed = reresizedImage;
				
			pBufferFilled->QueueBuffer();
		}
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}
	return;
}



#pragma region tracking functions



double* CBaumerCamera::getCoordinates() {

	return coordinate;
}


void CBaumerCamera::trackingColorCalibration()
{


#pragma region colorCalibration

	//some boolean variables for different functionality within this
	//program
	bool trackObjects = true;
	bool useMorphOps = true;
	calibrationMode = true;
	//Matrix to store each frame of the webcam feed
	Mat HSV;
	//x and y values for the location of the object
	//matrix storage for binary threshold image
	Mat threshold;
	int x = 0, y = 0;
	double depth = 0;

	bool depthImage_found = false;
	Mat combiImage;
	


	cv::namedWindow(windowName);


	//set mouse callback function to be active on "Webcam Feed" window
	//we pass the handle to our "frame" matrix so that we can draw a rectangle to it
	//as the user clicks and drags the mouse
	cv::setMouseCallback(windowName,  clickAndDrag_Rectangle, &cameraFeed);

	////circle
	vector<int> v3fCircles;  // 3 element vector of floats, this will be the pass by reference output of HoughCircles()
							 //initiate mouse move and drag to false 
	mouseIsDragging = false;
	mouseMove = false;
	rectangleSelected = false;

	//start an infinite loop where webcam feed is copied to cameraFeed matrix
	//all of our operations will be performed within this loop

	int i = 0;
	Size size(620, 480);
	Mat copy;
	Mat depthFeed;
	Mat depthSelected;
	cv::waitKey(100);

#pragma endregion

	Mat imgThresholded;
	int counter = 0;
	vector<int> H;
	vector <int>S;
	vector<int>V;
	bool imageCapured = false;


	std::cout << "CAMERA START" << std::endl;
	std::cout << "############" << std::endl << std::endl;
	std::clock_t start;
	//START DataStream acquisition
	try
	{
		//pDataStream->StartAcquisition(20000);
		pDataStream->StartAcquisitionContinuous();
		std::cout << "5.1.12.  DataStream started " << std::endl;
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}

	//START CAMERA
	try
	{
		std::cout << "5.1.12.  " << pDevice->GetModel() << " started " << std::endl;
		pDevice->GetRemoteNode("AcquisitionStart")->Execute();
	}
	catch (BGAPI2::Exceptions::IException& ex)
	{
		returncode = 0 == returncode ? 1 : returncode;
		std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
		std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
		std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
	}



	while (true)
	{

		BGAPI2::Buffer * pBufferFilled = NULL;
		try
		{
			pBufferFilled = pDataStream->GetFilledBuffer(1000); //timeout 1000 msec
			if (pBufferFilled == NULL)
			{
				std::cout << "Error: Buffer Timeout after 1000 msec" << std::endl;
				imageCapured = false;
			}
			else if (pBufferFilled->GetIsIncomplete() == true)
			{
				std::cout << "Error: Image is incomplete" << std::endl;
				// queue buffer again
				imageCapured = false;
				pBufferFilled->QueueBuffer();
			}
			else
			{
				// queue buffer again
				cv::Mat imOriginal = cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC1, (char *)((bo_uint64)(pBufferFilled->GetMemPtr()) + pBufferFilled->GetImageOffset()));
				cv::Mat imTransformBGR8 = cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC3); //memory allocation
				bo_uint64 imTransformBGR8_FrameID = pBufferFilled->GetFrameID();
				if (pBufferFilled->GetPixelFormat() == "BayerRG8")
				{
					cv::cvtColor(imOriginal, imTransformBGR8, CV_BayerBG2BGR); //to BGR
					std::cout << " Image " << std::setw(5) << pBufferFilled->GetFrameID() << " transformed by cv::cvtColor() to 'BGR8'" << std::endl;
				}

				Mat reresizedImage;
				cv::Size size(640, 480);
				resize(imTransformBGR8, reresizedImage, size);
				cameraFeed = reresizedImage;
				imageCapured = true;
				pBufferFilled->QueueBuffer();
			}
		}
		catch (BGAPI2::Exceptions::IException& ex)
		{
			returncode = 0 == returncode ? 1 : returncode;
			std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
			std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
			std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
		}
		
		if (imageCapured == true)
		{
			try
			{
				cv::Mat temp;
				//convert frame from BGR to HSV colorspace
				cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
				//set HSV values from user selected region
				recordHSV_Values(cameraFeed, HSV);
				//filter HSV image between values and store filtered image to
				//threshold matrix
				inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshold);
				//perform morphological operations on thresholded image to eliminate noise
				//and emphasize the filtered object(s)
				if (useMorphOps)
					morphOps(threshold);

				if (trackObjects)
				{
					if (rectangleSelected)
						trackingStarted = true;
					temp = cameraFeed.clone();
					trackFilteredObject(x, y, threshold, temp);


				}


				if (calibrationMode == true) {

					//create slider bars for HSV filtering
					//createTrackbars();
					//imshow(windowName1, HSV);
					cv::imshow(windowName2, threshold);
					//imshow("depth", depthFeed);
				}
				else {

					destroyWindow(windowName1);
					destroyWindow(windowName2);
					destroyWindow(trackbarWindowName);
				}

				cv::imshow(windowName, temp);
				////imshow("depth", depthFeed);

				auto key = cv::waitKey(30);
				if (key == 'q') break;

				if (key == 'c')
				{

					H.push_back(H_MIN);
					H.push_back(H_MAX);
					S.push_back(S_MIN);
					S.push_back(S_MAX);
					V.push_back(V_MIN);
					V.push_back(V_MAX);
					OutputDebugString(L"calibration finished\n");
				}

			}
			catch (std::exception & e)
			{
			}
		}
		
		
	}
	cv::destroyAllWindows();

	
		b_max = *std::max_element(H.begin(), H.end());
		g_max = *std::max_element(S.begin(), S.end());
		r_max = *std::max_element(V.begin(),V.end());

		b_min = *std::min_element(H.begin(), H.end());
		g_min = *std::min_element(S.begin(), S.end());
		r_min = *std::min_element(V.begin(), V.end());





		OutputDebugString(L"calibration finished\n");
		int posX = -1, posY = -1;
		imageCapured = false;
		while (true)
		{

			BGAPI2::Buffer * pBufferFilled = NULL;
			try
			{
				pBufferFilled = pDataStream->GetFilledBuffer(1000); //timeout 1000 msec
				if (pBufferFilled == NULL)
				{
					std::cout << "Error: Buffer Timeout after 1000 msec" << std::endl;
					imageCapured = false;
				}
				else if (pBufferFilled->GetIsIncomplete() == true)
				{
					std::cout << "Error: Image is incomplete" << std::endl;
					// queue buffer again
					imageCapured = false;
					pBufferFilled->QueueBuffer();
				}
				else
				{
					// queue buffer again
					cv::Mat imOriginal = cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC1, (char *)((bo_uint64)(pBufferFilled->GetMemPtr()) + pBufferFilled->GetImageOffset()));
					cv::Mat imTransformBGR8 = cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC3); //memory allocation
					bo_uint64 imTransformBGR8_FrameID = pBufferFilled->GetFrameID();
					if (pBufferFilled->GetPixelFormat() == "BayerRG8")
					{
						cv::cvtColor(imOriginal, imTransformBGR8, CV_BayerBG2BGR); //to BGR
						std::cout << " Image " << std::setw(5) << pBufferFilled->GetFrameID() << " transformed by cv::cvtColor() to 'BGR8'" << std::endl;
					}

					Mat reresizedImage;
					cv::Size size(640, 480);
					resize(imTransformBGR8, reresizedImage, size);
					cameraFeed = reresizedImage;
					imageCapured = true;
					pBufferFilled->QueueBuffer();
				}
			}
			catch (BGAPI2::Exceptions::IException& ex)
			{
				returncode = 0 == returncode ? 1 : returncode;
				std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
				std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
				std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
			}

			if (imageCapured == true)
			{
				namedWindow("Original", WINDOW_AUTOSIZE);// Create a window for display.
				try {

					cvtColor(cameraFeed, imgThresholded, CV_BGR2HSV); // Convert captured frame from BGR to HSV
					inRange(imgThresholded, Scalar(b_min, g_min, r_min), Scalar(b_max, g_max, r_max), imgThresholded); // Threshold the image

																													   // Morphological opening (removes small objects from the foreground)
					cv::erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
					cv::dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

					// Morphological closing (removes small holes from the foreground)
					//dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
					//erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


					// Calculate the moments of the thresholded image
					Moments oMoments = moments(imgThresholded);
					double dM01 = oMoments.m01;
					double dM10 = oMoments.m10;
					double dArea = oMoments.m00;


					// If Area <= 10000, assume there are no objects in image and it's due to noise
					if (dArea > 5000) {
						// Calculate position of ball
						posX = dM10 / dArea;
						posY = dM01 / dArea;

						circle(cameraFeed, Point(posX, posY), 10, Scalar(0, 0, 255), -1);

						start = std::clock();
						std::cout << "[ x , y ] = [" << posX << " , " << posY << " ] " << start << endl;


						char buffer[100];
						sprintf_s(buffer, "time before tracking: %d   x=%i  y=%i\n", duration_cast<milliseconds>(
							system_clock::now().time_since_epoch()), posX, posY);
						OutputDebugStringA(buffer);


					}


					cvtColor(cameraFeed, imgThresholded, CV_BGR2HSV);
					cv::imshow("Original ", cameraFeed);
					auto key = cv::waitKey(30);
					if (key == 'q') break;


				}
				catch (cv::Exception &e)
				{
					std::cout << e.msg << endl;
				}

			}

		}

	


		//OutputDebugString(L"calibration finished\n");
}



void  CBaumerCamera::on_trackbar(int, void*)
{//This function gets called whenever a
 // trackbar position is changed

 //for now, this does nothing.
}



void CBaumerCamera::createTrackbars() {
	//create window for trackbars


	namedWindow(trackbarWindowName, 0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf_s(TrackbarName, "H_MIN", H_MIN);
	sprintf_s(TrackbarName, "H_MAX", H_MAX);
	sprintf_s(TrackbarName, "S_MIN", S_MIN);
	sprintf_s(TrackbarName, "S_MAX", S_MAX);
	sprintf_s(TrackbarName, "V_MIN", V_MIN);
	sprintf_s(TrackbarName, "V_MAX", V_MAX);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
	//the max value the trackbar can move (eg. H_HIGH), 
	//and the function that is called whenever the trackbar is moved(eg. on_trackbar)
	//                                  ---->    ---->     ---->      
	createTrackbar("H_MIN", trackbarWindowName, &H_MIN, 255);
	createTrackbar("H_MAX", trackbarWindowName, &H_MAX, 255);
	createTrackbar("S_MIN", trackbarWindowName, &S_MIN, 255);
	createTrackbar("S_MAX", trackbarWindowName, &S_MAX, 255);
	createTrackbar("V_MIN", trackbarWindowName, &V_MIN, 255);
	createTrackbar("V_MAX", trackbarWindowName, &V_MAX, 255);


}

void clickAndDrag_Rectangle(int event, int x, int y, int flags, void* param) {
	//only if calibration mode is true will we use the mouse to change HSV values
	if (calibrationMode == true) {
		//get handle to video feed passed in as "param" and cast as Mat pointer
		Mat* videoFeed = (Mat*)param;

		if (event == CV_EVENT_LBUTTONDOWN && mouseIsDragging == false)
		{
			//keep track of initial point clicked
			initialClickPoint = cv::Point(x, y);
			//user has begun dragging the mouse
			mouseIsDragging = true;
		}
		/* user is dragging the mouse */
		if (event == CV_EVENT_MOUSEMOVE && mouseIsDragging == true)
		{
			//keep track of current mouse point
			currentMousePoint = cv::Point(x, y);
			//user has moved the mouse while clicking and dragging
			mouseMove = true;
		}
		/* user has released left button */
		if (event == CV_EVENT_LBUTTONUP && mouseIsDragging == true)
		{
			//set rectangle ROI to the rectangle that the user has selected
			rectangleROI = Rect(initialClickPoint, currentMousePoint);

			//reset boolean variables
			mouseIsDragging = false;
			mouseMove = false;
			rectangleSelected = true;
		}

		if (event == CV_EVENT_RBUTTONDOWN) {
			//user has clicked right mouse button
			//Reset HSV Values
			H_MIN = 0;
			S_MIN = 0;
			V_MIN = 0;
			H_MAX = 255;
			S_MAX = 255;
			V_MAX = 255;

		}
		if (event == CV_EVENT_MBUTTONDOWN) {

			//user has clicked middle mouse button
			//enter code here if needed.
		}
	}
}


void CBaumerCamera::recordHSV_Values(cv::Mat frame, cv::Mat hsv_frame) {

	//save HSV values for ROI that user selected to a vector
	if (mouseMove == false && rectangleSelected == true) {

		//clear previous vector values
		if (H_ROI.size()>0) H_ROI.clear();
		if (S_ROI.size()>0) S_ROI.clear();
		if (V_ROI.size()>0)V_ROI.clear();
		//if the rectangle has no width or height (user has only dragged a line) then we don't try to iterate over the width or height
		if (rectangleROI.width<1 || rectangleROI.height<1) std::cout << "Please drag a rectangle, not a line" << endl;
		else {
			for (int i = rectangleROI.x; i<rectangleROI.x + rectangleROI.width; i++) {
				//iterate through both x and y direction and save HSV values at each and every point
				for (int j = rectangleROI.y; j<rectangleROI.y + rectangleROI.height; j++) {
					//save HSV value at this point
					H_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[0]);
					S_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[1]);
					V_ROI.push_back((int)hsv_frame.at<cv::Vec3b>(j, i)[2]);
				}
			}
		}
		//reset rectangleSelected so user can select another region if necessary
		rectangleSelected = false;
		//set min and max HSV values from min and max elements of each array

		if (H_ROI.size()>0) {
			//NOTE: min_element and max_element return iterators so we must dereference them with "*"
			H_MIN = *std::min_element(H_ROI.begin(), H_ROI.end());
			H_MAX = *std::max_element(H_ROI.begin(), H_ROI.end());
			std::cout << "MIN 'H' VALUE: " << H_MIN << endl;
			std::cout << "MAX 'H' VALUE: " << H_MAX << endl;
		}
		if (S_ROI.size()>0) {
			S_MIN = *std::min_element(S_ROI.begin(), S_ROI.end());
			S_MAX = *std::max_element(S_ROI.begin(), S_ROI.end());
			std::cout << "MIN 'S' VALUE: " << S_MIN << endl;
			std::cout << "MAX 'S' VALUE: " << S_MAX << endl;
		}
		if (V_ROI.size()>0) {
			V_MIN = *std::min_element(V_ROI.begin(), V_ROI.end());
			V_MAX = *std::max_element(V_ROI.begin(), V_ROI.end());
			std::cout << "MIN 'V' VALUE: " << V_MIN << endl;
			std::cout << "MAX 'V' VALUE: " << V_MAX << endl;
		}

	}

	if (mouseMove == true) {
		//if the mouse is held down, we will draw the click and dragged rectangle to the screen
		rectangle(frame, initialClickPoint, cv::Point(currentMousePoint.x, currentMousePoint.y), cv::Scalar(0, 255, 0), 1, 8, 0);
	}


}

string CBaumerCamera::intToString(int number) {


	std::stringstream ss;
	ss << number;
	return ss.str();
}

void CBaumerCamera::drawObject(int x, int y, Mat &frame) {

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!


	//'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window)
	try
	{

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
	}

	catch (cv::Exception & e)
	{

		std::cout << e.msg << endl;
	}

}


void CBaumerCamera::morphOps(Mat &thresh) {

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

	cv::erode(thresh, thresh, erodeElement);
	cv::erode(thresh, thresh, erodeElement);


	cv::dilate(thresh, thresh, dilateElement);
	cv::dilate(thresh, thresh, dilateElement);

}




bool CBaumerCamera::trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed) {

	bool returnValue = false;
	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	//use moments method to find our filtered object
	double refArea = 0;
	int largestIndex = 0;
	bool objectFound = false;
	if (hierarchy.size() > 0) {
		int numObjects = hierarchy.size();
		//if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
		if (numObjects<MAX_NUM_OBJECTS) {
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we save a reference area each
				//iteration and compare it to the area in the next iteration.
				if (area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea) {
					x = moment.m10 / area;
					y = moment.m01 / area;
					objectFound = true;
					refArea = area;
					//save index of largest contour to use with drawContours
					largestIndex = index;
				}
				else objectFound = false;


			}
			//let user know you found an object
			if (objectFound == true) {
				putText(cameraFeed, "Tracking Object", Point(0, 50), 2, 1, Scalar(0, 255, 0), 2);
				//draw object location on screen
				drawObject(x, y, cameraFeed);
				//draw largest contour
				//drawContours(cameraFeed, contours, largestIndex, Scalar(0, 255, 255), 2);
				returnValue = true;
			}

		}
		else
		{
			putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
			returnValue = false;
		}

		return returnValue;
	}
}



void  CBaumerCamera::createChessboardPostion(Size boardSize, float squareEdgeLenth, vector<Point3f>& corners)
{

	for (int i=0;i<boardSize.height;i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(Point3f(j*squareEdgeLenth, i*squareEdgeLenth, 0.0f));
		}

	}
	
}






void CBaumerCamera::getChessboardCorners(vector<cv::Mat> images, vector<vector<cv::Point2f>>& allFoundCorners, bool showResults)
{

	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		vector<Point2f> pointBuf;
		// Change Size from Chessbord 
		bool found = findChessboardCorners(*iter, Size(9, 6), pointBuf, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
		if (found)
		{

			allFoundCorners.push_back(pointBuf);
		}
		if (showResults)
		{

			drawChessboardCorners(*iter, cv::Size(9, 6), pointBuf, found);
			cv::imshow("looking for corners", *iter);
			cv::waitKey(0);

		}



	}
}

void CBaumerCamera::cameraCalibration(vector<cv::Mat> calibrationImages, cv::Size boardSize, float squareEdgelength, cv::Mat& _cameraMatrix, cv::Mat &_distanceCoefficients)
{
	vector<vector<cv::Point2f>> checkerboardImageSpacePoints;
	getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

	vector<vector<Point3f>> worldSpaceCornerPoints(1);
	createChessboardPostion(boardSize, squareEdgelength,worldSpaceCornerPoints[0]);
	worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);


	vector<cv::Mat> rVectors, tVectors;
	distanceCoefficients = cv::Mat::zeros(8, 1, CV_64F);

	calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints, boardSize, _cameraMatrix,_distanceCoefficients,rVectors,tVectors);

}


bool saveCameraCalibration(string name, cv::Mat _cameraMatrix,cv::Mat _distanceCoefficients)
{
	
	ofstream outstream(name);
	if (outstream)
	{
		uint16_t rows = _cameraMatrix.rows;
		uint16_t columns = _cameraMatrix.cols;

		outstream << rows << endl;
		outstream << columns << endl;



		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double value = _cameraMatrix.at<double>(r, c);
				outstream << value << endl;

			}

		}

		rows = _distanceCoefficients.rows;
		columns = _distanceCoefficients.cols;
		cout << "rows: " << rows << "| columns: " << columns<<endl;



		outstream << rows << endl;
		outstream << columns << endl;


		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double value = _distanceCoefficients.at<double>(r, c);
				outstream << value << endl;

			}

		}

		outstream.close();
		return true;





	}
	return false;

}

bool CBaumerCamera::loadCameraCalibration(string name)
{
	ifstream instream(name);


	uint16_t rows;
	uint16_t columns;

	instream >> rows;
    instream >> columns;

	cameraMatrix = cv::Mat(Size(columns, rows),CV_64F);

	if(instream)
	{
		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double read =0.0f;
				instream >> read;
				cameraMatrix.at<double>(r, c) = read;
				cout << cameraMatrix.at<double>(r, c) << endl;

			}

		}

		//Distance coefficients
		instream >> rows;
		instream >> columns;
		distanceCoefficients = cv::Mat::zeros(rows, columns, CV_64F);

		for (int r = 0; r < rows; r++)
		{
			for (int c = 0; c < columns; c++)
			{
				double read = 0.0f;
				instream >> read;
				distanceCoefficients.at<double>(r, c) = read;
				cout << distanceCoefficients.at<double>(r, c) << endl;

			}

		}

		instream.close();
		return true;
	}
	return false;
}





void CBaumerCamera::runCameraCalibration(CRobotCommunication & robot)
{

	double x, y;
	cv::Mat drawToFrame;
	
	vector<cv::Mat>savedImages;

	vector<vector<cv::Point2f>> markerCorners, rejectedCandidates;

	BGAPI2::Buffer * pBufferFilled = NULL;
	int framesPerSecond = 30;


	cv::namedWindow("Baumer TXG14cf", CV_WINDOW_AUTOSIZE);

	while (true)
	{
		try
		{
			Sleep(15);
			pBufferFilled = pDataStream->GetFilledBuffer(1500); //timeout 1000 msec
			if (pBufferFilled == NULL)
			{
				std::cout << "Error: Buffer Timeout after 1000 msec" << std::endl;
			}
			else if (pBufferFilled->GetIsIncomplete() == true)
			{
				//std::cout << "Error: Image is incomplete" << std::endl;
				// queue buffer again
				pBufferFilled->QueueBuffer();
			}
			else
			{


				// queue buffer again
				cv::Mat imOriginal = cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC1, (char *)((bo_uint64)(pBufferFilled->GetMemPtr()) + pBufferFilled->GetImageOffset()));
				cv::Mat imTransformBGR8 = cv::Mat((int)pBufferFilled->GetHeight(), (int)pBufferFilled->GetWidth(), CV_8UC3); //memory allocation
				bo_uint64 imTransformBGR8_FrameID = pBufferFilled->GetFrameID();
				if (pBufferFilled->GetPixelFormat() == "BayerRG8")
				{
					cv::cvtColor(imOriginal, imTransformBGR8, CV_BayerBG2BGR); //to BGR
					//std::cout << " Image " << std::setw(5) << pBufferFilled->GetFrameID() << " transformed by cv::cvtColor() to 'BGR8'" << std::endl;
				}

				Mat reresizedImage;
				cv::Size size(640, 480);
				resize(imTransformBGR8, reresizedImage, size);
				cameraFeed = reresizedImage;

				pBufferFilled->QueueBuffer();


				vector <cv::Vec2f> foundPoints;
				bool found = false;

				found = findChessboardCorners(cameraFeed, chessboardDimensions, foundPoints, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
				cameraFeed.copyTo(drawToFrame);
				drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);

				if (found)
				{

					cv::imshow("Baumer TXG14cf", drawToFrame);

				}
				else
				{
					cv::imshow("Baumer TXG14cf", cameraFeed);
				}
				char character =cv::waitKey(1000 / framesPerSecond);

				switch (character)
				{

				case 'd':
					cout <<endl << "drive robots by key" << endl;
					robot.driveByKey(x,y);
					cout << endl << "driving finished" << endl;
				case ' ':
					if (found)
					{
						cout << "Image save" << endl;
						cv::Mat temp;
						cameraFeed.copyTo(temp);
						savedImages.push_back(temp);


					}

				case 13: 
					//startcalibration // enter button
					if (savedImages.size() > 20)
					{

						
						cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
						saveCameraCalibration("CameraCoefficients", cameraMatrix, distanceCoefficients);
						cout << "calibration finished" << endl;
						return ;
					}
					

					break;
				case 27: //escape button
					//exit
					return ;
					break;

				}



			}
		}
		catch(BGAPI2::Exceptions::IException& ex)
		{

			returncode = 0 == returncode ? 1 : returncode;
			std::cout << "ExceptionType:    " << ex.GetType() << std::endl;
			std::cout << "ErrorDescription: " << ex.GetErrorDescription() << std::endl;
			std::cout << "in function:      " << ex.GetFunctionName() << std::endl;
		
		}
		
	}



}





#pragma endregion




