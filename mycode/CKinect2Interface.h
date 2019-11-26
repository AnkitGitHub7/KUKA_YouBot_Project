/**
* @version 11/08/2018
*/
#pragma once
#include <iostream>
#include <vector>
#include <Kinect.h>

#include "opencv\highgui.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2\core\core.hpp"
#include "opencv2\video\video.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv\cv.h"
#include <mutex>
#include "CCoordinate.h"

static std::mutex MutexColor;
static std::mutex MutexDeph;



typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long uint64;
typedef char int8;
typedef short int16;
typedef int int32;
typedef long long int64;

class CKinect2Interface
{
public:

	CKinect2Interface();
	~CKinect2Interface();
	void initialize_kinect();
	void release_kinect();
	int openKinect();
	IColorFrameReader* get_colorframereader();
	IDepthFrameReader* get_depthframereader();
	ICoordinateMapper* get_coordinatemapper();
	bool get_colorframe(cv::Mat &colorMat, float COLORSCALE);
	bool getCoordinate(int colorX, int colorY, CCoordinate &coord, double frameRatio);
	bool get_depthframe(cv::Mat &depthMat);
	//bool get_depthframe();
	cv::Mat mapCoordinate();

	// fills and gives the depthBuffer
	std::vector<UINT16> getDepthBuffer();
	//	fills and gives the colorBuffer
	std::vector<uint32>	getColorBuffer();

	template<class Interface>
	inline void SafeRelease(Interface *& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL) {
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}


private:
	HRESULT hResult;

	int colorWidth;
	int colorHeight;
	int depthWidth;
	int depthHeight;

	unsigned int colorBufferSize;
	unsigned int depthBufferSize;

	std::vector<uint32> colorBuffer;
	std::vector<UINT16> depthBuffer;


	IKinectSensor* pSensor;
	IColorFrameSource* pColorSource;
	IFrameDescription* pColorDescription;
	IDepthFrameSource* pDepthSource;
	IFrameDescription* pDepthDescription;
	UINT16 maxDepth;
	UINT16 minDepth;
	unsigned int colorBytesPerPixel;

	IColorFrameReader* pColorReader;
	IDepthFrameReader* pDepthReader;
	ICoordinateMapper* pCoordinateMapper;

};

