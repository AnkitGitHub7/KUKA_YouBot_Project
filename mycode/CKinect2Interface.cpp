/**
* @version 11/08/2018
*/
#include "stdafx.h"
#include <thread>
#include <chrono>
#include <ppl.h>
#include "CKinect2Interface.h"


using namespace cv;
using namespace std;

CKinect2Interface::CKinect2Interface()
{
	hResult = S_OK;

	pSensor = nullptr;

	colorWidth = 0;
	colorHeight = 0;
	depthWidth = 0;
	depthHeight = 0;
	colorBytesPerPixel = 0;

}
CKinect2Interface::~CKinect2Interface()
{
	if (pSensor)
	{
		pSensor->Close();
		pSensor->Release();
	}
}
void CKinect2Interface::initialize_kinect()
{
	openKinect();
}

int CKinect2Interface::openKinect()
{
	/*
	ERROR_CHECK(::GetDefaultKinectSensor(&pSensor));
	ERROR_CHECK(pSensor->Open());


	cout << "Kinect is " << (isOpen ? "Open" : "Not Open") << endl;
	*/




	// Create Sensor Instance	
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult)) {
		std::cerr << "Error : GetDefaultKinectSensor" << endl;
		return -1;
	}


	// Open Sensor
	hResult = pSensor->Open();
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::Open()" << endl;
		return -1;
	}

	BOOLEAN isOpen = false;
	pSensor->get_IsOpen(&isOpen);
	if (!isOpen)
	{
		std::cerr << "Kinect is not open" << endl;
		return -1;
	}
	else
	{
		std::cerr << "Kinect is open" << endl;
	}


	pColorReader = get_colorframereader();
	pDepthReader = get_depthframereader();
	pCoordinateMapper = get_coordinatemapper();

	colorBufferSize = colorWidth * colorHeight * 4 * sizeof(unsigned char);
	depthBufferSize = depthWidth * depthHeight * sizeof(unsigned short);

	// To Reserve Color Frame Buffer
	//std::vector<RGBQUAD> colorBuffer( colorWidth * colorHeight );
	colorBuffer.resize(colorWidth * colorHeight);

	// To Reserve Depth Frame Buffer
	//std::vector<UINT16> depthBuffer( depthWidth * depthHeight );
	depthBuffer.resize(depthWidth * depthHeight);


	return 1;
}

IColorFrameReader* CKinect2Interface::get_colorframereader()
{
	// Retrieved Color Frame Source	
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << endl;
		//return -1;
	}
	// Open Color Frame Reader
	//IColorFrameReader* pColorReader;
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult)) {
		std::cerr << "Error : IColorFrameSource::OpenReader()" << endl;
		//return -1;
	}
	// Retrieved Color Frame Size

	hResult = pColorSource->get_FrameDescription(&pColorDescription);
	if (FAILED(hResult)) {
		std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << endl;
		//return -1;
	}
	pColorDescription->get_Width(&colorWidth); // 1920
	pColorDescription->get_Height(&colorHeight); // 1080
	pColorDescription->get_BytesPerPixel(&colorBytesPerPixel);

	return pColorReader;
}
IDepthFrameReader* CKinect2Interface::get_depthframereader()
{
	//// DO get depth image	

	// Retrieved Depth Frame Source	
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_DepthFrameSource()" << endl;
		//return -1;
	}
	// Open Depth Frame Reader
	//IDepthFrameReader* pDepthReader;
	hResult = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(hResult)) {
		std::cerr << "Error : IDepthFrameSource::OpenReader()" << endl;
		//return -1;
	}
	// Retrieved Depth Frame Size

	hResult = pDepthSource->get_FrameDescription(&pDepthDescription);
	if (FAILED(hResult)) {
		std::cerr << "Error : IDepthFrameSource::get_FrameDescription()" << endl;
		//return -1;
	}
	pDepthDescription->get_Width(&depthWidth); // 512
	pDepthDescription->get_Height(&depthHeight); // 424	

												 // getting maximum and minimum reliable depth value
	pDepthSource->get_DepthMaxReliableDistance(&maxDepth);
	pDepthSource->get_DepthMinReliableDistance(&minDepth);

	//cout << "Max Depth: " << maxDepth << " mm" << endl;
	//cout << "Min Depth: " << minDepth << " mm" << endl;


	return pDepthReader;
}
ICoordinateMapper* CKinect2Interface::get_coordinatemapper()
{
	//// Retrieved Coordinate Mapper
	//ICoordinateMapper* pCoordinateMapper;
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult)) {
		std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << endl;
		//return -1;
	}

	return pCoordinateMapper;
}


bool CKinect2Interface::get_colorframe(cv::Mat &colorMat, float COLORSCALE)
{
	bool retValue = false;
	cv::Mat colorBufferMat(colorHeight, colorWidth, CV_8UC4);
	colorMat.create(colorHeight * COLORSCALE, colorWidth * COLORSCALE, CV_8UC4);

	// Acquire Latest Color Frame
	IColorFrame* pColorFrame = nullptr;
	hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hResult)) {

		// Retrieved Color Data for show in window
		hResult = pColorFrame->CopyConvertedFrameDataToArray(colorBufferSize,
			reinterpret_cast<BYTE*>(colorBufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);

		if (SUCCEEDED(hResult)) {

			cv::resize(colorBufferMat, colorMat, cv::Size(), COLORSCALE, COLORSCALE);

			////This flips the image
			//cv::flip(colorMat, colorMat, 1);

			// Retrieved Color Data for pcl
			hResult = pColorFrame->CopyConvertedFrameDataToArray(colorBuffer.size() * sizeof(RGBQUAD),
				reinterpret_cast<BYTE*>(&colorBuffer[0]), ColorImageFormat::ColorImageFormat_Bgra);
			if (FAILED(hResult)) {
				std::cerr << "Error : IColorFrame::CopyConvertedFrameDataToArray()" << endl;
				retValue = false;
			}
			else
				retValue = true;
		}
		else retValue = false;
	}
	else
		retValue = false;
	SafeRelease(pColorFrame);
	return retValue;
}


bool CKinect2Interface::get_depthframe(cv::Mat &depthMat)
{
	cv::Mat bufferMat(depthHeight, depthWidth, CV_16UC1);
	bool retValue = false;
	// Acquire Latest Depth Frame
	IDepthFrame* pDepthFrame = nullptr;
	hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hResult))
	{
		unsigned int bufferSize = depthWidth * depthHeight * sizeof(unsigned short);
		unsigned short* pBuffer = nullptr;
		hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, reinterpret_cast<UINT16**>(&bufferMat.data));
		if (SUCCEEDED(hResult))
		{
			pDepthFrame->CopyFrameDataToArray((UINT)depthBuffer.size(), &depthBuffer[0]);
			// for colorful depthmap
			cv::Mat img0 = cv::Mat::zeros(depthHeight, depthWidth, CV_8UC1);
			double scale = 255.0 / (maxDepth - minDepth);
			bufferMat.convertTo(img0, CV_8UC1, scale);
			depthMat = img0;
			//applyColorMap(img0, depthMat, cv::COLORMAP_JET);

			////// for grey depthmap
			/////* Processing*/
			////bufferMat.convertTo(depthMat, CV_8U, -255.0f / 8000.0f, 255.0f);
			retValue = true;
		}
	}
	SafeRelease(pDepthFrame);
	return retValue;
}



std::vector<UINT16> CKinect2Interface::getDepthBuffer()
{

	IDepthFrame* pDepthFrame = nullptr;
	hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hResult))
	{
		pDepthFrame->CopyFrameDataToArray((UINT)depthBuffer.size(), &depthBuffer[0]);
	}
	else
	{
		cout << "Depthbuffer is not filled!" << endl;
	}

	SafeRelease(pDepthFrame);
	return depthBuffer;
}

std::vector<uint32>	CKinect2Interface::getColorBuffer()
{

	IColorFrame* pColorFrame = nullptr;
	hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hResult))
	{
		pColorFrame->CopyConvertedFrameDataToArray(colorBuffer.size() * sizeof(RGBQUAD),
			reinterpret_cast<BYTE*>(&colorBuffer[0]), ColorImageFormat::ColorImageFormat_Bgra);
	}
	else
	{
		cout << "Colorbuffer is not filled!" << endl;
	}

	SafeRelease(pColorFrame);
	return colorBuffer;
}

cv::Mat CKinect2Interface::mapCoordinate()
{
	getDepthBuffer();
	getColorBuffer();
	cv::Mat colorMat;

	if (!colorBuffer.empty() && !depthBuffer.empty())
	{
		// Retrieve Mapped Coordinates
		std::vector<ColorSpacePoint> colorSpacePoints(depthWidth * depthHeight);
		pCoordinateMapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], colorSpacePoints.size(), &colorSpacePoints[0]);


		// Mapping Color to Depth Resolution
		std::vector<BYTE> buffer(depthWidth * depthHeight * colorBytesPerPixel);

		Concurrency::parallel_for(0, depthHeight, [&](const int depthY) {
			const unsigned int depthOffset = depthY * depthWidth;
			for (int depthX = 0; depthX < depthWidth; depthX++) {
				unsigned int depthIndex = depthOffset + depthX;
				const int colorX = static_cast<int>(colorSpacePoints[depthIndex].X + 0.5f);
				const int colorY = static_cast<int>(colorSpacePoints[depthIndex].Y + 0.5f);
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
					const unsigned int colorIndex = (colorY * colorWidth + colorX) * colorBytesPerPixel;
					depthIndex = depthIndex * colorBytesPerPixel;
					buffer[depthIndex + 0] = colorBuffer[colorIndex + 0];
					buffer[depthIndex + 1] = colorBuffer[colorIndex + 1];
					buffer[depthIndex + 2] = colorBuffer[colorIndex + 2];
					buffer[depthIndex + 3] = colorBuffer[colorIndex + 3];
				}
			}
		});

		colorMat = cv::Mat(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
	}
	else
	{
		cout << "ColorBuffer Size = " << colorBuffer.size() << endl;
		cout << "DpthBuffer Size = " << depthBuffer.size() << endl;
		cout << "Either depth- or colorBuffer is empty!!" << endl;
	}

	return colorMat;

}

//bool CKinect2Interface::getCoordinate(int colorX, int colorY, CCoordinate &coord, double frameRatio)
//{
//	HRESULT hr;
//
//	colorX = colorX / frameRatio;
//	colorY = colorY / frameRatio;
//
//	int reducedWidth = colorWidth * frameRatio;
//	int reducedHeight = colorHeight * frameRatio;
//
//	if (colorX < 1920 && colorY < 1080)
//	{
//		CameraSpacePoint *csp = new CameraSpacePoint[colorWidth * colorHeight];
//
//		hr = pCoordinateMapper->MapColorFrameToCameraSpace(depthBuffer.size(), &depthBuffer[0], colorWidth * colorHeight, csp);
//		if (SUCCEEDED(hr))
//		{
//			coord.setX(csp[(1920 * (uint16)(colorY)) + (uint16)(colorX)].X);
//			coord.setY(csp[(1920 * (uint16)(colorY)) + (uint16)(colorX)].Y);
//			coord.setZ(csp[(1920 * (uint16)(colorY)) + (uint16)(colorX)].Z);
//		}
//
//		delete[] csp;
//		return true;
//	}
//	return false;
//}


bool CKinect2Interface::getCoordinate(int colorX, int colorY, CCoordinate &coord, double frameRatio)
{
	HRESULT hr;

	colorX = colorX / frameRatio;
	colorY = colorY / frameRatio;

	//cout <<endl<< "[Color_X, Color_Y] = [" << colorX << " , "<<colorY<<" ]"<< endl;

	vector < double > x_values;
	vector < double > y_values;
	vector < double > z_values;
	double lowestZ = 0;
	int pos = 0;
	int offset = 3;

	double x = 0;
	double y = 0;
	double z = 0;
	CameraSpacePoint *csp = new CameraSpacePoint[colorWidth * colorHeight];

	if (colorX < 1920 && colorY < 1080)
	{
		hr = pCoordinateMapper->MapColorFrameToCameraSpace(depthBuffer.size(), &depthBuffer[0], colorWidth * colorHeight, csp);
		if (SUCCEEDED(hr))
		{
			if (((1080 - offset) <= colorY || colorY <= offset) || ((1920 - offset) <= colorX || colorX <= offset))
			{
				coord.setX(csp[(1920 * (uint16)(colorY)) + (uint16)(colorX)].X);
				coord.setY(csp[(1920 * (uint16)(colorY)) + (uint16)(colorX)].Y);
				coord.setZ(csp[(1920 * (uint16)(colorY)) + (uint16)(colorX)].Z);
				delete[] csp;
				return true;
			}

			else
			{
				for (int i = colorX - offset; i <= colorX + offset; i++)
				{
					for (int j = colorY - offset; j <= colorY + offset; j++)
					{
						x_values.push_back(csp[(1920 * (uint16)(j)) + (uint16)(i)].X);
						y_values.push_back(csp[(1920 * (uint16)(j)) + (uint16)(i)].Y);
						z_values.push_back(csp[(1920 * (uint16)(j)) + (uint16)(i)].Z);
					}
				}


				// to see the choosen Area
				for (int k = 0; k < z_values.size(); k++)
				{
					if (k == 0)
					{
						lowestZ = 100;
						//cout << "[";
					}
					if (k % (2 * offset + 1) == 0 && k != 0)
					{
						//cout << endl;
					}
					if (!isinf(z_values[k]) && z_values[k] < lowestZ)
					{
						lowestZ = z_values[k];
						pos = k;
					}

					//cout << z_values[k] << " ";
				}
				//cout << endl;
				z = lowestZ;
				x = x_values[pos];
				y = y_values[pos];
				coord.setAllCoordinates(x, y, z);
				delete[] csp;
				return true;
			}
		}
	}
	return false;
}


void CKinect2Interface::release_kinect()
{
	SafeRelease(pColorReader);
	SafeRelease(pDepthReader);
	SafeRelease(pCoordinateMapper);

	SafeRelease(pColorSource);
	SafeRelease(pDepthSource);

	SafeRelease(pColorDescription);
	SafeRelease(pDepthDescription);

	if (pSensor)
	{
		pSensor->Close();
	}
	SafeRelease(pSensor);
}

