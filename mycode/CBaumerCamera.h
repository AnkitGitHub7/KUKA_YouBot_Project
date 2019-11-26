#pragma once

#include <fstream>

#pragma once
//openCv includes
#include "opencv\highgui.h"
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp> 
#include <opencv2\video\video.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/types.hpp"
#include "opencv\cv.h"
#include "opencv2\aruco.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2\ccalib.hpp>



// including of the baumer librarys 
#include "bgapi2_genicam.hpp"
#include "bgapi2_def.h"
#include "bgapi2_featurenames.h"
#include  <list>
#include <map>
#include "CRobotCommunication.h"




typedef void(*my_callback_t) (double,double);

using namespace std;






//Color Calibration 

//initial min and max HSV filter values.
//these will be changed using trackbars
 static int H_MIN;
 static int H_MAX;
 static int S_MIN;
 static int S_MAX;
 static int V_MIN;
 static int V_MAX;
//default capture width and height
 static const int FRAME_WIDTH = 800;
 static const int FRAME_HEIGHT = 600;
//max number of objects to be detected in frame
 static const int MAX_NUM_OBJECTS = 50;
//minimum and maximum object area
 static const int MIN_OBJECT_AREA = 20 * 20;
 static const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;
//names that will appear at the top of each window
 static const string windowName = "Original Image";
 static const string windowName1 = "HSV Image";
 static const string windowName2 = "Thresholded Image";
 static const string windowName3 = "After Morphological Operations";
 static const string trackbarWindowName = "Trackbars";

static bool calibrationMode;//used for showing debugging windows, trackbars etc.

static bool mouseIsDragging;//used for showing a rectangle on screen as user clicks and drags mouse
static bool mouseMove;
static bool rectangleSelected;
static bool trackingStarted;
static cv::Point initialClickPoint, currentMousePoint; //keep track of initial point clicked and current position of mouse
static  cv::Rect rectangleROI; //this is the ROI that the user has selected
static vector<int> H_ROI, S_ROI, V_ROI;// HSV values from the click/drag ROI region stored in separate vectors so that we can sort them easily




class CBaumerCamera
{
	//private Attributes
	private:
		BGAPI2::SystemList *systemList = NULL;
		BGAPI2::System * pSystem = NULL;
		BGAPI2::String sSystemID;

		BGAPI2::InterfaceList *interfaceList = NULL;
		BGAPI2::Interface * pInterface = NULL;
		BGAPI2::String sInterfaceID;

		BGAPI2::DeviceList *deviceList = NULL;
		BGAPI2::Device * pDevice = NULL;
		BGAPI2::String sDeviceID;

		BGAPI2::DataStreamList *datastreamList = NULL;
		BGAPI2::DataStream * pDataStream = NULL;
		BGAPI2::String sDataStreamID;

		BGAPI2::BufferList *bufferList = NULL;
		BGAPI2::Buffer * pBuffer = NULL;
		BGAPI2::String sBufferID;
		int returncode = 0;


#pragma region robot tracking attributes

		int minHue, maxHue, minSat, maxSat, minValue, maxValue;


		bool depthImage_found = false;
		bool detected = false;

		// Size value for resizing OpenCV images

		

		// for time calculation
		time_t start = 0;
		time_t end = 0;
		int counter = 0;


		//bgr values min and max for calibration

		double b_min;
		double g_min;
		double r_min;

		double b_max;
		double g_max;
		double r_max;



		// getter for coordiantes
		double coordinate[2];
		
		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
		cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();

		
		

#pragma endregion
// calibrationg of the camera

		const float calibrationSquareDimension = 0.0243;
		const float arucoSquareDimension = 0.0;
		const cv::Size chessboardDimensions = cv::Size(6, 9);
		cv::Mat distanceCoefficients;
		cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);




public:


	
	double*getCoordinates();
	CBaumerCamera();
	~CBaumerCamera();
	
	
	
	

private:


	int initSystem();


	///<summary>
	///initialization of the System Class 
	///</summary>
	/// <returns>
	/// returns true if the intialization of the system succeeds 
	/// </returns>
	/// <exception cref="BGAPI2::Exceptions::IException">
	/// throws exception if initalization wasn't successful
	/// </exception>
	int initInterface();
	/// <summary>
	/// This method loads the device List 
	/// </summary>
	/// <returns>
	/// returns an iteger value which can be used to check if everything 
	/// was correct
	/// </returns>
	int initDataStreamsAndBuffers();
	int registerCallbackFunctions();


	void on_trackbar(int, void*);
	void createTrackbars();
	
	void recordHSV_Values(cv::Mat frame, cv::Mat hsv_frame);
	string intToString(int number);
	void drawObject(int x, int y, cv::Mat &frame);
	void morphOps(cv::Mat &thresh);
	bool trackFilteredObject(int &x, int &y, cv::Mat threshold, cv::Mat &cameraFeed);
	void cameraCalibration(vector<cv::Mat> calibrationImages, cv::Size boardSize, float squareEdgelength, cv::Mat& _cameraMatrix, cv::Mat &_distanceCoefficients);

public:
	
	
	int initializeCameraImagePolling();
	int startAquisitonPolling();
	int AquisitionPolling(double & x, double& y);
	int startThreaFunctionAquisitionPolling(double & x, double& y);
	int startAquisition(double & x,double & y);
	void createChessboardPostion(cv::Size boardSize, float sqareEdgeLenth, vector<cv::Point3f>& corners);
	void getChessboardCorners(vector<cv::Mat> images, vector<vector<cv::Point2f>> &allFoundCorners, bool showResults );
	void runCameraCalibration(CRobotCommunication & robot);
	bool loadCameraCalibration(string name);
	void initBaumerCamera();
	
	int stopAquisition();
	void trackingColorCalibration();
	
};


