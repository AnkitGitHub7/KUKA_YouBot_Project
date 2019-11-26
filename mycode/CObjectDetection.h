/**
* @version 11/08/2018
*/
#pragma once
#include <fstream>
#include <assert.h> 

//include own headers
#include "CEventQueue.h"
#include "CKinect2Interface.h"
#include "CCoordinate.h"

using namespace std;
using namespace cv;
using namespace Eigen;

// enum types for different color
enum color_type { ORANGE, YELLOW, RED, BLUE, GREEN };

class CObjectDetection
{
private:
	ofstream out_data;
	CEventQueue &m_evtQueue;
	CKinect2Interface myKinect;
	friend class CRoute;


protected:
	double m_coordinate[4];

public:
	CObjectDetection(CEventQueue & eventQueue);
	CObjectDetection();
	~CObjectDetection();
	double  getFps(int &counter, time_t start, time_t end);
	void setHSV(color_type color, int& minHue, int& maxHue, int& minSat, int& maxSat, int& minValue, int& maxValue);
	CCoordinate doTransformation(Matrix4d tMatrixE, CCoordinate inputCoord);
	CCoordinate getRealWorldCoordinate(CCoordinate camCoord);
	CCoordinate getCoordinateInRobotFrame(CCoordinate realCoord);
	CCoordinate retransformToRealCoord(double phi, double deltaX, CCoordinate inputCoord);
	void transformToParabolaPlane(double phi, double deltaX, vector<CCoordinate> const& in_Coords, vector<CCoordinate> &out_coordinates);
	bool lineFit(vector<double> x, vector<double> y, double &m, double &c);
	bool polyFit(vector<CCoordinate> const& coordinates, double &coeff_a, double &coeff_b, double &coeff_c, double &slope, double &y_int);
	bool getLandingCordinate(vector<CCoordinate> const& coordinates, CCoordinate &landingCoord);
	void putInOutputFile(vector<CCoordinate> const& coords, string coordinateSet);
	void printCoordinates(vector<CCoordinate> const& coords, string coordinateSet);
	void generateParabola(double a, double b, double c, vector<CCoordinate> & outCoords);
	void analyseParabola(double a, double b, double c, double phi, double deltaX);
	void runKinect();
	void runTest();
	void Testklasse();
	void initilization_Kinect();
	CKinect2Interface getKinect(CKinect2Interface &kinect)
	{
		kinect = myKinect;
	};
};

