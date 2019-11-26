/**
* @version 11/08/2018
*/

#include "stdafx.h"
#include "CObjectDetection.h"





CObjectDetection::CObjectDetection(CEventQueue & eventQueueRef) :m_evtQueue(eventQueueRef)
{
	//	Object will be detected in the Main Programm
	m_coordinate[0] = 0;
	m_coordinate[1] = 0;
	m_coordinate[2] = 0;
	m_coordinate[3] = 0;

	// for output file
	out_data.open("Coordinates.txt");
	out_data.clear();
}

void CObjectDetection::Testklasse()
{
	m_coordinate[0] = 75 - 40;  //m_RobotCoordinateX m_coordinate[0] = 75-40, because the start of the platform is 40cm behind the net;
	m_coordinate[1] = 75;	//m_RobotCoordinateY

							//get coordinates from camera + triangulation code
	m_coordinate[2] = 100;	// 143.18; m_BallCoordinateX
	m_coordinate[3] = 120;	// 6.18; m_BallCoordinateY


	//m_evtQueue.addEvent(EV_BallEndPos);

}


CObjectDetection::~CObjectDetection()
{
}

double CObjectDetection::getFps(int &counter, time_t start, time_t end)
{
	double sec;
	double fps;

	time(&end);
	sec = difftime(end, start);
	fps = counter / sec;
	if (counter > 30)
		printf("%.2f fps\n", fps);
	// overflow protection
	if (counter == (INT_MAX - 1000))
		counter = 0;

	return fps;

}

void CObjectDetection::setHSV(color_type color, int& minHue, int& maxHue, int& minSat, int& maxSat, int& minValue, int& maxValue)
{
	switch (color)
	{
	case ORANGE:
		minHue = 8;
		maxHue = 25;
		minSat = 107;
		maxSat = 222;
		minValue = 210;
		maxValue = 255;
		break;

	case YELLOW:
		minHue = 10;
		maxHue = 70;
		minSat = 120;
		maxSat = 255;
		minValue = 100;
		maxValue = 255;
		break;

	case RED:
		minHue = 174;
		maxHue = 179;
		minSat = 189;
		maxSat = 218;
		minValue = 193;
		maxValue = 255;
		break;

	case GREEN:
		minHue = 0;
		maxHue = 179;
		minSat = 29;
		maxSat = 199;
		minValue = 107;
		maxValue = 226;
		break;

		//case ORANGE:
		//	minHue = 104;
		//	maxHue = 109;
		//	minSat = 117;
		//	maxSat = 253;
		//	minValue = 157;
		//	maxValue = 247;
		//	break;

	default:
		break;
	}
}

CCoordinate CObjectDetection::doTransformation(Matrix4d tMatrixE, CCoordinate inputCoord)
{
	Matrix<double, 4, 1> cordVec;
	Vector4d cord_V1 = inputCoord.coord2Vec();
	Matrix<double, 4, 1> t_cordVec; // Transformed coordinate
	Vector4d cord_V2 = tMatrixE * cord_V1;
	CCoordinate transformed_coord;
	transformed_coord.vec2Coord(cord_V2);
	return transformed_coord;
}

CCoordinate CObjectDetection::getCoordinateInRobotFrame(CCoordinate realCoord)
{
	Matrix4d tMatrixE;
	tMatrixE << 0, 0, -1, 5.049,
		-1, 0, 0, 1.762,
		0, 1, 0, 0,
		0, 0, 0, 1;
	return(doTransformation(tMatrixE, realCoord));
}

CCoordinate CObjectDetection::getRealWorldCoordinate(CCoordinate camCoord)
{
	Matrix4d tMatrixE;
	tMatrixE << 1, 0, 0, 1,
		0, 1, 0, 1.285,
		0, 0, 1, 0,
		0, 0, 0, 1;
	return(doTransformation(tMatrixE, camCoord));
}

CCoordinate CObjectDetection::retransformToRealCoord(double phi, double deltaX, CCoordinate inputCoord)
{
	Matrix4d tMatrixE;

	tMatrixE << cos(phi), 0, sin(phi), deltaX,
		0, 1, 0, 0,
		-sin(phi), 0, cos(phi), 0,
		0, 0, 0, 1;

	return(doTransformation(tMatrixE, inputCoord));
}


void CObjectDetection::transformToParabolaPlane(double phi, double deltaX, vector<CCoordinate> const& in_Coords, vector<CCoordinate> &out_coordinates)
{

	Matrix4d tMatrixE;

	tMatrixE << cos(phi), 0, -sin(phi), -deltaX * cos(phi),
		0, 1, 0, 0,
		sin(phi), 0, cos(phi), -deltaX * sin(phi),
		0, 0, 0, 1;

	for (int h = 0; h < in_Coords.size(); h++)
	{
		out_coordinates.push_back(doTransformation(tMatrixE, in_Coords[h]));
	}
}

bool CObjectDetection::lineFit(vector<double> x, vector<double> y, double &m, double &c)
{
	if (x.size() != y.size())
	{
		return false;
	}
	double n = x.size();
	double sumX = 0;
	double sumY = 0;

	for (int i = 0; i < y.size(); i++)
	{
		sumX += x[i];
		sumY += y[i];
	}

	double avgX = sumX / n;
	double avgY = sumY / n;

	double numerator = 0.0;
	double denominator = 0.0;

	for (int i = 0; i<n; ++i) {
		numerator += (x[i] - avgX) * (y[i] - avgY);
		denominator += (x[i] - avgX) * (x[i] - avgX);
	}

	if (denominator == 0) {
		cout << "Cannot calculate LineFit !!!" << endl;
		return false;
	}

	m = numerator / denominator;
	c = y[0] - m * x[0];

	return true;

}


bool CObjectDetection::polyFit(vector<CCoordinate> const&  coordinates, double &coeff_a, double &coeff_b, double &coeff_c, double &slope, double &y_int)
{
	if (!coordinates.empty())
	{
		int n = 2;									// degree of polynomial
		int N = coordinates.size();					//
		int i, j, k, l;								// running variables
		cout.precision(4);                        //set precision
		cout.setf(ios::fixed);

		vector<CCoordinate> transformedCoords;

		vector<double> x_values;
		vector<double> y_values;
		vector<double> z_values;


		for (vector<CCoordinate>::const_iterator it = coordinates.begin(); it != coordinates.end(); ++it)
		{
			x_values.push_back((*it).getX());
			y_values.push_back((*it).getY());
			z_values.push_back((*it).getZ());
		}

		// calling linefit to get the angle

		if (!lineFit(z_values, x_values, slope, y_int))
		{
			return false;
		}
		double phi = atan(slope);

		// transforming coordinates to parabola frame
		transformToParabolaPlane(phi, y_int, coordinates, transformedCoords);


		////For Testing the coordinates in Parabola Frame
		//putInOutputFile(transformedCoords, "Parabola Frame Cordiantes");

		// clearing all vectors to put the transformed coordinate values
		x_values.clear();
		y_values.clear();
		z_values.clear();

		for (vector<CCoordinate>::iterator it = transformedCoords.begin(); it != transformedCoords.end(); ++it)
		{
			x_values.push_back((*it).getZ());
			y_values.push_back((*it).getY());
			z_values.push_back((*it).getX());
		}

		double *sigma_X;
		sigma_X = new double[2 * n + 1];                       //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
		for (i = 0; i < 2 * n + 1; i++)
		{
			sigma_X[i] = 0;
			for (j = 0; j < N; j++)
				sigma_X[i] = sigma_X[i] + pow(x_values[j], i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
		}
		double B[3][4], a[3];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients

		for (i = 0; i <= n; i++)
			for (j = 0; j <= n; j++)
				B[i][j] = sigma_X[i + j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
		double *sigma_Y;
		sigma_Y = new double[n + 1];                   //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
		for (i = 0; i < n + 1; i++)
		{
			sigma_Y[i] = 0;
			for (j = 0; j < N; j++)
				sigma_Y[i] = sigma_Y[i] + pow(x_values[j], i)*y_values[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
		}
		for (i = 0; i <= n; i++)
			B[i][n + 1] = sigma_Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
		n = n + 1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations


		for (i = 0; i < n - 1; i++)            //loop to perform the gauss elimination
			for (k = i + 1; k < n; k++)
			{
				double t = B[k][i] / B[i][i];
				for (j = 0; j <= n; j++)
					B[k][j] = B[k][j] - t * B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
			}
		for (i = n - 1; i >= 0; i--)                //back-substitution
		{                        //x is an array whose values correspond to the values of x,y,z..
			a[i] = B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
			for (j = 0; j < n; j++)
				if (j != i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
					a[i] = a[i] - B[i][j] * a[j];
			a[i] = a[i] / B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
		}

		for (i = 0; i < n; i++)
		{
			if (i == 0)
				coeff_c = a[i];
			else if (i == 1)
				coeff_b = a[i];
			else if (i == 2)
				coeff_a = a[i];
		}
		return true;
	}
	return false;
}


bool CObjectDetection::getLandingCordinate(vector<CCoordinate> const& coordinates, CCoordinate &landingCoord)
{
	bool retValue = false;
	double a = 0;
	double b = 0;
	double c = 0;
	double slope = 0;
	double y_int = 0;
	double x = 0;
	double y = 0;
	double z = 0;
	double beta = 0;
	CCoordinate tempCoord;
	if (polyFit(coordinates, a, b, c, slope, y_int))
	{
		////Analysing parabola
		//analyseParabola(a, b, c, atan(slope), y_int);

		//// Output of Equation line and of parabola
		//out_data << "****************************************************" << endl;
		//out_data << "******** Results of Polyfit & Linefit **************" << endl;
		//out_data << "****************************************************" << endl;
		//out_data << "Angle of Projectile to the Z-Axis: " << atan(slope) * 180 / 3.1416 << " degree" << endl;
		//out_data << "Equation of line: y = (" << slope << ")*x + (" << y_int << ")" << endl;
		//out_data << "Equation of parabola: " << "y = (" << a << ")*x^2 + (" << b << ")*x + (" << c << ")" << endl << endl;


		//out_data << "****************************************************" << endl;
		//out_data << "************* Results of calculation ***************" << endl;
		//out_data << "****************************************************" << endl;



		double disc = pow(b, 2) - 4 * a * c;
		if (disc >= 0)
		{
			double range = (-b - sqrt(disc)) / (2 * a);
			cout << "Calculated Range: " << range << endl;
			x = 0;
			y = 0;
			z = range;

			tempCoord.setAllCoordinates(x, y, z);
			landingCoord = retransformToRealCoord(atan(slope), y_int, tempCoord);
			retValue = true;
		}
		else
		{
			cout << "ERROR: imaginary solution !!!" << endl;
			retValue = false;
		}
	}

	return retValue;
}

void CObjectDetection::putInOutputFile(vector<CCoordinate> const& coords, string coordinateSet)
{


	out_data << coordinateSet << ": " << endl;
	out_data << "______________________________________________" << endl;

	out_data << "x = [";
	for (int i = 0; i < coords.size(); i++)
	{
		out_data << coords[i].getX() << " ";
	}
	out_data << "];" << endl;

	out_data << "y = [";
	for (int i = 0; i < coords.size(); i++)
	{
		out_data << coords[i].getY() << " ";
	}
	out_data << "];" << endl;

	out_data << "z = [";
	for (int i = 0; i < coords.size(); i++)
	{
		out_data << coords[i].getZ() << " ";
	}
	out_data << "];" << endl << endl;

}

void CObjectDetection::printCoordinates(vector<CCoordinate> const & coords, string coordinateSet)
{
	cout << endl;
	for (int i = 0; i < coords.size(); i++)
	{
		cout << i + 1 << ": " << coords[i] << endl;
	}
	cout << endl;
}

void CObjectDetection::generateParabola(double a, double b, double c, vector<CCoordinate>& outCoords)
{
	double h = -b / (2 * a);
	double k = ((4 * a*c) - (b*b)) / (4 * a);

	double dz = 0.2;

	CCoordinate tempCoord;

	double disc = pow(b, 2) - 4 * a * c;

	double end = (-b - sqrt(disc)) / (2 * a);
	double start = (-b + sqrt(disc)) / (2 * a);

	double i = start;

	while (i <= end)
	{
		tempCoord.setX(0);
		tempCoord.setZ(i);
		tempCoord.setY(a*i*i + b * i + c);
		outCoords.push_back(tempCoord);
		tempCoord.resetCoordinates();
		i += dz;
	}
}

void CObjectDetection::analyseParabola(double a, double b, double c, double phi, double deltaX)
{
	// generating parabola coordinate
	vector<CCoordinate> parabolaCoords;
	generateParabola(a, b, c, parabolaCoords);
	vector<CCoordinate> transformaedParabolaCoords;

	for (int i = 0; i < parabolaCoords.size(); i++)
	{
		CCoordinate tCoord = retransformToRealCoord(phi, deltaX, parabolaCoords[i]);
		transformaedParabolaCoords.push_back(tCoord);
	}
	putInOutputFile(transformaedParabolaCoords, "Ideal Parabola Coords in Real Frame");
}

void CObjectDetection::initilization_Kinect()
{
	// Kinect interface object and initialization
	myKinect.initialize_kinect();

}


void CObjectDetection::runKinect()
{
	m_coordinate[0] = 75 - 40;  //m_RobotCoordinateX
	m_coordinate[1] = 75;	//m_RobotCoordinateY
							// Kinect interface object and initialization
	CKinect2Interface myKinect;
	myKinect.initialize_kinect();


	// Setting HSV for the respective color
	int minHue, maxHue, minSat, maxSat, minValue, maxValue;
	setHSV(ORANGE, minHue, maxHue, minSat, maxSat, minValue, maxValue);

	// For x,y,Z cordiante for tracked object
	CCoordinate camCoord;		// coordinate in camera axis
	CCoordinate realCoord;		// coordinate in real world
	CCoordinate landingCoord;	// landing coordinate real world
	CCoordinate landingCoordRobotFrame; // landing coordinate robot world

	// OpenCV Matrices
	Mat cameraFeed;				//Matrix to store each frame of the webcam feed
	Mat HSV;					//matrix storage for HSV image
	Mat imgThresholded;			//matrix storage for binary threshold image
	Mat depthFeed;				//Matrix to store each frame of the depth cam feed
	Mat depthSelected;

	//// for ROI
	//Rect roi = Rect(x, y, w, h);

	bool depthImage_found = false;
	bool detected = false;

	// Size value for resizing OpenCV images
	Size size(620, 480);


	// for fps calculation
	time_t f_start = 0;
	time_t f_end = 0;
	int f_counter = 0;

	double resx, resy;

	bool start = false;

	// for real
	int i = 0;
	int k = 0;
	std::vector<CCoordinate> coordinates;
	std::vector<CCoordinate> allCoordinates;


	// for filters
	int countInf = 0;
	double oldZ = 0;
	double oldY = 0;

	bool startTakingCoords = false;

	// Variable for scaling the image size taken from kinect
	double  frameRatio = 0.4;

	waitKey(10);
	while (1)
	{
		// for fps
		if (f_counter == 0)
		{
			time(&f_start);
		}

		//store image to matrix
		myKinect.get_colorframe(cameraFeed, frameRatio);

		//// selecting roi image
		//Mat image_roi = cameraFeed(roi);

		// for depthmap
		if (myKinect.get_depthframe(depthFeed))
		{
			//imshow("depth", depthFeed);
			depthImage_found = true;
		}

		auto keyStart = cv::waitKey(1);
		if (keyStart == 'k')
			start = true;

		if (depthImage_found)
		{
			cvtColor(cameraFeed, imgThresholded, CV_BGR2HSV); // Convert captured frame from BGR to HSV
			inRange(imgThresholded, Scalar(minHue, minSat, minValue), Scalar(maxHue, maxSat, maxValue), imgThresholded); // Threshold the image

																														 // Morphological opening (removes small objects from the foreground)
			erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

			// Morphological closing (removes small holes from the foreground)
			dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
			erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

			// Calculate the moments of the thresholded image
			Moments oMoments = moments(imgThresholded);
			double dM01 = oMoments.m01;
			double dM10 = oMoments.m10;
			double dArea = oMoments.m00;


			// If Area <= 10000, assume there are no objects in image and it's due to noise<
			if (dArea >= 1000)
			{
				// Calculate position of ball
				int posX = dM10 / dArea;

				int posY = dM01 / dArea;

				circle(cameraFeed, Point(posX, posY), 10, Scalar(0, 0, 255), -1);

				if (myKinect.getCoordinate(posX, posY, camCoord, frameRatio))
				{


					// getting real world coordinate
					realCoord = getRealWorldCoordinate(camCoord);
					//cout << "Real Frame Coordinate: " << realCoord << endl;
					//cout << "Robot Frame Coordinate: " << getCoordinateInRobotFrame(realCoord) << endl;




					if (true)
					{
						allCoordinates.push_back(realCoord);
						if (startTakingCoords)
						{
							k++;
						}

						// projectile calculation with real coord
						if (!isinf(realCoord.getX()) || !isinf(realCoord.getY()) || !isinf(realCoord.getZ()))
						{
							if (!isnan(realCoord.getX()) || !isnan(realCoord.getY()) || !isnan(realCoord.getZ()))
							{
								if (realCoord.getZ() < 2.5)
								{
									startTakingCoords = true;
									cout << "Real Coordinate: " << realCoord << endl;
									coordinates.push_back(realCoord);
									i++;

									if(i==7 || realCoord.getZ() >= 2 || (k-i) > 2)
									{
										if (getLandingCordinate(coordinates, landingCoord))
										{
											cout << endl << "Landing coordinate Real Frame : " << landingCoord << endl;
											landingCoordRobotFrame = getCoordinateInRobotFrame(landingCoord);
											cout << "Landing coordinate Robot Frame  : " << landingCoordRobotFrame << endl;

											//putInOutputFile(allCoordinates, "All Coordinates");
											//printCoordinates(allCoordinates, "All Coordinates");
											//allCoordinates.clear();

											//putInOutputFile(coordinates, "Real Coordinates");
											//printCoordinates(coordinates, "Real Coordinates");
											//coordinates.clear();

											break;


										}
										else
										{
											m_evtQueue.addEvent(EV_BallOutOfRange);
											cout << "Too less value to calculate landing coordinate."<< endl;
											return;
										}
									}

								}
							}
						}

					}
				}
			}
			////imshow("Thresholded ", imgThresholded);
			imshow("Original ", cameraFeed);


			////Printing fps
			//f_counter++;
			//getFps(f_counter, f_start, f_end);
		}

		auto key = cv::waitKey(1);
		if (key == 'q') break;
	}
	cv::destroyAllWindows();
	myKinect.release_kinect();
	out_data.close();

	//todo: connect predicted coordinates
	m_coordinate[2] = 100 * landingCoordRobotFrame.getX();	//m_BallCoordinateX
	m_coordinate[3] = 100 * landingCoordRobotFrame.getY();	//m_BallCoordinateY

	if (((m_coordinate[2] <0) && (m_coordinate[2] > 150)) ||( (m_coordinate[3] < 0)&&(m_coordinate[3] > 150))) {
	
		m_evtQueue.addEvent(EV_BallOutOfRange);
		cout << " OUT OF RANGE !!!!! " << endl;
		return;
	}
	else
	{
		m_evtQueue.addEvent(EV_BallEndPos);
		return;
	}
}



void CObjectDetection::runTest()
{
	CCoordinate landingCoord;

	//// For coordinatesSet_1
	//vector<CCoordinate> coordinatesSet_1;
	//CCoordinate c1(1.2858, 0.8888, 0.999);
	//CCoordinate c2(1.2596, 1.2046, 1.315);
	//CCoordinate c3(1.2363, 1.6102, 1.947);
	//CCoordinate c4(1.2423, 1.7155, 2.249);
	//CCoordinate c5(1.1762, 1.7176, 2.431);
	//CCoordinate c6(1.0887, 0.4823, 3.765);
	//coordinatesSet_1.push_back(c1);
	//coordinatesSet_1.push_back(c2);
	//coordinatesSet_1.push_back(c3);
	//coordinatesSet_1.push_back(c4);
	//coordinatesSet_1.push_back(c5);
	//coordinatesSet_1.push_back(c6);
	//if (getLandingCordinate(coordinatesSet_1, landingCoord))
	//{
	//	//cout << "%%%%%%%%%%%%%%%	Landing coordinate : " << landingCoord << endl;
	//	coordinatesSet_1.clear();
	//	cout << "X: " << landingCoord.getX() << endl;
	//	cout << "Z: " << landingCoord.getZ() << endl;
	//}


	//// For coordinatesSet_2
	//vector<CCoordinate> coordinatesSet_2;
	//CCoordinate c1(1.1405, 1.0515, 0.884);
	//CCoordinate c2(1.1243, 1.3461, 1.096);
	//CCoordinate c3(1.1186, 1.5263, 1.297);
	//CCoordinate c4(1.0944, 1.6736, 1.47);
	//CCoordinate c5(1.0801, 1.7358, 1.671);
	//CCoordinate c6(1.0155, 1.3111, 2.129);
	//CCoordinate c7(0.9917, 0.9807, 2.306);
	//CCoordinate c8(0.9781, 0.5901, 2.43);
	//coordinatesSet_2.push_back(c1);
	//coordinatesSet_2.push_back(c2);
	//coordinatesSet_2.push_back(c3);
	//coordinatesSet_2.push_back(c4);
	//coordinatesSet_2.push_back(c5);
	//coordinatesSet_2.push_back(c6);
	//coordinatesSet_2.push_back(c7);
	//coordinatesSet_2.push_back(c8);
	//if (getLandingCordinate(coordinatesSet_2, landingCoord))
	//{
	//	//cout << "%%%%%%%%%%%%%%%	Landing coordinate : " << landingCoord << endl;
	//	coordinatesSet_2.clear();
	//	cout << "X: " << landingCoord.getX() << endl;
	//	cout << "Z: " << landingCoord.getZ() << endl;
	//}

	// For coordinatesSet_6
	vector<CCoordinate> coordinatesSet_6;
	CCoordinate c1(0.9773, 1.2431, 1.14);
	CCoordinate c2(0.9951, 1.5157, 1.358);
	CCoordinate c3(1.0286, 1.6389, 1.58);
	CCoordinate c4(1.0469, 1.7377, 1.785);
	CCoordinate c5(1.114, 1.6472, 2.132);
	CCoordinate c6(1.1406, 1.4569, 2.382);
	CCoordinate c7(1.2481, 0.275, 3.566);
	coordinatesSet_6.push_back(c1);
	coordinatesSet_6.push_back(c2);
	coordinatesSet_6.push_back(c3);
	coordinatesSet_6.push_back(c4);
	coordinatesSet_6.push_back(c5);
	coordinatesSet_6.push_back(c6);
	coordinatesSet_6.push_back(c7);
	////For Testing the coordinates in Real Frame
	cout << endl << "Coordinates in Real Frame:" << endl;
	for (vector<CCoordinate>::iterator it = coordinatesSet_6.begin(); it != coordinatesSet_6.end(); ++it)
	{
		cout << (*it) << endl;
	}
	if (getLandingCordinate(coordinatesSet_6, landingCoord))
	{
		cout << "Landing coordinate : " << landingCoord << endl << endl;
	}

}



