/**
* @version 11/08/2018
*/
#pragma once

#include <iostream>
#include <string>
#include "eigen3/Eigen/Dense"

class CCoordinate
{
private:
	double m_X, m_Y, m_Z;
public:
	CCoordinate(double x = 0, double y = 0, double z = 0);
	void setX(double x);
	void setY(double y);
	void setZ(double z);
	double getX() const;
	double getY() const;
	double getZ() const;
	friend std::ostream& operator<<(std::ostream& out, const CCoordinate& rhs);
	void resetCoordinates();
	void calculateDeltasTo(CCoordinate const & coordinate2, double & dx, double & dy, double & dz);
	void getAllCoordinates(double &x, double &y, double &z);
	void setAllCoordinates(double x, double y, double z);
	Eigen::Vector4d coord2Vec();
	void vec2Coord(Eigen::Vector4d vec);

	~CCoordinate();
};

