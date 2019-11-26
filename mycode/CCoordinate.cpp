/**
* @version 11/08/2018
*/
#include "stdafx.h"
#include "CCoordinate.h"

using namespace std;

CCoordinate::~CCoordinate()
{
}

CCoordinate::CCoordinate(double x, double y, double z)
{
	m_X = x;
	m_Y = y;
	m_Z = z;
}

void CCoordinate::setX(double x)
{
	m_X = x;
}

void CCoordinate::setY(double y)
{
	m_Y = y;
}

void CCoordinate::setZ(double z)
{
	m_Z = z;
}

double CCoordinate::getX() const
{
	return m_X;
}

double CCoordinate::getY() const
{
	return m_Y;
}

double CCoordinate::getZ() const
{
	return m_Z;
}


std::ostream & operator<<(std::ostream & out, const CCoordinate & rhs)
{
	out << "[ " << rhs.getX() << ", " << rhs.getY() << ", " << rhs.getZ() << " ]";
	return out;
}

void CCoordinate::resetCoordinates()
{
	m_X = 0;
	m_Y = 0;
	m_Z = 0;
}

void CCoordinate::calculateDeltasTo(CCoordinate const & coordinate2, double & dx, double & dy, double & dz)
{
	dx = this->getX() - coordinate2.getX();
	dy = this->getY() - coordinate2.getY();
	dz = this->getZ() - coordinate2.getZ();
}

void CCoordinate::getAllCoordinates(double & x, double & y, double & z)
{
	x = m_X;
	y = m_Y;
	z = m_Z;
}

void CCoordinate::setAllCoordinates(double x, double y, double z)
{
	m_X = x;
	m_Y = y;
	m_Z = z;
}

Eigen::Vector4d CCoordinate::coord2Vec()
{
	return Eigen::Vector4d(this->getX(), this->getY(), this->getZ(), 1);
}

void CCoordinate::vec2Coord(Eigen::Vector4d vec)
{
	this->setX(vec[0]);
	this->setY(vec[1]);
	this->setZ(vec[2]);
}



