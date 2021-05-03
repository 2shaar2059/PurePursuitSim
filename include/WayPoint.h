#pragma once
#include "CoordinatePoint.h"
#include <string>

class WayPoint {

private: CoordinatePoint point;
	double curvatureAtPoint;
	double TargetVelocityAtPoint;
	double DistanceAlongPathAtPoint;

public:
	WayPoint(CoordinatePoint p1);

	WayPoint(double x, double y);

	CoordinatePoint getPoint();

	double distanceTo(WayPoint wp);

	void setPoint(CoordinatePoint point);

	double getCurvatureAtPoint();

	void setCurvatureAtPoint(double curvatureAtPoint);

	double getTargetVelocityAtPoint();

	void setTargetVelocityAtPoint(double targetVelocityAtPoint);

	double getDistanceAlongPathAtPoint();

	void setDistanceAlongPathAtPoint(double distanceAlongPathAtPoint);

	bool operator==(WayPoint wayPoint);

	bool operator!=(WayPoint wayPoint);

	WayPoint operator/(double scale);

	WayPoint operator*(double scale);

	WayPoint operator+(WayPoint wayPoint);

	WayPoint operator-(WayPoint wayPoint);

	WayPoint getNormalized();

	std::string toString();
};