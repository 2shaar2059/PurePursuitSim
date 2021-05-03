#pragma once
#include <math.h>
#include <string>

class CoordinatePoint {

private:
	double x;
	double y;

public:
	CoordinatePoint();

	CoordinatePoint(double x, double y);

	void setCoordinatePoint(double x, double y);
	double getX();

	double getY();

	void setX(double x);

	void setY(double y);


	/*
	 * curvature is defined as (4*A)/(|p1-p2|*|p2-p3|*|p3-p1|) where A is the
	 * area enclosed by the triangle formed by three points, p1 ("this"
	 * coordinate point), p2, and p3, and |p1-p2| is the distance between points
	 * p1 and p2.
	 */
	double getCurvatureFromThreePoints(CoordinatePoint p1, CoordinatePoint p2);

	/*
	 * returns distance between "this" coordinate point and p1
	 */
	double distanceTo(CoordinatePoint p1);

	void setCoordinatePoint(CoordinatePoint p1);

	CoordinatePoint operator/(double scale);

	CoordinatePoint operator*(double scale);

	CoordinatePoint operator+(CoordinatePoint p1);

	CoordinatePoint operator-(CoordinatePoint p1);

	CoordinatePoint getNormalized();

	std::string toString();
};
