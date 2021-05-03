#pragma once
#include "WayPoint.h"
#include "CoordinatePoint.h"
#include "math.h"

class Vector {
private:
	double XComponent;
	double YComponent;
	double magnitude;

public:
	/*
	* create a blank vector	*/
	Vector();
	/*
	* create a vector pointing from CoordinatePoint p1 to CoordinatePoint p2
	*/
	Vector(CoordinatePoint p1, CoordinatePoint p2);
	/*
	 * create a vector from a CoordinatePoint p1
	 */
	Vector(CoordinatePoint p1);
	/*
	 * create a CoordinatePoint from "this" Vector
	 */
	CoordinatePoint toCoordinatePoint();
	/*
	 * create a vector with components XComponent and YComponent
	 */
	Vector(double XComponent, double YComponent);

	double getXComponent();

	double getYComponent();

	double getmagnitude();

	std::string toString();

	/**
	 * @param v1
	 * @return "this" vector crossed with v1
	 */
	double Cross(Vector v1);

	/**
	 * @param v1
	 * @return "this" vector dotted with v1
	 */
	double Dot(Vector v1);

	/**
	 * @param v1
	 * @return "this" vector added to v1
	 */
	Vector Add(Vector v1);

	/**
	 * @param v1
	 * @return "this" vector minus v1
	 */
	Vector Subtract(Vector v1);

	Vector ScalarMultiply(double s);

	/**
	 *
	 * @param v1
	 * @return the scalar projection of "this" vector onto v1
	 */
	double proj(Vector v1);
};