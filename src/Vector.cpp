#include "Vector.h"
#include "WayPoint.h"
#include "CoordinatePoint.h"
#include "math.h"


	/*
	 * create a blank vector
	 */
	Vector::Vector() {
		XComponent = 0;
		YComponent = 0;
		magnitude = 0;
	}

	/*
	 * create a vector pointing from CoordinatePoint p1 to CoordinatePoint p2
	 */
	Vector::Vector(CoordinatePoint p1, CoordinatePoint p2) {
		XComponent = p2.getX() - p1.getX();
		YComponent = p2.getY() - p1.getY();
		magnitude = p1.distanceTo(p2);
	}
	
	/*
	 * create a vector from a CoordinatePoint p1
	 */
	Vector::Vector(CoordinatePoint p1) {
		XComponent = p1.getX();
		YComponent = p1.getY();
		magnitude = sqrt((XComponent * XComponent) + (YComponent * YComponent));
	}

	/*
	 * create a CoordinatePoint from "this" Vector
	 */
	CoordinatePoint Vector::toCoordinatePoint() {
		return CoordinatePoint(this->getXComponent(), this->getYComponent());
	}

	/*
	 * create a vector with components XComponent and YComponent
	 */
	Vector::Vector(double XComponent, double YComponent) {
		this->XComponent = XComponent;
		this->YComponent = YComponent;
		magnitude = sqrt((XComponent * XComponent) + (YComponent * YComponent));
	}

	double Vector::getXComponent() {
		return XComponent;
	}

	double Vector::getYComponent() {
		return YComponent;
	}

	double Vector::getmagnitude() {
		return magnitude;
	}

	std::string Vector::toString() {
		return ("<" + std::to_string(XComponent) + ", " + std::to_string(YComponent) + ">");
	}

	/**
	 * @param v1
	 * @return "this" vector crossed with v1
	 */
	double Vector::Cross(Vector v1) {
		return (this->XComponent * v1.getYComponent()) - (this->YComponent * v1.getXComponent());
	}

	/**
	 * @param v1
	 * @return "this" vector dotted with v1
	 */
	double Vector::Dot(Vector v1) {
		return (this->XComponent * v1.getXComponent()) + (this->YComponent * v1.getYComponent());
	}

	/**
	 * @param v1
	 * @return "this" vector added to v1
	 */
	Vector Vector::Add(Vector v1) {
		return Vector(this->XComponent + v1.getXComponent(), this->YComponent + v1.getYComponent());
	}

	/**
	 * @param v1
	 * @return "this" vector minus v1
	 */
	Vector Vector::Subtract(Vector v1) {
		return Vector(this->XComponent - v1.getXComponent(), this->YComponent - v1.getYComponent());
	}

	Vector Vector::ScalarMultiply(double s) {
		return Vector(this->XComponent * s, this->YComponent * s);
	}

	/**
	 * 
	 * @param v1
	 * @return the scalar projection of "this" vector onto v1
	 */
	double Vector::proj(Vector v1) {
		return (this->Dot(v1) / v1.getmagnitude());
	}