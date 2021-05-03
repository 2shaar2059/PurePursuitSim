#include "CoordinatePoint.h"

#include <math.h>
#include <string>

CoordinatePoint::CoordinatePoint() {
	x = y = 0;
}

CoordinatePoint::CoordinatePoint(double x, double y) {
	this->x = x;
	this->y = y;
}

void CoordinatePoint::setCoordinatePoint(double x, double y) {
	this->x = x;
	this->y = y;
}

void CoordinatePoint::setCoordinatePoint(CoordinatePoint p1) {
	this->x = p1.getX();
	this->y = p1.getY();
}

double CoordinatePoint::getX() {
	return x;
}

double CoordinatePoint::getY() {
	return y;
}

void CoordinatePoint::setX(double x) {
	this->x = x;
}

void CoordinatePoint::setY(double y) {
	this->y = y;
}

/*
 * curvature is defined as (4*A)/(|p1-p2|*|p2-p3|*|p3-p1|) where A is the
 * area enclosed by the triangle formed by three points, p1 ("this"
 * coordinate point), p2, and p3, and |p1-p2| is the distance between points
 * p1 and p2.
 */
double CoordinatePoint::getCurvatureFromThreePoints(CoordinatePoint p1, CoordinatePoint p2) {
	double len1 = this->distanceTo(p1);
	double len2 = p1.distanceTo(p2);
	double len3 = p2.distanceTo(*this);
	double semiPerimeter = 0.5 * (len1 + len2 + len3);
	double area = (sqrt(semiPerimeter * (semiPerimeter - len1) * (semiPerimeter - len2) * (semiPerimeter - len3))); // Heron's Formula
	double curvature = (4 * area) / (len1 * len2 * len3);
	return curvature;
}

/*
 * returns distance between "this" coordinate point and p1
 */
double CoordinatePoint::distanceTo(CoordinatePoint p1) {
	return (sqrt(pow(p1.getX() - this->getX(), 2.0) + pow(p1.getY() - this->getY(), 2.0)));
}

CoordinatePoint CoordinatePoint::operator*(double scale) {
	return CoordinatePoint(this->x*scale, this->y*scale);
}

CoordinatePoint CoordinatePoint::operator/(double scale) {
	return CoordinatePoint(this->x/scale, this->y/scale);
}

CoordinatePoint CoordinatePoint::operator+(CoordinatePoint p1) {
	return CoordinatePoint(this->x+p1.x, this->y+p1.y);
}

CoordinatePoint CoordinatePoint::operator-(CoordinatePoint p1) {
	return CoordinatePoint(this->x-p1.x, this->y-p1.y);
}

CoordinatePoint CoordinatePoint::getNormalized() {
	double magnitude = sqrt(pow(this->x,2)+pow(this->y,2));
	return CoordinatePoint((*this)/magnitude);
}

std::string CoordinatePoint::toString() {
	return ("(" + std::to_string(x) + ", " + std::to_string(y) + ")");
}
