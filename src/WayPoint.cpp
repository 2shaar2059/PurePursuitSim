#include "WayPoint.h"
#include "CoordinatePoint.h"
#include <string>

WayPoint::WayPoint(CoordinatePoint p1) {
	this->point = p1;
	this->curvatureAtPoint = 0;
	this->TargetVelocityAtPoint = 0;
	this->DistanceAlongPathAtPoint = 0;
}

WayPoint::WayPoint(double x, double y) {
	this->point = CoordinatePoint(x, y);
	this->curvatureAtPoint = 0;
	this->TargetVelocityAtPoint = 0;
	this->DistanceAlongPathAtPoint = 0;
}

CoordinatePoint WayPoint::getPoint() {
	return point;
}

double WayPoint::distanceTo(WayPoint wp) {
	return (this->point.distanceTo(wp.getPoint()));
}

void WayPoint::setPoint(CoordinatePoint point) {
	this->point = point;
}


double WayPoint::getCurvatureAtPoint() {
	return curvatureAtPoint;
}

void WayPoint::setCurvatureAtPoint(double curvatureAtPoint) {
	this->curvatureAtPoint = curvatureAtPoint;
}

double WayPoint::getTargetVelocityAtPoint() {
	return TargetVelocityAtPoint;
}

void WayPoint::setTargetVelocityAtPoint(double targetVelocityAtPoint) {
	TargetVelocityAtPoint = targetVelocityAtPoint;
}

double WayPoint::getDistanceAlongPathAtPoint() {
	return DistanceAlongPathAtPoint;
}

void WayPoint::setDistanceAlongPathAtPoint(double distanceAlongPathAtPoint) {
	DistanceAlongPathAtPoint = distanceAlongPathAtPoint;
}


bool WayPoint::operator==(WayPoint wayPoint) {
	if (this->curvatureAtPoint == wayPoint.getCurvatureAtPoint()
	        && this->TargetVelocityAtPoint == wayPoint.getTargetVelocityAtPoint()
	        && this->DistanceAlongPathAtPoint == wayPoint.getDistanceAlongPathAtPoint()) {
		return true;
	}
	else {
		return false;
	}
}

WayPoint WayPoint::getNormalized() {
	return WayPoint(this->point.getNormalized());
}

bool WayPoint::operator!=(WayPoint wp) {
	return !(*this == wp);
}

WayPoint WayPoint::operator*(double scale) {
	return WayPoint(this->point * scale);
}

WayPoint WayPoint::operator/(double scale) {
	return WayPoint(this->point / scale);
}

WayPoint WayPoint::operator+(WayPoint wp) {
	return WayPoint(this->point + wp.point);
}

WayPoint WayPoint::operator-(WayPoint wp) {
	return WayPoint(this->point - wp.point);
}

std::string WayPoint::toString()
{
	return ("Point:" + point.toString() + " CurvatureAtPoint:" + std::to_string(curvatureAtPoint) + " TargetVelocityAtPoint:"
	        + std::to_string(TargetVelocityAtPoint) + " DistanceAlongPathAtPoint:" + std::to_string(DistanceAlongPathAtPoint));
}