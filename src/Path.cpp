#include "WayPoint.h"
#include "Vector.h"
#include "Path.h"
#include "mousePointer.h"

#include "math.h"
#include "vector"
#include <algorithm>
#include <iostream>

Path::Path(std::vector <WayPoint> unsmoothedPoints, double a, double b, double tolerance, double max_vel, double max_centrip_accel, bool inToM) {
	this->unsmoothedPoints = unsmoothedPoints;

	if (inToM) { //if we need to convert inch values to meters
		for (int i = 0; i < this->unsmoothedPoints.size(); i++) {
			this->unsmoothedPoints[i] = this->unsmoothedPoints[i] * 0.0254;
		}
	}


	std::vector <WayPoint> evenlySpacedPoints = this->unsmoothedPoints;

	int initialNumberOfPoints = unsmoothedPoints.size();

	double wayPointSpacing = 0.05; //[m]
	for (int i = 0; i < evenlySpacedPoints.size()/* - initialNumberOfPoints*/; i++) {
		if (i == evenlySpacedPoints.size() - 1) {
			break;
		}
		if (evenlySpacedPoints[i].distanceTo(evenlySpacedPoints[i + 1]) > wayPointSpacing) {
			WayPoint newPoint = evenlySpacedPoints[i] + (evenlySpacedPoints[i + 1] - evenlySpacedPoints[i]).getNormalized() * wayPointSpacing;
			evenlySpacedPoints.insert(evenlySpacedPoints.begin() + i + 1, newPoint);
		}
	}
	std::vector <WayPoint> smoothedPoints = evenlySpacedPoints;


	/*
		for (int interations = 0; interations < 1; interations++) {
			for (int i = 1; i < smoothedPoints.size() - 1; i++) {
				smoothedPoints[i] = (smoothedPoints[i - 1] + smoothedPoints[i + 1]) / 2.0;
			}
		}
	*/

	/*std::vector <WayPoint> smoothedPoints = this->unsmoothedPoints;

		double change = tolerance;
		while (change >= tolerance) {
			change = 0.0;
			for (int i = 1; i < unsmoothedPoints.size() - 1; i++) {
				Vector previousPointSmooth =  Vector(smoothedPoints[i - 1].getPoint());
				Vector currentPointSmooth =  Vector(smoothedPoints[i].getPoint());
				Vector nextPointSmooth =  Vector(smoothedPoints[i + 1].getPoint());
				Vector currentPointUnsmooth =  Vector(smoothedPoints[i].getPoint());

				double temp1 = smoothedPoints[i].getPoint().getX();
				double temp2 = smoothedPoints[i].getPoint().getY();
				smoothedPoints[i].getPoint()
				.setCoordinatePoint(currentPointSmooth
				                    .Add(currentPointUnsmooth.Subtract(currentPointSmooth).ScalarMultiply(a)
				                         .Add(previousPointSmooth.Add(nextPointSmooth)
				                              .Subtract(currentPointSmooth.ScalarMultiply(2.0)).ScalarMultiply(b)))
				                    .toCoordinatePoint());
				change += abs(temp1 - smoothedPoints[i].getPoint().getX());
				change += abs(temp2 - smoothedPoints[i].getPoint().getY());
			}
		}*/


	double maxCurvature = max_centrip_accel / pow(max_vel, 2);
	smoothedPoints[0].setTargetVelocityAtPoint(max_vel);
	for (int i = 1; i < smoothedPoints.size() - 1; i++) {
		double curvature = smoothedPoints[i].getPoint().getCurvatureFromThreePoints(smoothedPoints[i - 1].getPoint(), smoothedPoints[i + 1].getPoint());
		smoothedPoints[i].setDistanceAlongPathAtPoint(smoothedPoints[i - 1].getPoint().distanceTo(smoothedPoints[i].getPoint()) + smoothedPoints[i - 1].getDistanceAlongPathAtPoint());
		smoothedPoints[i].setCurvatureAtPoint(curvature);
		smoothedPoints[i].setTargetVelocityAtPoint(std::min(max_vel, abs(sqrt(max_centrip_accel / curvature))));
	}
	int lastidx = smoothedPoints.size() - 1;
	smoothedPoints[lastidx].setDistanceAlongPathAtPoint(smoothedPoints[lastidx - 1].distanceTo(smoothedPoints[lastidx]) + smoothedPoints[lastidx - 1].getDistanceAlongPathAtPoint());

	/*
	smoothedPoints[0].setCurvatureAtPoint(0.0);
	smoothedPoints[0].setTargetVelocityAtPoint(max_vel);
	smoothedPoints[0].setDistanceAlongPathAtPoint(0.0);

	smoothedPoints[smoothedPoints.size() - 1].setCurvatureAtPoint(0.0);
	smoothedPoints[smoothedPoints.size() - 1].setTargetVelocityAtPoint(0.0);

	for (int i = 1; i < smoothedPoints.size(); i++) {
		smoothedPoints[i].setDistanceAlongPathAtPoint(smoothedPoints[i - 1].getPoint().distanceTo(smoothedPoints[i].getPoint()) + smoothedPoints[i - 1].getDistanceAlongPathAtPoint());
		if (i != smoothedPoints.size() - 1) {
			double curvature = smoothedPoints[i].getPoint().getCurvatureFromThreePoints(smoothedPoints[i - 1].getPoint(), smoothedPoints[i + 1].getPoint());
			smoothedPoints[i].setCurvatureAtPoint(curvature);
			smoothedPoints[i].setTargetVelocityAtPoint(std::min(max_vel, abs(sqrt(max_centrip_accel / curvature))));
		}
	}
	smoothedPoints[smoothedPoints.size() - 1].setCurvatureAtPoint(0.0);*/

	this->smoothedPoints = smoothedPoints;
}

std::vector <WayPoint> Path::getSmoothedPoints() {
	return smoothedPoints;
}

void Path::setSmoothedPoints(std::vector <WayPoint> smoothedPoints) {
	this->smoothedPoints = smoothedPoints;
}

std::vector <WayPoint> Path::getunsmoothedPoints() {
	return unsmoothedPoints;
}

void Path::setunsmoothedPoints(std::vector <WayPoint> unsmoothedPoints) {
	this->unsmoothedPoints = unsmoothedPoints;
}

int Path::getidxOfLastClosestPoint() {
	return this->idxOfLastClosestPoint;
}

void Path::setidxOfLastClosestPoint(int idx) {
	this->idxOfLastClosestPoint = idx;
}


int Path::idxOfClosestPointTo(CoordinatePoint robotLocation) {
	std::vector <double> distances;
	for (int i = 0; i < this->smoothedPoints.size(); i++) {
		distances.push_back(this->smoothedPoints[i].getPoint().distanceTo(robotLocation));
	}
	return std::min_element(distances.begin(), distances.end()) - distances.begin();
}

/**
 *
 * @param path   array of coordinate points
 * @param robotX current robot X position
 * @param robotY current robot Y position
 * @return a point on path that is closest to the robot
 */
WayPoint Path::closestPointTo(CoordinatePoint robotLocation) {
	return this->smoothedPoints[this->idxOfClosestPointTo(robotLocation)];
}


WayPoint Path::closestUnvisitedPointTo(CoordinatePoint p1) {
	double next_distance = 0;
	double next_next_distance = 0;

	for (int i = this->idxOfLastClosestPoint; i < this->smoothedPoints.size(); i++) {
		next_distance = p1.distanceTo(this->smoothedPoints[i].getPoint());
		next_next_distance = p1.distanceTo(this->smoothedPoints[i + 1].getPoint());

		if (next_next_distance > next_distance) {
			idxOfLastClosestPoint = i;
			break;
		}
	}
	return this->smoothedPoints[idxOfLastClosestPoint];
}

/**
 *
 * @param startPointofSegment starting point of the line segment
 * @param endPointofSegment   end point of the line segment
 * @param robotLocation       center point of the circle (Robot Location) as a
 *                            CoordinatePoint with radius of lookahead distance
 * @param lookaheadDistance   radius of the circle (lookahead distance)
 * @return the Coordinate Point on the line segment (defined by startPointofSegment
 *         and endPointofSegment) that is lookaheadDistance away from the robot
 *         location
 */
CoordinatePoint Path::findLookaheadPoint(CoordinatePoint robotLocation, double lookaheadDistance) {
	bool pointFound = false;
	CoordinatePoint startPointofSegment =  CoordinatePoint();
	CoordinatePoint endPointofSegment =  CoordinatePoint();
	Vector d = Vector();


	for (int i = this->idxOfLastClosestPoint; i < this->getSmoothedPoints().size() - 1  && !pointFound; i++) {
		startPointofSegment = this->smoothedPoints[i].getPoint();
		endPointofSegment = this->smoothedPoints[i + 1].getPoint();

		d =  Vector(startPointofSegment, endPointofSegment);
		Vector f =  Vector(robotLocation, startPointofSegment);

		double a = d.Dot(d);
		double b = 2 * (f.Dot(d));
		double c = f.Dot(f) - (lookaheadDistance * lookaheadDistance);
		double discriminant = b * b - 4 * a * c;
		double t = 0;

		if (discriminant < 0) {
			//System.out.println("No Intersection");
		} else {
			discriminant = sqrt(discriminant);
			double t1 = (-b - discriminant) / (2 * a);
			double t2 = (-b + discriminant) / (2 * a);

			if (t1 >= 0 && t1 <= 1) {
				t = t1;
				pointFound = true;
				this->idxOfLastClosestPoint = i;
			}
			if (t2 >= 0 && t2 <= 1) {
				t = t2;
				pointFound = true;
				this->idxOfLastClosestPoint = i;
			}
		}
		d = d.ScalarMultiply(t);
	}/*
	
		for (int i = this->getSmoothedPoints().size() - 2; i >= 0 && !pointFound; i--) {
			startPointofSegment = this->smoothedPoints[i].getPoint();
			endPointofSegment = this->smoothedPoints[i + 1].getPoint();

			d =  Vector(startPointofSegment, endPointofSegment);
			Vector f =  Vector(robotLocation, startPointofSegment);

			double a = d.Dot(d);
			double b = 2 * (f.Dot(d));
			double c = f.Dot(f) - (lookaheadDistance * lookaheadDistance);
			double discriminant = b * b - 4 * a * c;
			double t = 0;

			if (discriminant < 0) {
				//System.out.println("No Intersection");
			} else {
				discriminant = sqrt(discriminant);
				double t1 = (-b - discriminant) / (2 * a);
				double t2 = (-b + discriminant) / (2 * a);

				if (t1 >= 0 && t1 <= 1) {
					t = t1;
					pointFound = true;
				}
				if (t2 >= 0 && t2 <= 1) {
					t = t2;
					pointFound = true;
				}
			}
			d = d.ScalarMultiply(t);
		}*/
	std::cout << this->idxOfLastClosestPoint << " " << robotLocation.toString() << " " << this->smoothedPoints[this->idxOfLastClosestPoint].getPoint().toString() << "\n";
	return  CoordinatePoint((startPointofSegment.getX() + d.getXComponent()),
	                        (startPointofSegment.getY() + d.getYComponent()));
}

std::string Path::toString() {
	std::string output = ("Unsmooth Path: \n");
	for (WayPoint W : this->unsmoothedPoints) {
		output += (W.toString() + "\n");
	}
	output += ("Smooth Path: \n");
	for (WayPoint W : this->smoothedPoints) {
		output += (W.toString() + "\n");
	}
	return output;
}