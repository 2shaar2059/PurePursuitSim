#pragma once
#include "WayPoint.h"
#include "Vector.h"
#include "math.h"
#include "vector"

class Path {
private:
	std::vector <WayPoint> unsmoothedPoints;
	std::vector <WayPoint> smoothedPoints;
	int idxOfLastClosestPoint = 0; //TODO FIX
public:
	Path(std::vector <WayPoint> unsmoothedPoints, double a, double b, double tolerance, double max_vel, double max_centrip_accel, bool inToM);
	
	std::vector <WayPoint> getSmoothedPoints();

	void setSmoothedPoints(std::vector <WayPoint> smoothedPoints);

	std::vector <WayPoint> getunsmoothedPoints();

	void setunsmoothedPoints(std::vector <WayPoint> unsmoothedPoints);

	int getidxOfLastClosestPoint();

	void setidxOfLastClosestPoint(int idx);

	int idxOfClosestPointTo(CoordinatePoint robotLocation);

	/**
	 *
	 * @param path   array of coordinate points
	 * @param robotX current robot X position
	 * @param robotY current robot Y position
	 * @return a point on path that is closest to the robot
	 */
	WayPoint closestPointTo(CoordinatePoint robotLocation);

	WayPoint closestUnvisitedPointTo(CoordinatePoint robotLocation);

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
	CoordinatePoint findLookaheadPoint(CoordinatePoint robotLocation, double lookaheadDistance);

	std::string toString();
};