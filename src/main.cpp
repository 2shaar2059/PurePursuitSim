/*Variables to tweak:
make sure the path is pretty smooth
	max velocity paramter in path
	how many points make up the path
	max centripetal acceleration paramter in path
	look ahead distance as a funciton of the robots current velocity
	PID constants for acheiving target velocities from voltage inputs
*/

#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include <unistd.h>
#include <eigen3/Eigen/Dense>
#include "matplotlibcpp.h"

#include "CoordinatePoint.h"
#include "Path.h"
#include "Vector.h"
#include "WayPoint.h"
#include "mousePointer.h"

namespace plt = matplotlibcpp;
using namespace Eigen;

#define PI 3.1415926535
#define print(x) std::cout<<(#x)<<":\n"<<(x)<<"\n\n";

#define NUM_STATES 5
#define NUM_CONTROLS 2


double robotTrackWidth = 1; //robot track width [m]
double Izz; //robot MOI about an axis perpendicular to the ground and goign through the robot's geometric center [kg*m^2]

double kA_L = 1; //left drive train acceleration constant
double kA_R = 1; //right drive train acceleration constant

double kV_L = .4; //left drive train voltage constant
double kV_R = .4; //right drive train voltage constant

double max_u = 12.0; // [volts]
double kdt = 0.02;

double max_vel = 5; //maximum robot velocity [m/s]
double max_centrip_accel = 1; //maximum robot centripetal acceleration [m/s^2]
double kp_L = 100;
double kp_R = 100;

bool stopRobot = false;


VectorXd f(VectorXd x, VectorXd u) {
	/*x = [x, y, theta, vel_L, vel_R].
	Returns x' = f(x, u)*/

	double x_dot = 0.5 * cos(x(2)) * (x(3) + x(4));
	double y_dot = 0.5 * sin(x(2)) * (x(3) + x(4));
	double theta_dot = (x(4) - x(3)) / robotTrackWidth;
	double vel_L_dot = (kV_L * u(0) - x(3)) / kA_L;
	double vel_R_dot = (kV_R * u(1) - x(4)) / kA_R;


	VectorXd xdot(NUM_STATES); xdot <<	x_dot,
	          y_dot,
	          theta_dot,
	          vel_L_dot,
	          vel_R_dot;
	return xdot;
}

VectorXd rk4(VectorXd x, VectorXd u, double dt) {
	/*Fourth order Runge-Kutta integration.
	Keyword arguments:
	x -- vector of states
	u -- vector of inputs (constant for dt)
	dt -- time for which to integrate
	*/
	double half_dt = dt * 0.5;
	VectorXd k1 = f(x, u);
	VectorXd k2 = f(x + half_dt * k1, u);
	VectorXd k3 = f(x + half_dt * k2, u);
	VectorXd k4 = f(x + dt * k3, u);

	return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

double getLookAheadDistance(double currentRobotVelocity) {
	return 0.5 + currentRobotVelocity / 3.5;
}

VectorXd getTargetWheelVelocities(VectorXd x, Path& path) {
	double distance = 0, previousDistance = 0, oldLeftTicks = 0, oldRightTicks = 0;

	CoordinatePoint robotLocation = CoordinatePoint(x(0), x(1));

	VectorXd TargetWheelVelocities(2);

	if (!stopRobot && robotLocation.distanceTo(path.getSmoothedPoints().back().getPoint()) > 0.1) {

		double currentRobotVel = 0.5 * (x(3) + x(4));

		CoordinatePoint lookAheadPoint = path.findLookaheadPoint(robotLocation, getLookAheadDistance(currentRobotVel));

		Vector l =  Vector(robotLocation, lookAheadPoint);

		double angle = x(2);
		Vector unitRobotY =  Vector(cos(angle), sin(angle));
		Vector unitRobotX =  Vector(sin(angle), -cos(angle));
		double x = l.proj(unitRobotX);
		double y = l.proj(unitRobotY);

		double signedCurvature = (2 * x) / (l.getmagnitude() * l.getmagnitude());
		double curvature = abs(signedCurvature);

		//double targetRobotVelocity = path.closestPointTo(robotLocation).getTargetVelocityAtPoint();
		double targetRobotVelocity = sqrt(max_centrip_accel / curvature);
		double target_vel_L = targetRobotVelocity * (2 + signedCurvature * robotTrackWidth) / 2.0;
		double target_vel_R = targetRobotVelocity * (2 - signedCurvature * robotTrackWidth) / 2.0;

		TargetWheelVelocities << target_vel_L, target_vel_R;
	}
	else {
		stopRobot = true;
		TargetWheelVelocities << 0.0, 0.0;
	}
	return TargetWheelVelocities;
}



void SimulateTime(double maxtime, VectorXd x_0, VectorXd u_0, Path& path) {
	double currentTime = 0.0;

	VectorXd x = x_0;
	VectorXd u = u_0;


	std::vector <double> x_pos_arr;
	std::vector <double> y_pos_arr;
	std::vector <double> theta_arr;
	std::vector <double> vel_L_arr;
	std::vector <double> vel_R_arr;
	std::vector <double> left_voltage_arr;
	std::vector <double> right_voltage_arr;

	std::vector <double> target_vel_L_arr;
	std::vector <double> target_vel_R_arr;

	std::vector <double> t;

	auto start = std::chrono::steady_clock::now();
	while (currentTime < maxtime) {
		VectorXd target_vel = getTargetWheelVelocities(x, path);

		u(0) = kp_L * (target_vel(0) - x(3));
		u(1) = kp_R * (target_vel(1) - x(4));

		if (u(0) < -max_u) u(0) = -max_u;
		else if (u(0) > max_u) u(0) = max_u;

		if (u(1) < -max_u) u(1) = -max_u;
		else if (u(1) > max_u) u(1) = max_u;

		x = rk4(x, u, kdt);

		x_pos_arr.push_back(x(0));
		y_pos_arr.push_back(x(1));
		theta_arr.push_back(x(2));
		vel_L_arr.push_back(x(3));
		vel_R_arr.push_back(x(4));
		left_voltage_arr.push_back(u(0));
		right_voltage_arr.push_back(u(1));

		t.push_back(currentTime);

		target_vel_L_arr.push_back(target_vel(0));
		target_vel_R_arr.push_back(target_vel(1));


		currentTime += kdt;
	}
	auto end = std::chrono::steady_clock::now();

	std::cout << "C++ Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";

	plt::subplot(4, 2, 1);
	std::vector <double> X_coords;
	std::vector <double> Y_coords;
	for (WayPoint wp : path.getunsmoothedPoints()) {
		CoordinatePoint cp = wp.getPoint();
		X_coords.push_back(cp.getX());
		Y_coords.push_back(cp.getY());
	}
	plt::plot(X_coords, Y_coords, "unsmooth target path");

	X_coords.clear();
	Y_coords.clear();

	for (WayPoint wp : path.getSmoothedPoints()) {
		CoordinatePoint cp = wp.getPoint();
		X_coords.push_back(cp.getX());
		Y_coords.push_back(cp.getY());
	}
	plt::plot(X_coords, Y_coords, "smooth target path");
	plt::grid(true);
	plt::legend();

	plt::subplot(4, 2, 2);
	plt::plot(x_pos_arr, y_pos_arr, "robot position");

	drawPathOnField(x_pos_arr, y_pos_arr);
	drawPathOnField(path.getunsmoothedPoints());
	showField();

	plt::plot(X_coords, Y_coords, "smooth target path");
	plt::grid(true);
	plt::legend();


	plt::subplot(4, 2, 3);
	plt::plot(t, target_vel_L_arr, "target_left_wheel_vel");
	plt::plot(t, vel_L_arr, "actual_left_wheel_vel");
	plt::grid(true);
	plt::legend();

	plt::subplot(4, 2, 4);
	plt::plot(t, target_vel_R_arr, "target_right_wheel_vel");
	plt::plot(t, vel_R_arr, "actual_right_wheel_vel");
	plt::grid(true);
	plt::legend();

	plt::subplot(4, 2, 5);
	plt::plot(t, left_voltage_arr, "left_voltage");
	plt::grid(true);
	plt::legend();

	plt::subplot(4, 2, 6);
	plt::plot(t, right_voltage_arr, "right_voltage");
	plt::grid(true);
	plt::legend();

	plt::subplot(4, 2, 7);
	plt::plot(t, x_pos_arr, "x_pos");
	plt::grid(true);
	plt::legend();

	plt::subplot(4, 2, 8);
	plt::plot(t, y_pos_arr, "y_pos");
	plt::grid(true);
	plt::legend();

	plt::show();
}

double degToRad(double deg) {
	return deg * PI / 180.0;
}


std::vector <WayPoint> rightHabRightRocket = {
	WayPoint(0, 0),
	WayPoint(0, 10),
	WayPoint(10, 20),
	WayPoint(20, 30),
	WayPoint(30, 35),
	WayPoint(40, 40),
	WayPoint(55, 45),
	WayPoint(65, 50),
	WayPoint(75, 55),
	WayPoint(85, 60),
	WayPoint(90, 68)
};

std::vector <WayPoint> straight = {
	WayPoint(0, 5),
	WayPoint(0, 10),
	WayPoint(0, 60)
};

std::vector <WayPoint> sinePath = {
	WayPoint(0, 0),
	WayPoint(90, 60),
	WayPoint(180, 0),
	WayPoint(270, -60),
	WayPoint(360, 0)
};

std::vector <WayPoint> spiralPath = {
	WayPoint(0, 0),
	WayPoint(320, 0),
	WayPoint(320, 200),
	WayPoint(0, 200),
	WayPoint(0, 90),
	WayPoint(269, 100)//this is problematic
};

std::vector <WayPoint> spiralPathOffset = {
	WayPoint(0, 0),
	WayPoint(240, 0),
	WayPoint(320, 0),
	WayPoint(320, 200),
	WayPoint(0, 200),
	WayPoint(0, 100),
	WayPoint(300, 100)//this is problematic
};

std::vector <WayPoint> sharpTurnGood = {
	WayPoint(0, 0),
	WayPoint(320, 0),
	WayPoint(320, 200)
};

std::vector <WayPoint> smoothTurn = {
	WayPoint(0, 0),
	WayPoint(270, 0),
	WayPoint(310, 10),
	WayPoint(320, 50),
	WayPoint(320, 200)
};

std::vector <WayPoint> sharpTurnBad = {
	WayPoint(0, 0),
	WayPoint(240, 0), //this is problematic
	WayPoint(320, 0),
	WayPoint(320, 200)
};


std::vector <WayPoint> smoothtest = {
	WayPoint(0, 0),
	WayPoint(50, -10),
	WayPoint(100, 0),
	WayPoint(150, 10),
	WayPoint(200, 0),
	WayPoint(250, -10),
	WayPoint(300, 0),
	WayPoint(350, 10)
};


std::vector <WayPoint> slalomPath = {
	WayPoint(60, 29),
	WayPoint(68, 31),
	WayPoint(80, 37),
	WayPoint(87, 44),
	WayPoint(107, 71),
	WayPoint(121, 81),
	WayPoint(134, 88),
	WayPoint(146, 92),
	WayPoint(162, 95),
	WayPoint(176, 97),
	WayPoint(193, 97),
	WayPoint(207, 96),
	WayPoint(221, 92),
	WayPoint(236, 84),
	WayPoint(246, 75),
	WayPoint(253, 68),
	WayPoint(264, 54),
	WayPoint(277, 43),
	WayPoint(291, 36),
	WayPoint(304, 35),
	WayPoint(318, 43),
	WayPoint(320, 78),
	WayPoint(304, 84),
	WayPoint(290, 78),
	WayPoint(281, 71),
	WayPoint(273, 62),
	WayPoint(257, 46),
	WayPoint(241, 38),
	WayPoint(226, 33),
	WayPoint(212, 31),
	WayPoint(198, 30),
	WayPoint(182, 30),
	WayPoint(151, 33),
	WayPoint(122, 40),
	WayPoint(103, 48),
	WayPoint(87, 60),
	WayPoint(73, 75)
};


int main() {

	//std::vector <WayPoint> userDrawn = drawAndGetPath();

	Path path = Path(slalomPath, .5, .5 , 0.001, max_vel, max_centrip_accel, true);
	std::cout << path.toString();

	VectorXd x_0(NUM_STATES);
	x_0 << 	path.getSmoothedPoints()[0].getPoint().getX(),//x
	    path.getSmoothedPoints()[0].getPoint().getY(),//y
	    degToRad(0),//theta
	    0.0,//velL
	    0.0;//velR

	VectorXd u_0(NUM_CONTROLS);
	u_0 << 	0.0,//left_voltage
	    0.0;//right_voltage


	SimulateTime(25, x_0, u_0, path);
	return 0;
}


