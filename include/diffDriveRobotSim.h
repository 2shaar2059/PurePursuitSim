#include <eigen3/Eigen/Dense>

#include "Path.h"

using namespace Eigen;

VectorXd f(VectorXd x, VectorXd u);

VectorXd rk4(VectorXd x, VectorXd u, double dt);

VectorXd getTargetVelocities(VectorXd x, Path path);

void SimulateTime(double maxtime, VectorXd x_0, VectorXd u_0, Path path);