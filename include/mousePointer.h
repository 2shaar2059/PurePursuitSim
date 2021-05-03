#include "WayPoint.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>

//PATH_NODES = //after path is drawn, how many nodes should it be reduced to befroe it is handed to the solver for optimization
std::vector <WayPoint>  drawAndGetPath();

void drawPathOnField(std::vector <double> X_coords, std::vector <double>  Y_coords);

void drawPathOnField(std::vector <WayPoint> WayPoints);

void showField();
