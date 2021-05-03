#include "mousePointer.h"
#include "WayPoint.h"

#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <vector>

using namespace std;
using namespace cv;

double xPixelsToInches(double xPixels) {return xPixels * 3.0 / 14.0;}
double yPixelsToInches(double yPixels) {return 180.0 - yPixels * 3.0 / 14.0;}
double inchesToMeteres(double in) {return in / (100.0 / 2.54);}

double inchesToxPixels(double in) {return in * 14.0 / 3.0;}
double inchesToyPixels(double in) {return (180.0 - in) * 14.0 / 3.0;}
double metersToInches(double m) {return m * (100.0 / 2.54);}


Mat field_orig = imread("field.jpg", IMREAD_COLOR);
Mat field = field_orig.clone();

bool draw = false;

vector<WayPoint> unoptimized_path;

void mouse_callback(int  event, int  x, int  y, int  flag, void *param) {
	if (event == EVENT_LBUTTONDOWN) {
		draw = true;
	}
	else if (event == EVENT_LBUTTONUP) {
		draw = false;
	}
	if (draw && event == EVENT_MOUSEMOVE) {
		unoptimized_path.push_back(WayPoint(xPixelsToInches(x), yPixelsToInches(y)));

		circle(field_orig, Point(x, y), 3, Scalar(0, 0, 0), 3);
	}
}

std::vector <WayPoint> drawAndGetPath() {
	namedWindow("field_orig");
	setMouseCallback("field_orig", mouse_callback);

	while (1) {
		int key = waitKey(10);
		if (key == (int)'c') { //press "c" key to clear path and redraw
			field_orig.copyTo(field);
			unoptimized_path.clear();
		}
		else if (key == (int)'s') { //press "s" key to save path to continue
			break;
		}
		else {
			imshow("field_orig", field_orig);
		}
	}
	destroyAllWindows();

	return unoptimized_path;
}

void drawPathOnField(std::vector <double> X_coords, std::vector <double> Y_coords) {
	for (int i = 0; i < X_coords.size(); i++) {
		int xPixel = inchesToxPixels(metersToInches(X_coords[i]));
		int yPixel = inchesToyPixels(metersToInches(Y_coords[i]));
		circle(field, Point(xPixel, yPixel), 1, Scalar(0, 128, 255), 3);	
	}
}

void drawPathOnField(std::vector <WayPoint> WayPoints) {
	for (int i = 0; i < WayPoints.size(); i++) {
		int xPixel = inchesToxPixels(metersToInches(WayPoints[i].getPoint().getX()));
		int yPixel = inchesToyPixels(metersToInches(WayPoints[i].getPoint().getY()));
		circle(field, Point(xPixel, yPixel), 1, Scalar(255, 128, 0), 3);
	}
}

void showField() {
	imshow("field", field);
}