#pragma once
#ifndef KU_TRAJECTORY_H
#define KU_TRAJECTORY_H

#include <cmath>
#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"
#include "../../KUNSProcess/KUNSLocalPathPlnnerPr/Kalman.h"

using namespace cv;
using namespace std;

struct KuTrajectory
{
	
	vector<float> Tra_x;
	vector<float> Tra_y;
	vector<float> Tra_th;
	Point2f LastResult;
	vector<float> Predicted_x;
	vector<float> Predicted_y;
	vector<Point2f> PredictedPt;
	float Avg_x;
	float Avg_y;
	float Avg_dth;
	float Avg_th;
	vector<float> dth;
	vector<float> th;
	vector<float> dx;
	vector<float> dy;
	int ID;
};

#endif /*KU_TRAJECTORY_H*/
