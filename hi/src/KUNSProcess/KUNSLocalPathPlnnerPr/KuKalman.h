#pragma once
#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
using namespace cv;
using namespace std;
// http://www.morethantechnical.com/2011/06/17/simple-kalman-filter-for-tracking-using-opencv-2-2-w-code/
class KuKalman
{

public:
	int ID;
	KalmanFilter* kukalman;
	double deltatime; 	
	Point2f LastResult;
	
public:
	Point2f GetPrediction();
	Point2f Update(Point2f p, bool DataCorrect);

	KuKalman(Point2f p,float dt=0.2,float Accel_noise_mag=0.5);
	KuKalman();
	~KuKalman();
};