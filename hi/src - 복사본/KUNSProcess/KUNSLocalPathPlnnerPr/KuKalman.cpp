#pragma once
#include "stdafx.h"
#include "KuKalman.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>


using namespace cv;
using namespace std;

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
KuKalman::KuKalman(Point2f pt,float dt,float Accel_noise_mag)
{
	//time increment (lower values makes target more "massive")
	deltatime = dt; //0.2

	// We don't know acceleration, so, assume it to process noise.
	// But we can guess, the range of acceleration values which can be achieved by tracked object. 
	// Process noise. (standard deviation of acceleration: ??2)
	// shows, woh much target can accelerate.
	//float Accel_noise_mag = 0.5; 

	//4 state variables, 2 measurements
	kukalman = new KalmanFilter( 4, 2, 0 );  
	// Transition matrix
	kukalman->transitionMatrix = (Mat_<float>(4, 4) << 1,0,deltatime,0,   0,1,0,deltatime,  0,0,1,0,  0,0,0,1);

	// init... 
	LastResult = pt; //initial point
	kukalman->statePre.at<float>(0) = pt.x; // x predicted state
	kukalman->statePre.at<float>(1) = pt.y; // y 

	kukalman->statePre.at<float>(2) = 0;
	kukalman->statePre.at<float>(3) = 0;

	kukalman->statePost.at<float>(0)=pt.x; //corrected state
	kukalman->statePost.at<float>(1)=pt.y;

	setIdentity(kukalman->measurementMatrix); // initializes scaled identity matrix //measurement matrix (H)

	kukalman->processNoiseCov=(Mat_<float>(4, 4) << //process noise covariance matrix (Q)
		pow(deltatime,4.0)/4.0	,0						,pow(deltatime,3.0)/2.0		,0,
		0						,pow(deltatime,4.0)/4.0	,0							,pow(deltatime,3.0)/2.0,
		pow(deltatime,3.0)/2.0	,0						,pow(deltatime,2.0)			,0,
		0						,pow(deltatime,3.0)/2.0	,0							,pow(deltatime,2.0));


	kukalman->processNoiseCov*=Accel_noise_mag;

	setIdentity(kukalman->measurementNoiseCov, Scalar::all(0.1)); //measurement noise covariance (R)

	setIdentity(kukalman->errorCovPost, Scalar::all(.1));

}

//---------------------------------------------------------------------------
KuKalman::~KuKalman()
{
	delete kukalman;
}

//---------------------------------------------------------------------------
Point2f KuKalman::GetPrediction()
{
	Mat prediction = kukalman->predict();
	LastResult=Point2f(prediction.at<float>(0),prediction.at<float>(1)); 
	return LastResult;
}
//---------------------------------------------------------------------------
Point2f KuKalman::Update(Point2f p, bool DataCorrect)
{
	Mat measurement(2,1,CV_32FC1);
	if(!DataCorrect)
	{
		measurement.at<float>(0) = LastResult.x;  //update using prediction
		measurement.at<float>(1) = LastResult.y;
	}
	else
	{
		measurement.at<float>(0) = p.x;  //update using measurements
		measurement.at<float>(1) = p.y;
	}
	// Correction
	Mat estimated = kukalman->correct(measurement);
	LastResult.x=estimated.at<float>(0);   //update using measurements
	LastResult.y=estimated.at<float>(1);
	return LastResult;
}
//---
