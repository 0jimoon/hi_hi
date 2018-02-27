
#pragma once
#include "KuKalman.h"
#include "HungarianAlg.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include "../../KUNSProcess/KUNSLocalPathPlnnerPr/KuTrajectory.h"

using namespace cv;
using namespace std;

class KuTrack
{
public:
	vector<Point2d> trace;
	static size_t NextTrackID;
	size_t track_id;
	size_t skipped_frames; 
	Point2f prediction;
	Point2f center;
	KuKalman* KF;
	KuTrack(Point2f p, float dt, float Accel_noise_mag);
	~KuTrack();
};


class KuTracker
{
public:

	float dt; 
	float Accel_noise_mag;
	double dist_thres;	
	int maximum_allowed_skipped_frames;	
	int max_trace_length;

	vector<KuTrack*> kutracks;
	void Update(vector<Point2f>& detections);
	KuTracker(float _dt, float _Accel_noise_mag, double _dist_thres=60, int _maximum_allowed_skipped_frames=10,int _max_trace_length=10);
	~KuTracker(void);
};

