#ifndef SENSOR_H
#define SENSOR_H

#include <iostream>
using namespace std;

class Sensor
{
public:
	static const int CELLSIZE = 100; //mm
	static const int ROBOT_RADIUS = 500;//mm
	static const int URG04LX_MAX_DISTANCE = 30000; //30m
	static const int URG04LX_MIN_DISTANCE = 50; //30m	
	static const int URG04LX_XOFFSET = 230; 
	static const int URG04LX_HEIGHT =300  ;
	static const int URG04LX_DATA_NUM181 = 181;

	static const int KINECT_SENSOR_FOV = 56;
	static const int KINECT_MAX_DISTANCE = 5000; //30m
	static const int KINECT_MIN_DISTANCE = 700; //30m	
	static const int KINECT_XOFFSET = 260; 
	static const int KINECT_HEIGHT =800;
	static const int KINECT_MAX_HEIGHT_DIST = 2000; //최대 높이 측정거리
	static const int KINECT_MIN_HEIGHT_DIST = 500; //최대 높이 측정거리
	static const int  KINECT_IMAGE_WIDTH= 640;
	static const int  KINECT_IMAGE_HEIGHT= 480;

	static const int  IMAGE_WIDTH =320;
	static const int  IMAGE_HEIGHT= 240;

	static const int  CEILING_IMAGE_WIDTH= 320;
	static const int  CEILING_IMAGE_HEIGHT= 240;

	static const int RANGEINDEX= 361;	

	static const int RPLIDAR_DATA_NUM181= 181;
	static const int RPLIDAR_MAX_DISTANCE = 5000; //30m
	static const int RPLIDAR_MIN_DISTANCE = 700; //30m
	static const int RPLIDAR_XOFFSET = 260;
	static const int RPLIDAR_HEIGHT =800;

	Sensor();
	~Sensor();
};

#endif /*SENSOR_H*/

