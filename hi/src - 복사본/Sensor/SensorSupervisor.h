#ifndef C_SENSOR_SUPERVISOR_H
#define C_SENSOR_SUPERVISOR_H


#include <cv.h>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/opencv.hpp"
#include<iostream>
#include<fstream>
#include "../KUNSPose/KuPose.h"
#include "Sensor.h"
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "./HokuyoURG04LXInterface/HokuyoURG04LXInterface.h" //��ƿƮ ������
#include "./KinectSensorInterface/KinectSensorInterface.h"
#include "./WheelActuatorInterface/PioneerWheelActuatorInterface.h"
#include "../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../MobileSupervisor/KuRobotParameter.h"
#include "./RplidarInterface/RplidarInterface.h"

using namespace std;
using namespace cv;

class SensorSupervisor: public KuSingletone <SensorSupervisor>
{
private:
	KuUtil m_KuUtil;
private:
	int m_KinectLogIdx;

	//sensor data
	KuPose m_EncoderDelPos;
	KuPose m_tempEncoderDelPos;
	int_1DArray m_nURGLaserData; //laser data;
	int_1DArray m_nKinectRangeData;	
	int_1DArray m_nHightDataOfRangeData;
	int_1DArray m_nRplidarData; //laser data;

	IplImage* m_IplKinectImage;
	IplImage* m_IplKinectDepthImg;
	KuPose* m_pKinectDataPos; 

	bool m_bDataRecoding;
	bool m_bDataPlay;
	ofstream m_DataLog;
	ifstream m_PlayLog;

	double 	*m_d3DKinectX;
	double 	*m_d3DKinectY;
	double  *m_d3DKinectZ;
	int* m_n3DKinectPixX;
	int* m_n3DKinectPixY;
	int* m_n3DKinectDataValidation;
	float	*m_f3DX;
	float	*m_f3DY;
	float	*m_f3DZ;
	bool	*m_bFlag3D;
	float* m_fKinectDistanceImage;
private:
	void init();

	void dataParsing();

	void recordEncoderDelPosData(KuPose EncoderDelPos);

	KuPose playEncoderDelPosData();
	void recordLaserData(int_1DArray  nLaserData);
	void playLaserData(int_1DArray  nData);
	void recordKinectRangeData(int_1DArray nKinectRangeData);
	void recordHightDataOfKinectRangeData(int_1DArray nHightDataOfKinectRangeData);
	void recordKinectData(IplImage* IplKinectImage, IplImage* IplKinectDepthImg, KuPose* pKinectDataPos);

	void playKinectData();
	void PlayHIghtDataOfKinectRangeData(int_1DArray nHightDataOfKinectRangeData);
	void PlayKinectRangeData(int_1DArray nKinectRangeData);

public:
	//������ ������ ��� �����ϴ� �κ�
	void stopAllSensor();

	//���� �� ������ �����ϴ� �κ�
	void completeDataRecording();

	//������ �����͸� �д� �κ�
	bool readSensorData();

	//���ڴ��� �޾ƿ�
	KuPose getEncoderDelPos();
	//�������� ������ �޾ƿ�
	int_1DArray getURG04LXLaserData();
	//Ű��Ʈ�� 2D �Ÿ� ���� �޾ƿ�
	int_1DArray getKinectRangeData();
	//Ű��Ʈ�� 2D �Ÿ����� ���� ���� ���� �޾ƿ�
	int_1DArray getHeightDataOfKinectRangeData();
	//Ű��Ʈ�� �̹��� ���� �޾ƿ�
	IplImage* getKinectImageData();
	//Ű��Ʈ��  ���� �̹��� ���� �޾ƿ�
	IplImage* getKinectDepthImageData();
	//Ű��Ʈ�� 3D  ���� �Ÿ� ���� �޾ƿ�
	KuPose* getKinectDataPos();

	float* getKinectDistanceImage();

	bool readOnlySensorData();
	void DataRecodingNPlay();
	bool connectionHokuyoutm();
	bool connectionKinect();
	bool connectionWheelactuator();

	bool connectionRplidar();

	int_1DArray getRplidarData();


	SensorSupervisor();
	~SensorSupervisor();

};

#endif