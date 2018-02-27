#ifndef KUNS_MAP_BUILDING_BEHAVIOR_H
#define KUNS_MAP_BUILDING_BEHAVIOR_H

#include <iostream>
#include <conio.h>
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSGUI/KuDrawingInfo.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../KUNSMap/KuMapRepository.h"
#include "../../MobileSupervisor/KuCommandMessage.h"
#include "../../KUNSProcess/KUNSLaserMapBuilderPr/KuLaserMapBuilderPr.h"
#include "../../KUNSProcess/KuICPLocalizerPr/KuICPLocalizerPr.h"
#include "../../MobileSupervisor/KuRobotParameter.h"
#include "../../Sensor/SensorSupervisor.h"
#include "../../KUNSMap/KuMapRepository.h"


using namespace std;
class MapBuildingBehavior :public KuThread
{
private:
	KuUtil m_KuUtil;
	int m_nThreadFuncPeriod;
	KuPose m_RobotPos, m_GoalPos;		
	KuPose m_EncoderDelPos;
	int_1DArray m_nLaserData; 

	bool m_bThreadFlag; 
	bool m_bIsThreadFuncGenerated; 
	KuLaserMapBuilderPr m_LaserMapBuilder;
	KuMap* m_pMap;
	KuICPLocalizerPr m_ICPLocalizer;
	IplImage* m_IplCeilingCamera;
	IplImage* m_IplKinectCamera;
	float* m_fKinectDistanceImage;
	int m_nMapSizeXm , m_nMapSizeYm ;
	KuMath m_Math;

private:
	void initialize();
	static void doThread(void* arg);
	void initialize(KuCommandMessage CMessage);
	void initICPProcess();
	void generateMapData2BMPFile(); //격자 지도를 BMP형태로 생성하는 함수
	void drawNaviData(); //주행 관련 정보를 그려주는 함수.
	



public:
	bool getBehaviorStates();
	bool execute(KuCommandMessage CMessage);
	void terminate();
	Localizer* getLocalizer();
	

	MapBuildingBehavior();
	~MapBuildingBehavior();

};

#endif 