/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description :Kanayama 알고리듬을 사용한 motion control을 시뮬레이션으로 수행하는 behavior 클래스
$Created on: 2012. 6. 13.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/


#ifndef KUNS_VIRTUAL_DWA_BEHAVIOR_H
#define KUNS_VIRTUAL_DWA_BEHAVIOR_H

#include <iostream>
#include <conio.h>
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSGUI/KuDrawingInfo.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../Sensor/VirtualSensor/KuVrWheelActuatorInterface.h"
#include "../../KUNSMap/KuMapRepository.h"
#include "../../KUNSProcess/KUNSPathPlannerPr/KuGradientPathPlannerPr.h"
#include "../../KUNSProcess/KUNSPathPlannerPr/KuPathSmoothingPr.h"
#include "../../Sensor/VirtualSensor/KuVrWheelActuatorInterface.h"
#include "../../KUNSProcess/KUNSDWAMotionControlPr/KuDWAPr.h"
#include "../../Sensor/VirtualSensor/KuVrHokuyoUTM30LXInterface.h"
#include "../../KUNSProcess/KUNSLaserMapBuilderPr/KuLaserMapBuilderPr.h"
#include "../../KUNSProcess/KUNSLaserMapBuilderPr/KuMapBuilderParameter.h"
#include "../../KUNSProcess/KUNSKanayamaMotionControlPr/KuKanayamaMotionControlPr.h"
#include "../../KUNSProcess/KUNSObstAvoidancePr/KuObstAvoidancePr.h"
#include "../../KUNSProcess/KUNSBuildingPOMapPr/KuBuildingPOMapPr.h"
#include "../../KUNSProcess/KUNSBuildingPOMapPr/KuBuildingPOMapParameter.h"

using namespace std;
class KuVrDWABh :public KuThread
{
	static const int INFINITY_VALUE = 100000;
	static const int LOCALGOAL_DISTANCE = 5000;

public:
	static const int KUNS = 1;
	static const int ROS = 2;



private:
	KuVrWheelActuatorInterface m_KuVrWheelActuator; 
	KuGradientPathPlannerPr m_KuPathPlanner; //경로계획기 인스턴스
	KuINIReadWriter* m_pINIReaderWriter;
 	KuPathSmoothingPr m_KuPathSmoothing;
	KuDWAPr m_KuDWA;
	//KuDWAPr_ROS m_KuDWA_ROS;
	KuMath m_math;
	KuThread m_KuThread_ROS;
	KuThread m_LocalPathplaningThread;
	KuKanayamaMotionControlPr m_KanayaMC;
	KuBuildingPOMapPr m_PBMPr;


private:
	KuPose m_RobotPos, m_GoalPos, m_TargetPos;	
	int m_nThreadFuncPeriod;
	list<KuPose> m_listPath;
	vector<KuPose> m_vecPath;
	bool m_bThreadFlag; 
	int_1DArray m_nLaserData;
	int_1DArray m_nLaserData_ROS;
	KuMap * m_pMap;
 	int m_nGoalArea;
	int m_nDistToTarget;
	int m_nMode;
	int** m_nMap;
	int m_nMapX;
	int m_nMapY;

private:
	KuUtil m_KuUtil;
	list<KuPose>  m_DetourPathList;
	int m_nLocalMapSPosX, m_nLocalMapSPosY;
	int m_nGlobalMapX, m_nGlobalMapY;
	int m_nLocalMapX, m_nLocalMapY;
	int m_nBuildingMapX, m_nBuildingMapY;
	bool m_bAvoidModeflag;
	int m_nMapSizeX;
	int m_nMapSizeY;
	KuPose m_LocalGoalPos;
	int m_nLocalGoalIndex;
	int m_nLocalGoalWayPointIndex;
	vector<KuPose> m_vecWayPoint;
	vector<KuPose> m_vecLocalPath;
	KuMap* m_pRefMap;
	KuMap *m_pLocalMap;
	KuMap *m_pOriginalMap; 
	KuLaserMapBuilderPr m_LaserMapBuilder;
	KuGradientPathPlannerPr m_KuLocalPathPlanner; //지역 경로 계획기 생성.
	int_1DArray m_nKinectRnageData;
	KuPose m_DelEncoderData;
	int m_nPrePathIndx;
	int m_nSelectedMinIndx;

private:
	int m_nDesiredVel ;


private:
	bool initialize(KuCommandMessage CMessage);
	static void doThread(void* arg);
	//static void doThread_ROS(void* arg);
	KuPose getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist);
	void checkVirtualObstacle(KuMap* pMap);
	void drawNaviData();
	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);

	static void doLocalPathplanningThread(void* arg);
	void initKanayamaProcess();
	bool checkObstacles(KuPose RobotPos,KuPose TargetPos,int_1DArray nLaserData181, double dTargetDist);

private:
	int_1DArray combinateLaserNKinect(int_1DArray nLaserData181, int_1DArray nKinectRnageData);
	bool checkObstacles(KuPose RobotPos,KuPose TargetPos, int_1DArray nLaserData181, int_1DArray nKinectRnageData ,double dTargetDist);
	void initMapbuildingProcess(bool bLocalMapflag);
	bool initPathplanningProcess();
	void calStartPathPoint( );

public:
	bool execute(KuCommandMessage CMessage);
	void terminate();
	void setMode(int nMode);
	bool getBehaviorStates();
public:
	KuVrDWABh();
	~KuVrDWABh();

};

#endif