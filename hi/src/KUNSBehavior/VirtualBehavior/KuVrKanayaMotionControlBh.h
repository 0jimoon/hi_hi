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


#ifndef KUNS_VIRTUAL_KANAYAMA_MOTION_CONTROL_BEHAVIOR_H
#define KUNS_VIRTUAL_KANAYAMA_MOTION_CONTROL_BEHAVIOR_H

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
#include "../../KUNSProcess/KUNSKanayamaMotionControlPr/KuKanayamaMotionControlPr.h"
#include "../../KUNSProcess/KUNSLocalPathPlnnerPr/KuLocalPathPlannerPr.h"
#include "../../KUNSProcess/KUNSWanderingObstaclePr/KuWanderingObstaclePr.h"
#include "../../Sensor/VirtualSensor/KuVrHokuyoUTM30LXInterface.h"


using namespace std;
class KuVrKanayaMotionControlBh :public KuThread
{
private:
	static const int INFINITY_VALUE = 100000;
	static const int GOAL_BOUNDARY = 1000;

private:
	KuThread m_KuThreadForKeyEvt;
	KuKanayamaMotionControlPr m_KanayaMC;
	KuMath m_math;
	KuGradientPathPlannerPr m_KuPathPlanner; //경로계획기 인스턴스
	KuPathSmoothingPr m_KuPathSmoothing;
	KuVrWheelActuatorInterface m_KuVrWheelActuator; 

	KuUtil m_KuUtil;
	KuINIReadWriter* m_pINIReaderWriter;
	int m_nThreadFuncPeriod;
	KuPose m_RobotPos, m_GoalPos;		 
	double m_dpreVel;
	double m_dpreRVel;

	list<KuPose> m_listPath;
	vector<KuPose> m_vecPath;
	bool m_bThreadFlag; 
	KuPose m_TargetPos;
	int m_nMapX;
	int m_nMapY;

private:
	int_1DArray m_nLaserData181;

private:
	int m_nDistToTarget;
	int m_nDesiredVel ;
	int m_nGoalArea ;

private:
	void initialize(KuCommandMessage CMessage);
	static void doThread(void* arg);
	void initKanayamaProcess();
	KuPose getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist);
	void drawNaviData();
	KuPose getStaticTargetPos(KuPose RobotPos,int nDistToTarget);

public:
	
	bool execute(KuCommandMessage CMessage);
	void terminate();
	bool getBehaviorStates();

	KuVrKanayaMotionControlBh();
	~KuVrKanayaMotionControlBh();

};

#endif /* KUNS_VIRTUAL_KANAYAMA_MOTION_CONTROL_BEHAVIOR_H */