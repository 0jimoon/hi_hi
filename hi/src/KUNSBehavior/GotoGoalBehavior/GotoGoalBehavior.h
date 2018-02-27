#ifndef KUNS_GO_TO_GOALL_BEHAVIOR_H
#define KUNS_GO_TO_GOALL_BEHAVIOR_H

#include <iostream>
#include <conio.h>
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSGUI/KuDrawingInfo.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../KUNSMap/KuMapRepository.h"
#include "../../KUNSProcess/KUNSPathPlannerPr/KuGradientPathPlannerPr.h"
#include "../../KUNSProcess/KUNSPathPlannerPr/KuPathSmoothingPr.h"
#include "../../Sensor/WheelActuatorInterface/PioneerWheelActuatorInterface.h"
#include "../../KUNSProcess/KUNSKanayamaMotionControlPr/KuKanayamaMotionControlPr.h"
#include "../../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../../KUNSProcess/KuLaserBasedParticleFilterLocalizerPr/KuLBPFLocalizerPr.h"
#include "../../Sensor/SensorSupervisor.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSProcess/KUNSObstAvoidancePr/KuObstAvoidancePr.h"
#include "../../KUNSProcess/KUNSBuildingPOMapPr/KuBuildingPOMapPr.h"
#include "../../KUNSProcess/KUNSBuildingPOMapPr/KuBuildingPOMapParameter.h"
#include "../../KUNSProcess/KUNSCombiningPOMapPr/KuCombiningPOMapPr.h"
#include "../../KUNSProcess/KUNSPathPlannerPr/KuGradientPathPlannerbasedPOMapPr.h"
#include "../../KUNSProcess/KUNSPathPlannerPr/KuGlobalParhPlannerPr.h"
#include "../../MobileSupervisor/KuPOIMapParameter.h"
#include "../../KUNSProcess/KuElevatorPr/KuElevatorPr.h"

using namespace std;

class GotoGoalBehavior 
{
	static const int INFINITY_VALUE = 100000;
	static const int GOAL_BOUNDARY = 2500;

private:
	CCriticalSection m_CriticalSection;
	KuKanayamaMotionControlPr m_KanayaMC;
	KuMath m_math;
	KuThread m_Thread;
	KuGradientPathPlannerPr m_KuPathPlanner; //경로계획기 인스턴스
	KuPathSmoothingPr m_KuPathSmoothing;
	KuUtil m_KuUtil;
	KuGlobalParhPlannerPr m_KuGPathPlannerPr;
	KuElevatorPr m_Elvator;

private:	
	KuPose m_RobotPos, m_GoalPos, m_WayPointPos;		 
	bool m_bThreadFlag; 
	list<KuPose> m_listPath;
	list<KuPose> m_listWayPoint;
	vector<KuPose> m_vecPath;
	vector<KuPose> m_vecWayPoint;
	vector<KuPose> m_vecImagePath;
	KuSmartPointer<KuMap> m_smtpMap; //지도정보를 저장할 공간.
	KuINIReadWriter* m_pINIReaderWriter;
	int m_nSetGoalPosID,m_nSetRobotPosID;
	int m_nBhID;
	KuPose m_TargetPos;
	KuPose m_CurWayPoint;
	bool m_bWaitflag;
	int m_nselectIdx;
	int m_nSelectTargetIdx;
	int m_nSelectedMinIndx;
	ofstream m_DataLog;

	vector<KuPose> m_vecElveWayPointPos;
	int m_nCurFloor;
	int m_nMapSizeXm;
	int m_nMapSizeYm;
	double m_dCellSize;
private:
	KuPose m_DelEncoderData;
	int_1DArray m_nLaserData181;
	int_1DArray m_nRplidarData181;
	int_1DArray m_nKinectRangeData;
	IplImage* m_IplKinectCamera;
	KuPose m_KinectDataPos[ Sensor::IMAGE_WIDTH* Sensor::IMAGE_HEIGHT]; 

private:
	int m_nDistToTarget;
	int m_nDesiredVel ;
	int m_nGoalArea ;
	int m_nMaxTVel;
	int m_nMinTVel;
	int m_nPrePathIndx;
private:
	bool initialize();
	static void doThread(void* arg);
	void startLocalizer(KuPose RobotPos);
	bool initialize(KuCommandMessage CMessage);
	KuPose getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist);
	KuPose getStaticTargetPos(KuPose RobotPos,int nDistToTarget);
	bool checkObstacles(KuPose RobotPos,KuPose TargetPos, int_1DArray nLaserData181, int_1DArray nKinectRnageData ,double dTargetDist);
	void initKanayamaProcess();
	bool initPathplanningProcess();
	void drawNaviData();
	KuPose checkWayPoint(KuPose TargetPos,KuPose RobotPos, int *nselectIdx ,bool *bWaitflag);
	bool checkObstacles(KuPose RobotPos,KuPose TargetPos,int_1DArray nLaserData181, double dTargetDist);
	void calStartPathPoint( );
	int** loadMap(string strMapFilePath);
	void initMapbuildingProcess( );
	KuVelocity getElevState(KuPose RobotPos,vector<KuPose> vecWayPointPos,bool *bterminate );
	//bool getElevState(KuPose RobotPos,vector<KuPose> vecWayPointPos,int_1DArray nLaserData, int_1DArray nKinectRangeData );
	//KuVelocity getElevState(KuPose RobotPos,vector<KuPose> vecWayPointPos,KuPose DelEncoderData,int_1DArray nLaserData, int_1DArray nKinectRangeData,bool *bterminate  );
	KuVelocity getElevState(KuPose RobotPos,vector<KuPose> *vecWayPointPos,KuPose DelEncoderData, int_1DArray nLaserData, int_1DArray nKinectRangeData,bool *bterminate  );	
	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);

public:	
	bool execute(KuCommandMessage CMessage);
	void terminate();
	bool getBehaviorStates();
	Localizer* getLocalizer();
	KuPose getCurWayPoint( );

public:
	GotoGoalBehavior();
	~GotoGoalBehavior();

};

#endif 