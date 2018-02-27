#pragma once
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSMap/KuMapRepository.h"
#include"../../KUNSProcess/KUNSWanderingObstaclePr/KuSimulMotionControllerPr.h"
#include "../../KUNSProcess/KUNSPathPlannerPr/KuGradientPathPlannerPr.h"
#include "../../KUNSGUI/KuDrawingInfo.h"
#include "../../KUNSProcess/KUNSVFHPlusPr/KuVFHPlusPr.h"
#include "KuVrRangeInterface.h"


struct WanderingObs{
	double xm;//위치 x [m]
	double ym;//위치 y [m]
	double tr; // 방향 [rad]

	double cri_xm;//위치 x [m]
	double cri_ym;//위치 y [m]
	double cri_tr; // 방향 [rad]

	double vel;// 속도 [m/s]
	double dist;//범위 x [m]
	double r_vel;//범위 x [m]

	int ID;
	int groupID;
	int groupNum;
	int patternID;
	int movingID;
	vector<KuPose> pathlist;
	double Gxm;//목적지 위치 x [m]
	double Gym;//목적지 위치 y [m]
};


class KuWanderingObstaclePr  :public KuSingletone <KuWanderingObstaclePr>, KuThread
{
private:
	CCriticalSection m_CriticalSection;

private:

	static const int RADIUS = 75;//mm //바퀴의 반경. 
	static const int ENCODER_RESOLUTION = 2048; 
	static const int GEAR_RATIO = 150;
	static const int BETWEEN_WHEEL = 500;
	static const int SAMPLENUM = 43; // = STOPNUM + MOVINGNUM + GROUPNUM

	static const int STOPID = 1;
	static const int MOVINGID = 2;
	static const int GROUPID2= 3;
	static const int GROUPID3= 4;
	static const int GROUPID4= 5;


	static const int STOPNUM = 5;
	static const int MOVINGNUM = 10;
	static const int GROUPNUM = 28;// = GROUP2NUM + GROUP3NUM + GROUP4NUM + GROUP5NUM

	static const int GROUP2NUM = 5;//10
	static const int GROUP3NUM = 3;//9
	static const int GROUP4NUM = 1;//4
	static const int GROUP5NUM = 1;//5

	static const int GROUPDIST = 2;
	static const int GROUPDEGREE = 5;


	//스레드 관련 변수----------------------
	bool m_bIsThreadFuncGenerated;
	bool m_doThreadFunc;
	int m_nThreadFuncPeriod;
	bool m_bSuspendFlag;
	//--------------------------------------
	KuGradientPathPlannerPr m_KuPathPlanner; //경로계획기 인스턴스

	KuMap* m_pMap;
	KuMap* m_pRefMap;
	KuPose m_RobotPos;
	KuMath m_math;
	int m_nMovingId;

	int m_nMapSizeX;
	int m_nMapSizeY;

	double m_dRightWheelVel;
	double m_dLeftWheelVel;
	double m_dReferenceT;
	double  m_dReferenceX;
	double m_dReferenceY;
	KuSimulMotionControllerPr m_SimulationMotionController;
	vector<KuPose> m_vecMultiRobot;
	vector<WanderingObs> m_vecMultiObs;
	vector<KuPose> m_vecObstacleAreas;
	int m_ngroupID;
	double* m_psensor_value;
	//	vector<tangent_bugAlg>  vectbugAlg;
	vector<KuVFHPlusPr>  vecKVFHPPr;


private:
	static void doThread(void* arg);
	bool executeBehavior();
	void suspend();
	void resume();
	void terminate();

	KuPose TempDrive(double dTargetX, double dTargetY, double dDesiredVel,KuPose RobotPos);
	KuPose move(double V, double W);
	KuPose calcEncoderData(double dLeftEncData,double dRightEncData);
	bool initialize(KuCommandMessage CMessage);
	bool checkBoundary(vector<KuPose>vecObstacleAreas,double dRandX,double dRandY,KuPose ObsPose);	
	vector<WanderingObs> getvecMultiObs();
	void setvecMultiObs(vector<WanderingObs> vecMultiObs);
	KuPose movingmotion( WanderingObs ObsPos);
	bool movingmotion( WanderingObs ObsPos,vector<WanderingObs> vecObsPos,WanderingObs* outObsPos);
	bool checkBetweenObstacles(WanderingObs WanderOb, vector<WanderingObs> vecObsPos);
	bool checkBetweenAllObstacles(WanderingObs WanderOb, vector<WanderingObs> vecObsPos);

	vector<WanderingObs> selGroupObs( WanderingObs ObsPos, int ngroupnum);
	void initObstacles();
	WanderingObs getGroupWanderingObs(WanderingObs ObsPos,vector<WanderingObs> vecMultiObs);
	vector<WanderingObs> movinggroupmotion( vector<WanderingObs> vecObsPos,vector<WanderingObs> vecMultiObsPos);
	bool checkBetweenWall(WanderingObs WanderOb);
	void checkVirtualObstacle(KuMap* pMap,vector<WanderingObs> vecMultiObs,vector<int>* vecnX,vector<int>* vecnY);
	void resetMap(KuMap* pMap,vector<int> vecnX,vector<int> vecnY);
	void getRandVelocity(double dDistance,double dRandThetaRad,double* dOutDistance,double* dOutRandThetaRad,int nidx);
	WanderingObs getGroupWanderingObsinBoundary(WanderingObs CriPos,WanderingObs ObsPos,vector<WanderingObs> vecMultiObs,int ngroupID,double *deltax,double *deltay);
	bool checkBetweenAllObstacles(WanderingObs WanderOb, vector<WanderingObs> vecObsPos,int ngroupID);


public:
	bool execute(KuCommandMessage CMessage);
	vector<KuPose>getvecMultiWanderingObstacles();
	void setvecMultiWanderingObstacles(vector<KuPose> vecMultiRobot);
	vector<KuPose>getvecMultiGoalPose();


public:
	KuWanderingObstaclePr(void);
	~KuWanderingObstaclePr(void);
};
