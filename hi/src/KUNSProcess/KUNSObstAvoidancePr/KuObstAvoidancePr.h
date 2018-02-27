
#ifndef KUNS_OBASTACLE_AVOIDENCE_H
#define KUNS_OBASTACLE_AVOIDENCE_H

#include <iostream>
#include <conio.h>
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../KUNSMap/KuMapRepository.h"
#include "../../KUNSProcess/KUNSPathPlannerPr/KuGradientPathPlannerPr.h"
#include "../../KUNSProcess/KUNSPathPlannerPr/KuPathSmoothingPr.h"
#include "../../KUNSProcess/KUNSLaserMapBuilderPr/KuLaserMapBuilderPr.h"
#include "../../KUNSProcess/KUNSLaserMapBuilderPr/KuMapBuilderParameter.h"
#include "../../KUNSGUI/KuDrawingInfo.h"

using namespace std;
class KuObstAvoidancePr :public KuSingletone <KuObstAvoidancePr>
{
	static const int INFINITY_VALUE = 100000;
	static const int LOCALGOAL_DISTANCE = 5000;
	static const int LOCALPATH_COST = 5;


private:
	KuGradientPathPlannerPr m_KuPathPlanner; //경로계획기 인스턴스
	KuPathSmoothingPr m_KuPathSmoothing;
	KuMath m_math;
	KuLaserMapBuilderPr m_LaserMapBuilder;
	KuGradientPathPlannerPr m_KuLocalPathPlanner; //지역 경로 계획기 생성.
	KuThread m_LocalPathplaningThread;
	KuTimer m_timer;

private:
	KuPose m_RobotPos, m_GoalPos, m_TargetPos;	
	int m_nThreadFuncPeriod;
	vector<KuPose> m_vecPath;
	vector<KuPose> m_vectempPath;
	bool m_bThreadFlag; 
	KuMap * m_pMap;
	bool m_bStandbyFlag;

private:

	int m_nCellSize;
	int m_nDistToTarget;

	int m_nLaserDataIDX;
	double m_dLaserSensorOffsetmm;
	double m_dLaserMinDistanceMM;
	double m_dLaserMaxDistanceMM;

	int m_nKinectRnageDataIDX;
	double m_dKinectSensorOffsetmm;
	double m_dKinectMaxDistanceMM;
	double m_dKinectMinDistanceMM;

	double m_dRobotRadius;


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

	int_1DArray m_nLaserData;
	int_1DArray m_nKinectRnageData;
	KuPose m_DelEncoderData;
	int m_nPrePathIndx;
	int m_nSelectedMinIndx;


private:
	bool initialize();
	KuPose getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist);
	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);
	static void doLocalPathplanningThread(void* arg);
	void doCriticalSection(int nID);

private:
	void generateLocalMap(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData181, int_1DArray nKinectRnageData);
	void copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap, int** nOriginalMap, KuPose RobotPos);
	void copyGlobalMapToLocalMap(int**nGlobalMap, int** nLocalMap, KuPose RobotPos, int* nLocalMapSPosX, int* nLocalMapSPosY);
	void copyOriginalMapToGlobalMap(int** nOriginalMap, int** nGlobalMap, KuPose RobotPos);
	void copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap,  KuPose RobotPos);
	int_1DArray combinateLaserNKinect(int_1DArray nLaserData181, int_1DArray nKinectRnageData);
	bool generateLocalPathforObsAvoidance(KuPose RobotPos,KuPose DelEncoderData);
	bool existObstaclePath(KuPose RobotPos, vector<KuPose> vecLocalPath, int nPathSize, KuMap* pLocalMap, int nLocalMapSPosX, int nLocalMapSPosY );
	bool generateLocalPath(KuPose RobotPos);
	bool checkObstacles(KuPose RobotPos,KuPose TargetPos, int_1DArray nLaserData181, int_1DArray nKinectRnageData ,double dTargetDist);
	KuPose generateDetourGoalPos(KuPose RobotPos, vector<KuPose> vecPath, int nPathSize);
	bool generateDetourPath(KuPose RobotPos, KuPose DetourGoalPos, KuMap* pLocalMap, 
		int nLocalMapSPosX, int nLocalMapSPosY, list<KuPose> *DetourPathList);
	list<KuPose> transferLocaltoGlobalPath( list<KuPose> LocalPath,KuPose RobotPose,int nLocalMapSPosX, int nLocalMapSPosY);
	void initMapbuildingProcess(bool bLocalMapflag);
	bool initPathplanningProcess();
	void calStartPathPoint( KuPose RobotPos );
	KuPose calPathPoint( KuPose RobotPos);


public:
	bool start(int nTime);
	void terminate();
	KuPose getTargetPos();
	void setData(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData,int_1DArray nKinectRnageData);
	void setPath(list<KuPose> listPath);
	list<KuPose> getPath();
	void setData(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nKinectRnageData);
	void setParameter(int nLaserDataIDX,double dLaserSensorOffsetmm,double dLaserMaxDistanceMM,double dLaserMinDistanceMM,
		int nKinectRnageDataIDX,double dKinectSensorOffsetmm,double dKinectMaxDistanceMM,double dKinectMinDistanceMM,
		int nCellSize,int nDistToTarget,double dRobotRadius);

	void setMap(int **nRefMap,int nMapSizeX,int nMapSizeY);

public:
	KuObstAvoidancePr();
	~KuObstAvoidancePr();

};

#endif