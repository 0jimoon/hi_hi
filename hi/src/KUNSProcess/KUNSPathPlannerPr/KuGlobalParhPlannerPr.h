
#ifndef KUNS_GLOBAL_PATHPLANNER_PROCESS_H
#define KUNS_GLOBAL_PATHPLANNER_PROCESS_H

#include <list>
#include <cmath>
#include <iostream>
#include <fstream>

#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../KUNSGUI/KuDrawingInfo.h"
#include "KuGradientPathPlannerbasedPOMapPr.h"
#include "KuPathSmoothingPr.h"
#include "../../MobileSupervisor/KuPOIMapParameter.h"
#include "KuGradientPathPlannerbasedSafeLinePr.h"

using namespace std;
class KuGlobalParhPlannerPr 
{
private:
	KuGradientPathPlannerbasedPOMapPr m_KuGPPPMPr;
	KuPathSmoothingPr m_KuPathSmoothing;
	KuGradientPathPlannerbasedSafeLinePr m_KUGPPSLPr;

private:
	int** m_nMap; //지도정보를 저장할 공간.
	double** m_dMap; //지도정보를 저장할 공간.
	vector<POIMapPara> m_vecPOIMap;
	vector<int**> m_vecnMap;
	vector<double**> m_vecdMap;

private:
	KuMath m_math;
	KuUtil m_KuUtil;

public:
    void setGridMap(int** nMap);
	void setPOMap(double** dMap);
	void setPOIMap(vector<POIMapPara> vecPOIMapData);

public:
    void setRadiusofRobot(int nRadiusofRobot,double dCellSizeM);
	void initializeMapSize(int nMapSizeX, int nMapSizeY);
	void initIntCost(int nRange);
	void setProMapWeight(double dProMapWeight);

public:
	list<KuPose> getPath();   
	int generatePath(float fGoalPosX,float fGoalPosY,int nGoalFloor, float fRobotPosX, float fRobotPosY,int nRobotFloor);
	float calPathDist();

	KuGlobalParhPlannerPr();
	~KuGlobalParhPlannerPr();

private:
	list<KuPose> m_listPath; //경로정보를 저장하는 vector.

    int m_nMapSizeX;
	int m_nMapSizeY;
	int m_nCSpaceObstacle;

	double m_dRadiusofRobot;
	int m_nCSpaceInfinity;

	KuPose m_RobotPos;
	KuPose m_GoalPos;
	double m_dProMapWeight;
    bool m_bPOIMapinitflag;
    bool m_bPOMinitflag;

private:
	void initialize();
	void clear();
	void initMap();
	
};

#endif
