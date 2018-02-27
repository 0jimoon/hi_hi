
#ifndef KUNS_PROBABILITY_BUILDINGMAP_H
#define KUNS_PROBABILITY_BUILDINGMAP_H

#include <iostream>
#include <conio.h>
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSGUI/KuDrawingInfo.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../KUNSMap/KuMapRepository.h"
#include "../../KUNSUtil/KUNSTimer/KuTimer.h"
#include "KuBuildingPOMapParameter.h"

using namespace std;
class KuBuildingPOMapPr :public KuSingletone <KuBuildingPOMapPr>
{


private:
	KuMath m_math;
	//KuCriticalSection m_CriticalSection;
	KuThread m_BuildingProMapThread;

private:
	KuPose m_RobotPos, m_GoalPos, m_TargetPos;	
	int m_nThreadFuncPeriod;
	bool m_bThreadFlag; 
	KuMap * m_pMap;

	double** m_dProMap; // 확률 격자 지도
	double** m_dProMinMap; // 확률 격자 지도
	int** m_nMap;
	int** m_nRefMap;

	int m_nDay;
	int m_nHour;
	int m_nDataNum;

private:
	KuUtil m_KuUtil;
	KuTimer m_timer;
	list<KuPose>  m_DetourPathList;
	int m_nLocalMapSPosX, m_nLocalMapSPosY;
	int m_nGlobalMapX, m_nGlobalMapY;
	int m_nBuildingMapX, m_nBuildingMapY;
	int m_nMapSizeX;
	int m_nMapSizeY;
	KuMap* m_pRefMap;
	KuMap *m_pLocalMap;
	KuMap *m_pOriginalMap; 
	int m_nMapTimeNum;
	int m_nMapNum;
	bool m_bStandbyFlag;

	int_1DArray m_nRangeData;
	int_1DArray m_ntempRangeData;

	int_1DArray m_nKinectRnageData;
	KuPose m_DelEncoderData;
	int m_nScanIdX;
	int m_nLaserMinDist, m_nLaserMaXDist;
	KuPose* m_LaserscannerConfiguration;
	vector<double**> m_vecProbabilityMap;

	int m_nCellSize;	//1cell 1000mm;
	double MIN_PROBABILITY;// 센서 모델이 갖는 최소 확률 값
	double MAX_PROBABILITY;// 센서 모델이 갖는 최대 확률 값
	double INITIAL_PROBABILITY;// 초기 확률(unknown region) : 0.5
	double GAUSSIAN_SD;//센서 모델의 가우시안 표준편차 값, 점유 영역의 폭을 조절
	double m_dThicknessofWall;// 벽의 두께 mm
	double m_dRadiusofRobot;//로봇반지름 400(mm)
	string m_strMapFilePath;

private:
	bool initialize(KuBuildingPOMapParameter InputParam);
	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);
	void updateBayesianformula(KuPose RobotPos,KuPose DelEncoderData,int nRangeIdx, int_1DArray nRangeData);
	bool loadProMap(string strMapFilePath);
	bool saveProMap(string strMapFilePath);
	bool loadProMap(string strMapFilePath,int nMapSizeX,int nMapSizeY, double** dMap);
	void saveProMap(string strMapFilePath,int nMapSizeX,int nMapSizeY, double** dMap);


private:
	void initMapbuildingProcess(bool bLocalMapflag);
	static void doProBuilingdThread(void* arg);


public:
	bool start(KuBuildingPOMapParameter InputParam);
	void terminate();
	void setData(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nRangeData);
	double** getProMap();
	bool saveProMap( );

public:
	KuBuildingPOMapPr();
	~KuBuildingPOMapPr();

};

#endif