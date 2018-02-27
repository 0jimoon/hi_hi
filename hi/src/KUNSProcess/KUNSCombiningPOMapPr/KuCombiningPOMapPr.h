
#ifndef KUNS_COMBINING_POMAP_H
#define KUNS_COMBINING_POMAP_H

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
#include "../../MobileSupervisor/KuRobotParameter.h"

using namespace std;
class KuCombiningPOMapPr :public KuSingletone <KuCombiningPOMapPr>
{


private:
	KuMath m_math;
	//KuCriticalSection m_CriticalSection;

private:
	KuPose m_RobotPos, m_GoalPos, m_TargetPos;	
	int m_nThreadFuncPeriod;
	bool m_bThreadFlag; 


private:
	KuUtil m_KuUtil;
	int m_nMapSizeX;
	int m_nMapSizeY;
	int m_nMapTimeNum;
	int m_nMapNum;
	
	int m_nDayNum;
	int m_nHourNum;
	int m_nTimeNum;

	int m_nHour[25];
	int m_nLastNum;

	double** m_dOutMap;

	vector<double**> m_vecProbabilityMap;
	KuSmartPointer<double**> m_dProbabilityMap;

	//int m_nCellSize;	//1cell 1000mm;
	double m_dRadiusofRobot;//로봇반지름 400(mm)
	string m_strMapFilePath;

private:
	bool loadProMap(string strMapFilePath,int nMapSizeX,int nMapSizeY, double** dMap);
	void saveProMap(string strMapFilePath,int nMapSizeX,int nMapSizeY, double** dMap);
	void calGradientMask(double** dProMap,int nX,int nY,double** dGradientMask,int nMapSizeX,int nMapSizeY);
	void calCorrespondentPro(double** dProGradient,double** dCurMap,double** dPreMap,int nX,int nY );
	void generateProMap( double**dCurTimeMap,double**dOutMap);
	void calNormalization(double** dMultiplicationPro,double** dOutNormalPro);
	double calConditionalPro(double**dCurTimeMap,double** dOutNormalPro,double dXPro,int nX,int nY );
	void loadLastMap(double**dCurTimeMap );
	void countPro(double**dGradPro,double**dNextMap, int nX, int nY,double dTotalNum );
	void calPostPro(double**dPostPro,double**dDatabaseMap, int nX, int nY,double** dNum);
	void calOccLikelihoodPro(double**dLikelihood,double**dDatabasetMap, int nX, int nY,double** dNum,double**dCurStateMap);
	void calEmptyLikelihoodPro(double**dLikelihood,double**dDatabasetMap, int nX, int nY,double** dNum,double**dCurStateMap);
	double  calPriProValue(int nMaskSize,double** dPosterioriPro,double** dOccLikelihood,double** dEmptyLikelihood,double dXPro  );

	void calPostProNum(double**dPosterioriNum,double**dDatabaseMap, int nX, int nY );
	void calOccLikelihoodNum(double**dLikelihoodNum,double**dDatabasetMap, int nX, int nY ,double**dCurStateMap);
	void calEmptyLikelihoodNum(double**dLikelihood,double**dDatabasetMap, int nX, int nY,double**dCurStateMap );


public:
	bool initialize(int nMapSizeX,int nMapSizeY );
	bool loadProMap(string strMapFilePath);
	bool saveProMap(string strMapFilePath,double** dProMap);
	void execute(string strProMapNameNPath );
	double** getMap( );

public:
	KuCombiningPOMapPr();
	~KuCombiningPOMapPr();

};

#endif


