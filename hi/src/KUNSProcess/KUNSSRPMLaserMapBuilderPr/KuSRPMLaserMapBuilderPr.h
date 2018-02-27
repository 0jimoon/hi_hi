#ifndef C_SRPM_LASER_MAPBUIDER_H
#define C_SRPM_LASER_MAPBUIDER_H

#include <iostream> 
#include <list>
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"
#include "KuSRPMMapBuilderParameter.h"
#include "../../Sensor/Sensor.h"
#include "../../Algorithm/KuICP/KuICP.h"

using namespace std;
class KuSRPMLaserMapBuilderPr 
{
private:
	KuUtil m_KuUtil;
	KuICP m_KuICP;
	KuMath m_Math;
private:
	KuPose m_RobotPos;
	int m_nScanIdX;
	int m_nLaserMinDist, m_nLaserMaXDist;
	int_1DArray m_nLaserData;
	KuPose* m_LaserscannerConfiguration;
	int m_nMapSizeX, m_nMapSizeY;
	
	int m_nCellSize;	//1cell 100mm;
	double MIN_PROBABILITY;// 센서 모델이 갖는 최소 확률 값
	double MAX_PROBABILITY;// 센서 모델이 갖는 최대 확률 값
	double INITIAL_PROBABILITY;// 초기 확률(unknown region) : 0.5
	double GAUSSIAN_SD;//센서 모델의 가우시안 표준편차 값, 점유 영역의 폭을 조절
	double m_dThicknessofWall;// 벽의 두께 mm
	double m_dRadiusofRobot;//로봇반지름 400(mm)
	double m_dLaserSensorXOffset;
	double** m_dProMap; // 확률 격자 지도
	int** m_nMap;
	int** m_nRefMap;
	int** m_nRefOriMap;
	bool m_bLocalMappingflag;

	KuMap* m_pMap; // 자율지도작성을 통해 형성된 지도를 저장하기 위한 공간.

	vector<int> m_vetnIndex;
	vector<KuCartesianCoordinate2D>m_CartesianCoordiNewRangeData;
	vector<KuCartesianCoordinate2D>m_CartesianCoordiLastRangeData;


private:
	void buildGridmapByBayesUpdate(KuPose RobotPos, int_1DArray nLaserData, int nLaserIdx, bool bupdateSpeedflag);//베이즈룰에 따른 확률값 업데이트 함수
	void doMorphologyCloseForOccupiedGrid(int** nMap);
	void  calculateDatafromMap( KuPose RobotPos, int_1DArray nLaserData, int nLaserIdx);
	inline void copyRangeData(int_1DArray  nData);
	void  calculateLaserDatafromMap( KuPose RobotPos,int** nMap);
	inline void copyRangeDatafromMap(	vector<int> nLaserData);
	double generateRobotPose( );
	void computeRobotPoseByICPFor2D(double dX, double dY, double dTheta);

	bool compareMapwithCADMap(int nX, int nY );

public:
	void initialize(KuSRPMMapBuilderParameter InputParam);
	void buildMap(KuSRPMMapBuilderParameter InputParam);
	double** getProMap(); //확률 맵데이터를 가져가는 함수
	int** getMap(); //맵데이터를 가져가는 함수
	void setSigma(double dSigma);//센서 모델의 가우시안 표준편차 값을 조정하는 함수
	KuMap* getpMap();
	void initMap();
	void setReferenceCADMap(int** nMap );
	void setReferenceOriMap(int** nMap );
	
	KuSRPMLaserMapBuilderPr(void);
	~KuSRPMLaserMapBuilderPr(void);
		
};
#endif