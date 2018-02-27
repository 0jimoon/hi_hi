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
	double MIN_PROBABILITY;// ���� ���� ���� �ּ� Ȯ�� ��
	double MAX_PROBABILITY;// ���� ���� ���� �ִ� Ȯ�� ��
	double INITIAL_PROBABILITY;// �ʱ� Ȯ��(unknown region) : 0.5
	double GAUSSIAN_SD;//���� ���� ����þ� ǥ������ ��, ���� ������ ���� ����
	double m_dThicknessofWall;// ���� �β� mm
	double m_dRadiusofRobot;//�κ������� 400(mm)
	double m_dLaserSensorXOffset;
	double** m_dProMap; // Ȯ�� ���� ����
	int** m_nMap;
	int** m_nRefMap;
	int** m_nRefOriMap;
	bool m_bLocalMappingflag;

	KuMap* m_pMap; // ���������ۼ��� ���� ������ ������ �����ϱ� ���� ����.

	vector<int> m_vetnIndex;
	vector<KuCartesianCoordinate2D>m_CartesianCoordiNewRangeData;
	vector<KuCartesianCoordinate2D>m_CartesianCoordiLastRangeData;


private:
	void buildGridmapByBayesUpdate(KuPose RobotPos, int_1DArray nLaserData, int nLaserIdx, bool bupdateSpeedflag);//������꿡 ���� Ȯ���� ������Ʈ �Լ�
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
	double** getProMap(); //Ȯ�� �ʵ����͸� �������� �Լ�
	int** getMap(); //�ʵ����͸� �������� �Լ�
	void setSigma(double dSigma);//���� ���� ����þ� ǥ������ ���� �����ϴ� �Լ�
	KuMap* getpMap();
	void initMap();
	void setReferenceCADMap(int** nMap );
	void setReferenceOriMap(int** nMap );
	
	KuSRPMLaserMapBuilderPr(void);
	~KuSRPMLaserMapBuilderPr(void);
		
};
#endif