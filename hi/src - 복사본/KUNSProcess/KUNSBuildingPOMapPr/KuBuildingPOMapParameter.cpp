#include "stdafx.h"
#include "KuBuildingPOMapParameter.h"
/**
@brief Korean: ������ �Լ�
@brief English:
*/
KuBuildingPOMapParameter::KuBuildingPOMapParameter()
{
	m_nLaserScanIdx = 181;
	m_nMinDistofSensorData = 30;//mm
	m_nMaxDistofSensorData = 30000;//mm
	m_nMapSizeXm = 100;//mm
	m_nMapSizeYm = 100;//mm
	m_dLaserOffset = 230;//mm
	m_nCellSize = 100;//mm
	m_dThicknessofWall = 30;//mm
	m_dRadiusofRobot = 400;//mm
	m_bMapBuildingspeedflag=false;
	m_dSigma=50;
	m_dXOffset=230;
}
/**
@brief Korean: �Ҹ��� �Լ�
@brief English:
*/
KuBuildingPOMapParameter::~KuBuildingPOMapParameter()
{
	
}
/**
@brief Korean: �κ������� �����ϴ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setDelRobotPos(KuPose DelRobotPos)
{
	m_DelRobotPos = DelRobotPos;
}
/**
@brief Korean: �κ������� �Ѱ��ִ� �Լ�
@brief English:
*/
KuPose KuBuildingPOMapParameter::getDelRobotPos()
{
	return m_DelRobotPos;
}
/**
@brief Korean: �κ���ġ�� �����ϴ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
}
/**
@brief Korean: �κ���ġ�� �Ѱ��ִ� �Լ�
@brief English:
*/
KuPose KuBuildingPOMapParameter::getRobotPos()
{
	return m_RobotPos;
}
/**
@brief Korean: �Ÿ� ������ �����ϴ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setLaserData(int_1DArray nLaserData)
{
	m_nLaserData = nLaserData;	
}
/**
@brief Korean: �Ÿ������� �������� �Լ�
@brief English:
*/
int_1DArray KuBuildingPOMapParameter::getLaserData()
{
	return m_nLaserData;
}
/**
@brief Korean: Index�� �����ϴ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setLaserScanIdx(int nLaserScanIdx)
{
	m_nLaserScanIdx = nLaserScanIdx;
}
/**
@brief Korean: Index�� �������� �Լ�
@brief English:
*/
int KuBuildingPOMapParameter::getLaserScanIdx()
{
	return m_nLaserScanIdx;
}
/**
@brief Korean: Sensor ���� X Offset�� �����ϴ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setLaserXOffset(double dOffset)
{
	m_dLaserOffset = dOffset;
}
/**
@brief Korean: Sensor ���� X Offset�� �������� �Լ�
@brief English:
*/
double KuBuildingPOMapParameter::getLaserXOffset()
{
	return m_dLaserOffset;
}
/**
@brief Korean: Sensor �������� �ּҰŸ��� �����ϴ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setMinDistofSensorData(int nDist)
{
	m_nMinDistofSensorData = nDist;	
}
/**
@brief Korean: Sensor �������� �ִ�Ÿ��� �����ϴ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setMaxDistofSensorData(int nDist)
{
	m_nMaxDistofSensorData = nDist;
}
/**
@brief Korean: Sensor �������� �ּҰŸ��� �������� �Լ�
@brief English:
*/
int KuBuildingPOMapParameter::getMinDistofSensorData()
{
	return m_nMinDistofSensorData;
}
/**
@brief Korean: Sensor �������� �ִ�Ÿ��� �������� �Լ�
@brief English:
*/
int KuBuildingPOMapParameter::getMaxDistofSensorData()
{
	return m_nMaxDistofSensorData;	
}
/**
@brief Korean: ������ ũ�⸦ �����ϴ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setMapSizeXmYm(int nMapSizeXm, int nMapSizeYm)
{
	m_nMapSizeXm = nMapSizeXm;
	m_nMapSizeYm = nMapSizeYm;
}
/**
@brief Korean: ������ ũ�⸦ �������� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::getMapSizeXmYm(int* nMapSizeXm, int* nMapSizeYm)
{
	*nMapSizeXm = m_nMapSizeXm;
	*nMapSizeYm = m_nMapSizeYm;
}
/**
@brief Korean: �����ۼ��� �ӵ��� ����Ű�� flag�� �����ϴ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setLaserUpdateSpeedflag(bool bMapBuildingspeedflag)
{
	m_bMapBuildingspeedflag = bMapBuildingspeedflag;	
}
/**
@brief Korean: �����ۼ��� �ӵ��� ����Ű�� flag�� �������� �Լ�
@brief English:
*/
bool KuBuildingPOMapParameter::getLaserUpdateSpeedflag()
{
	 return m_bMapBuildingspeedflag;	
}
/**
@brief Korean: ������ ũ�⸦ �������ִ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setCellSize(int nCellSize)
{
	m_nCellSize = nCellSize;	
}
/**
@brief Korean: ������ ũ�⸦ �������� �Լ�
@brief English:
*/
int KuBuildingPOMapParameter::getCellSize()
{
	return m_nCellSize;	
}
/**
@brief Korean: ���� �α��� �������ִ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setThicknessofWall(double dThicknessofWall)
{
	m_dThicknessofWall = dThicknessofWall;	
}
/**
@brief Korean: ���� �α��� �������� �Լ�
@brief English:
*/
double KuBuildingPOMapParameter::getThicknessofWall()
{
	return m_dThicknessofWall;	
}
/**
@brief Korean: �κ��� �������� �������ִ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setRadiusofRobot(double dRadiusofRobot)
{
	m_dRadiusofRobot = dRadiusofRobot;	
}
/**
@brief Korean: �κ��� �������� �������� �Լ�
@brief English:
*/
double KuBuildingPOMapParameter::getRadiusofRobot()
{
	return m_dRadiusofRobot;	
}
/**
@brief Korean: �ñ׸��� �������ִ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setSigma(double dSigma)
{
	m_dSigma = dSigma;	
}
/**
@brief Korean: �ñ׸��� �������� �Լ�
@brief English:
*/
double KuBuildingPOMapParameter::getSigma()
{
	return m_dSigma;	
}

/**
@brief Korean: �ñ׸��� �������ִ� �Լ�
@brief English:
*/
void KuBuildingPOMapParameter::setXOffset(double dXOffset)
{
	m_dXOffset = dXOffset;	
}
/**
@brief Korean: �ñ׸��� �������� �Լ�
@brief English:
*/
double KuBuildingPOMapParameter::getXOffset()
{
	return m_dXOffset;	
}

void KuBuildingPOMapParameter::setPath(string strMapFilePath)
{
	m_strMapFilePath=strMapFilePath;
}

string KuBuildingPOMapParameter::getPath()
{
	return m_strMapFilePath;
}