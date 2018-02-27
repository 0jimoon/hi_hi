#include "stdafx.h"
#include "KuMapBuilderParameter.h"
/**
@brief Korean: ������ �Լ�
@brief English:
*/
KuMapBuilderParameter::KuMapBuilderParameter()
{
	m_nLaserScanIdx = 181;
	m_nMinDistofSensorData = 50;//mm
	m_nMaxDistofSensorData = 20000;//mm
	m_nMapSizeXm = 100;//mm
	m_nMapSizeYm = 100;//mm
	m_dLaserOffset = 230;//mm
	m_nCellSize = 100;//mm
	m_dThicknessofWall = 50;//mm
	m_dRadiusofRobot = 400;//mm
	m_bMapBuildingspeedflag=false;
	m_dSigma=50;
}
/**
@brief Korean: �Ҹ��� �Լ�
@brief English:
*/
KuMapBuilderParameter::~KuMapBuilderParameter()
{

}
/**
@brief Korean: �κ������� �����ϴ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setDelRobotPos(KuPose DelRobotPos)
{
	m_DelRobotPos = DelRobotPos;
}
/**
@brief Korean: �κ������� �Ѱ��ִ� �Լ�
@brief English:
*/
KuPose KuMapBuilderParameter::getDelRobotPos()
{
	return m_DelRobotPos;
}
/**
@brief Korean: �κ���ġ�� �����ϴ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
}
/**
@brief Korean: �κ���ġ�� �Ѱ��ִ� �Լ�
@brief English:
*/
KuPose KuMapBuilderParameter::getRobotPos()
{
	return m_RobotPos;
}
/**
@brief Korean: �Ÿ� ������ �����ϴ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setLaserData(int_1DArray nLaserData)
{
	m_nLaserData = nLaserData;	
}
/**
@brief Korean: �Ÿ������� �������� �Լ�
@brief English:
*/
int_1DArray KuMapBuilderParameter::getLaserData()
{
	return m_nLaserData;
}
/**
@brief Korean: Index�� �����ϴ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setLaserScanIdx(int nLaserScanIdx)
{
	m_nLaserScanIdx = nLaserScanIdx;
}
/**
@brief Korean: Index�� �������� �Լ�
@brief English:
*/
int KuMapBuilderParameter::getLaserScanIdx()
{
	return m_nLaserScanIdx;
}
/**
@brief Korean: Sensor ���� X Offset�� �����ϴ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setLaserXOffset(double dOffset)
{
	m_dLaserOffset = dOffset;
}
/**
@brief Korean: Sensor ���� X Offset�� �������� �Լ�
@brief English:
*/
double KuMapBuilderParameter::getLaserXOffset()
{
	return m_dLaserOffset;
}
/**
@brief Korean: Sensor �������� �ּҰŸ��� �����ϴ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setMinDistofSensorData(int nDist)
{
	m_nMinDistofSensorData = nDist;	
}
/**
@brief Korean: Sensor �������� �ִ�Ÿ��� �����ϴ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setMaxDistofSensorData(int nDist)
{
	m_nMaxDistofSensorData = nDist;
}
/**
@brief Korean: Sensor �������� �ּҰŸ��� �������� �Լ�
@brief English:
*/
int KuMapBuilderParameter::getMinDistofSensorData()
{
	return m_nMinDistofSensorData;
}
/**
@brief Korean: Sensor �������� �ִ�Ÿ��� �������� �Լ�
@brief English:
*/
int KuMapBuilderParameter::getMaxDistofSensorData()
{
	return m_nMaxDistofSensorData;	
}
/**
@brief Korean: ������ ũ�⸦ �����ϴ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setMapSizeXmYm(int nMapSizeXm, int nMapSizeYm)
{
	m_nMapSizeXm = nMapSizeXm;
	m_nMapSizeYm = nMapSizeYm;
}
/**
@brief Korean: ������ ũ�⸦ �������� �Լ�
@brief English:
*/
void KuMapBuilderParameter::getMapSizeXmYm(int* nMapSizeXm, int* nMapSizeYm)
{
	*nMapSizeXm = m_nMapSizeXm;
	*nMapSizeYm = m_nMapSizeYm;
}
/**
@brief Korean: ������ ũ�⸦ �����ϴ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setMapSizeXYAI(int nMapSizeXAI, int nMapSizeYAI)
{
	m_nMapSizeXAI = nMapSizeXAI;
	m_nMapSizeYAI = nMapSizeYAI;
}
/**
@brief Korean: ������ ũ�⸦ �������� �Լ�
@brief English:
*/
void KuMapBuilderParameter::getMapSizeXYAI(int* nMapSizeXAI, int* nMapSizeYAI)
{
	*nMapSizeXAI = m_nMapSizeXAI;
	*nMapSizeYAI = m_nMapSizeYAI;
}
/**
@brief Korean: �����ۼ��� �ӵ��� ����Ű�� flag�� �����ϴ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setLaserUpdateSpeedflag(bool bMapBuildingspeedflag)
{
	m_bMapBuildingspeedflag = bMapBuildingspeedflag;	
}
/**
@brief Korean: �����ۼ��� �ӵ��� ����Ű�� flag�� �������� �Լ�
@brief English:
*/
bool KuMapBuilderParameter::getLaserUpdateSpeedflag()
{
	 return m_bMapBuildingspeedflag;	
}
/**
@brief Korean: ������ ũ�⸦ �������ִ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setCellSize(int nCellSize)
{
	m_nCellSize = nCellSize;	
}
/**
@brief Korean: ������ ũ�⸦ �������� �Լ�
@brief English:
*/
int KuMapBuilderParameter::getCellSize()
{
	return m_nCellSize;	
}
/**
@brief Korean: ���� �α��� �������ִ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setThicknessofWall(double dThicknessofWall)
{
	m_dThicknessofWall = dThicknessofWall;	
}
/**
@brief Korean: ���� �α��� �������� �Լ�
@brief English:
*/
double KuMapBuilderParameter::getThicknessofWall()
{
	return m_dThicknessofWall;	
}
/**
@brief Korean: �κ��� �������� �������ִ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setRadiusofRobot(double dRadiusofRobot)
{
	m_dRadiusofRobot = dRadiusofRobot;	
}
/**
@brief Korean: �κ��� �������� �������� �Լ�
@brief English:
*/
double KuMapBuilderParameter::getRadiusofRobot()
{
	return m_dRadiusofRobot;	
}
/**
@brief Korean: �ñ׸��� �������ִ� �Լ�
@brief English:
*/
void KuMapBuilderParameter::setSigma(double dSigma)
{
	m_dSigma = dSigma;	
}
/**
@brief Korean: �ñ׸��� �������� �Լ�
@brief English:
*/
double KuMapBuilderParameter::getSigma()
{
	return m_dSigma;	
}
