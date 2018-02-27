#include "stdafx.h"
#include "KuSRPMMapBuilderParameter.h"
/**
@brief Korean: ������ �Լ�
@brief English:
*/
KuSRPMMapBuilderParameter::KuSRPMMapBuilderParameter()
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
	m_bLocalMappingflag=false;

}
/**
@brief Korean: �Ҹ��� �Լ�
@brief English:
*/
KuSRPMMapBuilderParameter::~KuSRPMMapBuilderParameter()
{
	
}
/**
@brief Korean: �κ������� �����ϴ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setDelRobotPos(KuPose DelRobotPos)
{
	m_DelRobotPos = DelRobotPos;
}
/**
@brief Korean: �κ������� �Ѱ��ִ� �Լ�
@brief English:
*/
KuPose KuSRPMMapBuilderParameter::getDelRobotPos()
{
	return m_DelRobotPos;
}
/**
@brief Korean: �κ���ġ�� �����ϴ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
}
/**
@brief Korean: �κ���ġ�� �Ѱ��ִ� �Լ�
@brief English:
*/
KuPose KuSRPMMapBuilderParameter::getRobotPos()
{
	return m_RobotPos;
}
/**
@brief Korean: �Ÿ� ������ �����ϴ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setLaserData(int_1DArray nLaserData)
{
	m_nLaserData = nLaserData;	
}
/**
@brief Korean: �Ÿ������� �������� �Լ�
@brief English:
*/
int_1DArray KuSRPMMapBuilderParameter::getLaserData()
{
	return m_nLaserData;
}
/**
@brief Korean: Index�� �����ϴ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setLaserScanIdx(int nLaserScanIdx)
{
	m_nLaserScanIdx = nLaserScanIdx;
}
/**
@brief Korean: Index�� �������� �Լ�
@brief English:
*/
int KuSRPMMapBuilderParameter::getLaserScanIdx()
{
	return m_nLaserScanIdx;
}
/**
@brief Korean: Sensor ���� X Offset�� �����ϴ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setLaserXOffset(double dOffset)
{
	m_dLaserOffset = dOffset;
}
/**
@brief Korean: Sensor ���� X Offset�� �������� �Լ�
@brief English:
*/
double KuSRPMMapBuilderParameter::getLaserXOffset()
{
	return m_dLaserOffset;
}
/**
@brief Korean: Sensor �������� �ּҰŸ��� �����ϴ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setMinDistofSensorData(int nDist)
{
	m_nMinDistofSensorData = nDist;	
}
/**
@brief Korean: Sensor �������� �ִ�Ÿ��� �����ϴ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setMaxDistofSensorData(int nDist)
{
	m_nMaxDistofSensorData = nDist;
}
/**
@brief Korean: Sensor �������� �ּҰŸ��� �������� �Լ�
@brief English:
*/
int KuSRPMMapBuilderParameter::getMinDistofSensorData()
{
	return m_nMinDistofSensorData;
}
/**
@brief Korean: Sensor �������� �ִ�Ÿ��� �������� �Լ�
@brief English:
*/
int KuSRPMMapBuilderParameter::getMaxDistofSensorData()
{
	return m_nMaxDistofSensorData;	
}
/**
@brief Korean: ������ ũ�⸦ �����ϴ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setMapSizeXmYm(int nMapSizeXm, int nMapSizeYm)
{
	m_nMapSizeXm = nMapSizeXm;
	m_nMapSizeYm = nMapSizeYm;
}
/**
@brief Korean: ������ ũ�⸦ �������� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::getMapSizeXmYm(int* nMapSizeXm, int* nMapSizeYm)
{
	*nMapSizeXm = m_nMapSizeXm;
	*nMapSizeYm = m_nMapSizeYm;
}
/**
@brief Korean: �����ۼ��� �ӵ��� ����Ű�� flag�� �����ϴ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setLaserUpdateSpeedflag(bool bMapBuildingspeedflag)
{
	m_bMapBuildingspeedflag = bMapBuildingspeedflag;	
}
/**
@brief Korean: �����ۼ��� �ӵ��� ����Ű�� flag�� �������� �Լ�
@brief English:
*/
bool KuSRPMMapBuilderParameter::getLaserUpdateSpeedflag()
{
	 return m_bMapBuildingspeedflag;	
}
/**
@brief Korean: ������ ũ�⸦ �������ִ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setCellSize(int nCellSize)
{
	m_nCellSize = nCellSize;	
}
/**
@brief Korean: ������ ũ�⸦ �������� �Լ�
@brief English:
*/
int KuSRPMMapBuilderParameter::getCellSize()
{
	return m_nCellSize;	
}
/**
@brief Korean: ���� �α��� �������ִ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setThicknessofWall(double dThicknessofWall)
{
	m_dThicknessofWall = dThicknessofWall;	
}
/**
@brief Korean: ���� �α��� �������� �Լ�
@brief English:
*/
double KuSRPMMapBuilderParameter::getThicknessofWall()
{
	return m_dThicknessofWall;	
}
/**
@brief Korean: �κ��� �������� �������ִ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setRadiusofRobot(double dRadiusofRobot)
{
	m_dRadiusofRobot = dRadiusofRobot;	
}
/**
@brief Korean: �κ��� �������� �������� �Լ�
@brief English:
*/
double KuSRPMMapBuilderParameter::getRadiusofRobot()
{
	return m_dRadiusofRobot;	
}
/**
@brief Korean: �ñ׸��� �������ִ� �Լ�
@brief English:
*/
void KuSRPMMapBuilderParameter::setSigma(double dSigma)
{
	m_dSigma = dSigma;	
}
/**
@brief Korean: �ñ׸��� �������� �Լ�
@brief English:
*/
double KuSRPMMapBuilderParameter::getSigma()
{
	return m_dSigma;	
}

/**
@brief Korean: 
@brief English:
*/
void KuSRPMMapBuilderParameter::setLocalMappingflag(bool bLocalMappingflag)
{
	m_bLocalMappingflag = bLocalMappingflag;	
}
/**
@brief Korean: 
@brief English:
*/
bool KuSRPMMapBuilderParameter::getLocalMappingflag()
{
	 return m_bLocalMappingflag;	
}