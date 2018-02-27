#include "stdafx.h"
#include "KuSRPMMapBuilderParameter.h"
/**
@brief Korean: 생성자 함수
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
@brief Korean: 소멸자 함수
@brief English:
*/
KuSRPMMapBuilderParameter::~KuSRPMMapBuilderParameter()
{
	
}
/**
@brief Korean: 로봇변위를 설정하는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setDelRobotPos(KuPose DelRobotPos)
{
	m_DelRobotPos = DelRobotPos;
}
/**
@brief Korean: 로봇변위를 넘겨주는 함수
@brief English:
*/
KuPose KuSRPMMapBuilderParameter::getDelRobotPos()
{
	return m_DelRobotPos;
}
/**
@brief Korean: 로봇위치를 설정하는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
}
/**
@brief Korean: 로봇위치를 넘겨주는 함수
@brief English:
*/
KuPose KuSRPMMapBuilderParameter::getRobotPos()
{
	return m_RobotPos;
}
/**
@brief Korean: 거리 정보를 설정하는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setLaserData(int_1DArray nLaserData)
{
	m_nLaserData = nLaserData;	
}
/**
@brief Korean: 거리정보를 내보내는 함수
@brief English:
*/
int_1DArray KuSRPMMapBuilderParameter::getLaserData()
{
	return m_nLaserData;
}
/**
@brief Korean: Index를 설정하는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setLaserScanIdx(int nLaserScanIdx)
{
	m_nLaserScanIdx = nLaserScanIdx;
}
/**
@brief Korean: Index를 내보내는 함수
@brief English:
*/
int KuSRPMMapBuilderParameter::getLaserScanIdx()
{
	return m_nLaserScanIdx;
}
/**
@brief Korean: Sensor 장착 X Offset을 설정하는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setLaserXOffset(double dOffset)
{
	m_dLaserOffset = dOffset;
}
/**
@brief Korean: Sensor 장착 X Offset을 내보내는 함수
@brief English:
*/
double KuSRPMMapBuilderParameter::getLaserXOffset()
{
	return m_dLaserOffset;
}
/**
@brief Korean: Sensor 측정가능 최소거리를 설정하는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setMinDistofSensorData(int nDist)
{
	m_nMinDistofSensorData = nDist;	
}
/**
@brief Korean: Sensor 측정가능 최대거리를 설정하는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setMaxDistofSensorData(int nDist)
{
	m_nMaxDistofSensorData = nDist;
}
/**
@brief Korean: Sensor 측정가능 최소거리를 내보내는 함수
@brief English:
*/
int KuSRPMMapBuilderParameter::getMinDistofSensorData()
{
	return m_nMinDistofSensorData;
}
/**
@brief Korean: Sensor 측정가능 최대거리를 내보내는 함수
@brief English:
*/
int KuSRPMMapBuilderParameter::getMaxDistofSensorData()
{
	return m_nMaxDistofSensorData;	
}
/**
@brief Korean: 지도의 크기를 설정하는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setMapSizeXmYm(int nMapSizeXm, int nMapSizeYm)
{
	m_nMapSizeXm = nMapSizeXm;
	m_nMapSizeYm = nMapSizeYm;
}
/**
@brief Korean: 지도의 크기를 내보내는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::getMapSizeXmYm(int* nMapSizeXm, int* nMapSizeYm)
{
	*nMapSizeXm = m_nMapSizeXm;
	*nMapSizeYm = m_nMapSizeYm;
}
/**
@brief Korean: 지도작성의 속도를 향상시키는 flag를 설정하는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setLaserUpdateSpeedflag(bool bMapBuildingspeedflag)
{
	m_bMapBuildingspeedflag = bMapBuildingspeedflag;	
}
/**
@brief Korean: 지도작성의 속도를 향상시키는 flag를 내보내는 함수
@brief English:
*/
bool KuSRPMMapBuilderParameter::getLaserUpdateSpeedflag()
{
	 return m_bMapBuildingspeedflag;	
}
/**
@brief Korean: 격자의 크기를 설정해주는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setCellSize(int nCellSize)
{
	m_nCellSize = nCellSize;	
}
/**
@brief Korean: 격자의 크기를 내보내는 함수
@brief English:
*/
int KuSRPMMapBuilderParameter::getCellSize()
{
	return m_nCellSize;	
}
/**
@brief Korean: 벽의 두깨를 설정해주는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setThicknessofWall(double dThicknessofWall)
{
	m_dThicknessofWall = dThicknessofWall;	
}
/**
@brief Korean: 벽의 두깨를 내보내는 함수
@brief English:
*/
double KuSRPMMapBuilderParameter::getThicknessofWall()
{
	return m_dThicknessofWall;	
}
/**
@brief Korean: 로봇의 반지름을 설정해주는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setRadiusofRobot(double dRadiusofRobot)
{
	m_dRadiusofRobot = dRadiusofRobot;	
}
/**
@brief Korean: 로봇의 반지름을 내보내는 함수
@brief English:
*/
double KuSRPMMapBuilderParameter::getRadiusofRobot()
{
	return m_dRadiusofRobot;	
}
/**
@brief Korean: 시그마를 설정해주는 함수
@brief English:
*/
void KuSRPMMapBuilderParameter::setSigma(double dSigma)
{
	m_dSigma = dSigma;	
}
/**
@brief Korean: 시그마를 내보내는 함수
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