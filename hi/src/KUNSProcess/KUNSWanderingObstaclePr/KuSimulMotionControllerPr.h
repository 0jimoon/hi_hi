#ifndef C_SIMULATION_MOTION_CONTROLLER_H
#define C_SIMULATION_MOTION_CONTROLLER_H


#include <list>
#include <iostream>
#include <fstream>
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../Sensor/VirtualSensor/KuVrWheelActuatorInterface.h"
#include "KuSimulLocalizer.h"



using namespace std;
class KuSimulMotionControllerPr
{
private:
	static const int GOAL_AREA = 300;
	static const int TARGET_DIST = 950; //mm단위이며, 목표지점을 선택하는 거리 즉 1.4를 목표지점으로 설정

private:
	
	KuVrWheelActuatorInterface m_KuVrWheelActuator; 

private:
	int m_bMotionDirection;
	double m_dMaxTVel;
	double m_dDesiredVel;
	int** m_nPath;
	double** m_dPath;
	int m_nPathCnt, m_nPathNoPointer;
	KuPose m_TargetPos;
	double m_dXVel;
	double m_dYVel;
	
	bool m_bAlignRobotAngleFlag;

private:
	KuPose clacNextTargetPoint(KuPose RobotPos);
	void move(double dTVal, double dRVal);	

public:
	void turnRight();
	void turnLeft();
	void stop();
	void TempDrive(double dTargetX, double dTargetY, double dDesiredVel,KuPose RobotPos);
	KuSimulMotionControllerPr();
	~KuSimulMotionControllerPr();	

public:
	void init();
	KuPose getTargetPos();
	bool move(KuPose RobotPos, list<KuPose> PathList,bool* bIsNewPath);
};

#endif
