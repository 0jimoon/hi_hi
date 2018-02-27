#ifndef KUNS_ELEVATOR_PROCESS_H
#define KUNS_ELEVATOR_PROCESS_H

#include <vector>
#include <list>
#include <iostream>

#include "../../KUNSPose/KuPose.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSMath/KuMath.h"
#include "../../kunsgui/KuDrawingInfo.h"
#include "../../Sensor/Sensor.h"
#include <cstdlib>

using namespace std;

class KuElevatorPr
{
private:
	int m_nState;
	KuPose m_ElvPos;
	KuPose m_FirstGoalPos;
	double m_dXmin;
	double m_dElvLength;//엘리베이터 길이
	double m_dElvWidth;//엘리베이터 문 너비
	int m_nCountNum;
	double m_dDelEncoderThetaDeg;
private:
	KuPose CalElvPose(int_1DArray KinectRangeData, KuPose RobotPos);//엘리베이터 중심 위치, 각도 계산
	KuPose FirstGoal(KuPose ElvPose,KuPose RobotPos);//첫번째 골포인트 정보줌
	double CriterionDistanceX(int_1DArray KinectRangeData);//기준이되는 거리값 설정
public:
	void init();
	int EnterElevatorState(int_1DArray KinectRangeData, KuPose POIElevatorPos,KuPose RobotPos, KuPose DelEncoder,double *dTransVel, double *dRotVel);
	KuElevatorPr();
	~KuElevatorPr();

};

#endif /*KUNS_ELEVATOR_PROCESS_H*/