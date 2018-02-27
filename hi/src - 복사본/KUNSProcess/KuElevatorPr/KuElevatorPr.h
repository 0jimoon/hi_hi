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
	double m_dElvLength;//���������� ����
	double m_dElvWidth;//���������� �� �ʺ�
	int m_nCountNum;
	double m_dDelEncoderThetaDeg;
private:
	KuPose CalElvPose(int_1DArray KinectRangeData, KuPose RobotPos);//���������� �߽� ��ġ, ���� ���
	KuPose FirstGoal(KuPose ElvPose,KuPose RobotPos);//ù��° ������Ʈ ������
	double CriterionDistanceX(int_1DArray KinectRangeData);//�����̵Ǵ� �Ÿ��� ����
public:
	void init();
	int EnterElevatorState(int_1DArray KinectRangeData, KuPose POIElevatorPos,KuPose RobotPos, KuPose DelEncoder,double *dTransVel, double *dRotVel);
	KuElevatorPr();
	~KuElevatorPr();

};

#endif /*KUNS_ELEVATOR_PROCESS_H*/