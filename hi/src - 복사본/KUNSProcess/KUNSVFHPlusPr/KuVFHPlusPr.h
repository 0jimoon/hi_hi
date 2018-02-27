#pragma once
#include "VFHPlus.h"
#include "mm.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../Sensor/Sensor.h"
#include "LocalMap.h"
#include "../../MobileSupervisor/KuRobotParameter.h"

class KuVFHPVelocity
{
public:
	double dTranslationVel;
	double dXVel;
	double dYVel;
	double dRotationDegVel;
};
class KuVFHPlusPr
{
private:
	VFHPlus m_VFHPlus;

	double _steer_direction ;
	double _wheel_radius;
	double _axle_length ;
	double _robot_radius;
	CLocalMap m_certainty_grid;
public:
	KuVFHPVelocity generateTRVelocity(KuPose TargetPos, KuPose RobotPos, int_1DArray  nSensorData );
	void init();
	void setRobotRadius(double robot_radius);
	void setVelocity(double dMaxVel,double dMinVel,double dRVel);
	void setSensorParameter(double dXoffset,double dYoffset,double dThetaoffset,double dMaxRange,double dMinRange);

public:
	KuVFHPlusPr(void);
	~KuVFHPlusPr(void);
};

