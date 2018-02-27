#include "StdAfx.h"
#include "KuVFHPlusPr.h"


KuVFHPlusPr::KuVFHPlusPr(void)
{
	_wheel_radius = 0.2;
	_steer_direction = 0;
	_axle_length = 0.4;
	_robot_radius = 0.2;
}


KuVFHPlusPr::~KuVFHPlusPr(void)
{
}
void KuVFHPlusPr::init()
{
	m_certainty_grid.Clear();
	m_certainty_grid.setSensorParameter(0.0,0.0,0.0,2.0	,0.1);
	m_VFHPlus.setVelocity(900,10,30);

	//m_VFHPlus.setRobotRadius(KuRobotParameter::getInstance()->getRobotRadius()/1000.0);

}
void KuVFHPlusPr::setVelocity(double dMaxVel,double dMinVel,double dRVel)
{
	m_VFHPlus.setVelocity(dMaxVel,dMinVel,dRVel);
}

void KuVFHPlusPr::setSensorParameter(double dXoffset,double dYoffset,double dThetaoffset,double dMaxRange,double dMinRange)
{
	m_certainty_grid.setSensorParameter(dXoffset,dYoffset,dThetaoffset,dMaxRange,dMinRange);
}

void KuVFHPlusPr::setRobotRadius(double robot_radius)
{
	_robot_radius=robot_radius;
	m_VFHPlus.setRobotRadius(robot_radius);
}

KuVFHPVelocity KuVFHPlusPr::generateTRVelocity(KuPose TargetPos, KuPose RobotPos, int_1DArray  nSensorData )
{
	KuVFHPVelocity VFHPVelocity;
	double dsensor_value[Sensor::RANGEINDEX]={0};
	double dtarget_direction=0 ;
	double dsteer_magnitude=0 ;

	for(int i=0;i<Sensor::RANGEINDEX;i++){	dsensor_value[i]=nSensorData[i]/1000.0;}
	
	double drobot_x=RobotPos.getXm();
	double drobot_y=RobotPos.getYm();
	double drobot_theta=RobotPos.getThetaRad()+M_PI;

	m_certainty_grid.MoveRobotPos (drobot_x, drobot_y, drobot_theta);
	m_certainty_grid.UpdateSensorValue (dsensor_value, Sensor::RANGEINDEX);
	m_VFHPlus.setCertaintyGrid(m_certainty_grid);

	m_VFHPlus.setTagetPos(  TargetPos.getXm(),TargetPos.getYm(),0 );
	m_VFHPlus.setRobotPos(  RobotPos.getXm(),RobotPos.getYm(),drobot_theta );

	m_VFHPlus.primary_polar_histogram();
	m_VFHPlus.binary_polar_histogram();
	m_VFHPlus.masked_polar_histogram();
	m_VFHPlus.selection_steering_angle();
	m_VFHPlus.speed_control();	

	m_VFHPlus.getspeed_control(&dsteer_magnitude,&_steer_direction);
	double v = dsteer_magnitude;
	double w = DeltaRad (_steer_direction, drobot_theta);

// 	double theta_r = v/_wheel_radius + w*_axle_length/(2*_wheel_radius);
// 	double theta_l = v/_wheel_radius - w*_axle_length/(2*_wheel_radius);
// 	VFHPVelocity.m_nTranslationVel=v*1000;
// 	VFHPVelocity.m_nRotationDegVel=w*R2D;
	VFHPVelocity.dTranslationVel=v;
	VFHPVelocity.dRotationDegVel=w;	

	return VFHPVelocity;
}

