#ifndef KUNS_DWA_ROS_H
#define KUNS_DWA_ROS_H

#include <list>
#include <iostream>
#include <cmath>
#include <iostream>
#include <assert.h>
#include <ctime>
#include <cstdlib>
#include <windows.h>
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"

using namespace std;

class KuDWAVelocity_ROS
{
public:
	int m_nTranslationVel;
	int m_nXVel;
	int m_nYVel;
	int m_nRotationDegVel;
};
//자료형 클래스

class KuDWAPr_ROS
{

private:
	static const int INFINITY_VALUE = 100000;
	static const int GOAL_AREA = 500; //800mm
	static const int INDEX =181;

private:	
	double m_dtime_c;	
	double m_dlaser_max;
	double m_dlaser_min;
	double m_dlaser_xoffset;
	double m_dvmax;	
	double m_dwmax;
	double m_dwmin;	
	double m_dv_res;	
	double m_dw_res;	
	double m_dvpmax;	
	double m_dwpmax;	
	unsigned int m_nIdX;		
	double m_dalpha, m_dbeta, m_gamma, m_ddelta;	
	double m_dradius;	
	double m_dvpbrake;	
	double m_dwpbrake;	
	double m_dgoal_x;		
	double m_dgoal_y;
	double	m_dTVel;
	double m_dRVel;

private:
	double  m_dEmergencyDistance;
	double * m_dCosData;
	double 	* m_dSinData;
	//_______________________________________________________________________


	double m_dObstacleX[INDEX];
	double m_dObstacleY[INDEX];
	bool m_bFirstMove;

	int m_GoalArea;

	double m_dDelTheta;
	double m_dAngularVel;

	KuPose m_GoalPos;
	double m_dXoffset;

	int m_nDistTime;


public:
	void InitVar();	
	void InitTheta();
	KuDWAVelocity_ROS generateTRVelocity(KuPose TargetPos, KuPose RobotPos, int_1DArray nSensorData );
	void setGoalPos(KuPose GoalPos);
	void setWeightParameter(double dAlphaH,double dAlphaC,double dAlphaV    );
	bool  generateObstacleModel(int_1DArray nSensorData);

public:		
	void setLaser_count ( unsigned int val );	
	void setGamma ( double val );		
	void setBeta ( double val );		
	void setAlpha ( double val );	
	void setWpmax ( double val );	
	void setWmin ( double val );
	void setWmax ( double val );
	void setW_res ( double val );	
	void setVpmax ( double val );		
	void setVmax ( double val );		
	void setV_res ( double val );	
	void setTime_c ( double val );	
	void setRadius ( double val );		
	void setLaser_max ( double val );	
	void setLaser_min ( double val );
	void setLaser_xoffset ( double val );
	static double rad2deg(double val);	
	static double deg2rad(double val);	
	void setVpbrake ( double val );	
	void setWpbrake ( double val );		
	static void fix_angle(double& angle);    	
	static double randab(double a, double b);		
	void setGoal_x ( double val );		
	void setGoal_y ( double val );	
	void setDelta ( double val );

private:	

	int circle_circle_intersection(double x0, double y0, double r0,double x1, double y1, double r1,double& xi, double& yi,double& xi_prime, double& yi_prime) ;		
	int circle_xaxis_intersection(double x,double y,double r,double& xi,double& xi_prime,double& yi,double& yi_prime) ;	
	double target_heading_score(double v, double w, double theta_goal) ;		
	double speed_score(double v, double w) ;	
	void compute_window(double v_curr, double w_curr,double& v_min,double& v_max,double& w_min,double& w_max) ;  
	double curvature_dist(double v,double w,double x_obs,double y_obs,double r_obs) ;   
	double circle_circle_dist(double x0,double y0,double r0,double x1,double y1,double r1) ;
	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);
	double isAngleDifference(double dWayPointX, double dWayPointY, double dRobotX, double dRobotY, double dRobotThetaRad);

public:		
	KuDWAPr_ROS();
	~KuDWAPr_ROS();


};

#endif 