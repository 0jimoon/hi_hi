#ifndef KUNS_DWA_H
#define KUNS_DWA_H

#include <list>
#include <iostream>
#include <cmath>
#include <iostream>
#include <assert.h>
//#include <sys/time.h>
#include <ctime>
#include <cstdlib>
#include <windows.h>
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KuUtil.h"

using namespace std;

class KuDWAVelocity
{
public:
	int m_nTranslationVel;
	int m_nXVel;
	int m_nYVel;
	int m_nRotationDegVel;
};

class DynamicWindow
{
private:
	int m_nLeftWheelVel;
	int m_nRightWheelVel;
	double m_dW; //Overall objective
	double m_dHeading;
	double m_dVel;
	double m_dClearance;

public:	
	void SetLeftWheelVel(int nLVel){ m_nLeftWheelVel = nLVel;}
	void SetRightWheelVel(int nRVel){ m_nRightWheelVel = nRVel;}
	void SetW(double dW){m_dW = dW;}
	void SetHeading(double dHeading){m_dHeading = dHeading;}
	void SetVel(double dVel){ m_dVel = dVel;}
	void SetClearance(double dClearance){ m_dClearance = dClearance;}

	int GetLeftWheelVel(){ return m_nLeftWheelVel;}
	int GetRightWheelVel(){ return m_nRightWheelVel;}
	double GetW(){ return m_dW;}
	double GetHeading(){ return m_dHeading;}
	double GetVel(){ return m_dVel;}
	double GetClearance(){ return m_dClearance;}

}; //자료형 클래스

class KuDWAPr
{

private:
	static const int INFINITY_VALUE = 100000;
	static const int GOAL_AREA = 500; //800mm
	static const int DWA_SIZE = 33; //-16~16 다시말해 -VELOCITYRANGE~ VELOCITYRANGE 의미
	static const int DWA_VELLIMIT = 100; // unit cm/sec
	static const int DWA_RIGHT_ACCELATION	= 40; // unit cm/sec^2
	static const int DWA_LEFT_ACCELATION	= 40; // unit cm/sec^2

	static const int DWA_R_ACCELATION	= 60; // unit cm/sec^2
	static const int DWA_T_ACCELATION	= 50; // unit cm/sec^2
	static const int INDEX =181;
	DynamicWindow m_DWAData[DWA_SIZE][DWA_SIZE]; //자료형 클래스 객체

	double m_dX, m_dY;
	int m_nMinVelocity;
	double	m_dTVel;
	double m_dRVel;
private:
	bool isAngleDifferenceLarge(double dWayPointX, double dWayPointY, double dRobotX, double dRobotY, double dRobotThetaRad);
	bool generateObstacleModel(int_1DArray nSensorData);
	void generateDynamicWinodw(int nVelLeftWheel,int nVelRightWheel);
	double calcMinCollisionTime(double dTimeToCollisionSet[INDEX]);

	double calcPredictedRadBetweenRobotandGoal(int nRightWheelVel, int nLeftWheelVel,double dGoalPosX, double dGoalPosY,double,double,double);
	void calcSpeedObject(bool bSpeedCostflag);
	void calcSpeedObjectTurn();
	void calcHeadingObject(double, double,double,double,double);
	void calcClearanceObject();
	void calcObjectiveFunc(int *nLeftWheelVel,int *nRightWheelVel);

	void moveByWheelVel(int nLeftWheelVel, int nRightWheelVel);
	bool checkGoalPos(double cur_posX,double cur_posY);
	void InitVar();	
	void initDWA();

private:
	int m_nLaserData[INDEX];
	double m_dDistBetweenWheel; //로봇 양바퀴사이의 거리 unit--> mm
	double m_dRadiusofRobot; //로봇 몸체 반경 
	int m_nSafeDistance;
	double  m_dEmergencyDistance;
	double * m_dCosData;
	double 	* m_dSinData;
	int m_nVelocityRange;
	int m_nMaxDiffRotate;
	//_______________________________________________________________________


	int m_nLeftWheelVel,m_nRightWheelVel;	
	double m_dObstacleX[INDEX];
	double m_dObstacleY[INDEX];
	bool m_bFirstMove;
	bool m_bEmergancy;
	int m_GoalArea;
	double m_dAlphaH;
	double m_dAlphaC;
	double m_dAlphaV;
	double m_dTurnDegree;
	double m_dDelTheta;
	double m_dAngularVel;
	float m_fRVelLimit;
	KuPose m_GoalPos;
	double m_dXoffset;
	double m_dMaxDistance;
	double m_dMinDistance ;
	int m_nIdX;
	int m_nDistTime;

public:
	void Init();
	void InitTheta();
	KuDWAVelocity generateTRVelocity(KuPose TargetPos, KuPose RobotPos, int_1DArray nSensorData );
	void setGoalPos(KuPose GoalPos);
	void setWeightParameter(double dAlphaH,double dAlphaC,double dAlphaV    );
	void setRobotParameter(double  dRadiusofRobot,double  dDistBetweenWheel);
	void setSensorParameter(int nIdX, double dXoffset,double dMaxDistance,double dMinDistance );
	void setHeadingMode();


	KuDWAPr();
	~KuDWAPr();
	public:		
	//	void motion_model_sampling_time(double maxtime,double currv, double currw,double theta_goal,float* ranges,float* bearings, double& v_found, double& w_found) 	;	
	  KuDWAVelocity motion_model_sampling_time(KuPose TargetPos, KuPose RobotPos, int_1DArray  nSensorData );

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
		static double rad2deg(double val);	
		static double deg2rad(double val);	
		void setVpbrake ( double val );	
		void setWpbrake ( double val );		
		static void fix_angle(double& angle);    	
		static double randab(double a, double b);		
		void setGoal_x ( double val );		
		void setGoal_y ( double val );	
		void setDelta ( double val );
protected:	

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

	protected:		double m_time_c;	
					double m_laser_max;
					double m_vmax;	
					double m_wmax;
					double m_wmin;	
					double m_v_res;	
					double m_w_res;	
					double m_vpmax;	
					double m_wpmax;	
					unsigned int m_laser_count;		
					double m_alpha, m_beta, m_gamma, m_delta;	
					double m_radius;	
					double m_vpbrake;	
					double m_wpbrake;	
					double m_goal_x;		
					double m_goal_y;

};

#endif 