
// VFF_clientDlg.h : header file
//

#pragma once
#include "LocalMap.h"

class VFHPlus
{

public:
	// 전역 좌표계에서 타겟의 위치와 방향
	double m_dtarget_x;
	double m_dtarget_y;
	double m_dtarget_theta;

	// 전역 좌표계에서 로봇의 위치와 방향
	double m_drobot_x;
	double m_drobot_y;
	double m_drobot_theta;
	
   double m_drobot_radius;


	 double m_dtarget_direction;

	 double m_dsteer_magnitude;
	 double m_dsteer_direction;

	 double m_primary_polar_histogram[360];
	 double m_binary_polar_histogram[360];
	 double m_masked_polar_histogram[360];

	 double m_dVmax;
	 double m_dVmin;
	 double m_dWmax;
	 double m_dSmax;

	 CLocalMap m_certainty_grid;

public:
	 inline int  H_ID(double theta);
	 void speed_control();
	 void selection_steering_angle();
	 void masked_polar_histogram();
	 void binary_polar_histogram();
	 void primary_polar_histogram();
	 void find_open_space (int &k_l, int &k_r);
	 double cost_function (double c);
	 void setCertaintyGrid(CLocalMap _certainty_grid);
	 void setTagetPos(double  dtarget_x,double  dtarget_y,double  dtarget_theta  );
	 void setRobotPos(double  drobot_x,double  drobot_y,double  dtarget_theta  );
	 void getspeed_control(double *steer_magnitude,double *steer_direction);
	 void setVelocity(double dVmax,double dVmin,double dWmax);
	 void setRobotRadius(double drobot_radius);

public:
	VFHPlus(void);
	~VFHPlus(void);
};

