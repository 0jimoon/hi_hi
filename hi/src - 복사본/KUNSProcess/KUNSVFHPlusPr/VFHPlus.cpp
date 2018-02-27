
#include "stdafx.h"
#include <math.h>
#include "mm.h"
#include "VFHPlus.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


VFHPlus::VFHPlus(void)
{

	// 전역 좌표계에서 타겟의 위치와 방향
	m_dtarget_x = 0.0;
	m_dtarget_y = 0.0;
	m_dtarget_theta = _DEG2RAD*0;

	// 전역 좌표계에서 로봇의 위치와 방향
	m_drobot_x = 0.0;
	m_drobot_y = 0.0;
	m_drobot_theta = _DEG2RAD*0;

	m_drobot_radius = 0.5;

	m_dtarget_direction = 0;
	m_dsteer_magnitude = 0;
	m_dsteer_direction = 0;

	m_dVmax=0.5;
	m_dVmin=0.05;
	m_dWmax=90*D2R;
	m_dSmax=0;

}


VFHPlus::~VFHPlus(void)
{
}

inline int VFHPlus:: H_ID(double theta)
{
	double alpha = 1.;
	return (int)(theta + 10*360. + alpha/2.)%360;
}

void VFHPlus::setTagetPos(double  dtarget_x,double  dtarget_y,double  dtarget_theta  )
{
	m_dtarget_x = dtarget_x;
	m_dtarget_y = dtarget_y;
	m_dtarget_theta = dtarget_theta;
}
void VFHPlus::setRobotPos(double  drobot_x,double  drobot_y,double  dtarget_theta  )
{
	m_drobot_x = drobot_x;
	m_drobot_y = drobot_y;
	m_drobot_theta = dtarget_theta;
}
void VFHPlus::setRobotRadius(double drobot_radius)
{
	m_drobot_radius = drobot_radius;
}
void VFHPlus::setCertaintyGrid(CLocalMap certainty_grid)
{
	m_certainty_grid=certainty_grid;
}
void VFHPlus::primary_polar_histogram()
{
	double L_max = CELL_SIZE*(WINDOW_SIZE-1)/2;

	memset(m_primary_polar_histogram, 0, 360*sizeof(double));

	for(int yi=0; yi<WINDOW_SIZE; yi++)
	{
		for(int xi=0; xi<WINDOW_SIZE; xi++)
		{
			double c_ij = m_certainty_grid.m_cells[yi][xi];

			if(0. < c_ij)
			{
				double dy_j = m_certainty_grid.CU2My(yi);
				double dx_i = m_certainty_grid.CU2Mx(xi);

				double d_ij = sqrt(dx_i*dx_i + dy_j*dy_j);
				double beta_ij = atan2(dy_j, dx_i);

				if (m_drobot_radius < d_ij) 
				{
					double a = 5.;
					double b = (a - 1.)/L_max*L_max;
					/*
					double m_ij = c_ij*c_ij*(a - b*d_ij*d_ij);
					double gamma_ij = asin(_robot_radius/d_ij);

					for (double i = (beta_ij-gamma_ij)*_RAD2DEG; i<(beta_ij+gamma_ij)*_RAD2DEG; i+=1.)
					{
					int index = H_ID(i);
					_primary_polar_histogram[index] = max(_primary_polar_histogram[index], m_ij);
					}
					*/
					double gamma_ij = asin(m_drobot_radius/d_ij);

					for (double i = beta_ij-gamma_ij; i < beta_ij+gamma_ij; i+=1.*_DEG2RAD)
					{
						double gamma = fabs(i - beta_ij);
						double r = d_ij*sin(gamma);

						if (r <= m_drobot_radius) {
							double d = d_ij*cos(gamma) - sqrt(m_drobot_radius*m_drobot_radius - r*r);
							double m = c_ij*c_ij*(a - b*d*d);

							int index = H_ID(i*_RAD2DEG);
							m_primary_polar_histogram[index] = max(m_primary_polar_histogram[index], m);
						}
					}
				}
			}
		}
	}
}

void VFHPlus::binary_polar_histogram()
{
	memset(m_binary_polar_histogram, 0, 360*sizeof(double));

	double Thigh = 3.5;
	double Tlow	= 3.5;

	for(int i = 0; i<360; i++)
	{
		if(m_primary_polar_histogram[i] < Tlow)
			m_binary_polar_histogram[i] = 0;
		else if(m_primary_polar_histogram[i] > Thigh)
			m_binary_polar_histogram[i] = 1;
	}

	for(int i = 0; i<360; i++)
	{
		if(m_primary_polar_histogram[i]<Thigh && m_primary_polar_histogram[i] > Tlow)
		{
			int index = H_ID(i-1);
			m_binary_polar_histogram[i] = m_binary_polar_histogram[index];
		}
	}
}

void VFHPlus::masked_polar_histogram()
{
	double r_l = 0.15;
	double r_r = 0.15;

	double dxl = r_l*cos(m_drobot_theta + M_PI_2);
	double dyl = r_l*sin(m_drobot_theta + M_PI_2);
	double dxr = r_r*cos(m_drobot_theta - M_PI_2);
	double dyr = r_r*sin(m_drobot_theta - M_PI_2);

	double phi_b = DeltaRad(m_drobot_theta+M_PI, 0.);
	double phi_l = phi_b;
	double phi_r = phi_b;

	for(int yi=0; yi<WINDOW_SIZE; yi++)
	{
		for(int xi=0; xi<WINDOW_SIZE; xi++)
		{
			if(m_certainty_grid.m_cells[yi][xi])
			{
				double dy_j = m_certainty_grid.CU2My(yi);
				double dx_i = m_certainty_grid.CU2Mx(xi);

				double beta_ij = atan2(dy_j, dx_i);
				double d_ij = sqrt(dx_i*dx_i + dy_j*dy_j);

				if (0. < DeltaRad(beta_ij, m_drobot_theta)) 
				{
					double d2_l = sqrt((dxl-dx_i)*(dxl-dx_i)+(dyl-dy_j)*(dyl-dy_j));

					if ((d2_l < r_l + m_drobot_radius) && (0. < DeltaRad(phi_l, beta_ij)))
					{
						phi_l = beta_ij;
					}
				}
				else
				{
					double d2_r = sqrt((dxr-dx_i)*(dxr-dx_i)+(dyr-dy_j)*(dyr-dy_j));

					if ((d2_r < r_r + m_drobot_radius) && (0. < DeltaRad(beta_ij, phi_r))) 
					{
						phi_r = beta_ij;
					}
				}
			}
		}
	}

	memcpy(m_masked_polar_histogram, m_binary_polar_histogram, 360*sizeof(double));

	phi_l = phi_l;
	phi_r = phi_l + DeltaRad(phi_r, phi_l);

	for(double i = phi_l; i<phi_r; i+=1.*_DEG2RAD)
	{
		int index = H_ID(i*_RAD2DEG);

		m_masked_polar_histogram[index] = 1.;
	}
}

void VFHPlus::find_open_space (int &k_l, int &k_r)
{
	for (int i = k_l; i<k_r; i++)
	{
		int index = H_ID(i);

		if(m_masked_polar_histogram[index])
		{
			k_r = i - 1;
			break;
		}
	}
}

double VFHPlus::cost_function (double c)
{
	double u1 = 3;
	double u2 = 1.2;
	double u3 = 1.;

	double g = 
		u1*fabs(DeltaRad(c, m_dtarget_direction)) + 
		u2*fabs(DeltaRad(c, m_drobot_theta)) + 
		u3*fabs(DeltaRad(c, m_dsteer_direction));

	return g;
}

void VFHPlus::selection_steering_angle()
{
	m_dtarget_direction = atan2(m_drobot_y-m_dtarget_y  , m_drobot_x-m_dtarget_x );

	double g_min = 1000.;
	int c_sel = -1;

	for (int k_l = 0; k_l<360; k_l++)
	{		
		int k_r = k_l + 360;

		find_open_space (k_l, k_r);

		int c_l, c_r;

		if (k_l + m_dSmax < k_r)
		{ 
			c_l = k_l + m_dSmax/2;
			c_r = k_r - m_dSmax/2;
			k_l = k_r;
		}
		else if (k_l <= k_r)
		{
			c_l = c_r = (k_r + k_l)/2;
			k_l = k_r;
		}
		else 
		{
			c_l = 0; c_r = -1;
		}

		for (int c_n = c_l; c_n <= c_r; c_n+=1) 
		{
			double g = cost_function (c_n*_DEG2RAD);
			if(g < g_min)
			{
				g_min = g;
				c_sel = c_n;
			}
		}
	}

	if (c_sel != -1) {
		m_dsteer_magnitude = m_dVmax*(1 - m_masked_polar_histogram[c_sel]/1.);
		m_dsteer_direction = c_sel*_DEG2RAD;		
	}
	else {
		m_dsteer_magnitude = 0.;
	}
}

void VFHPlus::speed_control()
{
	if (m_dsteer_magnitude < m_dVmin) m_dsteer_magnitude = m_dVmin;

	double Omega = DeltaRad (m_dsteer_direction, m_drobot_theta);

	m_dsteer_magnitude = m_dsteer_magnitude*(1. - fabs(Omega)/m_dWmax);
	if (m_dsteer_magnitude < m_dVmin) m_dsteer_magnitude = m_dVmin;
}

void VFHPlus::getspeed_control(double *steer_magnitude,double *steer_direction)
{

	(*steer_magnitude)= m_dsteer_magnitude;
	(*steer_direction)= m_dsteer_direction;
}

void VFHPlus::setVelocity(double dVmax,double dVmin,double dWmax)
{
	m_dVmax= dVmax/1000.0;
	m_dVmin= dVmin/1000.0;
	m_dWmax= dWmax*D2R;

}
