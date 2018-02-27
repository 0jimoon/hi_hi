#include "StdAfx.h"
#include "mm.h"
#include "LocalMap.h"

#include "VFHPlus.h"


CLocalMap ::CLocalMap( )
 {
	Clear ();

	m_drobot_theta=0.0;
	m_nprev_pos_x=0.0;
	m_nprev_pos_y=0.0;
	// 로봇 좌표계를 기준으로 센서의 설치 위치
	m_dsensor_x = 0.25;
	m_dsensor_y = 0.0;
	m_dsensor_theta = D2R*0.;
	// 센서의 최소, 최대 감지 거리
	m_dsensor_min_range = 0.03;
	m_dsensor_max_range = 30.;
	// 센서의 스캔 시작 각도와 끝 각도 및 해상도
	m_dsensor_scan_range = D2R*181;
	m_nsensor_scan_count=181;

}
CLocalMap::~CLocalMap ()
{
}
void CLocalMap::setSensorParameter (double dsensor_x,double dsensor_y,double dsensor_theta
	,double dsensor_max_range,double dsensor_min_range)
{
	m_dsensor_x = dsensor_x;
	m_dsensor_y = dsensor_y;
	m_dsensor_theta = dsensor_theta;

	m_dsensor_min_range = dsensor_min_range;
	m_dsensor_max_range = dsensor_max_range;

}
void CLocalMap::DrawLine (int x1, int y1, int x2, int y2, long value)
{
	int deltax = abs(x2 - x1);        // The difference between the x's
	int deltay = abs(y2 - y1);        // The difference between the y's
	int x = x1;                       // Start x off at the first pixel
	int y = y1;                       // Start y off at the first pixel
	int xinc1, xinc2;
	int yinc1, yinc2;
	int den, num, numadd;
	int numpixels, curpixel;
	
	if (x2 >= x1)	xinc1 = 1,	xinc2 = 1;
	else			xinc1 = -1, xinc2 = -1;
	if (y2 >= y1) 	yinc1 = 1,	yinc2 = 1;
	else			yinc1 = -1,	yinc2 = -1;
	
	if (deltax >= deltay) {        // There is at least one x-value for every y-value
		xinc1 = 0;                  // Don't change the x when numerator >= denominator
		yinc2 = 0;                  // Don't change the y for every iteration
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;         // There are more x-values than y-values
	}
	else {                         // There is at least one y-value for every x-value
		xinc2 = 0;                  // Don't change the x for every iteration
		yinc1 = 0;                  // Don't change the y when numerator >= denominator
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;         // There are more y-values than x-values
	}
	// value = ~value;
	
	for (curpixel = 0; curpixel <= numpixels; ++curpixel) {
		if (IsIn (x, y))	m_cells[y][x] = value;
		else				break;

		num += numadd;              // Increase the numerator by the top of the fraction
		if (num >= den) {            // Check if numerator >= denominator
			num -= den;               // Calculate the new numerator value
			x += xinc1;               // Change the x as appropriate
			y += yinc1;               // Change the y as appropriate
		}
		x += xinc2;                 // Change the x as appropriate
		y += yinc2;                 // Change the y as appropriate
	}
}

void CLocalMap::Shift (int dx, int dy)
{
	int i;

	if (dx == 0 && dy == 0) { }
	else if (0 <= dy && dy < WINDOW_SIZE) {
		if (0 <= dx && dx < WINDOW_SIZE) {
			for (i=WINDOW_SIZE-1; dy<=i; i--) {
				memmove (&m_cells[i][dx], &m_cells[i-dy][0], (WINDOW_SIZE-dx)*sizeof(long));
				memset (&m_cells[i][0], 0, dx*sizeof(long));
			}
			for (; 0<=i; i--) {
				memset (&m_cells[i][0], 0, WINDOW_SIZE*sizeof(long));
			}
		}
		else if (-WINDOW_SIZE < dx && dx < 0) {
			dx = -dx;
			for (i=WINDOW_SIZE-1; dy<=i; i--) {
				memmove (&m_cells[i][0], &m_cells[i-dy][dx], (WINDOW_SIZE-dx)*sizeof(long));
				memset (&m_cells[i][WINDOW_SIZE-dx], 0, dx*sizeof(long));
			}
			for (; 0<=i; i--) {
				memset (&m_cells[i][0], 0, WINDOW_SIZE*sizeof(long));
			}
		}
	}
	else if (-WINDOW_SIZE < dy && dy < 0) {
		dy = -dy;
		if (0 <= dx && dx < WINDOW_SIZE) {
			for (i=0; i<WINDOW_SIZE-dy; ++i) {
				memmove (&m_cells[i][dx], &m_cells[i+dy][0], (WINDOW_SIZE-dx)*sizeof(long));
				memset (&m_cells[i][0], 0, dx*sizeof(long));
			}
			for (; i<WINDOW_SIZE; ++i) {
				memset (&m_cells[i][0], 0, WINDOW_SIZE*sizeof(long));
			}
		}
		else if (-WINDOW_SIZE < dx && dx < 0) {
			dx = -dx;
			for (i=0; i<WINDOW_SIZE-dy; ++i) {
				memmove (&m_cells[i][0], &m_cells[i+dy][dx], (WINDOW_SIZE-dx)*sizeof(long));
				memset (&m_cells[i][WINDOW_SIZE-dx], 0, dx*sizeof(long));
			}
			for (; i<WINDOW_SIZE; ++i) {
				memset (&m_cells[i][0], 0, WINDOW_SIZE*sizeof(long));
			}
		}
	}
}

void CLocalMap::Clear ()
{
	memset (m_cells, 0, sizeof (m_cells));
}

void CLocalMap::MoveRobotPos (double pos_x, double pos_y, double pos_theta)
{
	m_drobot_theta = pos_theta;

	int ix = INT_(CU(pos_x));
	int iy = INT_(CU(pos_y));
	
	int dx = ix - m_nprev_pos_x;
	int dy = iy - m_nprev_pos_y;

	m_nprev_pos_x = ix;
	m_nprev_pos_y = iy;

	Shift (-dx, -dy);
}

void CLocalMap::UpdateSensorValue (double val[], int no)
{
	m_nsensor_scan_count=no;
	double resolution = m_dsensor_scan_range/(m_nsensor_scan_count - 1);

	for (int i=0; i<m_nsensor_scan_count; ++i) {
		double v = val[i];

		if (m_dsensor_min_range < v) {
			// 임시로, 90%만 선을 그리도록 한다...
			v *= 0.9;	

			double obstacle_x = m_dsensor_x*cos(m_drobot_theta) - m_dsensor_y*sin(m_drobot_theta);
			double obstacle_y = m_dsensor_x*sin(m_drobot_theta) + m_dsensor_y*cos(m_drobot_theta);
			//double obstacle_theta = m_drobot_theta + m_dsensor_theta -m_dsensor_scan_range/2. + i*resolution;
			double obstacle_theta = m_drobot_theta + m_dsensor_theta+i*D2R-M_PI_2;

			obstacle_x += v*cos(obstacle_theta);
			obstacle_y += v*sin(obstacle_theta);
			
			int sx = INT_(M2CUx (m_dsensor_x));
			int sy = INT_(M2CUy (m_dsensor_y));
			int ex = INT_(M2CUx (obstacle_x));
			int ey = INT_(M2CUy (obstacle_y));

			DrawLine (sx, sy, ex, ey, 0);
		}
	}
	for (int i=0; i<m_nsensor_scan_count; ++i) {
		double v = val[i];

		if (m_dsensor_min_range < v) {
			double obstacle_x = m_dsensor_x*cos(m_drobot_theta) - m_dsensor_y*sin(m_drobot_theta);
			double obstacle_y = m_dsensor_x*sin(m_drobot_theta) + m_dsensor_y*cos(m_drobot_theta);
			//double obstacle_theta = m_drobot_theta + m_dsensor_theta -m_dsensor_scan_range/2. + i*resolution;
			double obstacle_theta = m_drobot_theta + m_dsensor_theta+i*D2R-M_PI_2;

			obstacle_x += v*cos(obstacle_theta);
			obstacle_y += v*sin(obstacle_theta);
			
			int ex = INT_(M2CUx (obstacle_x));
			int ey = INT_(M2CUy (obstacle_y));

			if (v < m_dsensor_max_range) {
				SetPixel (ex, ey, 1);
// 				for(int m=-1;m<2;m++)
// 				{
// 					for(int n=-1;n<2;n++)
// 					{
// 						if(WINDOW_SIZE<ex+n||0>ex+n||WINDOW_SIZE<ey+m||0>ey+m) continue;
// 						SetPixel (ex+n, ey+m, 1);
// 					}
// 				}
				
			}
		}
	}
}

