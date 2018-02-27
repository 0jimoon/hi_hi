
#include "KuDWAPr.h"


KuDWAPr::KuDWAPr()
{
	srand(time(NULL));
	InitVar();	//1 순서가 중요함. InitVar()
	InitTheta( );
	double alpha = 0.3;
	setAlpha(alpha);	
	double beta = 0.4;
	setBeta(beta);
	double gamma = 0.3;
	setGamma(gamma);	
	double wpmax = deg2rad(360);
	setWpmax(wpmax);
	double wmin = deg2rad(-90);
	setWmax(wmin);
	double wmax = deg2rad(90);
	setWmax(wmax);
	double wpbrake  = deg2rad(100);
	setWpbrake(wpbrake );
	double vmax = 1.0;    
	setVmax(vmax);
	double vpmax = 1.0;
	setVpmax(vpmax);
	double vpbrake = 0.3;
	setVpbrake(vpbrake);
	double tcmax = 0.1;
	setTime_c(tcmax);
	double laser_max = 20.0;
	setLaser_max(laser_max);
	double robot_radius = 0.42;
	setRadius(robot_radius);

}

KuDWAPr::~KuDWAPr()
{
	if(m_dCosData!=NULL)
		delete [] m_dCosData;
	if(m_dSinData!=NULL)
		delete [] m_dSinData;

}
/**
@brief Korean: 모든 전역 변수에 대한 초기화를 하는 함수 
@brief English: 
*/
void KuDWAPr::InitVar()
{
	m_dAlphaH =8; //heading //3
	m_dAlphaC = 5; //clearance //5 
	m_dAlphaV = 2; //velocity //2
	memset(m_DWAData,0,sizeof(m_DWAData));

	m_GoalArea=2000;
	m_dTurnDegree = 10; // 각도 오차

	m_dDelTheta = 0.;
	m_nLeftWheelVel = m_nRightWheelVel =0;
	m_fRVelLimit = M_PI; 

	m_dDistBetweenWheel = 500;
	m_dRadiusofRobot = 420/10; //--> mm to cm //로봇 반경
	m_bFirstMove = true;
	m_dCosData=NULL;
	m_dSinData=NULL;
	m_nMaxDiffRotate=120;
	m_nVelocityRange=(int)DWA_SIZE/2.0;
	m_nMinVelocity=10;

	m_dXoffset=23;//cm
	m_dMaxDistance=2000;//cm
	m_dMinDistance=5;//cm
	m_dEmergencyDistance=35;//cm
	m_nIdX=181;
	m_dTVel=0;
	m_dRVel=0;
	m_laser_count=181;
}

/**
@brief Korean: 각도 값을 look-up tablbe로 작성하는 함수
@brief English: 
*/
void KuDWAPr::InitTheta( )
{
	if(m_dCosData==NULL)
		m_dCosData = new double[m_nIdX];
	if(m_dSinData==NULL)
		m_dSinData = new double[m_nIdX];

	for(int i = 0; i <m_nIdX; i++){
		double	dAngleRad = (i-(int)(m_nIdX/2.0))*D2R;
		m_dCosData[i]=cos(dAngleRad);
		m_dSinData[i]=sin(dAngleRad);

	}
}
/**
@brief Korean: 재 시작하는 경우 초기화 해주는 부분
@brief English: 
*/
void KuDWAPr::Init()
{
	memset(m_DWAData,0,sizeof(m_DWAData));

	m_dDelTheta = 0.;
	m_nLeftWheelVel = m_nRightWheelVel =0;
	m_fRVelLimit = M_PI; 
	m_bFirstMove = true;
}

/**
@brief Korean: 로봇의 Parameter를 설정하는 함수
@brief English: 
*/
void KuDWAPr::setRobotParameter(double  dRadiusofRobot,double  dDistBetweenWheel)
{
	m_dRadiusofRobot=dRadiusofRobot;
	m_dDistBetweenWheel=dDistBetweenWheel;
}
/**
@brief Korean: DWA의 Weight를 설정하는 함수
@brief English: 
*/
void KuDWAPr::setWeightParameter(double dAlphaH,double dAlphaC,double dAlphaV )
{
	m_dAlphaH = dAlphaH;
	m_dAlphaC =dAlphaC;
	m_dAlphaV = dAlphaV;
}
/**
@brief Korean: 센서 파라미터를 받아오는 함수
@brief English: 
*/
void KuDWAPr::setSensorParameter(int nIdX, double dXoffset,double dMaxDistance,double dMinDistance )
{
	if(INDEX!=nIdX) printf("You must change index parameter");
	m_nIdX=nIdX;
	m_dXoffset= dXoffset/10.;
	m_dMaxDistance=dMaxDistance/10.;
	m_dMinDistance = dMinDistance/10.;
}

/**
@brief Korean: 목적지를 설정하는 함수
@brief English: 
*/
void KuDWAPr::setGoalPos(KuPose GoalPos)
{
	m_GoalPos = GoalPos;
}

/**
@brief Korean: 거리 값을 이용하여 장애물 영역을 만드는 함수
@brief English: 
*/
bool  KuDWAPr::generateObstacleModel(int_1DArray nSensorData)
{
	int nDistance = 0;
	double dEmergencyDist=10;
	memset(m_dObstacleX,0,sizeof(m_dObstacleX));
	memset(m_dObstacleY,0,sizeof(m_dObstacleY));
	m_nDistTime=m_dMaxDistance;
	for(int i = 0; i <m_nIdX; i++){

		nDistance = nSensorData[i]/10; //스캐너에서 받아오는 값의 단위는 mm단위이다. //mm->cm

		if(nDistance<m_nDistTime) m_nDistTime=nDistance;

		m_dObstacleX[i] = (((double)nDistance * m_dCosData[i] )+m_dXoffset)/100;
		m_dObstacleY[i] = (((double)nDistance * m_dSinData[i]))/100;	

		if(i<=45||i>=135)
			dEmergencyDist=10;//10cm
		else
			dEmergencyDist=(double)m_dEmergencyDistance*((int)m_nIdX/2.0-abs(i-(int)m_nIdX/2.0))/((int)m_nIdX/2.0);

		if(nDistance< dEmergencyDist ){			
			return  true;
		}		
		else if(nDistance<=m_dMinDistance){
			m_dObstacleX[i] =10000;
			m_dObstacleY[i] =10000;
			continue;		
		}
	}
	return false;
}	

/*!    \fn KuDWAPr::circle_circle_intersection(double x0, double y0, double r0,double x1, double y1, double r1,double *xi, double *yi,double& xi_prime, double& yi_prime) const */

int KuDWAPr::circle_circle_intersection(double x0, double y0, double r0,double x1, double y1, double r1,double& xi, double&  yi,double& xi_prime, double& yi_prime) 
{	
	double a, dx, dy, d, h, rx, ry;	
	double x2, y2;  	
	/* dx and dy are the vertical and horizontal distances between	
	* the circle centers.	*/
	dx = x1 - x0;	
	dy = y1 - y0;	
	/* Determine the straight-line distance between the centers. */	
	d = hypot(dx,dy); // Suggested by Keith Briggs

	/* Check for solvability. */	
	if (d > (r0 + r1))		{		
		/* no solution. circles do not intersect. */		
		return 0;
	}	

	if (d < fabs(r0 - r1))	{	
		/* no solution. one circle is contained in the other */	
		return 0;
	}  
	/* 'point 2' is the point/ where the line through the circle
	* intersection points /crosses the line between the circle
	* centers.  	
	*/	
	/* Determine the distance from point 0 to point 2. */	

	a = ((r0*r0) - ((r1)*(r1)) + (d*d)) / (2.0 * d) ;	

	/* Determine the coordinates of point 2. */	
	x2 = x0 + (dx * a/d);
	y2 = y0 + (dy * a/d);  
	/* Determine the distance from point 2 to either of the	* intersection points.	*/
	h = sqrt((r0*r0) - (a*a)); 
	/* Now determine the offsets of the intersection points from	* point 2.	*/
	rx = -dy * (h/d);	
	ry = dx * (h/d);
	/* Determine the absolute intersection points. */
	xi = x2 + rx;
	xi_prime = x2 - rx;	
	yi = y2 + ry;
	yi_prime = y2 - ry;
	return 1;
}

/*!    \fn KuDWAPr::circle_xaxis_intersection(double x,double y,double r,double& xi,double& xi_prime,double& yi,double& yi_prime) const */

int KuDWAPr::circle_xaxis_intersection(double x,double y,double r,double& xi,double& xi_prime,double& yi,double& yi_prime) 
{	
	if ( (r*r) < (y*y)) {		
		return 0;
	}	else {	
		xi = x + sqrt(r*r - y*y);		
		xi_prime = x - sqrt(r*r - y*y);	
		yi = 0;		
		yi_prime = 0;	
		return 1;
	}}
/*!    \fn KuDWAPr::circle_circle_dist(double x0,double y0,double r0,double x1,double y1,double r1) const */

double KuDWAPr::circle_circle_dist(double x0,double y0,double r0,double x1,double y1,double r1) 
{	
	double dx = x0 - x1;
	double dy = y0 - y1;	
	double d = hypot(dy,dx);	
	if (d > (r0 + r1))	
		return d - (r0 + r1);	
	else if (d < fabs(r0 - r1))	
		return fabs(r0 - r1) - d;
	else	
		return 0;
}

/*!    \fn KuDWAPr::curvature_dist(v,w,x_obs,y_obs,r_obs) const */

double KuDWAPr::curvature_dist(double v,double w,double x_obs,double y_obs,double r_obs) 
{	
	double xi, xi_prime, yi, yi_prime;	
	double r=100;
	double y_c=0;	
	double x_c = 0;

	if (w!=0) {
		r = fabs(v/w);	
		y_c = v/w;		
		if (!circle_circle_intersection(x_c,y_c,r,x_obs,y_obs,r_obs,xi,yi,xi_prime,yi_prime))	
			return 1;
	}	
	else {	
		if (!circle_xaxis_intersection(x_obs,y_obs,r_obs,xi,yi,xi_prime,yi_prime))		
			return 1;
	}	

	double theta_p1 = atan2(yi-y_c,xi);	

	double theta1;
	if (w >= 0)	
		theta1 = M_PI/2.0 + theta_p1;
	else if (w < 0)	
		theta1 = M_PI/2.0 - theta_p1;

	double dist1 = r*theta1;

	double theta_p2 = atan2(yi_prime-y_c,xi_prime);	
	double theta2;

	if (w >= 0)		
		theta2 = M_PI/2.0 + theta_p2;
	else if (w < 0)	
		theta2 = M_PI/2.0 - theta_p2;	

	double dist2 = r*theta2;

	if (dist1 <= dist2)		
		return dist1/m_laser_max;
	else		
		return dist2/m_laser_max;
}

/*!    \fn KuDWAPr::target_heading_score(double v, double w, double theta_goal) */
double KuDWAPr::target_heading_score(double v, double w, double theta_goal) 
{	
	double delta = 2.0;
	double score = (1.0 - fabs(theta_goal - w*1.0)/M_PI);	
	double val = 1.0 + delta*pow(fabs(theta_goal/M_PI),0.5);
	return val*score;
	// 	return (1 - fabs(theta_goal - w*2.0)/M_PI);
}
/*!    \fn KuDWAPr::speed_score(double v, double w) const */
double KuDWAPr::speed_score(double v, double w) 
{	
	return (v)/(m_vmax);

}
/**
@brief Korean: 로봇의 위치가 목적지 위치의 각도 오차가 큰경우를 판별하는 함수
@brief English: 
*/
double KuDWAPr::isAngleDifference(double dWayPointX, double dWayPointY, double dRobotX, double dRobotY, double dRobotThetaRad)
{
	double dThetaRad=0.;
	dThetaRad = atan2(dWayPointY - dRobotY ,dWayPointX - dRobotX) - dRobotThetaRad;
	if(dThetaRad > M_PI) dThetaRad -= 2*M_PI;
	else if(dThetaRad < -M_PI) dThetaRad += 2*M_PI;
	return dThetaRad;
}

KuDWAVelocity KuDWAPr::motion_model_sampling_time(KuPose TargetPos, KuPose RobotPos, int_1DArray  nSensorData )
{    

	KuDWAVelocity generatedVel;

	double dRobotPosX = RobotPos.getX()/10; //mm->cm
	double dRobotPosY = RobotPos.getY()/10;// mm->cm
	double dRobotPosT = RobotPos.getThetaRad(); //degree 단위를 radian으로 바꿔준다.

	double dTempGoalPosX = TargetPos.getX()/10;
	double dTempGoalPosY = TargetPos.getY()/10;  

	if(generateObstacleModel(nSensorData))
	{
		generatedVel.m_nTranslationVel = 0;
		generatedVel.m_nRotationDegVel =0;
		return generatedVel;
	}

	double maxtime=m_time_c*0.9;
	double currv=m_dTVel;
	double currw=m_dRVel;
	double v_found;
	double w_found;

	double theta_goal= isAngleDifference( dTempGoalPosX,  dTempGoalPosY,  dRobotPosX,  dRobotPosY,  dRobotPosT);

	if (theta_goal >= 10*D2R&& true ==m_bFirstMove ) {	  
		generatedVel.m_nTranslationVel = 0;
		generatedVel.m_nRotationDegVel =theta_goal*R2D;
		m_dTVel=0;
		m_dRVel=theta_goal;
		return generatedVel;
	}
	else
	{
		m_bFirstMove=false;
	}

	double minv=0,minw=0,maxv=0,maxw=0;	

	compute_window(currv,currw,minv,maxv,minw,maxw);

	assert(maxv >= minv);
	assert(maxw >= minw);	

	double f_found = 0;	
	double v = 0;	
	double w = 0;
	unsigned int samples=0;	


	LARGE_INTEGER present;		
	double elapsed =0.0;

	while(elapsed < maxtime) {	
		startTimeCheck(present);
		samples++;	
		v = randab(minv,maxv);	
		w = randab(minw,maxw);	
		double dist_score = 1.0;	
		if (v != 0) {	
			for ( int o=0; o<m_laser_count; o++) {		
				if ( (sqrt(m_dObstacleX[o]*m_dObstacleX[o] + m_dObstacleY[o]*m_dObstacleY[o]) >= m_laser_max) ) 		
					continue;	
				if (m_dObstacleX[o]*m_dObstacleX[o] + m_dObstacleY[o]*m_dObstacleY[o] <= m_radius*m_radius)
				{		
					continue;		
				}			
				double dist = curvature_dist(v,w,m_dObstacleX[o],m_dObstacleY[o],m_radius);		

				if (dist <=0) {			
					dist = 1.0;		
				}			
				if (dist < dist_score) {		
					dist_score = dist;		
				}		
			}	
		}	
		if ( (v>0.5*sqrt(2.0*dist_score*m_laser_max*m_vpbrake)) || (w>0.5*sqrt(2.0*dist_score*m_laser_max*m_wpbrake))) {		
	
			continue;		
		}	
		else {	
			double vel_score = speed_score(v,w);		
			double h_score = target_heading_score(v,w,theta_goal);			
			double tmp_val = m_alpha*h_score + m_beta*dist_score + m_gamma*vel_score;	
			if ( tmp_val > f_found&&dist_score!=0) {		
				f_found = tmp_val;		
				v_found = v;			
				w_found = w;		
				generatedVel.m_nTranslationVel = v_found*1000;
				generatedVel.m_nRotationDegVel =w_found*R2D;
				m_dTVel=v_found;
				m_dRVel=w_found;
			}	
		}		
			elapsed +=(double) finishTimeCheck(present)/1000;
		//	printf("elapsed=%f\n",elapsed);
	}
	// 	std::cout<<"Samples: "<<samples<<"\n";	
	if (f_found == 0) {	
		std::cerr<<"Warning, no value found!\n";	
		generatedVel.m_nTranslationVel = 0;
		generatedVel.m_nRotationDegVel =0;
		m_dTVel=0;
		m_dRVel=0;
	}
	if(fabs(m_dRVel)>45*D2R)m_dRVel=0;
		return generatedVel;
}
void KuDWAPr::compute_window(double v_curr, double w_curr,double& minv,double& maxv,double& minw,double& maxw) 
{
	if (v_curr < 0)	
		v_curr = 0;
	minv = v_curr - m_vpmax*m_time_c;	
	if (minv < 0)		
		minv = 0;	
	maxv = v_curr + m_vpmax*m_time_c;
	if (maxv > m_vmax)	
		maxv = m_vmax;	
	minw = w_curr - m_wpmax*m_time_c;	
	if (minw < m_wmin)	
		minw = m_wmin;	
	maxw = w_curr + m_wpmax*m_time_c;
	if (maxw > m_wmax)		
		maxw = m_wmax;	
	if (maxw < m_wmin)		
		maxw = m_wmin;	
	if (minv > maxv) {	
		minv = 0;	
		maxv = 0;	
		minw = m_wmin;	
		maxw = m_wmax;
	}
	if (minw > maxw) {		
		minv = 0;	
		maxv = 0;
		minw = m_wmin;	
		maxw = m_wmax;
	}	minv = 0;
	// 	std::cout<<"Vmin Vmax Wmin Wmax: "<<minv<<" "<<maxv<<" "<<KuDWAPr::rad2deg(minw)<<" "<<KuDWAPr::rad2deg(maxw)<<"\n";

}

void KuDWAPr::setAlpha ( double val )
{	
	m_alpha = val;
}

void KuDWAPr::setBeta (	double val  )
{	
	m_beta = val;
}

void KuDWAPr::setGamma ( double val )
{	
	m_gamma = val;
}
void KuDWAPr::setLaser_count ( unsigned int val )
{	
	m_laser_count = val;
}
void KuDWAPr::setLaser_max ( double val )
{	
	m_laser_max = val;
}
void KuDWAPr::setRadius ( double val )
{
	m_radius = val;
}
void KuDWAPr::setTime_c ( double val )
{	
	m_time_c = val;
}

void KuDWAPr::setV_res ( double val )
{	
	m_v_res = val;
}

void KuDWAPr::setVmax ( double val )
{	
	m_vmax = val;
}
void KuDWAPr::setVpmax ( double val )
{
	m_vpmax = val;
}
void KuDWAPr::setW_res ( double val )
{	
	m_w_res = val;
}

void KuDWAPr::setWmax ( double val )
{	
	m_wmax = val;
}
void KuDWAPr::setWmin ( double val )
{	
	m_wmin = val;
}
void KuDWAPr::setWpmax ( double val )
{	
	m_wpmax = val;
}
/*!    \fn KuDWAPr::rad2deg(double val) */
double KuDWAPr::rad2deg(double val)
{
	return val*180.0/M_PI;
}
/*!    \fn KuDWAPr::deg2rad(double val) */
double KuDWAPr::deg2rad(double val)
{
	return val*M_PI/180.0;
}
void KuDWAPr::setVpbrake ( double val )
{	
	m_vpbrake = val;
}
void KuDWAPr::setWpbrake ( double val )
{	
	m_wpbrake = val;
}
/*!    \fn KuDWAPr::fix_angle(double) */
void KuDWAPr::fix_angle(double& angle)
{	
	angle = rad2deg(angle);	
	if (angle >= 360)	
		angle = angle - 360.0 * (double)((int)angle / 360);	
	if (angle < -360)	
		angle = angle + 360.0 * (double)((int)angle / -360);	
	if (angle <= -180)		
		angle = + 180.0 + (angle + 180.0);
	if (angle > 180)	
		angle = - 180.0 + (angle - 180.0);
	angle = deg2rad(angle);
}

/*!\fn KuDWAPr::rand(double a, double b) */
double KuDWAPr::randab(double min, double max)
{	
	double r = double(rand())/double(RAND_MAX);
	return (max-min)*r + min;
}
void KuDWAPr::setGoal_x ( double val )
{
	m_goal_x = val;
}

void KuDWAPr::setGoal_y ( double val )
{
	m_goal_y = val;
}

void KuDWAPr::setDelta ( double val )
{	
	m_delta = val;
}
void KuDWAPr::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}

float KuDWAPr::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}