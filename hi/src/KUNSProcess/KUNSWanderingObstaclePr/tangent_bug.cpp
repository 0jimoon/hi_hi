#include "stdafx.h"
#include "tangent_bug.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


tangent_bugAlg::tangent_bugAlg(void)
{
	// 전역 좌표계에서 타겟의 위치와 방향
	 _target_x = 10;
	 _target_y = 7;
	 _target_theta = _DEG2RAD*0;

	// 전역 좌표계에서 로봇의 위치와 방향
	 _robot_x = 1.5;
	 _robot_y = 1.5;
	 _robot_theta = _DEG2RAD*0;

	// 센서의 최소, 최대 감지 거리
	 _sensor_min_range = 0.03;
	 _sensor_max_range = 2.;
	// 센서의 스캔 시작 각도와 끝 각도 및 해상도
	 _sensor_scan_range = _DEG2RAD*360;
	// 센서 값의 개수와 값을 저장하는 배열
	 _sensor_scan_count = 361;
	 for (int i=0;i<_sensor_scan_count;i++) { _sensor_value[i] = 0; }
	 
	// 차동바퀴형 로봇의 바퀴 크기 및 바퀴간 거리
	 _wheel_radius = 0.2;
	 _axle_length = 0.4;

	 _robot_diameter = 0.5;

	 _target_direction = 0;
	 _steer_magnitude = 0;
	 _steer_direction = 0;

	 _motion_to_goal = false;
	 _d_followed = 1000.;
}

tangent_bugAlg::~tangent_bugAlg(void)
{
}

void tangent_bugAlg::setRobotpose(double robot_x,double robot_y,double robot_th )
{
	_robot_x = robot_x;
	_robot_y = robot_y;
	_robot_theta = robot_th;

}
void tangent_bugAlg::setGoalpose(double target_x,double target_y,double target_th )
{
	_target_x = target_x;
	_target_y = target_y;
	_target_theta = target_th;

}
void tangent_bugAlg::getValue(double* steer_magnitude,double* steer_direction)
{
	(* steer_magnitude)=_steer_magnitude;
	(* steer_direction)=DeltaRad (_steer_direction, _robot_theta);
}

inline int tangent_bugAlg::H_ID(double theta)
{
	double alpha = 1.;
	return (int)(theta + 10*360. + alpha/2.)%360;
}
void tangent_bugAlg::setSensorData(double* psensor_value)
{
	for(int i=0; i<361;i++)
	{
		_sensor_value[i]=psensor_value[i];
	}
}
void tangent_bugAlg::expand_obstacle()
{
	double tmp[361];
	double robot_radius = _robot_diameter/2.;

	for (int j=0; j<_sensor_scan_count; ++j) {
		tmp[j] = _sensor_max_range;
	}

	for (int j=0; j<_sensor_scan_count; ++j) {
		double distance = _sensor_value[j];

		if (distance < _sensor_max_range) {
			double beta = _DEG2RAD*j;
			double gamma = asin(robot_radius/distance);

			for (double i = beta-gamma; i < beta+gamma; i+=1.*_DEG2RAD) {
				double gamma_i = fabs(i - beta);
				double r = distance*sin(gamma_i);

				if (r <= robot_radius) {
					double d_i = distance*cos(gamma_i) - sqrt(robot_radius*robot_radius - r*r);
					// if (d_i < _sensor_min_range) d_i = _sensor_min_range;

					int index = H_ID(i*_RAD2DEG);
					tmp[index] = min(tmp[index], d_i);
				}
			}
		}
	}

	tmp[360] = tmp[0];
	memcpy (_sensor_value, tmp, sizeof(double)*_sensor_scan_count);
}


double tangent_bugAlg::heuristic_distance ()
{
	double dx = _target_x - _robot_x;
	double dy = _target_y - _robot_y;

	return sqrt (dx*dx + dy*dy);
}

double tangent_bugAlg::heuristic_distance (double angle, double magnitude)
{
	double dtx = _target_x - _robot_x;
	double dty = _target_y - _robot_y;

	double dsx = magnitude*cos(angle);
	double dsy = magnitude*sin(angle);

	dtx -= dsx;
	dty -= dsy;

	return sqrt (dtx*dtx + dty*dty) + sqrt (dsx*dsx + dsy*dsy);
}

bool tangent_bugAlg::find_min_distance_of_path (double &distance_, double &direction_)
{
	double min_distance = 1e6;
	double min_direction = 0;

	for (int i=0; i<_sensor_scan_count-1; ++i) {
		if ((_sensor_value[i] == _sensor_max_range && _sensor_value[i+1] < _sensor_max_range) ||
			(_sensor_value[i] <  _sensor_max_range && _sensor_value[i+1] == _sensor_max_range) ||
			(_robot_diameter < fabs(_sensor_value[i] - _sensor_value[i+1])) ) {

			double direction = _robot_theta + _DEG2RAD*(i + 0.5 - _sensor_scan_count/2);
			double distance = heuristic_distance (direction, (_sensor_value[i] + _sensor_value[i+1])/2.);

			if (distance < min_distance) {
				min_distance = distance;
				min_direction = direction;
			}
		}
	}

	if (min_distance < 1e6) {
		direction_ = min_direction;
		distance_  = min_distance;
		return true;
	}
	else {
		return false;
	}
}

bool tangent_bugAlg::motion_to_goal ()
{
	// 로봇과 목표지점 사이에 장애물이 감지되는지 확인
	int index = H_ID(_RAD2DEG*DeltaRad(_target_direction, _robot_theta) - _sensor_scan_count/2);
	bool not_intersect = _sensor_value[index] == _sensor_max_range;
	
	if (not_intersect) {
		// 로봇과 목표지점 사이에 장애물이 감지되지 않았다.
		// 로봇은 목표지점을 향해 진행한다.
		_steer_direction = _target_direction;
		_steer_magnitude = V_max;

		_d_followed = heuristic_distance ();
	}
	else {
		// 로봇과 목표지점 사이에 장애물이 감지되었다.
		// 로봇-장애물의 경계점-목표지점을 잇는 경로들 중에서 최단경로를 선택하여 진행한다.
		double direction;
		double d_reach;

		if (find_min_distance_of_path (d_reach, direction)) {
			_steer_direction = direction;
			_steer_magnitude = V_max;

			// 더이상 최단경로의 거리를 줄여나갈 수 없는 상황이다.
			// boundary_following으로 전환.
			if (d_reach > _d_followed + 0.1) {
				d_reach = _d_followed - 0.1;
				_motion_to_goal = false;
			}
		}
		else {
			// 로봇이 빠져나갈 구멍이 보이지 않는다.
			// 네비게이션 실패!
			_steer_magnitude = 0.;
			return false;
		}

		_d_followed = d_reach;
	}
	return true;
}

bool tangent_bugAlg::find_index_of_min_distance(int &index)
{
	double min_distance = _sensor_max_range;
	int min_index = -1;

	for (int i=0; i<_sensor_scan_count; ++i) {
		double distance = _sensor_value[i];
		if (distance < min_distance) {
			min_distance = distance;
			min_index = i;
		}
	}

	if (min_index != -1) {
		index = min_index;
		return true;
	}
	else {
		return false;
	}
}

bool tangent_bugAlg::seek_right_side_waypoint (int base_index, double &distance, double &direction)
{
	for (int i=1; i<_sensor_scan_count; ++i) {
		int ii = H_ID(base_index + i);

		if ((_sensor_value[ii] == _sensor_max_range) ||
			(_robot_diameter < fabs(_sensor_value[ii] - _sensor_value[ii-1])) ) {
				
			direction = _robot_theta + _DEG2RAD*(ii + 0.5 - _sensor_scan_count/2);
			distance = heuristic_distance (direction, (_sensor_value[ii] + _sensor_value[ii-1])/2.);
			return true;
		}
	}
	return false;
}

bool tangent_bugAlg::seek_left_side_waypoint (int base_index, double &distance, double &direction)
{
	for (int i=0; i<_sensor_scan_count-1; ++i) {
		int ii = H_ID(base_index - i);

		if ((_sensor_value[ii] == _sensor_max_range) ||
			(_robot_diameter < fabs(_sensor_value[ii] - _sensor_value[ii+1])) ) {
				
			direction = _robot_theta + _DEG2RAD*(ii - 0.5 - _sensor_scan_count/2);
			distance = heuristic_distance (direction, (_sensor_value[ii] + _sensor_value[ii+1])/2.);
			return true;
		}
	}
	return false;
}

bool tangent_bugAlg::boundary_following ()
{
	// 로봇과 목표지점 사이에 장애물이 감지되는지 확인
	int index = H_ID(_RAD2DEG*DeltaRad(_target_direction, _robot_theta) - _sensor_scan_count/2);
	bool not_intersect = _sensor_value[index] == _sensor_max_range;
	
	if (not_intersect) {
		// 로봇과 목표지점 사이에 장애물이 감지되지 않았다.
		// 로봇은 목표지점을 향해 진행한다.
		_motion_to_goal = true;
	}

	int base_index;

	if (find_index_of_min_distance (base_index)) {
		bool ret;
		double direction = 0.;
		double distance = 1e6;

		if (base_index < _sensor_scan_count/2) {
			// 장애물과 가장 가까운 점이 로봇의 오른 쪽에 있음.
			ret = seek_right_side_waypoint (base_index, distance, direction);
		}
		else {
			// 장애물과 가장 가까운 점이 로봇의 왼 쪽에 있음.
			ret = seek_left_side_waypoint (base_index, distance, direction);
		}

		if (ret) {
			_steer_direction = direction;
			_steer_magnitude = V_max;

			if (distance < _d_followed) {
				_steer_magnitude = 0.;
				_motion_to_goal = true;
			}
		}
		else {
			return false;
		}	
	}
	else {
		_motion_to_goal = true;
	}

	return true;
}

bool tangent_bugAlg::tangent_bug()
{
	_target_direction = atan2(_target_y - _robot_y, _target_x - _robot_x);

	if (_motion_to_goal) {
		return motion_to_goal ();
	}
	else { 
		return boundary_following ();
	}
}

void tangent_bugAlg::speed_control()
{
	if (_steer_magnitude < V_min) _steer_magnitude =V_min;

	double Omega = DeltaRad (_steer_direction, _robot_theta);

	_steer_magnitude = _steer_magnitude*(1. - fabs(Omega)/Omega_max);
	if (_steer_magnitude < V_min) _steer_magnitude = V_min;
}