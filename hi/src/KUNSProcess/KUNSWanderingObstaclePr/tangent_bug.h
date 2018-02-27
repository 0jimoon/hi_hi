
// VFF_clientDlg.h : header file
//

#pragma once
#include <math.h>
#include "mm.h"
#include "../../KUNSUtil/KuUtil.h"


#define V_max		0.9
#define V_min		0.05
#define Omega_max	(_DEG2RAD*90)

class tangent_bugAlg
{

private:

// 전역 좌표계에서 타겟의 위치와 방향
 double _target_x;
 double _target_y;
 double _target_theta;

// 전역 좌표계에서 로봇의 위치와 방향
 double _robot_x;
 double _robot_y;
 double _robot_theta;

// 센서의 최소, 최대 감지 거리
 double _sensor_min_range;
 double _sensor_max_range;
// 센서의 스캔 시작 각도와 끝 각도 및 해상도
 double _sensor_scan_range;
// 센서 값의 개수와 값을 저장하는 배열
 int _sensor_scan_count;
 double _sensor_value[361];

// 차동바퀴형 로봇의 바퀴 크기 및 바퀴간 거리
 double _wheel_radius;
 double _axle_length;
 double _robot_diameter;


 double _target_direction;
 double _steer_magnitude;
 double _steer_direction;


 bool _motion_to_goal ;
 double _d_followed ;



 inline int H_ID(double theta);
 double heuristic_distance ();
 double heuristic_distance (double angle, double magnitude);
 bool find_min_distance_of_path (double &distance_, double &direction_);
 bool motion_to_goal ();
 bool find_index_of_min_distance(int &index);
 bool seek_right_side_waypoint (int base_index, double &distance, double &direction);
 bool seek_left_side_waypoint (int base_index, double &distance, double &direction);
 bool boundary_following ();

 public:
	 void setRobotpose(double robot_x,double robot_y,double robot_th );
	 void setGoalpose(double target_x,double target_y,double target_th );
	 void expand_obstacle();
	 bool tangent_bug();
	 void speed_control();
	 void getValue(double* steer_magnitude,double* steer_direction);
	 void setSensorData(double* psensor_value);
public:
	tangent_bugAlg(void);
	~tangent_bugAlg(void);
};
