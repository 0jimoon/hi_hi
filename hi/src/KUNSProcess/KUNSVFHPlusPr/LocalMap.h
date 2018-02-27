#pragma once
#include <vector>
#include "../../KUNSMath/KuMath.h"

using namespace std;

// Size of the window is 33 x 33 cells
// with a cell size of 10cm x 10cm

#define CELL_SIZE		0.1		// m
#define WINDOW_SIZE		33

class CLocalMap {
private:
	double m_drobot_theta;//rad
	int m_nprev_pos_x;
	int m_nprev_pos_y;

	double m_dsensor_x;
	double m_dsensor_y;
	double m_dsensor_theta;//rad

	// 센서의 최소, 최대 감지 거리
	double m_dsensor_min_range;//m
	double m_dsensor_max_range;//m

	// 센서의 스캔 시작 각도와 끝 각도 및 해상도
	double m_dsensor_scan_range;

	// 센서 값의 개수와 값을 저장하는 배열
	int m_nsensor_scan_count;
public:
	long m_cells[WINDOW_SIZE][WINDOW_SIZE];

	inline double CU (double p)	{ return (p + CELL_SIZE/2)/CELL_SIZE; }
	// 
	inline int M2CUx (double x)	{ return (int)(WINDOW_SIZE/2 + x/CELL_SIZE); }
	inline int M2CUy (double y)	{ return (int)(WINDOW_SIZE/2 + y/CELL_SIZE); }
	inline double CU2Mx (int x)	{ return (x - WINDOW_SIZE/2)*CELL_SIZE; }
	inline double CU2My (int y)	{ return (y - WINDOW_SIZE/2)*CELL_SIZE; }

public:
	CLocalMap ();
	~CLocalMap ();

	inline bool IsIn (int x, int y) { return (0 <= x && x < WINDOW_SIZE) && (0 <= y && y < WINDOW_SIZE); }
	inline void SetPixel (int x, int y, long value) { if (IsIn(x, y)) m_cells[y][x] = value; }
	void DrawLine (int x1, int y1, int x2, int y2, long value);
	void Shift (int dx, int dy);
	void Clear ();

	void MoveRobotPos (double pos_x, double pos_y, double pos_theta);
	void UpdateSensorValue (double value[], int no);

	void setSensorParameter (double dsensor_x,double dsensor_y,double dsensor_theta
		,double dsensor_max_range,double dsensor_min_range);
};

