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
	static const int DWA_SIZE = 41; //-16~16 다시말해 -VELOCITYRANGE~ VELOCITYRANGE 의미
	static const int DWA_VELLIMIT = 150; // unit cm/sec
	static const int DWA_RIGHT_ACCELATION	= 120; // unit cm/sec^2
	static const int DWA_LEFT_ACCELATION	= 120; // unit cm/sec^2

	static const int MAX_TRANSVEL = 1500; // unit cm/sec^2
	static const int MAX_ROTVEL	= 30; // unit cm/sec^2
	static const int  SAFE_DISTANCE = 15;

	static const int INDEX =181;


	DynamicWindow m_DWAData[DWA_SIZE][DWA_SIZE]; //자료형 클래스 객체

	double m_dX, m_dY;
	int m_nMinVelocity;
	double	m_dTVel;
	double m_dRVel;
	double m_dRotateVelLimite; 
	double m_ditratetime;
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
	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);
private:
	int m_nLaserData[INDEX];
	double m_dDistBetweenWheel; //로봇 양바퀴사이의 거리 unit--> mm
	double m_dRadiusofRobot; //로봇 몸체 반경 
	
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
	
};

#endif 