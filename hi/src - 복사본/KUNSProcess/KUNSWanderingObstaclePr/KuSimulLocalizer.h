#ifndef C_SIMULATION_LOCALIZER_H
#define C_SIMULATION_LOCALIZER_H



#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"

using namespace std;
class KuSimulLocalizer   : public KuSingletone <KuSimulLocalizer>
{
private:

	static const int RADIUS = 75;//mm //바퀴의 반경. 
	static const int ENCODER_RESOLUTION = 2048; 
	static const int GEAR_RATIO = 150;
	static const int BETWEEN_WHEEL = 500;
	
private:
	

	//for encoder
	double m_dEncData[2];
	double m_dReferenceLeftWheelEncoderCount;
	double m_dReferenceRightWheelEncoderCount;
	double m_dLeftWheelEncoderCount;
	double m_dRightWheelEncoderCount;
	double m_dLeftWheelDistance;
	double m_dRightWheelDistance;
	double m_dAverageWheelDistance;
	double m_dDistance2RobotCenter;
	double m_dDeltaX;
	double m_dDeltaY;
	double m_dDeltaT; 
	double m_dReferenceX;
    double m_dReferenceY;
    double m_dReferenceT;
	//------------------------------------	
	
	//vel;
	double m_dRightWheelVel;
	double m_dLeftWheelVel;
	
	//for robotPos
	KuPose m_RobotPos;	
	KuPose m_DelEncoderPos;

	double m_dOldOdometryPos[3];
	double m_dOldRefPos[3];
	double m_dDelOdoPos[3];

	static KuSimulLocalizer* thisInstance; //싱글톤으로 만들기 위한 변수.


	// 엔코더에 기반하여 상대적인 이동량의 누적치를 계산하는 기능
	double m_dAccumulatedDeltaMovement;
	double m_dAccumulatedDeltaAngle;

private:
	void InitVariable();
	void calcEncoderData();
	void computeAccumulatedDeltaMovement(KuPose EncoderDelPos);

public:
	void setRobotPos(KuPose RobotPos);
	KuPose getRobotPos();
	KuPose getDelEncoderData();
	bool isAccDeltaMovementOver(double dMovement, double dAngle);
	void moveByVelocityXYT(double X, double Y, double Theta);

	void SetRobotY(double dRobotPosY);
	void SetRobotX(double dRobotPosX);
	void SetRobotTH(double dRobotPosTH);
	void Vel2EncData(double V, double W);
	
	
	static KuSimulLocalizer* getInstance(); //외부에서 싱글톤 객체를 얻기 위한 함수.
	void ReleaseInstance();//외부에서 싱글톤 객체 해제하기 위한 함수.
	KuSimulLocalizer();
	~KuSimulLocalizer();



};

#endif 
