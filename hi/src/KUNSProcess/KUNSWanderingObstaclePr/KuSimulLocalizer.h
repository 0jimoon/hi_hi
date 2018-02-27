#ifndef C_SIMULATION_LOCALIZER_H
#define C_SIMULATION_LOCALIZER_H



#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"

using namespace std;
class KuSimulLocalizer   : public KuSingletone <KuSimulLocalizer>
{
private:

	static const int RADIUS = 75;//mm //������ �ݰ�. 
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

	static KuSimulLocalizer* thisInstance; //�̱������� ����� ���� ����.


	// ���ڴ��� ����Ͽ� ������� �̵����� ����ġ�� ����ϴ� ���
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
	
	
	static KuSimulLocalizer* getInstance(); //�ܺο��� �̱��� ��ü�� ��� ���� �Լ�.
	void ReleaseInstance();//�ܺο��� �̱��� ��ü �����ϱ� ���� �Լ�.
	KuSimulLocalizer();
	~KuSimulLocalizer();



};

#endif 
