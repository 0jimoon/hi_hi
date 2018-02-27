/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mecanical Engineering Korea University                                    
(c) Copyright 2007 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : active media���� pioneer �κ��� �ӵ� �� ���ڵ����� �����ϴ� ����� �����ϴ� Ŭ����
$Data: 2007/09                                                                           
$Author: Joong-Tae Park                                                                      
______________________________________________________________________________________________*/

#ifndef PIONEER_WHEEL_ACTUATOR_INTERFACE_H
#define PIONEER_WHEEL_ACTUATOR_INTERFACE_H



#include "../../src/KUNSPose/KuPose.h"
#include "../../src/KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../src/KUNSUtil/KUNSThread/KuThread.h"
#include <Aria.h> //aria.h ��Ŭ��� ��ġ�� �ٲٸ� �ȵ�.

using namespace std;

class PioneerWheelActuatorInterface : public KuSingletone <PioneerWheelActuatorInterface>
{
protected:
	ArRobot m_arRobot;	

private:

	ArPose m_arDeadPose;
	ArPose m_arLastDeadPose;

private:
	static const int TVEL_INCREMENT = 20; //20mm/sec �����ӵ��� ������.
	static const int RVEL_INCREMENT = 1; // 1deg/sec ȸ���ӵ��� ������.
	int MAX_TRANSLATION_VELOCITY;
	static const int MAX_ROTATION_VELOCITY = 60; //30deg/sec

	double m_dTranslationVelocity;
	double m_dRotationVelocity;
	
	
public:
	bool connect(string strSerialPort);
	bool disConnect();
	void stop();
	bool moveByTRVelocity(double translationVelocity, double rotationalVelocity);
	double getX();
	double getY();
	double getThetaDeg();
	double getTVelIncrement();
	double getRVelIncrement();
	double getTVel();
	double getRVel();
	double getVoltage();
	KuPose getDelEncoderData();
	void setMaxTransVel(int nVel);

	PioneerWheelActuatorInterface();
	~PioneerWheelActuatorInterface();
};
#endif



	

