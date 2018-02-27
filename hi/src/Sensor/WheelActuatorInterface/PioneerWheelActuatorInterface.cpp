#include "stdafx.h"
#include "PioneerWheelActuatorInterface.h"

/*************************************************************
* FUNCTION NAME : on()
* DISCRIPTION : servo를 on 시킨다.
* ARGUMENT :
* RETURN : void
* AUTHOR Joongtea,park
* REVISION 2007/09
*************************************************************/
PioneerWheelActuatorInterface::PioneerWheelActuatorInterface()
{
	MAX_TRANSLATION_VELOCITY = 500;
}

PioneerWheelActuatorInterface::~PioneerWheelActuatorInterface()
{

}
/**
 @brief Korean: 로봇에  연결한다.
 @brief English: 
*/
bool PioneerWheelActuatorInterface::connect(string strSerialPort)
{
	string strParam = "-rp";
	string test = "t";

	int argc=3;
	char** argv=NULL;
	argv = new char*[argc];
	argv[0] = new char[20]; //t
	argv[1] = new char[20]; //-p
	argv[2] = new char[20]; //port
	

	memcpy(argv[0],test.c_str(), 20);
	memcpy(argv[1],strParam.c_str(), 20);
	memcpy(argv[2],strSerialPort.c_str(),20);
	
	
	Aria::init();
 	ArArgumentParser arParser(&argc,argv);
 	arParser.loadDefaultArguments();
	ArSimpleConnector simpleConnector(&arParser);
	
	delete [] argv;
 
 	if(!simpleConnector.connectRobot( &m_arRobot)){ 
 		//로봇과 연결 실패된 상황
		cout<<"Could not connect to the robot!!!"<<endl;
 		return false;
	}else if (!Aria::parseArgs()){
		Aria::logOptions();
		Aria::exit(2);
		return 2;
	}
	else{ //로봇하고 연결된 상황
 
	//	m_arRobot.disableSonar(); //초음파 센서 비활성화
		 
		m_arRobot.runAsync(true);
		// turn on the motors, turn off amigobot sounds
		m_arRobot.enableMotors();
		m_arRobot.comInt(ArCommands::SOUNDTOG, 0);


		m_arDeadPose = m_arRobot.getEncoderPose();
		m_arLastDeadPose = m_arDeadPose;


		m_arRobot.lock();
		ArLog::log(ArLog::Normal, "simpleConnect: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV",
			m_arRobot.getX(), m_arRobot.getY(), m_arRobot.getTh(), m_arRobot.getVel(), m_arRobot.getBatteryVoltage());
		m_arRobot.unlock();
 
 	}
 
// 	  Aria::logOptions();
// 	  while(1);
	return true;

}

/**
 @brief Korean: 로봇의 남은 베터리 양을 받아온다.
 @brief English: 
*/
double PioneerWheelActuatorInterface::getVoltage()
{
	return m_arRobot.getBatteryVoltage();
}

/**
 @brief Korean: 로봇과의 연결을 끝는다.
 @brief English: 
*/
bool PioneerWheelActuatorInterface::disConnect()
{

 	bool bRobotState;
 	m_arRobot.lock();
 	bRobotState = m_arRobot.disconnect();
 	m_arRobot.unlock();
 	m_arRobot.stopRunning(bRobotState);
 	//m_arRobotCon.close();
 	Aria::shutdown();
	return true;

}

/**
 @brief Korean: 로봇의 움직임을 멈춘다.
 @brief English: 
*/
void PioneerWheelActuatorInterface::stop()
{
	m_dTranslationVelocity = m_dRotationVelocity =0.0;
	m_arRobot.lock();
	m_arRobot.setRotVel(m_dRotationVelocity);
	m_arRobot.setVel(m_dTranslationVelocity);
	m_arRobot.unlock();
}

/**
 @brief Korean: 병진 속도와 회전속도로 로봇을 움직인다.
 @brief English: 
*/
bool PioneerWheelActuatorInterface::moveByTRVelocity(double translationVelocity, double rotationalVelocity)
{
 	m_arRobot.lock();
 
 	m_dTranslationVelocity = translationVelocity;
 	if(m_dTranslationVelocity > MAX_TRANSLATION_VELOCITY) m_dTranslationVelocity = MAX_TRANSLATION_VELOCITY;
 	
 	
 	m_dRotationVelocity = rotationalVelocity;
 	if(m_dRotationVelocity > MAX_ROTATION_VELOCITY) m_dRotationVelocity = MAX_ROTATION_VELOCITY;
 
 
 	m_arRobot.setVel(m_dTranslationVelocity);
 	m_arRobot.setRotVel(m_dRotationVelocity);
 	
 	m_arRobot.unlock();
	return 0;
}

/**
 @brief Korean: 다른 클래스에서 로봇의 X변화량을 가져간다.
 @brief English: 
*/
double PioneerWheelActuatorInterface::getX()
{
	return m_arRobot.getX();
}
/**
 @brief Korean: 다른 클래스에서 로봇의 Y변화량을 가져간다.
 @brief English: 
*/
double PioneerWheelActuatorInterface::getY()
{
	return m_arRobot.getY();
}

/**
 @brief Korean: 다른 클래스에서 degree 단위로 로봇의 각도의 변화량 가져간다.
 @brief English: 
*/
double PioneerWheelActuatorInterface::getThetaDeg()
{
	return m_arRobot.getTh();
}
/**
 @brief Korean: 다른 클래스에서 병진속도의 가감속 값을 가져간다.
 @brief English: 
*/
double PioneerWheelActuatorInterface::getTVelIncrement()
{
	return (double)TVEL_INCREMENT;
}
/**
 @brief Korean: 다른 클래스에서 회진속도의 가감속 값을 가져간다.
 @brief English: 
*/
double PioneerWheelActuatorInterface::getRVelIncrement()
{
	return (double)RVEL_INCREMENT;
}
/**
 @brief Korean: 다른 클래스에서 병진속도 값을 가져간다.
 @brief English: 
*/
double PioneerWheelActuatorInterface::getTVel()
{
	return m_dTranslationVelocity; //m_arRobot.getVel();
}
/**
 @brief Korean: 다른 클래스에서 회진속도 값을 가져간다.
 @brief English: 
*/
double PioneerWheelActuatorInterface::getRVel()
{
	return m_dRotationVelocity; //m_arRobot.getRotVel();
}
 /**
 @brief Korean: 다른 클래스에서 X,Y,Theta의 변화량의 값을 가져간다.
 @brief English: 
*/
KuPose PioneerWheelActuatorInterface::getDelEncoderData()
{
	m_arDeadPose = m_arRobot.getRawEncoderPose();
	double dX, dY;
	
	KuPose Pose;
	
	dX = m_arDeadPose.getX() - m_arLastDeadPose.getX();
	dY = m_arDeadPose.getY() - m_arLastDeadPose.getY();
	
	Pose.setX( dX * cos(-m_arLastDeadPose.getTh()*M_PI/180.0) + dY * sin(m_arLastDeadPose.getTh()*M_PI/180.0)  );
	Pose.setY(dX * sin(-m_arLastDeadPose.getTh()*M_PI/180.0) + dY * cos(-m_arLastDeadPose.getTh()*M_PI/180.0));
	Pose.setThetaDeg(m_arDeadPose.getTh() - m_arLastDeadPose.getTh());
	
	m_arLastDeadPose = m_arDeadPose;
	return Pose;
}
 /**
 @brief Korean: 로봇의 최대 속도를 설정한다.
 @brief English: 
*/
void PioneerWheelActuatorInterface::setMaxTransVel(int nVel)
{
	MAX_TRANSLATION_VELOCITY = nVel; //nVelmm/sec
}
