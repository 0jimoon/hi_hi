#include "stdafx.h"
#include "KuSimulLocalizer.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif


// initialize pointer
KuSimulLocalizer* KuSimulLocalizer::thisInstance = 0;

KuSimulLocalizer::KuSimulLocalizer()
{

	m_dAccumulatedDeltaMovement = 0.0;
	m_dAccumulatedDeltaAngle = 0.0;

	InitVariable();
	m_RobotPos.init(); //로봇 위치 초기화
		
}

KuSimulLocalizer::~KuSimulLocalizer()
{
	
}

/*************************************************************
* FUNCTION NAME : getInstance()
* DISCRIPTION : get Sole Instance
* ARGUMENT :  
* RETURN :CMonteCarlo* - Sole Instance
* AUTHOR Joongtea,park Yong-Ju Lee, Byung-Doo Yim
* REVISION 2006/09/30
*************************************************************/
KuSimulLocalizer* KuSimulLocalizer::getInstance()
{
	if( thisInstance == 0 ){
	//	cout << "[KuSimulLocalizer] : Create Sole Instance.." << endl;
	        thisInstance = new KuSimulLocalizer;
	}
	return thisInstance;
}
/*************************************************************
* FUNCTION NAME : ReleaseInstance()
* DISCRIPTION : delete Sole Instance
* ARGUMENT :  
* RETURN :CMonteCarlo* - Sole Instance
* AUTHOR Joongtea,park Yong-Ju Lee, Byung-Doo Yim
* REVISION 2006/09/30
*************************************************************/
void KuSimulLocalizer::ReleaseInstance()
{
    
	delete thisInstance;
}

void KuSimulLocalizer::InitVariable()
{
	//for encoder
	m_dEncData[0] =0.;
	m_dEncData[1] =0.;
	m_dReferenceLeftWheelEncoderCount =0.;
	m_dReferenceRightWheelEncoderCount=0.;
	m_dLeftWheelEncoderCount=0.;
	m_dRightWheelEncoderCount=0.;
	m_dLeftWheelDistance=0.;
	m_dRightWheelDistance=0.;
	m_dAverageWheelDistance=0.;
	m_dDistance2RobotCenter=0.;
	m_dDeltaX =0.;
	m_dDeltaY =0.;
	m_dDeltaT =0.; 
	m_dReferenceX = 0;
    m_dReferenceY = 0;
    m_dReferenceT = 0;
	//------------------------------------
// 	m_dOldRefPos[0] = 10300;
// 	m_dOldRefPos[1] = 36500;
// 	m_dOldRefPos[2] = 0; //unit deg

	//memset(m_dDelOdoPos,0,sizeof(m_dDelOdoPos));

	m_dRightWheelVel = m_dLeftWheelVel =0.;
}



void KuSimulLocalizer::Vel2EncData(double V, double W)
{
	m_dRightWheelVel = V + (BETWEEN_WHEEL * W)/2.;
	m_dLeftWheelVel = V - (BETWEEN_WHEEL * W)/2.;
	

    //10으로 나눈 이유는 0.1초마다 엔코더를 세기 때문에 
	m_dEncData[0] += m_dLeftWheelVel / 10 * (GEAR_RATIO * ENCODER_RESOLUTION) /
					 (2 * M_PI * RADIUS);

	m_dEncData[1] += m_dRightWheelVel / 10 * (GEAR_RATIO * ENCODER_RESOLUTION) /
					 (2 * M_PI * RADIUS);
}
void KuSimulLocalizer::moveByVelocityXYT(double X, double Y, double Theta)
{
	m_DelEncoderPos.setX(X); 
	m_DelEncoderPos.setY(Y);
	m_DelEncoderPos.setThetaRad(Theta); 

	m_dReferenceX +=  X * cos( m_dReferenceT ) + Y * sin( -m_dReferenceT +Theta/2.0);
	m_dReferenceY +=  X * sin( m_dReferenceT ) + Y * cos( m_dReferenceT  +Theta/2.0);
	m_dReferenceT =  m_dReferenceT+Theta;



	if(m_dReferenceT > M_PI){
		m_dReferenceT -= 2*M_PI;
	}
	else if(m_dReferenceT < -M_PI){
		m_dReferenceT += 2*M_PI;
	}

}


void KuSimulLocalizer::SetRobotX(double dRobotPosX)
{
	m_dReferenceX = dRobotPosX;
}

void KuSimulLocalizer::SetRobotY(double dRobotPosY)
{
	m_dReferenceY = dRobotPosY;
}
void KuSimulLocalizer::SetRobotTH(double dRobotPosTH)
{
	m_dReferenceT = dRobotPosTH*D2R;
}

void KuSimulLocalizer::setRobotPos(KuPose RobotPos)
{
	m_RobotPos = RobotPos;
	m_dReferenceX = RobotPos.getX();
	m_dReferenceY = RobotPos.getY();
	m_dReferenceT = RobotPos.getThetaRad();
}

KuPose KuSimulLocalizer::getDelEncoderData()
{
	return m_DelEncoderPos;
}

KuPose KuSimulLocalizer::getRobotPos()
{
	calcEncoderData(); 
	m_RobotPos.setX( m_dReferenceX );
	m_RobotPos.setY( m_dReferenceY );
	m_RobotPos.setThetaRad( m_dReferenceT );

	return m_RobotPos;
}

void KuSimulLocalizer::computeAccumulatedDeltaMovement(KuPose EncoderDelPos)
{
	m_dAccumulatedDeltaMovement += sqrt(EncoderDelPos.getX()*EncoderDelPos.getX() + EncoderDelPos.getY()*EncoderDelPos.getY());
	m_dAccumulatedDeltaAngle += fabs(EncoderDelPos.getThetaDeg());
}

bool KuSimulLocalizer::isAccDeltaMovementOver(double dMovement, double dAngle)
{
	if (m_dAccumulatedDeltaMovement > dMovement || m_dAccumulatedDeltaAngle > dAngle) {
		m_dAccumulatedDeltaMovement = 0.0;
		m_dAccumulatedDeltaAngle = 0.0;
		return true;
	}
	else return false;
}
void KuSimulLocalizer::calcEncoderData()
{
	m_dLeftWheelEncoderCount = m_dEncData[0];
	m_dRightWheelEncoderCount = m_dEncData[1];

	m_dLeftWheelDistance = ( (m_dLeftWheelEncoderCount - m_dReferenceLeftWheelEncoderCount)
		/ (GEAR_RATIO * ENCODER_RESOLUTION)
		) * 2 * M_PI * RADIUS;


	m_dRightWheelDistance = ( (m_dRightWheelEncoderCount - m_dReferenceRightWheelEncoderCount )
		/ (GEAR_RATIO * ENCODER_RESOLUTION)
		) * 2 * M_PI * RADIUS;

	m_dReferenceLeftWheelEncoderCount = m_dLeftWheelEncoderCount;
	m_dReferenceRightWheelEncoderCount = m_dRightWheelEncoderCount;

	m_dAverageWheelDistance = (m_dLeftWheelDistance + m_dRightWheelDistance)/2;

	m_dDeltaT = (m_dRightWheelDistance - m_dLeftWheelDistance) / BETWEEN_WHEEL;

	if(fabs(m_dDeltaT) >= 0.0017f ) {
		m_dDistance2RobotCenter = m_dAverageWheelDistance / m_dDeltaT;
		m_dDeltaY = m_dDistance2RobotCenter - ( m_dDistance2RobotCenter * cos(m_dDeltaT) );
		m_dDeltaX = m_dDistance2RobotCenter * sin(m_dDeltaT);
	}
	else {
		m_dDistance2RobotCenter = 0.0f;
		m_dDeltaY = 0;
		m_dDeltaX = m_dAverageWheelDistance;
	}

	//상대좌표를 절대좌표로 변환하는것.
	m_dReferenceX +=  m_dDeltaX * cos( m_dReferenceT ) + m_dDeltaY * sin( -m_dReferenceT +m_dDeltaT/2.0);
	m_dReferenceY +=  m_dDeltaX * sin( m_dReferenceT ) + m_dDeltaY * cos( m_dReferenceT  +m_dDeltaT/2.0);
	m_dReferenceT =  m_dReferenceT+m_dDeltaT;


	if(m_dReferenceT > M_PI){
		m_dReferenceT -= 2*M_PI;
	}
	else if(m_dReferenceT < -M_PI){
		m_dReferenceT += 2*M_PI;
	}

	m_DelEncoderPos.setX(m_dDeltaX); 
	m_DelEncoderPos.setY(m_dDeltaY);
	m_DelEncoderPos.setThetaRad(m_dDeltaT); 
	computeAccumulatedDeltaMovement(m_DelEncoderPos);
	
}