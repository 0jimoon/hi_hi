#include "stdafx.h"
#include "KuSimulMotionControllerPr.h"

KuSimulMotionControllerPr::KuSimulMotionControllerPr()
{

	m_nPathNoPointer = 0;
	m_bAlignRobotAngleFlag = true;
	m_dMaxTVel = 0.7;
	m_dDesiredVel = 700;
	m_bMotionDirection=1;

	m_nPath=NULL;
	m_dPath=NULL;

	m_dXVel = 0;
	m_dYVel = 0;
}



void KuSimulMotionControllerPr::init()
{
	if(m_nPath!=NULL && m_dPath!=NULL){
		for(int i=0;i<3;i++){
			free(m_nPath[i]);
			free(m_dPath[i]);
		}
		free(m_nPath);
		free(m_dPath);

		m_nPath = NULL;
		m_dPath = NULL;
	}

	m_nPathNoPointer = 0;
	m_bAlignRobotAngleFlag = true;
}

KuSimulMotionControllerPr::~KuSimulMotionControllerPr()
{
	
}

void KuSimulMotionControllerPr::turnRight()
{
	KuSimulLocalizer::getInstance()->Vel2EncData(0, -30*D2R);
}

void KuSimulMotionControllerPr::turnLeft()
{
	KuSimulLocalizer::getInstance()->Vel2EncData(0, 30*D2R);
}

void KuSimulMotionControllerPr::stop()
{
	KuSimulLocalizer::getInstance()->Vel2EncData(0, 0);
}

void KuSimulMotionControllerPr::move(double dTVal, double dRVal)
{
	KuSimulLocalizer::getInstance()->Vel2EncData(dTVal, dRVal*D2R);
}



void KuSimulMotionControllerPr::TempDrive(double dTargetX, double dTargetY, double dDesiredVel,KuPose RobotPos)
{
	double dMaxV = m_dMaxTVel;     // 500mm/s�� �κ��� �� �� �ִ� �ִ� �ӵ�
	double dMaxW = 50*D2R; //50degree/s�� �κ��� �� �� �ִ� �ִ� ���ӵ�

	double dkx = 0.4;   //�κ��� �������� ������ �غ��ϴ� ����. ���� �κ��� ��� �ʹ� ũ�� �ⷷ�̰�, �ʹ� ������ ���ϴ� �ӵ����� ������ ������.
	double dky = 1.0;   //�κ��� ������� ������ �غ��ϴ� ����. ������ ũ���� ������, ũ�� �Ҿ����ϰ�, ������ ������� ������ ���� ����.
	double dkth = 1.2;  //�������� ���� �κ��� ����(heading)�� �غ��ϴ� ����. ũ�� �ſ� �ⷷ�̰�(Ư�� �ʱ⿡ ���ڸ����� �������� ���� �� ��), ������ �� ������ ����.


	// ��ǥ��ġ�� �κ���ġ ������ ���� ���.
	// ��ǥ��ġ�� �κ���ġ�� ������ǥ�� ����. �κ��� �����(v,w) �κ���ǥ�� ����.
	// ���� ������ǥ�� �󿡼��� ���� �����ǥ��� ��ȯ�ؾ� ��.
	
	double derrorX = (dTargetX/1000. - RobotPos.getXm()) * cos(-RobotPos.getThetaRad()) + (dTargetY/1000.-RobotPos.getYm()) * sin(RobotPos.getThetaRad());
	double derrorY = (dTargetX/1000. - RobotPos.getXm()) * sin(-RobotPos.getThetaRad()) + (dTargetY/1000.-RobotPos.getYm()) * cos(-RobotPos.getThetaRad());
	double derrorThRad = atan2(dTargetY/1000.-RobotPos.getYm(), dTargetX/1000. - RobotPos.getXm()) - RobotPos.getThetaRad();
	if(derrorThRad > M_PI ) derrorThRad = derrorThRad - 2*M_PI;
	if(derrorThRad <-M_PI ) derrorThRad = derrorThRad + 2*M_PI;
	 

	//Kanayama�� ���� �̿��ؼ� ���� ���̱� ����, �� ��ǥ���� ���� ���� ���� �κ��� �ӵ� ���.
	double dTVel = dDesiredVel/1000.*cos(derrorThRad) + dkx*derrorX;
	double dRotVel = 0 + dDesiredVel/1000.*(dky*derrorY + dkth*sin(derrorThRad));

	
	if (dTVel>dMaxV) dTVel=dMaxV; 
	if (dRotVel>dMaxW) dRotVel=dMaxW;
	if (dRotVel<-dMaxW) dRotVel=-dMaxW; 

	if(fabs(derrorX*1000)<100){
//		stop();
		move(dTVel*1000, dRotVel*R2D);
	}
	else{
		move(dTVel*1000, dRotVel*R2D);
	}

}


KuPose KuSimulMotionControllerPr::getTargetPos()
{
	return m_TargetPos;
}


bool KuSimulMotionControllerPr::move(KuPose RobotPos, list<KuPose> PathList, bool* bIsNewPath)
{

	if(PathList.size() <= 2) return true;

	list<KuPose>::iterator it;

	if(*bIsNewPath){
		*bIsNewPath = false;
		//��� ����Ʈ �� �迭�� ����
		m_nPath = (int**)calloc(3, sizeof(int*));
		m_dPath = (double**)calloc(3,sizeof(double*));
		for(int i=0;i<3; i++){
			m_nPath[i] = (int*)calloc(PathList.size(),sizeof(int));
			m_dPath[i] = (double*)calloc(PathList.size(),sizeof(double));
		}

		int nIdx=0;
		for(it=PathList.begin(); it!=PathList.end(); it++, nIdx++){
			m_nPath[0][nIdx] = (int)it->getX();
			m_nPath[1][nIdx] = (int)it->getY();
			m_nPath[2][nIdx] = (int)it->getThetaDeg();

			m_dPath[0][nIdx] = it->getX();
			m_dPath[1][nIdx] = it->getY();
			m_dPath[2][nIdx] = it->getThetaDeg();


		}
		m_nPathCnt = PathList.size() -1 ; //����Ʈ �� �������� ������ ���� �� �ִ�. 
		m_nPathNoPointer = 2;

	}
	int nPathX, nPathY,nPathThDeg;
	double dPathX, dPathY,dPathThDeg;

	bool bIsPathEnded;
	int nVel = 500;
	bIsPathEnded = false;

	nPathX = (m_nPath[0][m_nPathNoPointer]);
	nPathY = (m_nPath[1][m_nPathNoPointer]);

	while( (!bIsPathEnded) && sqrt( pow((RobotPos.getX() - nPathX),2) + pow((RobotPos.getY() - nPathY),2) ) < TARGET_DIST ) {
		m_nPathNoPointer = m_nPathNoPointer + 1;

		if(m_nPathNoPointer >= m_nPathCnt) {
			bIsPathEnded = true;
			m_nPathNoPointer = m_nPathCnt;
		}

		nPathX = (m_nPath[0][m_nPathNoPointer]);
		nPathY = (m_nPath[1][m_nPathNoPointer]);
		nPathThDeg = m_nPath[2][m_nPathNoPointer];

		dPathX = m_dPath[0][m_nPathNoPointer];
		dPathY = m_dPath[1][m_nPathNoPointer];
		dPathThDeg = m_dPath[2][m_nPathNoPointer];

	}
	if ((bIsPathEnded) && ((pow( RobotPos.getX() - (double)nPathX,2)+ pow(RobotPos.getY() - (double)nPathY,2))< GOAL_AREA*GOAL_AREA )){
		stop();
		return true;
	}
	else{

		m_TargetPos.setX(nPathX);
		m_TargetPos.setY(nPathY);

		if(m_bAlignRobotAngleFlag==true){			

			double dAngleDiff = atan2(m_TargetPos.getY()-RobotPos.getY(),m_TargetPos.getX()-RobotPos.getX()) - RobotPos.getThetaRad();
			if (dAngleDiff >= M_PI){
				dAngleDiff -= 2.0*M_PI;
			}			
			else if (dAngleDiff <= -M_PI){
				dAngleDiff += 2.0*M_PI;
			}

			if (dAngleDiff > M_PI*1/9) {
				turnLeft();
				return false;
			}	
			else if (dAngleDiff < -M_PI*1/9) {
				turnRight();
				return false;;
			}
			m_bAlignRobotAngleFlag = false;
		}

		TempDrive(nPathX,nPathY,(int)m_dDesiredVel,RobotPos);

	}

	return false;
}

