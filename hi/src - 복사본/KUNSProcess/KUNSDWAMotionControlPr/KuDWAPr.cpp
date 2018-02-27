#include "stdafx.h"
#include "KuDWAPr.h"

KuDWAPr::KuDWAPr()
{
	srand(time(NULL));
	InitVar();	//1 순서가 중요함. InitVar()
	InitTheta( );
}

KuDWAPr::~KuDWAPr()
{
	if(m_dCosData!=NULL)
		delete [] m_dCosData;
	if(m_dSinData!=NULL)
	delete [] m_dSinData;
			
}
/**
@brief Korean: 모든 전역 변수에 대한 초기화를 하는 함수 
@brief English: 
*/
void KuDWAPr::InitVar()
{
  	m_dAlphaH =3; //heading //3
 	m_dAlphaC = 8; //clearance //5 
  	m_dAlphaV = 2; //velocity //2
	memset(m_DWAData,0,sizeof(m_DWAData));

	m_GoalArea=2000;
	m_dTurnDegree = 10; // 각도 오차

	m_dDelTheta = 0.;
	m_nLeftWheelVel = m_nRightWheelVel =0;
	m_fRVelLimit = M_PI; 

	m_dDistBetweenWheel = 480;
	m_dRadiusofRobot = 260/10; //--> mm to cm //로봇 반경
	m_bFirstMove = true;
	m_dCosData=NULL;
	m_dSinData=NULL;
	m_nMaxDiffRotate=180;
	m_nVelocityRange=(int)DWA_SIZE/2.0;
	m_nMinVelocity=10;

	m_dXoffset=23;//cm
	m_dMaxDistance=2000;//cm
	m_dMinDistance=5;//cm
	m_dEmergencyDistance=35;//cm
	m_nIdX=181;
	m_ditratetime=0.1;//sec
	m_dRotateVelLimite=(2.0*DWA_VELLIMIT)/(m_dDistBetweenWheel/10.0);
}

/**
@brief Korean: 각도 값을 look-up tablbe로 작성하는 함수
@brief English: 
*/
void KuDWAPr::InitTheta( )
{
	if(m_dCosData==NULL)
		m_dCosData = new double[m_nIdX];
	if(m_dSinData==NULL)
		m_dSinData = new double[m_nIdX];

	for(int i = 0; i <m_nIdX; i++){
	double	dAngleRad = (i-(int)(m_nIdX/2.0))*D2R;
		m_dCosData[i]=cos(dAngleRad);
		m_dSinData[i]=sin(dAngleRad);

	}
}
/**
@brief Korean: 재 시작하는 경우 초기화 해주는 부분
@brief English: 
*/
void KuDWAPr::Init()
{
	m_dDelTheta = 0.;
	m_nLeftWheelVel = m_nRightWheelVel =0;
	m_bFirstMove = true;
}

/**
@brief Korean: 로봇의 Parameter를 설정하는 함수
@brief English: 
*/
void KuDWAPr::setRobotParameter(double  dRadiusofRobot,double  dDistBetweenWheel)
{
	m_dRadiusofRobot=dRadiusofRobot/10.0;
	m_dDistBetweenWheel=dDistBetweenWheel;
}
/**
@brief Korean: DWA의 Weight를 설정하는 함수
@brief English: 
*/
void KuDWAPr::setWeightParameter(double dAlphaH,double dAlphaC,double dAlphaV )
{
	m_dAlphaH = dAlphaH;
	m_dAlphaC =dAlphaC;
	m_dAlphaV = dAlphaV;
}
/**
@brief Korean: 센서 파라미터를 받아오는 함수
@brief English: 
*/
void KuDWAPr::setSensorParameter(int nIdX, double dXoffset,double dMaxDistance,double dMinDistance )
{
	if(INDEX!=nIdX) printf("You must change index parameter");
	m_nIdX=nIdX;
	m_dXoffset= dXoffset/10.;
	m_dMaxDistance=dMaxDistance/10.;
	m_dMinDistance = dMinDistance/10.;
}

/**
@brief Korean: 목적지를 설정하는 함수
@brief English: 
*/
void KuDWAPr::setGoalPos(KuPose GoalPos)
{
	m_GoalPos = GoalPos;
}

/**
@brief Korean: 거리 값을 이용하여 장애물 영역을 만드는 함수
@brief English: 
*/
bool KuDWAPr::generateObstacleModel(int_1DArray nSensorData)
{
	int nDistance = 0;
	double dEmergencyDist=30;
	memset(m_dObstacleX,0,sizeof(m_dObstacleX));
	memset(m_dObstacleY,0,sizeof(m_dObstacleY));

	for(int i = 0; i <m_nIdX; i++){
	
		nDistance = nSensorData[i]/10; //스캐너에서 받아오는 값의 단위는 mm단위이다. //mm->cm

		m_dObstacleX[i] = ((double)nDistance * m_dCosData[i] )+m_dXoffset;
		m_dObstacleY[i] = ((double)nDistance * m_dSinData[i]);	
	
		 if(nDistance<=m_dMinDistance){

			m_dObstacleX[i] =10000;
			m_dObstacleY[i] =10000;
						
			continue;		
		}

		 if(nDistance<dEmergencyDist&&abs(i-90)<atan2(m_dRadiusofRobot,dEmergencyDist))
		 {
			 return true;
		 }
		 else if(nDistance<dEmergencyDist*(double)(abs(i-90)/90.0))
		 {
			 return true;
		 }

	}
	return false;
}	

/**
@brief Korean: 왼쪽 바퀴, 오른쪽 바퀴 속도를 이용하여 Dynamic Window를 작성하는 함수
@brief English: 
*/
void KuDWAPr::generateDynamicWinodw(int nVelLeftWheel,int nVelRightWheel)
{
	memset(m_DWAData,0,sizeof(m_DWAData));

	int nWindow1 = 0;
	int nWindow2 = 0;
	int nTempRVel = 0;
	int nTempLVel = 0;
	double dVelocity = 0.;
	double dAngularVel = 0.;

	int nMaxLeftWheelVelocity = 0;
	int nMinLeftWheelVelocity = 0;
	int nMaxRightWheelVelocity = 0;
	int nMinRightWheelVelocity = 0;	

	nMaxLeftWheelVelocity = (int)(nVelLeftWheel + m_nVelocityRange);
	nMinLeftWheelVelocity = (int)(nVelLeftWheel - m_nVelocityRange);

	nMaxRightWheelVelocity = (int)(nVelRightWheel + m_nVelocityRange);
	nMinRightWheelVelocity = (int)(nVelRightWheel - m_nVelocityRange);
		
	for(int i = nMinRightWheelVelocity; i <= nMaxRightWheelVelocity ; i++,	nWindow1++){
		for(int j = nMinLeftWheelVelocity,nWindow2 = 0; j<= nMaxLeftWheelVelocity; j++,	nWindow2++){

		if(i >= DWA_VELLIMIT) nTempRVel = DWA_VELLIMIT;
		else if(i <= -DWA_VELLIMIT) nTempRVel =- DWA_VELLIMIT;
		else nTempRVel=i;
	
		if(j >= DWA_VELLIMIT) nTempLVel = DWA_VELLIMIT;
		else if(j <= -DWA_VELLIMIT) nTempLVel = -DWA_VELLIMIT;
		else  nTempLVel=j;
		
		if(m_nMinVelocity>abs(nTempRVel)+abs(nTempLVel))
		{
			nTempRVel=-1000;
			nTempLVel=-1000;
		}
		else if((nTempRVel*nTempLVel)>0&&nTempLVel<0)
		{
			nTempRVel=-1000;
			nTempLVel=-1000;
		}
		m_DWAData[nWindow1][nWindow2].SetRightWheelVel(nTempRVel);
		m_DWAData[nWindow1][nWindow2].SetLeftWheelVel(nTempLVel);
		}
	}
}

/**
@brief Korean: 로봇의 속도에 대한 속도 가중치를 계산하는 함수
@brief English: 
*/
void KuDWAPr::calcSpeedObject(bool bSpeedCostflag)
{
	double dVelForward;
	double dRVelRad;

	for(int i=0; i<DWA_SIZE ; i++){
		for (int j=0; j<DWA_SIZE;j++){
 			dVelForward=(m_DWAData[i][j].GetRightWheelVel() + m_DWAData[i][j].GetLeftWheelVel() ) / 2.;
 			dRVelRad = (m_DWAData[i][j].GetRightWheelVel()- m_DWAData[i][j].GetLeftWheelVel() ) / (m_dDistBetweenWheel/10.);
  			double dVelWeight=( (dVelForward+DWA_VELLIMIT)/((double)DWA_VELLIMIT*2.0)
  				+(fabs(dRVelRad)+(m_dRotateVelLimite))/(2.0*m_dRotateVelLimite))/2.0;

		//	m_DWAData[i][j].SetVel(((double)DWA_VELLIMIT+(dVelForward) )/((double)2.0*DWA_VELLIMIT));

		//double dVelWeight=(abs(m_DWAData[i][j].GetRightWheelVel()) + abs(m_DWAData[i][j].GetLeftWheelVel()))/((double)DWA_VELLIMIT*2.0);
			m_DWAData[i][j].SetVel(dVelWeight);
		//	m_DWAData[i][j].SetVel(dVelForward);
	
		}
	}

}
/**
@brief Korean: 로봇을 회전시키기 위한 속도 가중치를 결정하는 함수
@brief English: 
*/
void KuDWAPr::calcSpeedObjectTurn()
{
	double dVelForward;
	for(int i=0; i<DWA_SIZE ; i++){
		for (int j=0; j<DWA_SIZE;j++){
			dVelForward=(m_DWAData[i][j].GetRightWheelVel() + m_DWAData[i][j].GetLeftWheelVel() ) / 2.;
			m_DWAData[i][j].SetVel(((double)DWA_VELLIMIT-fabs(dVelForward) )/(double)2.0*DWA_VELLIMIT);
		}
	}

}

/**
@brief Korean: 목적지를 향하기 위한 heading 비용 계산하는 함수
@brief English: 
*/
void KuDWAPr::calcHeadingObject(double dGoalX,double dGoalY,double dRobotX, double dRobotY,double dRobotT)
{
	for(int i=0; i<DWA_SIZE ; i++){
		for(int j=0 ; j <DWA_SIZE ; j++){

			m_DWAData[i][j].SetHeading(0);
			m_dDelTheta = calcPredictedRadBetweenRobotandGoal( m_DWAData[i][j].GetRightWheelVel(), m_DWAData[i][j].GetLeftWheelVel(),dGoalX,dGoalY,dRobotX,dRobotY,dRobotT);

			if(m_dDelTheta >= M_PI) m_dDelTheta -= 2*M_PI;
			else if(m_dDelTheta < -M_PI) m_dDelTheta += 2*M_PI;
	
			m_DWAData[i][j].SetHeading( 1 - fabs(m_dDelTheta)/M_PI ) ; 

		}
	}
}

/**
@brief Korean: 목적지를 향하기 위한 theta 값을 계산하는 함수
@brief English: 
*/
double KuDWAPr::calcPredictedRadBetweenRobotandGoal(int nRightWheelVel, int nLeftWheelVel,double dGoalPosX,double dGoalPosY,double dRobotPosX,double dRobotPosY,double dRobotPosT)
{ 
	double dPredicRobotPos[3]; //x,y,t

	//predict Pose
	double dPredictedIC=0.; //instantaneous center of rotation(IC로 칭함 보통)
	double dPredictedLeftWheelMovedDist=0.; //정지할 때까지의 예측된 왼쪽 바퀴의 이동거리  
	double dPredictedRightWheelMovedDist=0.; //정지할 때까지의 예측된 오른쪽 바퀴의 이동거리  
	double dPredictedRobotMovedDist=0.; //예측된 로봇의 이동거리 
	double dPredictedRobotThetaRad=0.; //예측된 로봇의 각도 
	double dPredictedLeftVelStopTime; 
	double dPredictedRightVelStopTime;
	double dPredictedDiffRad ;

	//좌륜 및 우륜 정지시간 계산
	dPredictedLeftVelStopTime = (fabs((double)nLeftWheelVel)) / (double)DWA_LEFT_ACCELATION; // acc unit cm/s2
	dPredictedRightVelStopTime =(fabs((double)nRightWheelVel)) / (double)DWA_RIGHT_ACCELATION; //  acc unit cm/s2

	dPredictedLeftWheelMovedDist = (double)nLeftWheelVel *dPredictedLeftVelStopTime; 
	dPredictedRightWheelMovedDist = (double)nRightWheelVel*dPredictedRightVelStopTime;
	//---------------------------------------------------------------------


	dPredictedRobotMovedDist = (dPredictedLeftWheelMovedDist + dPredictedRightWheelMovedDist) / 2.;
	dPredictedRobotThetaRad = (dPredictedRightWheelMovedDist - dPredictedLeftWheelMovedDist) / (m_dDistBetweenWheel/10.); 
	
	if(dPredictedRobotThetaRad >= M_PI ) dPredictedRobotThetaRad -= 2*M_PI;
	else if(dPredictedRobotThetaRad < -M_PI) dPredictedRobotThetaRad += 2*M_PI;

	dPredicRobotPos[0] = dRobotPosX + dPredictedRobotMovedDist * cos(dPredictedRobotThetaRad/2.0+dRobotPosT);
	dPredicRobotPos[1] = dRobotPosY + dPredictedRobotMovedDist * sin(dPredictedRobotThetaRad/2.0+dRobotPosT);  
	dPredicRobotPos[2] = dRobotPosT + dPredictedRobotThetaRad;

	dPredictedDiffRad = atan2((dGoalPosY - dPredicRobotPos[1] ),(dGoalPosX - dPredicRobotPos[0] )) - dPredicRobotPos[2];
		
	return  dPredictedDiffRad;  

}
/**
@brief Korean: 장애물 회피를 위한 비용 계산함수
@brief English: 
*/
double KuDWAPr::calcMinCollisionTime(double dTimeToCollisionSet[INDEX])
{
	double dMinCollisionTime = INFINITY_VALUE;

	for(int i=0; i<m_nIdX; i++){
		if(dMinCollisionTime > dTimeToCollisionSet[i]){
			dMinCollisionTime = dTimeToCollisionSet[i];
		}
	}
	return dMinCollisionTime;
}
/**
@brief Korean: 장애물 회피를 위한 비용 계산함수
@brief English: 
*/
void KuDWAPr::calcClearanceObject()
{
	double dTVel = 0.; //병진속도
	double dRVelRad = 0.; //회전속도, 여기서는rad
	double dThetaRad0=0.;
	double dThetaRad=0.;
	double dIC=0.;
	double dTimeToStop=0.; 
	double dMaxTimeToStop =(double)DWA_VELLIMIT/(double)max(DWA_RIGHT_ACCELATION,DWA_LEFT_ACCELATION);
	double dTimetoStopofRightWheel=0;
	double dTimetoStopofLeftWheel=0;;

	double * dTimeToCollisionSet;
	dTimeToCollisionSet = new double [m_nIdX];

	for(int i=0;i<m_nIdX;i++){
		dTimeToCollisionSet[i] = INFINITY_VALUE;
	}

	double dTest =0;
	double dTimeToCollision=0.;

	double  dSafeDistance=0;
	for(int i=0; i<DWA_SIZE; i++){
		for(int j=0; j<DWA_SIZE; j++){

			m_DWAData[i][j].SetClearance(0); //초기화
			if(m_DWAData[i][j].GetRightWheelVel() ==-1000) continue;

			dTVel = ( m_DWAData[i][j].GetRightWheelVel() + m_DWAData[i][j].GetLeftWheelVel() ) / 2.;
			dRVelRad = (m_DWAData[i][j].GetRightWheelVel()- m_DWAData[i][j].GetLeftWheelVel() ) / (m_dDistBetweenWheel/10.);

			if(dTVel<0) continue;

			dTimetoStopofRightWheel=(double)abs(m_DWAData[i][j].GetRightWheelVel() ) /DWA_RIGHT_ACCELATION;;
			dTimetoStopofLeftWheel=(double)abs(m_DWAData[i][j].GetLeftWheelVel() ) /DWA_LEFT_ACCELATION ;

			dTimeToStop = max(dTimetoStopofRightWheel,dTimetoStopofLeftWheel);			
			 dSafeDistance=0;
			if(0==dRVelRad){ //병진 운동만 하는경우이다. 
				for(int k=0; k<m_nIdX ; k++){
					if(fabs(m_dObstacleY[k]) > m_dRadiusofRobot+SAFE_DISTANCE) continue; 
					else if(fabs(m_dObstacleY[k]) < (m_dRadiusofRobot+SAFE_DISTANCE)) dSafeDistance=SAFE_DISTANCE;
					else dSafeDistance=0;
					double dCollisonDist = m_dObstacleX[k] -  sqrt(pow(m_dRadiusofRobot+dSafeDistance ,2) - pow(m_dObstacleY[k],2));
					dTimeToCollisionSet[k] = dCollisonDist /dTVel;
				}
			}				
			else {
				dIC = dTVel/dRVelRad; //IC를 구하는 식.
				for(int k = 0; k<m_nIdX; k++){
					double dCollisonDist = sqrt(pow(m_dObstacleX[k],2) + pow(m_dObstacleY[k]-dIC,2) );
		
				//	if(sqrt(pow(m_dObstacleX[k],2) + pow(m_dObstacleY[k],2))  < (m_dRadiusofRobot+m_nSafeDistance)){
					if(fabs(m_dObstacleY[k]) < (m_dRadiusofRobot+SAFE_DISTANCE)){
						double r = double(rand())/double(RAND_MAX);
						dSafeDistance=SAFE_DISTANCE*r; 
					}
					else  dSafeDistance=0;					
					if( dCollisonDist > fabs(fabs(dIC)-(m_dRadiusofRobot+dSafeDistance)) && dCollisonDist < (fabs(dIC) +( m_dRadiusofRobot+dSafeDistance)) ){
						dThetaRad0 =M_PI/2 + dIC/fabs(dIC)*atan2( m_dObstacleY[k] - dIC, m_dObstacleX[k]);
						double temp=(dCollisonDist*dCollisonDist+ dIC*dIC - (m_dRadiusofRobot+dSafeDistance)*(m_dRadiusofRobot+dSafeDistance) )/(2.*fabs(dIC)*dCollisonDist);
						if(temp < -1 )	temp =-1; //예외처리 acos()의 범위는 -1~1사이임.
						else if(temp >1)  temp =1;
						dThetaRad = dThetaRad0 - acos(temp);
						dTimeToCollisionSet[k] = fabs(dThetaRad / dRVelRad);
					}					
				}
			}

			//---------------------------------------------------------------------------------------------------------
			dTimeToCollision = calcMinCollisionTime(dTimeToCollisionSet);

			if(dTimeToCollision <= dTimeToStop){ 
				m_DWAData[i][j].SetClearance(0);  //collision 
			}
			else if( dTimeToCollision < dMaxTimeToStop){ 
				double dClearancVal = (dTimeToCollision-dTimeToStop) / (dMaxTimeToStop-dTimeToStop);
			//	double  dVelWeight=(dTVel/DWA_VELLIMIT+fabs(dRVelRad)/((2.0*DWA_VELLIMIT)/m_dDistBetweenWheel/10.0))/2.0;
 				double dVelWeight=( (dTVel+DWA_VELLIMIT)/((double)DWA_VELLIMIT*2.0)
 					+(fabs(dRVelRad)+(m_dRotateVelLimite))/(2.0*m_dRotateVelLimite))/2.0;
				m_DWAData[i][j].SetClearance(dClearancVal*dVelWeight);	
//				m_DWAData[i][j].SetClearance(dClearancVal);	
			}
			else{		
				double dVelWeight=( (dTVel+DWA_VELLIMIT)/((double)DWA_VELLIMIT*2.0)
					+(fabs(dRVelRad)+(m_dRotateVelLimite))/(2.0*m_dRotateVelLimite))/2.0;				
				m_DWAData[i][j].SetClearance(1*dVelWeight); 	
//				m_DWAData[i][j].SetClearance(1); 
			}

			for(int k=0;k<m_nIdX;k++){
				dTimeToCollisionSet[k] = INFINITY_VALUE;
			}
		} //end of for i	
	}//end of for j
	delete  [] dTimeToCollisionSet;
}

/**
@brief Korean: 모든 비용 합중  가중치가 가장 큰 속도를 선택하는 함수
@brief English: 
*/

void KuDWAPr::calcObjectiveFunc(int *nLeftWheelVel,int *nRightWheelVel)
{
	int max_i = -1;
	int max_j = -1;
	double dCalW =0.;
	double dmaxWeight=0.;
	double dClear=0;
	for(int i=0 ; i<DWA_SIZE ; i++){
		for(int j=0 ; j<DWA_SIZE ; j++){
			dCalW = m_dAlphaH * m_DWAData[i][j].GetHeading() + 
				m_dAlphaV * m_DWAData[i][j].GetVel()+ 
				m_dAlphaC * m_DWAData[i][j].GetClearance();

			m_DWAData[i][j].SetW(dCalW);

			if(dmaxWeight < dCalW/*&&m_DWAData[i][j].GetClearance()>=dClear*/&&m_DWAData[i][j].GetClearance()!=0){
				dClear=m_DWAData[i][j].GetClearance();
				dmaxWeight = dCalW;
				max_i = i;
				max_j = j;
			}			
		}
	}

	if(max_i==-1&&max_j==-1)
	{
		double dRVel = (m_nRightWheelVel*10 - m_nLeftWheelVel*10) / m_dDistBetweenWheel;
	
		if(fabs(dRVel*R2D)>=5)
		{
			*nLeftWheelVel=m_nLeftWheelVel;
			*nRightWheelVel=m_nRightWheelVel;
		}
		else
		{
			*nLeftWheelVel=0;
			*nRightWheelVel=0;
		}
	}
	else
	{

	*nLeftWheelVel = m_DWAData[max_i][max_j].GetLeftWheelVel();
	*nRightWheelVel =m_DWAData[max_i][max_j].GetRightWheelVel();

	}
}



/**
@brief Korean: 로봇의 위치가 목적지 경계선에 있는지 검사하는 함수
@brief English: 
*/
bool KuDWAPr::checkGoalPos(double cur_posX,double cur_posY)
{
	double dGoalX = m_GoalPos.getX()/10;
	double dGoalY = m_GoalPos.getY()/10;

	double dDifferX = pow(cur_posX - dGoalX,2);
	double dDifferY = pow(cur_posY - dGoalY,2);

	if( dDifferX+dDifferY <= pow(((double)m_GoalArea/10.) ,2) ){
		return true;
	}
	else{
		return false;
	}

}

/**
@brief Korean: 로봇의 위치가 목적지 위치의 각도 오차가 큰경우를 판별하는 함수
@brief English: 
*/
bool KuDWAPr::isAngleDifferenceLarge(double dWayPointX, double dWayPointY, double dRobotX, double dRobotY, double dRobotThetaRad)
{
	double dThetaRad=0.;
	dThetaRad = atan2(dWayPointY - dRobotY ,dWayPointX - dRobotX) - dRobotThetaRad;
	if(dThetaRad > M_PI) dThetaRad -= 2*M_PI;
	else if(dThetaRad < -M_PI) dThetaRad += 2*M_PI;
	static bool bcompleteTurn=false;
	if((fabs(dThetaRad) > D2R*m_nMaxDiffRotate )&&bcompleteTurn)
	{
		Init();
		bcompleteTurn=false;
		return true;
	}
 else	if ( (fabs(dThetaRad) > D2R*m_dTurnDegree ) ){
		return true;
	}
	else{
			 bcompleteTurn=true;
		return false;
	}		
}

/**
 @brief Korean: 타겟지점까지의  최적의  v,w를 제공하는 함수.
 @brief English: 
*/
KuDWAVelocity KuDWAPr::generateTRVelocity(KuPose TargetPos, KuPose RobotPos, int_1DArray  nSensorData )
{
	LARGE_INTEGER present;		
	startTimeCheck(present);

	KuDWAVelocity generatedVel;

	double dRobotPosX = RobotPos.getX()/10; //mm->cm
	double dRobotPosY = RobotPos.getY()/10;// mm->cm
	double dRobotPosT = RobotPos.getThetaRad(); //degree 단위를 radian으로 바꿔준다.

	double dTempGoalPosX = TargetPos.getX()/10;
	double dTempGoalPosY = TargetPos.getY()/10;  

	if(generateObstacleModel(nSensorData))
	{
		generatedVel.m_nTranslationVel = 0;
		generatedVel.m_nRotationDegVel =0;
		m_nLeftWheelVel = m_nRightWheelVel =0;
		return generatedVel;
	}

	generateDynamicWinodw( m_nLeftWheelVel, m_nRightWheelVel );


	if(true==isAngleDifferenceLarge(dTempGoalPosX, dTempGoalPosY, dRobotPosX, dRobotPosY, dRobotPosT) && true==m_bFirstMove ){
		calcSpeedObjectTurn();		
	}else{
		m_bFirstMove = false;		
		bool bSpeedCostflag= checkGoalPos(dRobotPosX,dRobotPosY);
		calcSpeedObject(bSpeedCostflag);
	}

	calcHeadingObject(dTempGoalPosX,dTempGoalPosY,dRobotPosX,dRobotPosY,dRobotPosT); //cal heading Object
	calcClearanceObject();	
	calcObjectiveFunc(&m_nLeftWheelVel,&m_nRightWheelVel); // DWA decides forward and angualr velocity

	if(fabs((double)m_nLeftWheelVel)<1&&fabs((double)m_nRightWheelVel)<1)
	{
		generatedVel.m_nTranslationVel = 0;
		generatedVel.m_nRotationDegVel =0;
	}
	else{
		double dTVel = ( m_nLeftWheelVel*10 + m_nRightWheelVel*10) /2.;
		double dRVel = (m_nRightWheelVel*10 - m_nLeftWheelVel*10) / m_dDistBetweenWheel;

		if(dRVel>MAX_ROTVEL*D2R)dRVel=MAX_ROTVEL*D2R;
		else if(dRVel<-MAX_ROTVEL*D2R) dRVel=-MAX_ROTVEL*D2R;

		double dTransVelLimite=DWA_VELLIMIT*10.0;

		if(dTVel>MAX_TRANSVEL)dTVel=MAX_TRANSVEL;
		else if(dTVel<-MAX_TRANSVEL) dTVel=-MAX_TRANSVEL;

		m_nRightWheelVel=(2.0*dTVel+m_dDistBetweenWheel*dRVel)/20.0;
		m_nLeftWheelVel=(2.0*dTVel-m_dDistBetweenWheel*dRVel)/20.0;

		dRVel = dRVel*R2D;

		generatedVel.m_nTranslationVel = dTVel;
		generatedVel.m_nRotationDegVel =dRVel;

		if(m_nLeftWheelVel>100)m_nLeftWheelVel=100;
		else if(m_nLeftWheelVel<-100)m_nLeftWheelVel=-100;
		if(m_nRightWheelVel>100)m_nRightWheelVel=100;
		else if(m_nRightWheelVel<-100)m_nRightWheelVel=-100;
	}
	m_ditratetime =(double) finishTimeCheck(present)/1000;
	printf("elapsed=%f\n",m_ditratetime);
	return generatedVel; 
}
void KuDWAPr::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}

float KuDWAPr::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}

