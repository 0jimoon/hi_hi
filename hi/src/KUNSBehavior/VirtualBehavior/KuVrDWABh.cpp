#include "StdAfx.h"
#include "KuVrDWABh.h"

KuVrDWABh::KuVrDWABh()
{
	//initialize(); //초기화 작업
	m_vecPath.clear();
	m_listPath.clear();
	m_vecWayPoint.clear();
	m_bThreadFlag = false;
	m_nPrePathIndx=-1;
	m_nLocalGoalIndex=3;
	m_nLocalGoalWayPointIndex=-1;
	m_pLocalMap=NULL;
	m_pOriginalMap=NULL;
	m_pMap= NULL;
		m_nMap=NULL;
	cout<<"[KuVrDWABh]: Instance is created!!!"<<endl;
}

KuVrDWABh::~KuVrDWABh()
{
	if(m_pLocalMap!=NULL)
	{
		delete [] m_pLocalMap;
		m_pLocalMap=NULL;
	}
	if(m_pOriginalMap!=NULL)
	{
		delete [] m_pOriginalMap;
		m_pOriginalMap=NULL;
	}
	if(m_pMap!=NULL)
	{
		delete [] m_pMap;
		m_pMap=NULL;
	}


	for(int i = 0 ; i < m_nMapX ; i++){
		delete[] m_nMap[i];
		m_nMap[i] = NULL;
	}
	delete[] m_nMap;
	cout<<"[KuVrDWABh]: Instance is destroyed!!!"<<endl;
}

/**
 @brief Korean: 초기화 작업을 수행하는 함수.
 @brief English: 
*/
bool KuVrDWABh::initialize(KuCommandMessage CMessage)
{
	m_bThreadFlag = true;
	m_vecPath.clear();
	m_vecLocalPath.clear();
	m_nGoalArea=500;
	m_nDistToTarget=3000;
	m_nMode=-1;

	m_nThreadFuncPeriod = CMessage.getBehaviorPeriod(); //스레드 함수 실행주기 입력

	m_nKinectRnageData=m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,0);
		
	initMapbuildingProcess(true);	
	if(!initPathplanningProcess()) {m_bThreadFlag = false; return false;}	
	KuObstAvoidancePr::getInstance()->setPath(m_listPath);
	bool bflag= KuObstAvoidancePr::getInstance()->start(20);
	KuDrawingInfo::getInstance()->setPath(m_listPath); //경로 화면에 표시

	m_KuVrWheelActuator.setRobotPos( m_RobotPos );

// 	m_KuDWA.setGoalPos( m_GoalPos);
// 	m_KuDWA.Init();
	
	initKanayamaProcess();
//	Sleep(1000);

	printf("!!KuVrDWABh initialize\n");

	return bflag;
}

/**
 @brief Korean: 현재 Behavior의 상태를 나타내는 함수
 @brief English: 
*/
bool KuVrDWABh::getBehaviorStates()
{
	return m_bThreadFlag;
}

void KuVrDWABh::calStartPathPoint( )
{
	KuPose RobotPos=KuDrawingInfo::getInstance()->getRobotPos();
	int nDistToTarget =KuRobotParameter::getInstance()->getTargetDistance();

	int  nMinIdx=-1;
	double dMinPathX = 0.0;
	double dMinPathY = 0.0;
	double dDist=0;
	bool bcheck= false;

	//로봇에서 부터의 최소가 되는 지점 및 거리값
	while( true ) {
		nMinIdx++;
		if(nMinIdx >= m_vecPath.size()) {
			nMinIdx = m_vecPath.size(); //배열인덱스 이기 떄문에 1을 빼줘야 한다.	
			break;
		}
		double dMinPathX = m_vecPath[nMinIdx].getX();
		double dMinPathY = m_vecPath[nMinIdx].getY();

		dDist=hypot(RobotPos.getX()-dMinPathX, RobotPos.getY()- dMinPathY);

		if(dDist>nDistToTarget&& bcheck== true)
		{
			m_nPrePathIndx = nMinIdx;
			m_nSelectedMinIndx = nMinIdx;
			break;
		}	
		else if(dDist<nDistToTarget)
		{
			bcheck= true;
		}
	}
	if(nMinIdx==-1) {
		m_nPrePathIndx = m_vecPath.size()-1;
		m_nSelectedMinIndx = m_vecPath.size()-1;
	}

	int nWayPointSize = m_vecWayPoint.size();
	double dWayPointX=0.0;
	double dWayPointY=0.0;

	for(int nPathIdx=m_nSelectedMinIndx; nPathIdx>1;nPathIdx--)
	{
		for(int nWayPointIdx=1; nWayPointIdx<nWayPointSize;nWayPointIdx++)
		{
			dWayPointX = m_vecWayPoint[nWayPointIdx].getX();
			dWayPointY = m_vecWayPoint[nWayPointIdx].getY();

			dDist=hypot(m_vecPath[nPathIdx].getX()-dWayPointX, m_vecPath[nPathIdx].getY()- dWayPointY);

			if(dDist < 500&&m_vecWayPoint[nWayPointIdx].getID()!=1)
			{
				m_vecWayPoint[nWayPointIdx].setID(1);
			}	
		}
	}
}
/**
@brief Korean: 
@brief English: 
*/
bool KuVrDWABh::initPathplanningProcess()
{
	//path planning start-----------------------------------------------------------------------------
	{
		m_RobotPos =KuDrawingInfo::getInstance()->getRobotPos();
		m_GoalPos = KuDrawingInfo::getInstance()->getGoalPos();
		m_KuPathPlanner.initializeMapSize(kuMapRepository::getInstance()->getMap()->getX(),
			kuMapRepository::getInstance()->getMap()->getY()	);
		m_KuPathPlanner.setMap(kuMapRepository::getInstance()->getMap()->getMap(),Sensor::CELLSIZE); 
		m_KuPathPlanner.initIntCost((int)(KuRobotParameter::getInstance()->getRobotRadius()/((double)100.0))+1 );
		m_KuPathPlanner.generatePath(m_GoalPos, m_RobotPos); //경로 생성
		m_listPath = m_KuPathPlanner.getPath();
		if(m_listPath.size()==0){
			m_bThreadFlag = false;	
			return false; 
		}//경로생성 실패의 경우
		m_listPath=m_KuPathSmoothing.smoothingPath(m_listPath);
	}

	//path planning  end-----------------------------------------------------------------------------
	list<KuPose>::iterator it;
	for(it=m_listPath.begin(); it!=m_listPath.end(); it++){
		m_vecPath.push_back(*it);
	}

	calStartPathPoint();

	//Goal Drawing start-----------------------------------------------------------------------------
	int nPathSize = m_vecPath.size()-1;
	double dPathX = m_vecPath[nPathSize].getX();
	double dPathY = m_vecPath[nPathSize].getY();
	m_GoalPos.setX(dPathX);
	m_GoalPos.setY(dPathY);
	KuDrawingInfo::getInstance()->setGoalPos(m_GoalPos);
	//Goal Drawing start-----------------------------------------------------------------------------


	//경로생성 완료==================================================================
	m_RobotPos =KuDrawingInfo::getInstance()->getRobotPos();
	KuDrawingInfo::getInstance()->setPath(m_listPath); //경로 화면에 표시

	return true; 
}
/**
@brief Korean: 
@brief English: 
*/
void KuVrDWABh::initMapbuildingProcess(bool bLocalMapflag)
{

	m_pRefMap = kuMapRepository::getInstance()->getMap(); 
	m_nMapSizeX =m_nGlobalMapX=m_nBuildingMapX=m_pRefMap->getX();
	m_nMapSizeY =m_nGlobalMapY=m_nBuildingMapY=m_pRefMap->getY();

	if(m_nMap==NULL)
	{

		m_nMapX=m_pRefMap->getX();
		m_nMapY=m_pRefMap->getY();

		m_nMap = new int*[m_nMapX];
		for(int i = 0 ; i < m_nMapX ; i++){
			m_nMap[i] = new int[m_nMapY];
		}
	}

	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			m_nMap[i][j] = m_pRefMap->getMap()[i][j];
		}
	}

	KuBuildingPOMapParameter InputParam;

	string strProMapNameNPath = KuRobotParameter::getInstance()->getProMapNameNPath();
	
	InputParam.setPath(strProMapNameNPath);

	InputParam.setMapSizeXmYm(m_nMapSizeX/10.0, m_nMapSizeY/10.0);

	m_PBMPr.start(InputParam);
	//KuDrawingInfo::getInstance()->setProObBuildingMap(m_PBMPr.getProMap());

}

/**
 @brief Korean: 스레드로 돌아가는 함수
 @brief English: 
*/
/*
void KuVrDWABh::doThread_ROS(void* arg)
{
	KuVrDWABh* pVDBh = (KuVrDWABh*)arg;

	LARGE_INTEGER present1;		
	double dtotalelapsed =0.0;
	pVDBh->startTimeCheck(present1);
	
	double dTargetDist=0;

	KuDWAVelocity_ROS generatedVel_ROS;

	pVDBh->m_RobotPos = pVDBh->m_KuVrWheelActuator.getRobotPos();
	
	pVDBh->checkVirtualObstacle(pVDBh->m_pMap);

	pVDBh->m_nLaserData = KuVrHokuyoUTM30LXInterface::getInstance()->getData181(pVDBh->m_RobotPos);//로봇의 위치에 따른 가상 맵에서 가상 레이저 데이터 받아옴

	if(pVDBh->m_math.calcDistBetweenPoses(pVDBh->m_GoalPos,pVDBh->m_RobotPos) < pVDBh->m_nGoalArea){
		pVDBh->terminate();
		return;
	}

	pVDBh->m_TargetPos = pVDBh->getTargetPos( pVDBh->m_RobotPos, pVDBh->m_nDistToTarget, &dTargetDist);


	generatedVel_ROS = pVDBh->m_KuDWA_ROS.generateTRVelocity( pVDBh->m_TargetPos,  pVDBh->m_RobotPos,   pVDBh->m_nLaserData   );


	pVDBh->m_KuVrWheelActuator.moveTRVelocity(generatedVel_ROS.m_nTranslationVel, generatedVel_ROS.m_nRotationDegVel);
	printf("T:%d, R:%d\n",generatedVel_ROS.m_nTranslationVel, generatedVel_ROS.m_nRotationDegVel);
	
	pVDBh->drawNaviData();

	dtotalelapsed =(double) pVDBh->finishTimeCheck(present1);
	printf("!!dtotalelapsed=%f\n",dtotalelapsed);

}*/
void KuVrDWABh::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}

float KuVrDWABh::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}

/**
@brief Korean: 
@brief English: 
*/
int_1DArray KuVrDWABh::combinateLaserNKinect(int_1DArray nLaserData181, int_1DArray nKinectRnageData)
{
	int nLaserMinDis=KuRobotParameter::getInstance()->getURG04LXLaserMinDist();
	int nKinectMinDis=KuRobotParameter::getInstance()->getKinectMinDist();

	int_1DArray nCombinateRangeData=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	int nStart=(Sensor::URG04LX_DATA_NUM181-Sensor::KINECT_SENSOR_FOV)/2.0;
	int nEnd=(Sensor::URG04LX_DATA_NUM181+Sensor::KINECT_SENSOR_FOV)/2.0;

	for(int i=0;i<Sensor::URG04LX_DATA_NUM181;i++)
	{
		nCombinateRangeData[i]=nLaserData181[i];

		if(i>nStart&&i<nEnd)
		{
			if(nLaserData181[i]>nKinectRnageData[i-nStart]&& nKinectRnageData[i-nStart]>nKinectMinDis)
			{
				nCombinateRangeData[i]=nKinectRnageData[i-nStart];
			}
		}
	}

	return nCombinateRangeData;
}



/**
 @brief Korean: 센서데이터를 이용하여 타겟점과 로봇 사이의 장애물 유무를 검사하는 함수
 @brief English: 
*/
bool KuVrDWABh::checkObstacles(KuPose RobotPos,KuPose TargetPos,int_1DArray nLaserData181, int_1DArray nKinectRnageData ,double dTargetDist)
{

	double dGradientX = TargetPos.getXm() - RobotPos.getXm();
	double dGradientY = TargetPos.getYm() - RobotPos.getYm();
	double dDistofObst=_hypot(TargetPos.getX()- RobotPos.getX(), TargetPos.getY()- RobotPos.getY());
	double dDistObstoRobot=dTargetDist;
	double dTempDistObstoRobot=dTargetDist;
	int nDistIndx=(int)dDistObstoRobot/100;
	double dKinectOffset= (double)KuRobotParameter::getInstance()->getKinectXOffset();
	double dLaserOffset= (double)KuRobotParameter::getInstance()->getURG04LXLaserXOffset();
	int nradiusofRobot=KuRobotParameter::getInstance()->getRobotRadius();

	for (int ndistance = nDistIndx; ndistance <dDistObstoRobot; ndistance += nDistIndx) {

		double dRayOfX =  RobotPos.getX()+ ndistance*dGradientX;
		double dRayOfY = RobotPos.getY()+ ndistance*dGradientY;

		dTempDistObstoRobot=_hypot(ndistance*dGradientX, ndistance*dGradientY);

		for(int i=0;i<Sensor::KINECT_SENSOR_FOV;i++){
			
			if(nLaserData181[i+66]<50)continue;
			if(nKinectRnageData[i] <50)continue;

			if((nKinectRnageData[i] != 1000000&&nKinectRnageData[i] >50)&&nKinectRnageData[i] <dDistofObst){
				double dAngleRad = (double)(i -  Sensor::KINECT_SENSOR_FOV/2) * D2R;
				double dX = RobotPos.getX() + ((double)nKinectRnageData[i] * cos(dAngleRad) + dKinectOffset ) * cos(RobotPos.getThetaRad()) + 
					((double)nKinectRnageData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());
				double dY = RobotPos.getY() + ((double)nKinectRnageData[i] * cos(dAngleRad) + dKinectOffset ) * sin(RobotPos.getThetaRad()) + 
					((double)nKinectRnageData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());
				
				double tempD=hypot(dRayOfX-dX, dRayOfY- dY);

				if(tempD<nradiusofRobot&&tempD>0 ){	return true;	}// 거리에거리에 따른 장애물 감지
			}
			else if((nLaserData181[i+66] != 1000000&&nLaserData181[i+66]>50)&&nLaserData181[i+66]<dDistofObst){
				double dAngleRad = (double)(i -  Sensor::KINECT_SENSOR_FOV/2) * D2R;
				double dX = RobotPos.getX() + ((double)nLaserData181[i+66]* cos(dAngleRad) + dLaserOffset ) * cos(RobotPos.getThetaRad()) + 
					((double)nLaserData181[i+66]* sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());
				double dY = RobotPos.getY() + ((double)nLaserData181[i+66] * cos(dAngleRad) + dLaserOffset ) * sin(RobotPos.getThetaRad()) + 
					((double)nLaserData181[i+66] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

				double tempD=hypot(dRayOfX-dX, dRayOfY- dY);

				if(tempD<nradiusofRobot&&tempD>0 ){	return true;	}// 거리에 따른 장애물 감지
			}
		}
	}	
	return false;
}
/**
@brief Korean: Kanayama  프로세스를 수행하기 위해 필요한 초기화 작업을 수행하는 함수.
@brief English: 
*/
void KuVrDWABh::initKanayamaProcess()
{
	//INI 파일로부터 kanayama motion controㅣ을 수행하기 위해 필요한 정보를 얻어온다.------------------------------------
	m_nDistToTarget = KuRobotParameter::getInstance()->getTargetDistance();
	m_nDesiredVel = KuRobotParameter::getInstance()->getDesiedVel();
	m_nGoalArea = KuRobotParameter::getInstance()->getGoalArea();
	double dKX = KuRobotParameter::getInstance()->getdKX();
	double dKY = KuRobotParameter::getInstance()->getdKY();
	double dKT = KuRobotParameter::getInstance()->getdKT();
	int nMaxTVel = KuRobotParameter::getInstance()->getMaxRobotVelocity();
	int nMinTVel = KuRobotParameter::getInstance()->getMinRobotVelocity();
	//==================================================================================================================
	m_KanayaMC.init();
	m_KanayaMC.setGain(dKX,dKY,dKT);	//게인 설정, 설정하지 않더라고 기본 게인값이 사용된다.
	m_KanayaMC.setMaxTRVel(nMaxTVel,30);
	m_KanayaMC.setMinTRVel(nMinTVel,0);
}
/**
 @brief Korean: 스레드로 돌아가는 함수
 @brief English: 
*/
void KuVrDWABh::doThread(void* arg)
{
	KuVrDWABh* pVDBh = (KuVrDWABh*)arg;
	LARGE_INTEGER present1;		
	double dtotalelapsed =0.0;
	pVDBh->startTimeCheck(present1);
	
	KuVelocity generatedVel;
	//double dTargetDist=0;

	pVDBh->m_RobotPos = pVDBh->m_KuVrWheelActuator.getRobotPos();
		
	pVDBh->checkVirtualObstacle(pVDBh->m_pRefMap);

	pVDBh->m_nLaserData = KuVrHokuyoUTM30LXInterface::getInstance()->getData181(pVDBh->m_RobotPos);//로봇의 위치에 따른 가상 맵에서 가상 레이저 데이터 받아옴

	pVDBh->m_PBMPr.setData( pVDBh->m_RobotPos, pVDBh->m_KuVrWheelActuator.getDelEncoderData(), pVDBh->m_nLaserData);

	if(pVDBh->m_math.calcDistBetweenPoses(pVDBh->m_GoalPos,pVDBh->m_RobotPos) < pVDBh->m_nGoalArea){
		pVDBh->terminate();
		return;
	}
	//pVDBh->m_TargetPos = pVDBh->getTargetPos( pVDBh->m_RobotPos, pVDBh->m_nDistToTarget, &dTargetDist);
	
	KuObstAvoidancePr::getInstance()->setData(pVDBh->m_RobotPos,pVDBh->m_DelEncoderData,pVDBh->m_nLaserData,pVDBh->m_nKinectRnageData);

	pVDBh->m_TargetPos=KuObstAvoidancePr::getInstance()->getTargetPos();

	if(pVDBh->m_TargetPos.getX()==0&&pVDBh->m_TargetPos.getY()==0)
	{
		return;
	}
	
	generatedVel = pVDBh->m_KanayaMC.generateTRVelocity(pVDBh->m_TargetPos, pVDBh->m_RobotPos, 100 );

	if(pVDBh->checkObstacles( pVDBh->m_RobotPos, pVDBh->m_TargetPos,pVDBh->m_nLaserData, 1000))
	{
// 		double dAngleDiff = atan2(pVDBh->m_TargetPos.getY()-pVDBh->m_RobotPos.getY(),pVDBh->m_TargetPos.getX()-pVDBh->m_RobotPos.getX()) - pVDBh->m_RobotPos.getThetaRad();
// 
// 		if(fabs(dAngleDiff*R2D)>5)
// 			pVDBh->m_KuVrWheelActuator.moveTRVelocity(0, generatedVel.m_nRotationDegVel);
// 		else
 			 pVDBh->m_KuVrWheelActuator.moveTRVelocity(0,0);
	}
	else
	{
		//	KuDWAVelocity generatedVel = pVDBh->m_KuDWA.generateTRVelocity(pVDBh->m_TargetPos, pVDBh->m_RobotPos, pVDBh->m_nLaserData  );
		pVDBh->m_KuVrWheelActuator.moveTRVelocity(generatedVel.m_nTranslationVel, generatedVel.m_nRotationDegVel);

	}

	pVDBh->drawNaviData();
	
	dtotalelapsed =(double) pVDBh->finishTimeCheck(present1);
	printf("!!dtotalelapsed=%f\n",dtotalelapsed);
}


/**
 @brief Korean: 센서데이터를 이용하여 타겟점과 로봇 사이의 장애물 유무를 검사하는 함수
 @brief English: 
*/
bool KuVrDWABh::checkObstacles(KuPose RobotPos,KuPose TargetPos,int_1DArray nLaserData181, double dTargetDist)
{

	double dGradientX = TargetPos.getXm() - RobotPos.getXm();
	double dGradientY = TargetPos.getYm() - RobotPos.getYm();
	double dDistofObst=hypot(TargetPos.getX()- RobotPos.getX(), TargetPos.getY()- RobotPos.getY());
	double dOffset= (double)KuRobotParameter::getInstance()->getURG04LXLaserXOffset();
	int nradiusofRobot=KuRobotParameter::getInstance()->getRobotRadius();
	int nDetectSenorrange= 181;
	int ninitSenorrange= (Sensor::URG04LX_DATA_NUM181 -nDetectSenorrange)/2;
	int_1DArray nLaserData=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

	for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++)
	{
		nLaserData[i]=nLaserData181[i];
	}

	for (int ndistance = 0; ndistance <dDistofObst; ndistance += nradiusofRobot) {

		double dRayOfX =  RobotPos.getX()+ ndistance*dGradientX;
		double dRayOfY = RobotPos.getY()+ ndistance*dGradientY;


		for(int i=0;i<nDetectSenorrange;i++){
			
			if(nLaserData181[i+ninitSenorrange]<30)continue;

		 if((nLaserData[i+ninitSenorrange] != 1000000||nLaserData[i+ninitSenorrange]>30)&&nLaserData[i+ninitSenorrange]<nradiusofRobot){
				double dAngleRad = (double)(i -  nDetectSenorrange/2) * D2R;
				double dX = RobotPos.getX() + ((double)nLaserData[i+ninitSenorrange]* cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
					((double)nLaserData[i+ninitSenorrange]* sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());
				double dY = RobotPos.getY() + ((double)nLaserData[i+ninitSenorrange] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
					((double)nLaserData[i+ninitSenorrange] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

				double tempD=hypot(dRayOfX-dX, dRayOfY- dY);

				if(tempD<nradiusofRobot&&tempD>0 ){	return true;	}// 거리에 따흔 장애물 감지
			}
		}
	}	
	return false;
}


/**
@brief Korean: 주행 관련 정보를 그려주는 함수.
@brief English: 
*/
void KuVrDWABh::drawNaviData()
{
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos); //로봇 위치 화면에 표시	
	KuDrawingInfo::getInstance()->setLaserData181(m_nLaserData);		
	KuDrawingInfo::getInstance()->setTargetPos(m_TargetPos); //타겟 위치 화면에 표시
	//KuDrawingInfo::getInstance()->setnMap(m_nMap,m_nMapX,m_nMapY);
	//KuDrawingInfo::getInstance()->setProObBuildingMap(m_PBMPr.getProMap());
}

// void KuVrDWABh::checkVirtualObstacle(KuMap* pMap)
// {
// 	KuPose VirtualObstaclePos = KuDrawingInfo::getInstance()->getVirtualObstaclePos();
// 
// 	int** nMap = pMap->getMap();
// 
// 	int nObsX = VirtualObstaclePos.getX()/100;
// 	int nObsY = VirtualObstaclePos.getY()/100;
// 	//if(nObsX==0 && nObsY==0) return;
// 
// 	int nSize=3;
// 	for(int i=nObsX-nSize; i<nObsX+nSize; i++ ){
// 		for(int j=nObsY-nSize; j<nObsY+nSize; j++){
// 			if(nObsX==0 && nObsY==0) continue;
// 			m_nMap[i][j] = KuMap::OCCUPIED_AREA;	
// 		}
// 	}
// 
// 	KuVrHokuyoUTM30LXInterface::getInstance()->connect( m_nMap );
// 
// 
// 	int nSize=3;
// 	for(int i=nObsX-nSize; i<nObsX+nSize; i++ ){
// 		for(int j=nObsY-nSize; j<nObsY+nSize; j++){
// 			if(nObsX==0 && nObsY==0) continue;
// 			m_nMap[i][j] = nMap[i][j];	
// 		}
// 	}
// }

void KuVrDWABh::checkVirtualObstacle(KuMap* pMap)
{
	//KuPose VirtualObstaclePos = KuDrawingInfo::getInstance()->getVirtualObstaclePos();

	vector<KuPose>  MultiObstaclePosVector; 
		
	 KuDrawingInfo::getInstance()->getMultiObstaclePosVector(&MultiObstaclePosVector);

	int** nMap = pMap->getMap();
	int nSize=3;

	for (int nPosNum=0;nPosNum<MultiObstaclePosVector.size();nPosNum++)
	{

		int nObsX = MultiObstaclePosVector[nPosNum].getX()/100;
		int nObsY =  MultiObstaclePosVector[nPosNum].getY()/100;

		for(int i=nObsX-nSize; i<nObsX+nSize; i++ ){
			for(int j=nObsY-nSize; j<nObsY+nSize; j++){
				if(nObsX==0 && nObsY==0) continue;
				m_nMap[i][j] = KuMap::OCCUPIED_AREA;	
			}
		}
	}

// 	int nObsX = VirtualObstaclePos.getX()/100;
// 	int nObsY = VirtualObstaclePos.getY()/100;
	//if(nObsX==0 && nObsY==0) return;


	KuVrHokuyoUTM30LXInterface::getInstance()->connect(kuMapRepository::getInstance()->getMap()->getMap(),
		kuMapRepository::getInstance()->getMap()->getX(),kuMapRepository::getInstance()->getMap()->getY());

	for (int nPosNum=0;nPosNum<MultiObstaclePosVector.size();nPosNum++)
	{

		int nObsX = MultiObstaclePosVector[nPosNum].getX()/100;
		int nObsY =  MultiObstaclePosVector[nPosNum].getY()/100;

		for(int i=nObsX-nSize; i<nObsX+nSize; i++ ){
			for(int j=nObsY-nSize; j<nObsY+nSize; j++){
				if(nObsX==0 && nObsY==0) continue;
					m_nMap[i][j] = nMap[i][j];	
			}
		}
	}
}

/**
 @brief Korean: 시뮬레이터를 종료한다.
 @brief English: 
*/
void KuVrDWABh::terminate()
{
	KuThread::terminate();
	KuObstAvoidancePr::getInstance()->terminate();
	m_KuVrWheelActuator.moveTRVelocity(0, 0);
	//KuDWABhSupervisor::getInstance()->notifyBehavirTermination();
	cout<<"[KuVrDWABh]:: Behavior is terminated!!!"<<endl;
	m_bThreadFlag = false;	
	m_PBMPr.terminate();
}

/**
 @brief Korean: 경로에서 타겟점을 선택하는 함수
 @brief English: 
*/
KuPose KuVrDWABh::getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist)
{
	int nPathInx=0;
	int nMinIdx=0;

	double dPathX; 
	double dPathY;
	double dMinPathX;
	double dMinPathY;

	double dMinDistfromPath=INFINITY_VALUE;
	double dDistMintoTarget=INFINITY_VALUE;
	double dDist=INFINITY_VALUE;
	double dnewDistToTarget=INFINITY_VALUE;


	int nselectedMinIndx=-1;
	int nselectedTargetIndx=-1;
	double  dMinTangentthetaDeg=0;
	double  dTargetTangentthetaDeg=0;

	int nPathSize = m_vecLocalPath.size();

	if(nPathSize<=1) return m_GoalPos;

	bool bselectedTargetflag= false;

	KuPose TargetPos,MinPos;


	//로봇에서 부터의 최소가 되는 지점 및 거리값
	while( true ) {

		nMinIdx++;
		if(nMinIdx >= nPathSize) {
			nMinIdx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
			break;
		}
		dMinPathX = m_vecLocalPath[nMinIdx].getX();
		dMinPathY = m_vecLocalPath[nMinIdx].getY();

		dDist=hypot(RobotPos.getX()-dMinPathX, RobotPos.getY()- dMinPathY);

		if(dDist< dMinDistfromPath)
		{
			dMinDistfromPath=dDist;
			MinPos.setX( dMinPathX);
			MinPos.setY( dMinPathY );
			nselectedMinIndx=nMinIdx;
		}		
	}
	//로봇에서부터의 최소가 되는 지점에서의 법선 벡터
	dMinTangentthetaDeg=atan2(m_vecLocalPath[nselectedMinIndx].getY()-m_vecPath[nselectedMinIndx-1].getY(),m_vecLocalPath[nselectedMinIndx].getX()-m_vecLocalPath[nselectedMinIndx-1].getX());

	//목표지점과 최소지점간의 거리
	dDistMintoTarget=sqrt(pow((double)nDistToTarget,2)-pow(dMinDistfromPath,2));
	nPathInx=0;

	//목표지점과 최소지점간의 거리에 따른  목표지점 선정
	while( true ) {

		nPathInx++;
		if(nPathInx >= nPathSize) {
			nPathInx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.

			if(bselectedTargetflag==false)
			{
				dPathX = m_vecLocalPath[nselectedMinIndx].getX();
				dPathY = m_vecLocalPath[nselectedMinIndx].getY();
				TargetPos.setX( dPathX );
				TargetPos.setY( dPathY );
				nselectedTargetIndx=nselectedMinIndx;
			}
			break;			
		}			
		dPathX = m_vecLocalPath[nPathInx].getX();
		dPathY = m_vecLocalPath[nPathInx].getY();			

		dDist=hypot(RobotPos.getX()-dPathX, RobotPos.getY()- dPathY);

		if(dDist< dDistMintoTarget&&nPathInx>=nselectedMinIndx)
		{
			dDist=dDistMintoTarget;
			TargetPos.setX( dPathX );
			TargetPos.setY( dPathY );
			nselectedTargetIndx=nPathInx;
			bselectedTargetflag=true;
		}	

	}

	//목표 지점에서의 법선 벡터
	dTargetTangentthetaDeg=atan2(m_vecLocalPath[nselectedTargetIndx].getY()-m_vecLocalPath[nselectedTargetIndx-1].getY(),m_vecLocalPath[nselectedTargetIndx].getX()-m_vecLocalPath[nselectedTargetIndx-1].getX());


	//새로운 로봇과 목표지점간의 거리 계산
	dnewDistToTarget=nDistToTarget;
	double dTempDist= fabs(fabs(dMinTangentthetaDeg)-fabs(dTargetTangentthetaDeg));
//	printf("dTempDist=%f\n",dTempDist);
	dTempDist=dTempDist/M_PI*nDistToTarget/80;

	if(dTempDist<1.0)	{dTempDist=1.0;}
	else if(dTempDist>3.0)	{ dTempDist=3.0;}

	dnewDistToTarget=dnewDistToTarget/(dTempDist);


	//로봇과 목표지점간의 거리에 따른 목표점 선택
	nPathInx=0;
	if(dMinDistfromPath>=dnewDistToTarget) dDistMintoTarget=300;
	else
	{
		dDistMintoTarget=sqrt(pow((double)dnewDistToTarget,2)-pow(dMinDistfromPath,2));
		if(dDistMintoTarget<300) dDistMintoTarget=300;
	}

	bselectedTargetflag= false;
	dTempDist=1000000000;

	while( true ) {

		nPathInx++;
		if(nPathInx >= nPathSize) {

			nPathInx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.

			if(bselectedTargetflag==false)
			{
				(*dTargetDist)=dDistMintoTarget;

				dPathX = m_vecLocalPath[nselectedTargetIndx].getX();
				dPathY = m_vecLocalPath[nselectedTargetIndx].getY();
				TargetPos.setX( dPathX );
				TargetPos.setY( dPathY );
				nselectedTargetIndx=nselectedMinIndx;
			}
			break;			
		}			
		dPathX = m_vecLocalPath[nPathInx].getX();
		dPathY = m_vecLocalPath[nPathInx].getY();			

		dDist=hypot(MinPos.getX()-dPathX, MinPos.getY()- dPathY);

		if(dDist<dTempDist&&dDist> dDistMintoTarget&&nPathInx>=nselectedMinIndx)
		{
			dTempDist=dDist;
			(*dTargetDist)=dTempDist;
			TargetPos.setX( dPathX );
			TargetPos.setY( dPathY );
			nselectedTargetIndx=nPathInx;
			bselectedTargetflag=true;
		}	

	}
	return TargetPos;
}

void KuVrDWABh::setMode(int nMode)
{
	m_nMode=nMode;
}

/**
 @brief Korean: KuVrDWABh를 실행시키는 함수
 @brief English: 
*/

bool KuVrDWABh::execute(KuCommandMessage CMessage)
{
	
	switch(CMessage.getCommandName()){
	case KuCommandMessage::START_THREAD:
		if(initialize(CMessage))	
		{
			KuThread::start(doThread,this,m_nThreadFuncPeriod); //메인 스레드 시작	
		}
		break;
	case KuCommandMessage::TERMINATE_THREAD:
		terminate();
		break;
	case KuCommandMessage::SUSPEND_THREAD:
		suspend();
		break;
	case KuCommandMessage::RESUME_THREAD:
		resume();
		break;
	default:break;
	}

	return true;
	
}
