#include "StdAfx.h"
#include "KuVrProControlBh.h"

KuVrProControlBh::KuVrProControlBh()
{
	m_vecPath.clear();
	m_listPath.clear();
	m_bThreadFlag = false;
	m_nMap=NULL;
	cout<<"[KuVrProControlBh]: Instance is created!!!"<<endl;
}

KuVrProControlBh::~KuVrProControlBh()
{
	for(int i = 0 ; i < m_nMapSizeX ; i++){
		delete[] m_nMap[i];
		m_nMap[i] = NULL;
	}
	delete[] m_nMap;

	cout<<"[KuVrProControlBh]: Instance is destroyed!!!"<<endl;
}

/**
 @brief Korean: 초기화 작업을 수행하는 함수.
 @brief English: 
*/
bool  KuVrProControlBh::initialize(KuCommandMessage CMessage)
{

	m_bThreadFlag = false;
	m_vecPath.clear();
	m_bThreadFlag = true;

	m_RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	m_GoalPos =  KuDrawingInfo::getInstance()->getGoalPos();
	m_nThreadFuncPeriod = CMessage.getBehaviorPeriod(); //스레드 함수 실행주기 입력
	m_nKinectLaserData=m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,5000);
	m_nLaserData=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,20000);	

	m_KuVrWheelActuator.setRobotPos( m_RobotPos );
	
	if(!initPathplanningProcess()) return false;

 	initKanayamaProcess();
	initMapbuildingProcess();

	return true;
}
void KuVrProControlBh::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}

float KuVrProControlBh::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}

int** KuVrProControlBh::loadMap(string strMapFilePath)
{
	FILE	*infile;
	infile = fopen(strMapFilePath.c_str() ,"rb");
	if(infile==NULL) {
		cout<< "[CMapRepository] : Error - Could not open map from "<<strMapFilePath<<endl;
		return false;
	}
	fclose(infile);

	IplImage* IplMapImage = cvLoadImage(strMapFilePath.c_str(),1);

	int nMapSizeX = IplMapImage->width;
	int nMapSizeY = IplMapImage->height;
	
	int** nMap=NULL;

	nMap = new int*[nMapSizeX];

	if(nMap){
		for(int i = 0 ; i < nMapSizeX ; i++){
			nMap[i] = new int[nMapSizeY];

		}
	}

	
	for(int nX=0; nX< IplMapImage->width; nX++){
		for(int nY=0; nY<IplMapImage->height; nY++){
			if(IplMapImage->imageData[(nX+nY*IplMapImage->width)*3] == (char)255 && //Blue
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+1] == (char)255 && //Green
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+2] == (char)255	){ //Red 

					nMap[nX][IplMapImage->height-1-nY] = KuMap::EMPTY_AREA;				

			}
			else if(IplMapImage->imageData[(nX+nY*IplMapImage->width)*3] == (char)0 && 
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+1] == (char)0 &&
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+2] == (char)0){

					nMap[nX][IplMapImage->height-1-nY] =  KuMap::OCCUPIED_AREA;
			}
			else if(IplMapImage->imageData[(nX+nY*IplMapImage->width)*3] == (char)0 && 
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+1] == (char)255 &&
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+2] == (char)0){ 

					nMap[nX][IplMapImage->height-1-nY] =  KuMap::LUGGAGE_AREA;

			}
			else if(IplMapImage->imageData[(nX+nY*IplMapImage->width)*3] == (char)0 && 
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+1] == (char)100 &&
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+2] == (char)0){ 

					nMap[nX][IplMapImage->height-1-nY] =  KuMap::DYNAMIC_ENVIRONMENT_AREA;

			}
			else if(IplMapImage->imageData[(nX+nY*IplMapImage->width)*3] == (char)255 && 
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+1] == (char)0 &&
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+2] == (char)255){ 

					nMap[nX][IplMapImage->height-1-nY] =  KuMap::FIXED_CAD_AREA;

			}
			else{
				nMap[nX][IplMapImage->height-1-nY] =  KuMap::UNKNOWN_AREA;				

			}
		}
	} 

	cvReleaseImage(&IplMapImage);

	return nMap;
}

/**
@brief Korean: 
@brief English: 
*/
bool KuVrProControlBh::initPathplanningProcess()
{
	//목적지까지의 경로를 생성하기 위한 과정---------------------------------------------
	m_RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	m_GoalPos = KuDrawingInfo::getInstance()->getGoalPos();


	m_KuPathPlanner.initializeMapSize(kuMapRepository::getInstance()->getMap()->getX(),
		kuMapRepository::getInstance()->getMap()->getY()
		);

	m_KuPathPlanner.setMap(kuMapRepository::getInstance()->getMap()->getMap(),Sensor::CELLSIZE);
	m_KuPathPlanner.generatePath(m_GoalPos, m_RobotPos); //경로 생성

	list<KuPose> listPath = m_KuPathPlanner.getPath();
	if(listPath.size()==0) return false; //경로생성 실패의 경우
	m_listPath=m_KuPathSmoothing.smoothingPath(listPath);

	list<KuPose>::iterator it;
	for(it=m_listPath.begin(); it!=m_listPath.end(); it++){
		m_vecPath.push_back(*it);
	}
	//경로생성 완료==================================================================

	KuDrawingInfo::getInstance()->setPath(m_listPath); //경로 화면에 표시

	return true;

}
/**
@brief Korean: Kanayama  프로세스를 수행하기 위해 필요한 초기화 작업을 수행하는 함수.
@brief English: 
*/
void KuVrProControlBh::initKanayamaProcess()
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
 @brief Korean: 현재 Behavior의 상태를 나타내는 함수
 @brief English: 
*/
bool KuVrProControlBh::getBehaviorStates()
{
	return m_bThreadFlag;
}

/**
@brief Korean: 
@brief English: 
*/
void KuVrProControlBh::initMapbuildingProcess( )
{

	m_pRefMap = kuMapRepository::getInstance()->getMap(); 
	m_nMapSizeX=m_pRefMap->getX();
	m_nMapSizeY=m_pRefMap->getY();

	if(m_nMap==NULL)
	{
			m_nMap = new int*[m_nMapSizeX];
		for(int i = 0 ; i < m_nMapSizeX ; i++){
			m_nMap[i] = new int[m_nMapSizeY];
		}
	}

	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			m_nMap[i][j] = m_pRefMap->getMap()[i][j];
		}
	}

	if(m_nCurFloor==1)
	{
		KuBuildingPOMapParameter InputParam;

		string strProMapNameNPath = KuRobotParameter::getInstance()->getProMapNameNPath();

		InputParam.setPath(strProMapNameNPath);

		InputParam.setMapSizeXmYm(m_nMapSizeX/10.0, m_nMapSizeY/10.0);

		KuBuildingPOMapPr::getInstance()->start(InputParam);
	}


	//KuDrawingInfo::getInstance()->setProObBuildingMap(KuProBuildingMapPr::getInstance()->getProMap());

}

void KuVrProControlBh::checkVirtualObstacle(KuMap* pMap)
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
 @brief Korean: 센서데이터를 이용하여 타겟점과 로봇 사이의 장애물 유무를 검사하는 함수
 @brief English: 
*/
bool KuVrProControlBh::checkObstacles(KuPose RobotPos,KuPose TargetPos,int_1DArray nLaserData181, double dTargetDist)
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
 @brief Korean: 스레드로 돌아가는 함수
 @brief English: 
*/
void KuVrProControlBh::doThread(void* arg)
{
	KuVrProControlBh* pKMC = (KuVrProControlBh*)arg;

	KuVelocity generatedVel;
	double dTargetDist=0.0;

	//센서 정보------------------------------------------------------------------
	pKMC->m_RobotPos  = pKMC->m_KuVrWheelActuator.getRobotPos();
	pKMC->m_RobotPos.setID(pKMC->m_nCurFloor);
	pKMC->checkVirtualObstacle(pKMC->m_pRefMap);
	pKMC->m_nLaserData = KuVrHokuyoUTM30LXInterface::getInstance()->getData181(pKMC->m_RobotPos);//로봇의 위치에 따른 가상 맵에서 가상 레이저 데이터 받아옴
	
	//데이터 베이스 구축-------------------------------------------------------------
	KuBuildingPOMapPr::getInstance()->setData( pKMC->m_RobotPos, pKMC->m_KuVrWheelActuator.getDelEncoderData(), pKMC->m_nLaserData);

	
	// 포인트 확인--------------------------------------------------------------
	if(pKMC->m_math.calcDistBetweenPoses(pKMC->m_GoalPos,pKMC->m_RobotPos) <pKMC->m_nGoalArea){
		pKMC->terminate();
		return;
	}
	else if(pKMC->m_math.calcDistBetweenPoses(pKMC->m_GoalPos,pKMC->m_RobotPos) <GOAL_BOUNDARY){
		pKMC->m_nDesiredVel=100;//goal 주변 속도 감소
	}

	for(int i=0; i<pKMC->m_vecElveWayPointPos.size();i++)
	{	
		if(	pKMC->m_nCurFloor==pKMC->m_vecElveWayPointPos[i].getID())
		{
			if(pKMC->m_math.calcDistBetweenPoses(pKMC->m_vecElveWayPointPos[i],pKMC->m_RobotPos) <GOAL_BOUNDARY){
				pKMC->m_nDesiredVel=100;//goal 주변 속도 감소
				pKMC->m_vecElveWayPointPos[i].setPro(1.0);
			}
		}
	}
	

	KuObstAvoidancePr::getInstance()->setData(pKMC->m_RobotPos,pKMC->m_DelEncoderData,pKMC->m_nLaserData,pKMC->m_nKinectLaserData);

	pKMC->m_TargetPos=KuObstAvoidancePr::getInstance()->getTargetPos();
	
	if(pKMC->m_TargetPos.getX()==0&&pKMC->m_TargetPos.getY()==0){return;}
	
	//모션제어------------------------------------------------------------ 
	if(pKMC->m_KanayaMC.getterminateflag()==true)
	{
		KuObstAvoidancePr::getInstance()->terminate();

		//엘레베이터 인식 
		bool bterminate=false;
		generatedVel = pKMC->getElevState(pKMC->m_RobotPos ,pKMC->m_vecElveWayPointPos,&bterminate );
		if(bterminate==true)
		{	
			list<KuPose> listLocalPath;
			list<KuPose>::iterator it;
			for(it=pKMC->m_listPath.begin(); it!=pKMC->m_listPath.end(); it++){
				if(pKMC->m_nCurFloor!=it->getID()){ continue;}
				listLocalPath.push_back(*it);
			}

			KuObstAvoidancePr::getInstance()->setPath(listLocalPath);
			bool bflag= KuObstAvoidancePr::getInstance()->start(20);
			pKMC->initKanayamaProcess();
			//pKMC->initMapbuildingProcess();
		}

	}
	else
	{
		generatedVel = pKMC->m_KanayaMC.generateTRVelocity(pKMC->m_TargetPos, pKMC->m_RobotPos, 100 );
	}


// 	if(pKMC->checkObstacles( pKMC->m_RobotPos, pKMC->m_TargetPos,pKMC->m_nLaserData, 1000))
// 	{
// 		pKMC->m_KuVrWheelActuator.moveTRVelocity(0,0);
// 	}
// 	else
// 	{
		pKMC->m_KuVrWheelActuator.moveTRVelocity(generatedVel.m_nTranslationVel, generatedVel.m_nRotationDegVel);
//	}

	//그리기------------------------------------------------------------------------
	pKMC->drawNaviData();

}
/**
@brief Korean: 주행 관련 정보를 그려주는 함수.
@brief English: 
*/
void KuVrProControlBh::drawNaviData()
{
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos); //로봇 위치 화면에 표시	
	KuDrawingInfo::getInstance()->setLaserData181(m_nLaserData);		
	KuDrawingInfo::getInstance()->setTargetPos(m_TargetPos); //타겟 위치 화면에 표시
		KuDrawingInfo::getInstance()->setLocalPath(KuObstAvoidancePr::getInstance()->getPath()); //경로 화면에 표시
	//KuDrawingInfo::getInstance()->setnMap(m_nMap,m_nMapX,m_nMapY);
	//KuDrawingInfo::getInstance()->setProObBuildingMap(KuProBuildingMapPr::getInstance()->getProMap());

}
KuVelocity KuVrProControlBh::getElevState(KuPose RobotPos,vector<KuPose> vecWayPointPos,bool *bterminate )
{
	(*bterminate)=false;
	KuVelocity generatedVel;
	int nTurnVel=10;
	KuPose WayPointPos;

	for(int i=0; i<vecWayPointPos.size();i++)
	{
		if(RobotPos.getID()==vecWayPointPos[i].getID())
		{
			WayPointPos=vecWayPointPos[i];
			vecWayPointPos[i].setPro(1.0);
		}
	}
	bool bBackTransflag=false;
	
	for(int i=0; i<vecWayPointPos.size();i++)
	{
		if(vecWayPointPos[i].getPro()!=1.0)
		{
			bBackTransflag=true;
		}
	}
	
	bool bTransflag=false;
	bool bRotateflag=false;

	generatedVel=excuteRotateState(RobotPos, WayPointPos,&bRotateflag);

	if(bRotateflag==false){return generatedVel;}
	

	if(!bBackTransflag){generatedVel=excuteTransState(RobotPos, WayPointPos,-100, &bTransflag);}
	else{generatedVel=excuteTransState(RobotPos, WayPointPos,100, &bTransflag);}

	if(bTransflag==false){return generatedVel;}

	(*bterminate)=true;
	for(int i=0; i<vecWayPointPos.size();i++)
	{
		if(vecWayPointPos[i].getPro()!=1.0)
		{
			m_nCurFloor=vecWayPointPos[i].getID();
			(*bterminate)=false;
			if(m_nCurFloor==1) KuBuildingPOMapPr::getInstance()->terminate();
			KuDrawingInfo::getInstance()->setCurFloor(m_nCurFloor);

			KuPose  RobotPos;
			RobotPos.setXm(vecWayPointPos[i].getXm()+1*cos(vecWayPointPos[i].getThetaRad()));
			RobotPos.setYm(vecWayPointPos[i].getYm()+1*sin(vecWayPointPos[i].getThetaRad()));
			RobotPos.setThetaRad(vecWayPointPos[i].getThetaRad()+180);
			m_KuVrWheelActuator.setRobotPos( RobotPos );
			KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
			Sleep(1000);

			m_pRefMap = kuMapRepository::getInstance()->getMap(); 
			m_nMapSizeX=m_pRefMap->getX();
			m_nMapSizeY=m_pRefMap->getY();

			for(int i=0; i<m_nMapSizeX;i++)
			{
				for(int j=0; j<m_nMapSizeY;j++)
				{
					m_nMap[i][j] = m_pRefMap->getMap()[i][j];
				}
			}
		}
	}	

	generatedVel.m_nTranslationVel = 0;
	generatedVel.m_nXVel = 0;
	generatedVel.m_nYVel = 0;
	generatedVel.m_nRotationDegVel = 0;
	return generatedVel;
}

KuVelocity KuVrProControlBh::excuteRotateState(KuPose RobotPos,KuPose WayPointPos,bool *bterminate)
{
	KuVelocity generatedVel;
	generatedVel.m_nTranslationVel = 0;
	generatedVel.m_nXVel = 0;
	generatedVel.m_nYVel = 0;
	generatedVel.m_nRotationDegVel = 0;
	(*bterminate)=false;

	int nTurnVel=10;
	double dAngleDiff = WayPointPos.getThetaRad() - RobotPos.getThetaRad();
	if (dAngleDiff >= M_PI){ dAngleDiff -= 2.0*M_PI; }			
	else if (dAngleDiff <= -M_PI){ dAngleDiff += 2.0*M_PI; }
	if (dAngleDiff > M_PI*1/180) { 
		generatedVel.m_nTranslationVel = 0;
		generatedVel.m_nXVel = 0;
		generatedVel.m_nYVel = 0;
		generatedVel.m_nRotationDegVel = (int)(nTurnVel);
		return generatedVel;
	}	
	else if (dAngleDiff < -M_PI*1/180) {
		generatedVel.m_nTranslationVel = 0;
		generatedVel.m_nXVel = 0;
		generatedVel.m_nYVel = 0;
		generatedVel.m_nRotationDegVel = (int)(-nTurnVel);
		return generatedVel;
	}
	(*bterminate)=true;
return generatedVel;
}
KuVelocity KuVrProControlBh::excuteTransState(KuPose RobotPos,KuPose WayPointPos,int nVel ,bool *bterminate)
{
	KuVelocity generatedVel;
	generatedVel.m_nTranslationVel=nVel;
	generatedVel.m_nXVel = 0;
	generatedVel.m_nYVel = 0;
	generatedVel.m_nRotationDegVel = 0;

	(*bterminate)=false;
	if(nVel>0){

		if(m_math.calcDistBetweenPoses(WayPointPos,RobotPos)>1000)
		{
			(*bterminate)=true;
		}
	}
	else if(nVel<0)
	{
		if(m_math.calcDistBetweenPoses(WayPointPos,RobotPos)<400)
		{
			(*bterminate)=true;
		}
	}
	return generatedVel;
}
KuVelocity KuVrProControlBh::getElevState(KuPose RobotPos,KuPose WayPointPos,bool *bterminate )
{
	(*bterminate)=false;
	KuVelocity generatedVel;
	int nTurnVel=10;

	double dAngleDiff = WayPointPos.getThetaRad() - RobotPos.getThetaRad();
	if (dAngleDiff >= M_PI){ dAngleDiff -= 2.0*M_PI; }			
	else if (dAngleDiff <= -M_PI){ dAngleDiff += 2.0*M_PI; }
	if (dAngleDiff > M_PI*1/180) { 
		generatedVel.m_nTranslationVel = 0;
		generatedVel.m_nXVel = 0;
		generatedVel.m_nYVel = 0;
		generatedVel.m_nRotationDegVel = (int)(nTurnVel);
		return generatedVel;
	}	
	else if (dAngleDiff < -M_PI*1/180) {
		generatedVel.m_nTranslationVel = 0;
		generatedVel.m_nXVel = 0;
		generatedVel.m_nYVel = 0;
		generatedVel.m_nRotationDegVel = (int)(-nTurnVel);
		return generatedVel;
	}
	(*bterminate)=true;

	generatedVel.m_nTranslationVel = 0;
	generatedVel.m_nXVel = 0;
	generatedVel.m_nYVel = 0;
	generatedVel.m_nRotationDegVel = 0;
	return generatedVel;
}

KuPose KuVrProControlBh::getStaticTargetPos(KuPose RobotPos,int nDistToTarget)
{
	int nPathInx=0;
	int nMinIdx=0;;
	bool bselectedTargetflag= false;
	KuPose TargetPos;
	int nPathSize = m_vecPath.size()-1;
	double dPathX; 
	double dPathY;
	double dMinPathX;
	double dMinPathY;
	double dDist;
	double dMinDistfromPath=INFINITY_VALUE;
	double dDistMintoTarget=INFINITY_VALUE;
	int nselectedMinIndx=-1;

	//로봇에서 부터의 최소가 되는 지점 및 거리값
	while( true ) {

		nMinIdx++;
		if(nMinIdx >= nPathSize) {
			nMinIdx = nPathSize; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
			break;
		}
		dMinPathX = m_vecPath[nMinIdx].getX();
		dMinPathY = m_vecPath[nMinIdx].getY();

		dDist=hypot(RobotPos.getX()-dMinPathX, RobotPos.getY()- dMinPathY);

		if(dDist< nDistToTarget)
		{
			dMinDistfromPath=dDist;
			nselectedMinIndx=nMinIdx;
		}	
		else if(nselectedMinIndx!=-1)
			break;
	}

	if(nselectedMinIndx==-1){
		dPathX = m_vecPath[nPathSize].getX();
		dPathY = m_vecPath[nPathSize].getY();
		TargetPos.setX( dPathX );
		TargetPos.setY( dPathY );
	}
	else
	{
		dPathX = m_vecPath[nselectedMinIndx].getX();
		dPathY = m_vecPath[nselectedMinIndx].getY();
		TargetPos.setX( dPathX );
		TargetPos.setY( dPathY );
	}




	//m_nSelectedMinIndx = nselectedMinIndx;

// 	dDistMintoTarget=sqrt(pow((double)nDistToTarget,2)-pow(dMinDistfromPath,2));
// 	nPathInx=0;
// 
// 	while(true) {
// 
// 		nPathInx++;
// 		if(nPathInx >= nPathSize) {
// 			nPathInx = nPathSize; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
// 
// 			if(bselectedTargetflag==false)
// 			{
// 				dPathX = m_vecPath[nselectedMinIndx].getX();
// 				dPathY = m_vecPath[nselectedMinIndx].getY();
// 				TargetPos.setX( dPathX );
// 				TargetPos.setY( dPathY );
// 			}
// 			break;			
// 		}
// 
// 		dPathX = m_vecPath[nPathInx].getX();
// 		dPathY = m_vecPath[nPathInx].getY();
// 
// 		dDist = hypot(RobotPos.getX()-dPathX, RobotPos.getY()- dPathY);
// 
// 		if(dDist< dDistMintoTarget&&nPathInx>=nselectedMinIndx)
// 		{
// 			TargetPos.setX( dPathX );
// 			TargetPos.setY( dPathY );
// 			bselectedTargetflag=true;
// 			//m_nSelectTargetIdx = nPathInx;
// 		}	
// 	}

	return TargetPos;
}
/**
 @brief Korean: 경로에서 타겟점을 선택하는 함수
 @brief English: 
*/
KuPose KuVrProControlBh::getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist)
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

	int nPathSize = m_vecPath.size();

	bool bselectedTargetflag= false;

	KuPose TargetPos,MinPos;


	//로봇에서 부터의 최소가 되는 지점 및 거리값
	while( true ) {

		nMinIdx++;
		if(nMinIdx >= nPathSize) {
			nMinIdx = nPathSize-1; //배열인덱스 이기 떄문에 1을 빼줘야 한다.
			break;
		}
		dMinPathX = m_vecPath[nMinIdx].getX();
		dMinPathY = m_vecPath[nMinIdx].getY();

		dDist=hypot(RobotPos.getX()-dMinPathX, RobotPos.getY()- dMinPathY);

		if(dDist< dMinDistfromPath)
		{
			dMinDistfromPath=dDist;
			MinPos.setX( dMinPathX);
			MinPos.setY( dMinPathY );
			nselectedMinIndx=nMinIdx;
		}		
	}
	if(nselectedMinIndx<0) return m_GoalPos;
	//로봇에서부터의 최소가 되는 지점에서의 법선 벡터
	dMinTangentthetaDeg=atan2(m_vecPath[nselectedMinIndx].getY()-m_vecPath[nselectedMinIndx-1].getY(),m_vecPath[nselectedMinIndx].getX()-m_vecPath[nselectedMinIndx-1].getX());

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
				dPathX = m_vecPath[nselectedMinIndx].getX();
				dPathY = m_vecPath[nselectedMinIndx].getY();
				TargetPos.setX( dPathX );
				TargetPos.setY( dPathY );
				nselectedTargetIndx=nselectedMinIndx;
			}
			break;			
		}			
		dPathX = m_vecPath[nPathInx].getX();
		dPathY = m_vecPath[nPathInx].getY();			

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
	dTargetTangentthetaDeg=atan2(m_vecPath[nselectedTargetIndx].getY()-m_vecPath[nselectedTargetIndx-1].getY(),m_vecPath[nselectedTargetIndx].getX()-m_vecPath[nselectedTargetIndx-1].getX());


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

				dPathX = m_vecPath[nselectedTargetIndx].getX();
				dPathY = m_vecPath[nselectedTargetIndx].getY();
				TargetPos.setX( dPathX );
				TargetPos.setY( dPathY );
				nselectedTargetIndx=nselectedMinIndx;
			}
			break;			
		}			
		dPathX = m_vecPath[nPathInx].getX();
		dPathY = m_vecPath[nPathInx].getY();			

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

/**
 @brief Korean: 시뮬레이터를 종료한다.
 @brief English: 
*/
void KuVrProControlBh::terminate()
{

	KuThread::terminate();
	m_bThreadFlag = false;	
	m_KuVrWheelActuator.moveTRVelocity(0,0);
	KuObstAvoidancePr::getInstance()->terminate();
	cout<<"[KuVrProControlBh]:: Behavior is terminated!!!"<<endl;
	if(m_nCurFloor==1) KuBuildingPOMapPr::getInstance()->terminate();
}


/**
 @brief Korean: KuVrProControlBh를 실행시키는 함수
 @brief English: 
*/

bool KuVrProControlBh::execute(KuCommandMessage CMessage)
{
	switch(CMessage.getCommandName()){
	case KuCommandMessage::START_THREAD:
		if(initialize(CMessage)){
			KuThread::start(doThread,this,100); //메인 스레드 시작	
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