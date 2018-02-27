#include "stdafx.h"
#include "GotoGoalBehavior.h"

GotoGoalBehavior::GotoGoalBehavior()
{
	m_vecPath.clear();
	m_listPath.clear();
	m_vecWayPoint.clear();
	m_listWayPoint.clear();
	m_bThreadFlag = false;
	m_bWaitflag=false;
	m_nselectIdx=0;
	m_nSelectTargetIdx=0;
	m_CurWayPoint.init();
	m_IplKinectCamera =  cvCreateImage(cvSize( Sensor::IMAGE_WIDTH, Sensor::IMAGE_HEIGHT),8,3);	
	memset(m_KinectDataPos,0,sizeof(m_KinectDataPos));

	cout<<"[GotoGoalBehavior]: Instance is created!!!"<<endl;
	m_IplKinectCamera = cvCreateImage(cvSize( Sensor::CEILING_IMAGE_WIDTH, Sensor::CEILING_IMAGE_HEIGHT),8,3);
}

GotoGoalBehavior::~GotoGoalBehavior()
{
	cout<<"[GotoGoalBehavior]: Instance is destroyed!!!"<<endl;
}

/**
 @brief Korean: 초기화 작업을 수행하는 함수.
 @brief English: 
*/
bool GotoGoalBehavior::initialize( )
{
	m_bThreadFlag = true;
	m_vecPath.clear();
	m_listPath.clear();
	m_vecWayPoint.clear();
	m_listWayPoint.clear();
	m_CurWayPoint.init();
	m_nselectIdx=-1;
	m_bWaitflag=false;
	m_nSelectTargetIdx=0;
	m_nPrePathIndx=-1;
// 	m_nMapSizeXm=100;
// 	m_nMapSizeYm=100;
	m_dCellSize=Sensor::CELLSIZE;//10cm

	m_nLaserData181=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	m_nRplidarData181=m_KuUtil.generateIntType1DArray(Sensor::RPLIDAR_DATA_NUM181,0);
	m_nKinectRangeData=m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,5000);
// 	m_nMapSizeXm=KuRobotParameter::getInstance()->getMapSizeXm();
// 	m_nMapSizeYm=KuRobotParameter::getInstance()->getMapSizeYm();

	//path planning start-----------------------------------------------------------------------------
// 	m_RobotPos =KuDrawingInfo::getInstance()->getRobotPos();
// 	m_GoalPos = KuDrawingInfo::getInstance()->getGoalPos();
// 	m_KuPathPlanner.setRadiusofRobotp(KuRobotParameter::getInstance()->getRobotRadius());
// 	m_KuPathPlanner.initIntCost(KuRobotParameter::getInstance()->getRobotRadius()/100.0+2);
// 	m_KuPathPlanner.initializeMapSize(kuMapRepository::getInstance()->getMap()->getX(),
// 		kuMapRepository::getInstance()->getMap()->getY()	);
// 	m_KuPathPlanner.setMap(kuMapRepository::getInstance()->getMap()); 
// 	m_KuPathPlanner.generatePath(m_GoalPos, m_RobotPos); //경로 생성
// 	m_listPath = m_KuPathPlanner.getPath();
// 	if(m_listPath.size()==0){
// 		m_bThreadFlag = false;	
// 		return false; 
// 	}//경로생성 실패의 경우
// 	m_listPath=m_KuPathSmoothing.smoothingPath(m_listPath);
	//path planning  end-----------------------------------------------------------------------------
	//경로생성 완료==================================================================
	
	initPathplanningProcess();
	//	return false;
	//initMapbuildingProcess( );
	startLocalizer(m_RobotPos); //Localizer를 시작한다.
	initKanayamaProcess();

	return true;
}
void GotoGoalBehavior::initMapbuildingProcess( )
{
	if(m_nCurFloor==1)
	{
		KuBuildingPOMapParameter InputParam;

		string strProMapNameNPath = KuRobotParameter::getInstance()->getProMapNameNPath();

		InputParam.setPath(strProMapNameNPath);

		InputParam.setMapSizeXmYm(m_nMapSizeXm, m_nMapSizeYm);

		KuBuildingPOMapPr::getInstance()->start(InputParam);
	}
	//KuDrawingInfo::getInstance()->setProObBuildingMap(KuProBuildingMapPr::getInstance()->getProMap());

}
int** GotoGoalBehavior::loadMap(string strMapFilePath)
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
			else if(IplMapImage->imageData[(nX+nY*IplMapImage->width)*3] == (char)0 && 
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+1] == (char)0 &&
				IplMapImage->imageData[(nX+nY*IplMapImage->width)*3+2] == (char)255){ 

					nMap[nX][IplMapImage->height-1-nY] =  KuMap::SAFE_AREA;					

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
@brief Korean: Kanayama  프로세스를 수행하기 위해 필요한 초기화 작업을 수행하는 함수.
@brief English: 
*/
void GotoGoalBehavior::initKanayamaProcess()
{
	//INI 파일로부터 kanayama motion controㅣ을 수행하기 위해 필요한 정보를 얻어온다.------------------------------------
	m_nDistToTarget =KuRobotParameter::getInstance()->getTargetDistance();
	m_nDesiredVel = KuRobotParameter::getInstance()->getDesiedVel();
	m_nGoalArea =KuRobotParameter::getInstance()->getGoalArea();
	double dKX =KuRobotParameter::getInstance()->getdKX();
	double dKY =KuRobotParameter::getInstance()->getdKY();
	double dKT = KuRobotParameter::getInstance()->getdKT();
	m_nMaxTVel=KuRobotParameter::getInstance()->getMaxRobotVelocity();
	m_nMinTVel=KuRobotParameter::getInstance()->getMinRobotVelocity();
	//==================================================================================================================
	m_KanayaMC.init();
	m_KanayaMC.setGain(dKX,dKY,dKT);	//게인 설정, 설정하지 않더라고 기본 게인값이 사용된다.
	m_KanayaMC.setMaxTRVel(m_nMaxTVel,30);
	m_KanayaMC.setMinTRVel(m_nMinTVel,0);
}

/**
 @brief Korean: 스레드로 돌아가는 함수
 @brief English: 
*/
void GotoGoalBehavior::doThread(void* arg)
{
	GotoGoalBehavior* pGGB = (GotoGoalBehavior*)arg;
	KuVelocity generatedVel;
	generatedVel.m_nRotationDegVel=generatedVel.m_nTranslationVel=0;
	
	LARGE_INTEGER t;
	pGGB->startTimeCheck(t);

	//센서--------------------------------------------------------------
	if(SensorSupervisor::getInstance()->readSensorData() ==false) return;
	pGGB->m_nLaserData181 = SensorSupervisor::getInstance()->getURG04LXLaserData();
	pGGB->m_DelEncoderData =SensorSupervisor::getInstance()->getEncoderDelPos();
	pGGB->m_IplKinectCamera = SensorSupervisor::getInstance()->getKinectImageData();
	pGGB->m_nKinectRangeData= SensorSupervisor::getInstance()->getKinectRangeData();
	pGGB->m_nRplidarData181= SensorSupervisor::getInstance()->getRplidarData();

//	KuPose* pKinectDataPos = SensorSupervisor::getInstance()->getKinectDataPos();
// 	int nSize = Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT;
// 	for(int i=0; i<nSize; i++){	pGGB->m_KinectDataPos[i] = pKinectDataPos[i];}
	//센서--------------------------------------------------------------

	double dSensortime = pGGB->finishTimeCheck(t);
	printf("dSensortime =%f\n",dSensortime );

	
	pGGB->startTimeCheck(t);
	// 위치추정-------------------------------------------------------------
	pGGB->m_RobotPos = KuLBPFLocalizerPr::getInstance()->estimateRobotPos(pGGB->m_nLaserData181, pGGB->m_DelEncoderData);	
	//pGGB->m_RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	// 위치추정-------------------------------------------------------------
	double dLocaltime = pGGB->finishTimeCheck(t);
	printf("dLocaltime =%f\n",dLocaltime );
	

	//데이터 베이스 구축(POMap)-------------------------------------------------------------
	if(pGGB->m_nCurFloor==1)
	{
		KuBuildingPOMapPr::getInstance()->setData( pGGB->m_RobotPos, pGGB->m_DelEncoderData, pGGB->m_nLaserData181);
	}
	//데이터 베이스 구축-------------------------------------------------------------


	pGGB->startTimeCheck(t);
	// 포인트 확인--------------------------------------------------------------
	if(pGGB->m_math.calcDistBetweenPoses(pGGB->m_GoalPos,pGGB->m_RobotPos) <pGGB->m_nGoalArea){
		pGGB->terminate();
		return;
	}
	else if(pGGB->m_math.calcDistBetweenPoses(pGGB->m_GoalPos,pGGB->m_RobotPos) <GOAL_BOUNDARY){
		pGGB->m_nDesiredVel=100;//goal 주변 속도 감소
	}

	for(int i=0; i<pGGB->m_vecElveWayPointPos.size();i++)
	{	
		if(	pGGB->m_nCurFloor==pGGB->m_vecElveWayPointPos[i].getID())
		{
			if(pGGB->m_math.calcDistBetweenPoses(pGGB->m_vecElveWayPointPos[i],pGGB->m_RobotPos) <GOAL_BOUNDARY){
				pGGB->m_nDesiredVel=100;//goal 주변 속도 감소
				pGGB->m_vecElveWayPointPos[i].setPro(1.0);
			}
		}
	}
	
	//KuObstAvoidancePr::getInstance()->setData(pGGB->m_RobotPos,pGGB->m_DelEncoderData,pGGB->m_nLaserData181,pGGB->m_nKinectRangeData);
	KuObstAvoidancePr::getInstance()->setData(pGGB->m_RobotPos,pGGB->m_DelEncoderData,pGGB->m_nRplidarData181,pGGB->m_nKinectRangeData);// 로컬패스 및 타겟지점 설정

	pGGB->m_TargetPos=KuObstAvoidancePr::getInstance()->getTargetPos();

//	if(pGGB->m_TargetPos.getX()==0&&pGGB->m_TargetPos.getY()==0){return;}
	// 포인트 확인--------------------------------------------------------------
	double dPointtime = pGGB->finishTimeCheck(t);
	printf("dPointtime =%f\n",dPointtime );

	pGGB->startTimeCheck(t);
	//모션제어------------------------------------------------------------ 
	if(pGGB->m_KanayaMC.getterminateflag()==true)
	{
		KuObstAvoidancePr::getInstance()->terminate();
		
		bool bElveterminate=false;
		//엘레베이터 인식 ---------------------------------------------------
		generatedVel=pGGB->getElevState( pGGB->m_RobotPos,&pGGB->m_vecElveWayPointPos,pGGB->m_DelEncoderData ,pGGB->m_nLaserData181, pGGB->m_nKinectRangeData,&bElveterminate  );
		//엘레베이터 인식 ---------------------------------------------------

		if(bElveterminate==true)
		{
			PioneerWheelActuatorInterface::getInstance()->stop();	
			Sleep(1000);
			pGGB->startLocalizer(pGGB->m_RobotPos);
			Sleep(1000);

			list<KuPose> listLocalPath;
			list<KuPose>::iterator it;
			for(it=pGGB->m_listPath.begin(); it!=pGGB->m_listPath.end(); it++){
				if(pGGB->m_nCurFloor!=it->getID()){ continue;}
				listLocalPath.push_back(*it);
			}
			KuObstAvoidancePr::getInstance()->setPath(listLocalPath);
			bool bflag= KuObstAvoidancePr::getInstance()->start(20);
			pGGB->initKanayamaProcess();
		}

	}
	else
	{
		generatedVel = pGGB->m_KanayaMC.generateTRVelocity(pGGB->m_TargetPos, pGGB->m_RobotPos, 100 );
	}


	if(pGGB->checkObstacles( pGGB->m_RobotPos, pGGB->m_TargetPos,pGGB->m_nLaserData181, 1000))
	{
		PioneerWheelActuatorInterface::getInstance()->stop();	
	}
	else
	{
		PioneerWheelActuatorInterface::getInstance()->moveByTRVelocity(generatedVel.m_nTranslationVel, generatedVel.m_nRotationDegVel);	
	}
	//모션제어------------------------------------------------------------ 
	double dMotiontime = pGGB->finishTimeCheck(t);
	printf("dMotiontime =%f\n",dMotiontime );

	// UI그리기 
	pGGB->drawNaviData();
}
KuVelocity GotoGoalBehavior::getElevState(KuPose RobotPos,vector<KuPose> *vecWayPointPos,KuPose DelEncoderData, int_1DArray nLaserData, int_1DArray nKinectRangeData,bool *bterminate  )
{
	KuLBPFLocalizerPr::getInstance()->terminate();	
	
	KuVelocity generatedVel;
	generatedVel.m_nRotationDegVel=0;
	generatedVel.m_nTranslationVel=0;
	(*bterminate)=false;
	KuPose WayPointPos;

	for(int i=0; i<(*vecWayPointPos).size();i++)
	{
		if(RobotPos.getID()==(*vecWayPointPos)[i].getID())
		{
			WayPointPos=(*vecWayPointPos)[i];
			(*vecWayPointPos)[i].setPro(1.0);
		}
	}
	//엘베 프로세스---
	bool bturnfloorflag=false;
	
	///////로봇 포즈 업데이트
	double dTransVel=0.; double dRotVel=0.;
	printf("시작 \n");
	int nState = m_Elvator.EnterElevatorState(nKinectRangeData,WayPointPos, RobotPos, DelEncoderData,&dTransVel, &dRotVel);
	printf("끝 \n");

	generatedVel.m_nRotationDegVel=dRotVel;
	generatedVel.m_nTranslationVel=dTransVel;

	if(nState>8)
	{
		bturnfloorflag=true;
	}
	else if(nState==0)
	{
		(*bterminate)=true;
	}
	//---------------

	//다 움직임	

	if(bturnfloorflag==true)
	{	
		for(int i=0; i<(*vecWayPointPos).size();i++)
		{
			if((*vecWayPointPos)[i].getPro()!=1.0)
			{
				m_nCurFloor=(*vecWayPointPos)[i].getID();
				if(m_nCurFloor==1) KuBuildingPOMapPr::getInstance()->terminate();

				KuPose  RobotPos;
				RobotPos.setXm((*vecWayPointPos)[i].getXm()+1.5*cos((*vecWayPointPos)[i].getThetaRad()+D2R*180));
				RobotPos.setYm((*vecWayPointPos)[i].getYm());
				RobotPos.setThetaRad((*vecWayPointPos)[i].getThetaRad()+D2R*180);
				RobotPos.setID(m_nCurFloor);
				KuDrawingInfo::getInstance()->setCurFloor(m_nCurFloor);
				KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
				KuLBPFLocalizerPr::getInstance()->setRobotPos(RobotPos);
			}
		}	
		
	}

	return generatedVel;
}

KuVelocity GotoGoalBehavior::getElevState(KuPose RobotPos,vector<KuPose> vecWayPointPos,bool *bterminate )
{
	(*bterminate)=false;
	KuVelocity generatedVel;
// 	int nTurnVel=10;
// 	KuPose WayPointPos;
// 
// 	for(int i=0; i<vecWayPointPos.size();i++)
// 	{
// 		if(RobotPos.getID()==vecWayPointPos[i].getID())
// 		{
// 			WayPointPos=vecWayPointPos[i];
// 			vecWayPointPos[i].setPro(1.0);
// 		}
// 	}
// 	bool bBackTransflag=false;
// 
// 	for(int i=0; i<vecWayPointPos.size();i++)
// 	{
// 		if(vecWayPointPos[i].getPro()!=1.0)
// 		{
// 			bBackTransflag=true;
// 		}
// 	}
// 
// 	bool bTransflag=false;
// 	bool bRotateflag=false;
// 
// 	generatedVel=excuteRotateState(RobotPos, WayPointPos,&bRotateflag);
// 
// 	if(bRotateflag==false){return generatedVel;}
// 
// 
// 	if(!bBackTransflag){generatedVel=excuteTransState(RobotPos, WayPointPos,-100, &bTransflag);}
// 	else{generatedVel=excuteTransState(RobotPos, WayPointPos,100, &bTransflag);}
// 
// 	if(bTransflag==false){return generatedVel;}
// 
// 	(*bterminate)=true;
// 	for(int i=0; i<vecWayPointPos.size();i++)
// 	{
// 		if(vecWayPointPos[i].getPro()!=1.0)
// 		{
// 			m_nCurFloor=vecWayPointPos[i].getID();
// 			(*bterminate)=false;
// 			if(m_nCurFloor==1) KuBuildingPOMapPr::getInstance()->terminate();
// 			KuDrawingInfo::getInstance()->setCurFloor(m_nCurFloor);
// 
// 			KuPose  RobotPos;
// 			RobotPos.setXm(vecWayPointPos[i].getXm()+1*cos(vecWayPointPos[i].getThetaRad()));
// 			RobotPos.setYm(vecWayPointPos[i].getYm()+1*sin(vecWayPointPos[i].getThetaRad()));
// 			RobotPos.setThetaRad(vecWayPointPos[i].getThetaRad());
// 			m_KuVrWheelActuator.setRobotPos( RobotPos );
// 			KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
// 			Sleep(1000);
// 
// 		}
// 	}	
// 
// 	generatedVel.m_nTranslationVel = 0;
// 	generatedVel.m_nXVel = 0;
// 	generatedVel.m_nYVel = 0;
// 	generatedVel.m_nRotationDegVel = 0;
	return generatedVel;
}

/**
@brief Korean: 주행 관련 정보를 그려주는 함수.
@brief English: 
*/
void GotoGoalBehavior::drawNaviData()
{
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos); //로봇 위치 화면에 표시		
	KuDrawingInfo::getInstance()->setLaserData181(m_nLaserData181);		
	KuDrawingInfo::getInstance()->setKinectGlobal3DPos(m_KinectDataPos);
	KuDrawingInfo::getInstance()->setKinectRangeData(m_nKinectRangeData);
	KuDrawingInfo::getInstance()->setKinectImageData(m_IplKinectCamera);		
	KuDrawingInfo::getInstance()->setTargetPos(m_TargetPos); //타겟 위치 화면에 표시
	KuDrawingInfo::getInstance()->setPath(m_listPath);
	KuDrawingInfo::getInstance()->setParticle( KuLBPFLocalizerPr::getInstance()->getParticle() );
	KuDrawingInfo::getInstance()->setLocalPath(KuObstAvoidancePr::getInstance()->getPath()); //경로 화면에 표시
}

/**
 @brief Korean: 경로에서 타겟점을 선택하는 함수
 @brief English: 
*/
KuPose GotoGoalBehavior::getStaticTargetPos(KuPose RobotPos,int nDistToTarget)
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

	if(nPathSize<1) return m_GoalPos;

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

		if(dDist>nDistToTarget&&nMinIdx>m_nPrePathIndx-1)
		{
			dMinDistfromPath=dDist;
			nselectedMinIndx=nMinIdx;
			m_nPrePathIndx=nMinIdx;
			break;
		}	
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


	m_nSelectedMinIndx = nselectedMinIndx;


	return TargetPos;
}

/**
 @brief Korean: 경로에서 타겟점을 선택하는 함수
 @brief English: 
*/
KuPose GotoGoalBehavior::getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist)
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
 @brief Korean: 센서데이터를 이용하여 타겟점과 로봇 사이의 장애물 유무를 검사하는 함수
 @brief English: 
*/
bool GotoGoalBehavior::checkObstacles(KuPose RobotPos,KuPose TargetPos,int_1DArray nLaserData181, double dTargetDist)
{

	double dGradientX = TargetPos.getXm() - RobotPos.getXm();
	double dGradientY = TargetPos.getYm() - RobotPos.getYm();
	double dDistofObst=hypot(TargetPos.getX()- RobotPos.getX(), TargetPos.getY()- RobotPos.getY());
	double dDistObstoRobot=dTargetDist;
	double dTempDistObstoRobot=dTargetDist;
	int nDistIndx=(int)dDistObstoRobot/100;
	double dOffset= (double)KuRobotParameter::getInstance()->getKinectXOffset();
	int nradiusofRobot=KuRobotParameter::getInstance()->getRobotRadius();
	int nDetectSenorrange= 60;
	int ninitSenorrange= (Sensor::URG04LX_DATA_NUM181 -nDetectSenorrange)/2+1;


	for (int ndistance = nDistIndx; ndistance <dDistObstoRobot; ndistance += nDistIndx) {

		double dRayOfX =  RobotPos.getX()+ ndistance*dGradientX;
		double dRayOfY = RobotPos.getY()+ ndistance*dGradientY;

		dTempDistObstoRobot=hypot(ndistance*dGradientX, ndistance*dGradientY);

		for(int i=0;i<nDetectSenorrange;i++){
			
			if(nLaserData181[i+ninitSenorrange]<50)continue;

		 if((nLaserData181[i+ninitSenorrange] != 1000000||nLaserData181[i+ninitSenorrange]>50)&&nLaserData181[i+ninitSenorrange]<dDistofObst){
				double dAngleRad = (double)(i -  nDetectSenorrange/2) * D2R;
				double dX = RobotPos.getX() + ((double)nLaserData181[i+ninitSenorrange]* cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
					((double)nLaserData181[i+66]* sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());
				double dY = RobotPos.getY() + ((double)nLaserData181[i+ninitSenorrange] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
					((double)nLaserData181[i+ninitSenorrange] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

				double tempD=hypot(dRayOfX-dX, dRayOfY- dY);

				if(tempD<nradiusofRobot&&tempD>0 ){	return true;	}// 거리에 따흔 장애물 감지
			}
		}
	}	
	return false;
}

/**
 @brief Korean: 센서데이터를 이용하여 타겟점과 로봇 사이의 장애물 유무를 검사하는 함수
 @brief English: 
*/
bool GotoGoalBehavior::checkObstacles(KuPose RobotPos,KuPose TargetPos,int_1DArray nLaserData181, int_1DArray nKinectRnageData ,double dTargetDist)
{

	double dGradientX = TargetPos.getXm() - RobotPos.getXm();
	double dGradientY = TargetPos.getYm() - RobotPos.getYm();
	double dDistofObst=hypot(TargetPos.getX()- RobotPos.getX(), TargetPos.getY()- RobotPos.getY());
	double dDistObstoRobot=dTargetDist;
	double dTempDistObstoRobot=dTargetDist;
	int nDistIndx=(int)dDistObstoRobot/100;
	double dOffset= (double)KuRobotParameter::getInstance()->getKinectXOffset();
	int nradiusofRobot=KuRobotParameter::getInstance()->getRobotRadius();

	for (int ndistance = nDistIndx; ndistance <dDistObstoRobot; ndistance += nDistIndx) {

		double dRayOfX =  RobotPos.getX()+ ndistance*dGradientX;
		double dRayOfY = RobotPos.getY()+ ndistance*dGradientY;

		dTempDistObstoRobot=hypot(ndistance*dGradientX, ndistance*dGradientY);

		for(int i=0;i<Sensor::KINECT_SENSOR_FOV;i++){
			
			if(nLaserData181[i+66]<50)continue;
			if(nKinectRnageData[i] <50)continue;

			if((nKinectRnageData[i] != 1000000||nKinectRnageData[i] >50)&&nKinectRnageData[i] <dDistofObst){
				double dAngleRad = (double)(i -  Sensor::KINECT_SENSOR_FOV/2) * D2R;
				double dX = RobotPos.getX() + ((double)nKinectRnageData[i] * cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
					((double)nKinectRnageData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());
				double dY = RobotPos.getY() + ((double)nKinectRnageData[i] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
					((double)nKinectRnageData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());
				
				double tempD=hypot(dRayOfX-dX, dRayOfY- dY);

				if(tempD<nradiusofRobot&&tempD>0 ){	return true;	}// 거리에 따흔 장애물 감지
			}
			else if((nLaserData181[i+66] != 1000000||nLaserData181[i+66]>50)&&nLaserData181[i+66]<dDistofObst){
				double dAngleRad = (double)(i -  Sensor::KINECT_SENSOR_FOV/2) * D2R;
				double dX = RobotPos.getX() + ((double)nLaserData181[i+66]* cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
					((double)nLaserData181[i+66]* sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());
				double dY = RobotPos.getY() + ((double)nLaserData181[i+66] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
					((double)nLaserData181[i+66] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

				double tempD=hypot(dRayOfX-dX, dRayOfY- dY);

				if(tempD<nradiusofRobot&&tempD>0 ){	return true;	}// 거리에 따흔 장애물 감지
			}
		}
	}	
	return false;
}

/**
@brief Korean: 웨이포인트에 타겟포인트가 도달했는지 검사하는 함수
@brief English: 
*/
KuPose GotoGoalBehavior::checkWayPoint(KuPose TargetPos,KuPose RobotPos, int *nselectIdx ,bool *bWaitflag)
{
	int nWayPointIdx=0;
	int nPathIdx=0;
	double dWayPointX;
	double dWayPointY;
	double dDist=INFINITY_VALUE;

	int nWayPointSize = m_vecWayPoint.size();
	int nPathSize = m_vecPath.size();
	/*m_vecWayPoint[0].setID(1);*/

	KuPose selectPos;
	if((*bWaitflag)==false){

		if(m_nSelectedMinIndx>nPathSize-1) m_nSelectedMinIndx=nPathSize-1;

		for(int nPathIdx=m_nSelectedMinIndx; nPathIdx>1;nPathIdx--)
		{
			for(int nWayPointIdx=1; nWayPointIdx<nWayPointSize;nWayPointIdx++)
			{

				dWayPointX = m_vecWayPoint[nWayPointIdx].getX();
				dWayPointY = m_vecWayPoint[nWayPointIdx].getY();

				if(nPathSize-1<nPathIdx) continue;

				dDist=hypot(m_vecPath[nPathIdx].getX()-dWayPointX, m_vecPath[nPathIdx].getY()- dWayPointY);

				if(dDist < 100&&m_vecWayPoint[nWayPointIdx].getID()!=1&&m_vecWayPoint[nWayPointIdx-1].getID()==1)
				{
					(*bWaitflag)=true;
					(*nselectIdx)=nWayPointIdx;		
				}	
			}
		}

		if((*bWaitflag)==true)
		{	
			if(nWayPointSize-1<(*nselectIdx)) return TargetPos;

			m_vecWayPoint[(*nselectIdx)].setID(1);
			selectPos.setX( m_vecWayPoint[(*nselectIdx)].getX());
			selectPos.setY( m_vecWayPoint[(*nselectIdx)].getY() );
			selectPos.setPro( m_vecWayPoint[(*nselectIdx)].getPro() );
			return selectPos;
		}
	}
	return TargetPos;
}
/**
 @brief Korean: 현재 사용중인  Localizer를 넘겨주는 함수
 @brief English: 
*/
Localizer* GotoGoalBehavior::getLocalizer()
{
	return KuLBPFLocalizerPr::getInstance();

}

/**
 @brief Korean: 현재 Behavior의 상태를 나타내는 함수
 @brief English: 
*/
bool GotoGoalBehavior::getBehaviorStates()
{
	return m_bThreadFlag;
}

/**
 @brief Korean: 클래스를 종료한다.
 @brief English: 
*/
void GotoGoalBehavior::terminate()
{
	m_Thread.terminate();
	PioneerWheelActuatorInterface::getInstance()->stop();
	m_bThreadFlag = false;	

}
/**
@brief Korean: 소요 시간을 측정하기 위해서 초기화하는 함수
@brief English: Initializes to count the duration
*/
void GotoGoalBehavior::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}
/**
@brief Korean: 측정된 소요 시간을 받아오는 함수
@brief English: Gets the estimated duration
*/
float GotoGoalBehavior::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}
/**
 @brief Korean: Localizer를 시작한다.
 @brief English: 
*/
void GotoGoalBehavior::startLocalizer(KuPose RobotPos)
{
	//INI 파일로부터 Particle filter을 수행하기 위해 필요한 정보를 얻어온다.------------------------------------
	int nMaxSampleNum = KuRobotParameter::getInstance()->getMaxParticleNum();
	int nMinSampleNum = KuRobotParameter::getInstance()->getMinParticleNUm();
	double dDevationforTrans = KuRobotParameter::getInstance()->getDeviationforTrans();
	double dDevationforRotae = KuRobotParameter::getInstance()->getDeviationforRotate();
	double dDevationforTransRotae = KuRobotParameter::getInstance()->getDeviationforTransRotate();
	//==================================================================================================================
	KuLBPFLocalizerPr::getInstance()->setDeviation(dDevationforTrans,dDevationforRotae,dDevationforTransRotae);
	KuLBPFLocalizerPr::getInstance()->setSampleNum(nMaxSampleNum,nMinSampleNum);

	int** nMap = kuMapRepository::getInstance()->getMap()->getMap();
	KuLBPFLocalizerPr::getInstance()->setMap(KuDrawingInfo::getInstance()->getMapSizeX(), KuDrawingInfo::getInstance()->getMapSizeY(),nMap);
	KuLBPFLocalizerPr::getInstance()->setRobotPos(RobotPos);	
 	KuLBPFLocalizerPr::getInstance()->spreadParticleNearRobot(RobotPos,0.3);		
 	KuLBPFLocalizerPr::getInstance()->start(50);	


}

/**
 @brief Korean: GotoGoalBehavior를 실행시키는 함수
 @brief English: 
*/
bool GotoGoalBehavior::execute(KuCommandMessage CMessage )
{
	
	switch(CMessage.getCommandName()){

	case KuCommandMessage::START_THREAD:	
		if(initialize()) 
			m_Thread.start(doThread,this,100); //메인 스레드 시작		
		break;
	case KuCommandMessage::TERMINATE_THREAD:
		terminate();
		break;
	case KuCommandMessage::SUSPEND_THREAD:
		m_Thread.suspend();
		break;
	case KuCommandMessage::RESUME_THREAD:
		m_Thread.resume();
		break;
	default:break;
	}

	return true;
}
bool GotoGoalBehavior::initPathplanningProcess()
{	
	 //목적지까지의 경로를 생성하기 위한 과정---------------------------------------------
    m_RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
    m_GoalPos = KuDrawingInfo::getInstance()->getGoalPos();

    //vector<POIMapPara>  vecPIOMapData=KuPOIMapParameter::getInstance()->getvecPIOMapData();
   // m_KuGPathPlannerPr.setPOIMap(vecPIOMapData);
    //int nSize=vecPIOMapData.size();


    m_KuGPathPlannerPr.setRadiusofRobot(Sensor::ROBOT_RADIUS+250,Sensor::CELLSIZE);
    m_KuGPathPlannerPr.initializeMapSize(kuMapRepository::getInstance()->getMap()->getX(),
                                         kuMapRepository::getInstance()->getMap()->getY());
     m_KuGPathPlannerPr.setGridMap(kuMapRepository::getInstance()->getMap()->getMap());
     cout<<"MapX"<<kuMapRepository::getInstance()->getMap()->getX()<<"MapY"<<kuMapRepository::getInstance()->getMap()->getY()<<endl;
/*
    for(int i=0; i<nSize;i++)
        {
            int nCurFloor=vecPIOMapData[i].Floor;
            string strMapNameNPath = KuRobotParameter::getInstance()->getMapNameNPath();
            memset(cMapPathName,0,sizeof(cMapPathName));
            sprintf_s(cMapPathName,"%s/IH_%dF.png", strMapNameNPath.c_str(),nCurFloor);
            KuRobotParameter::getInstance()->setCurfloor(nCurFloor);
            int** nMap=loadMap(cMapPathName);
            m_KuGPathPlannerPr.setGridMap(nMap);

            if(i==0){ m_KuGPathPlannerPr.setPOMap(dPOMap);}
            else
            {
                double** dMap=NULL;
                dMap = new double*[nMapSizeX];
                if(dMap){for(int i = 0 ; i < nMapSizeX ; i++){dMap[i] = new double[nMapSizeY];}	}
                m_KuGPathPlannerPr.setPOMap(dMap);
            }
        }
 */
    m_KuGPathPlannerPr.generatePath(m_GoalPos.getXm(),m_GoalPos.getYm(),m_GoalPos.getID(),m_RobotPos.getXm(),m_RobotPos.getYm(),m_RobotPos.getID());
    m_listPath=m_KuGPathPlannerPr.getPath();

    if(m_listPath.size()==0) return false; //경로생성 실패의 경우
/*
    m_KuPathPlanner.initializeMapSize(kuMapRepository::getInstance()->getMap()->getX(),
                                      kuMapRepository::getInstance()->getMap()->getY()
                                     );


    m_KuPathPlanner.setMap(kuMapRepository::getInstance()->getMap());
    m_KuPathPlanner.generatePath(m_GoalPos, m_RobotPos); //경로 생성
      list<KuPose> listPath = m_KuPathPlanner.getPath();
    if(listPath.size()==0) return false; //경로생성 실패의 경우
    m_listPath=m_KuPathSmoothing.smoothingPath(listPath);
*/
    list<KuPose>::iterator it;
    for(it=m_listPath.begin(); it!=m_listPath.end(); it++){
        m_vecPath.push_back(*it);
    }
    //경로생성 완료==================================================================

    KuDrawingInfo::getInstance()->setPath(m_listPath); //경로 화면에 표시


    KuObstAvoidancePr::getInstance()->setPath(m_listPath);
    bool bflag= KuObstAvoidancePr::getInstance()->start(20);

	
	return true; 
}

// bool GotoGoalBehavior::initPathplanningProcess()
// {	
// 	m_listWayPoint=KuDrawingInfo::getInstance()->getWayPointList();
// 	m_listPath=KuDrawingInfo::getInstance()->getPath();
// 	//load path & waypoint -----------------------------------------------------------------------------
// 
// 	if(KuDrawingInfo::getInstance()->getDirectionofPathflag()==false)
// 	{
// 		m_listPath.reverse();
// 		m_listWayPoint.reverse();
// 		KuDrawingInfo::getInstance()->setPath(m_listPath);
// 		KuDrawingInfo::getInstance()->setWayPointList(m_listWayPoint);	
// 		//KuDrawingInfo::getInstance()->setDirectionofPathflag(false);
// 	}
// 
// 	//path planning start-----------------------------------------------------------------------------
// 	if(m_listPath.size()==0)
// 	{
// 		m_RobotPos =KuDrawingInfo::getInstance()->getRobotPos();
// 		m_GoalPos = KuDrawingInfo::getInstance()->getGoalPos();
// 		m_KuPathPlanner.initializeMapSize(kuMapRepository::getInstance()->getMap()->getX(),
// 			kuMapRepository::getInstance()->getMap()->getY()	);
// 		m_KuPathPlanner.setMap(kuMapRepository::getInstance()->getMap()); 
// 		m_KuPathPlanner.initIntCost((int)(KuRobotParameter::getInstance()->getRobotRadius()/((double)100.0))+1 );
// 		m_KuPathPlanner.generatePath(m_GoalPos, m_RobotPos); //경로 생성
// 		m_listPath = m_KuPathPlanner.getPath();
// 		if(m_listPath.size()==0){
// 			m_bThreadFlag = false;	
// 			return false; 
// 		}//경로생성 실패의 경우
// 		m_listPath=m_KuPathSmoothing.smoothingPath(m_listPath);
// 	}
// 	//path planning  end-----------------------------------------------------------------------------
// 
//  	list<KuPose>::iterator itway;
// 
// 	for(itway=m_listWayPoint.begin(); itway!=m_listWayPoint.end(); itway++){
// 		m_vecWayPoint.push_back(*itway);
// 	}
// 
// 	list<KuPose>::iterator it;
// 	for(it=m_listPath.begin(); it!=m_listPath.end(); it++){
// 		m_vecPath.push_back(*it);
// 	}
// 
// 	calStartPathPoint();
// 
// 	//Goal Drawing start-----------------------------------------------------------------------------
// 	int nPathSize = m_vecPath.size()-1;
// 	double dPathX = m_vecPath[nPathSize].getX();
// 	double dPathY = m_vecPath[nPathSize].getY();
// 	m_GoalPos.setX(dPathX);
// 	m_GoalPos.setY(dPathY);
// 	KuDrawingInfo::getInstance()->setGoalPos(m_GoalPos);
// 	//Goal Drawing start-----------------------------------------------------------------------------
// 
// 
// 	//경로생성 완료==================================================================
// 	m_RobotPos =KuDrawingInfo::getInstance()->getRobotPos();
// 	KuDrawingInfo::getInstance()->setPath(m_listPath); //경로 화면에 표시
// 
// 	return true; 
// }
void GotoGoalBehavior::calStartPathPoint( )
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
KuPose GotoGoalBehavior::getCurWayPoint()
{
	return m_CurWayPoint;
}