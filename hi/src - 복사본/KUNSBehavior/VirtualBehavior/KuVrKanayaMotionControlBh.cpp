#include "StdAfx.h"
#include "KuVrKanayaMotionControlBh.h"


KuVrKanayaMotionControlBh::KuVrKanayaMotionControlBh()
{

	cout<<"[KuVrKanayaMotionControlBh]: Instance is created!!!"<<endl;
}

KuVrKanayaMotionControlBh::~KuVrKanayaMotionControlBh()
{
	cout<<"[KuVrKanayaMotionControlBh]: Instance is destroyed!!!"<<endl;
}
/**
 @brief Korean: 현재 Behavior의 상태를 나타내는 함수
 @brief English: 
*/
bool KuVrKanayaMotionControlBh::getBehaviorStates()
{
	return m_bThreadFlag;
}

/**
 @brief Korean: 초기화 작업을 수행하는 함수.
 @brief English: 
*/
void KuVrKanayaMotionControlBh::initialize(KuCommandMessage CMessage)
{
	m_bThreadFlag = false;
	m_vecPath.clear();
	m_bThreadFlag = true;
	m_dpreVel=0.0;
	m_dpreRVel=0.0;
	m_RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	m_GoalPos =  KuDrawingInfo::getInstance()->getGoalPos();
	m_nThreadFuncPeriod = CMessage.getBehaviorPeriod(); //스레드 함수 실행주기 입력
	m_nMapX=kuMapRepository::getInstance()->getMap()->getX();
	m_nMapY=kuMapRepository::getInstance()->getMap()->getY();
	//path planning start-----------------------------------------------------------------------------
	m_KuPathPlanner.initializeMapSize(m_nMapX,m_nMapY);
	m_KuPathPlanner.setMap(kuMapRepository::getInstance()->getMap()->getMap(),Sensor::CELLSIZE); 
	m_KuPathPlanner.initIntCost((int)(KuRobotParameter::getInstance()->getRobotRadius()/((double)100.0))+1);
	m_KuPathPlanner.generatePath(m_GoalPos, m_RobotPos); //경로 생성
	m_listPath = m_KuPathPlanner.getPath();
	if(m_listPath.size()==0){
		m_bThreadFlag = false;	
	}//경로생성 실패의 경우
	m_listPath=m_KuPathSmoothing.smoothingPath(m_listPath);
	//path planning  end-----------------------------------------------------------------------------

	list<KuPose>::iterator it;
	for(it=m_listPath.begin(); it!=m_listPath.end(); it++){
		m_vecPath.push_back(*it);
	}
	
	m_KuVrWheelActuator.setRobotPos( m_RobotPos );

	//경로생성 완료==================================================================
	KuDrawingInfo::getInstance()->setPath(m_listPath); //경로 화면에 표시

 	initKanayamaProcess();
	
	KuVrHokuyoUTM30LXInterface::getInstance()->connect(kuMapRepository::getInstance()->getMap()->getMap(),
		kuMapRepository::getInstance()->getMap()->getX(),kuMapRepository::getInstance()->getMap()->getY());

	KuLocalPathPlannerPr::getInstance()->setPath(m_listPath);
	bool bflag= KuLocalPathPlannerPr::getInstance()->start(20);

}

/**
@brief Korean: Kanayama  프로세스를 수행하기 위해 필요한 초기화 작업을 수행하는 함수.
@brief English: 
*/
void KuVrKanayaMotionControlBh::initKanayamaProcess()
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
void KuVrKanayaMotionControlBh::doThread(void* arg)
{
	KuVrKanayaMotionControlBh* pKMC = (KuVrKanayaMotionControlBh*)arg;

	KuVelocity generatedVel;
	double dTargetDist=0.0;
	vector<KuPose> vecWanderObs;
	vecWanderObs= KuWanderingObstaclePr::getInstance()->getvecMultiWanderingObstacles();

	pKMC->m_RobotPos  = pKMC->m_KuVrWheelActuator.getRobotPos();
	pKMC->m_nLaserData181= KuVrHokuyoUTM30LXInterface::getInstance()->getData181(pKMC->m_RobotPos,vecWanderObs);
	
		
	if(pKMC->m_math.calcDistBetweenPoses(pKMC->m_GoalPos,pKMC->m_RobotPos) <pKMC->m_nGoalArea){
		pKMC->terminate();
		return;
	}

	else if(pKMC->m_math.calcDistBetweenPoses(pKMC->m_GoalPos,pKMC->m_RobotPos) <GOAL_BOUNDARY){
		pKMC->m_nDesiredVel=100;//goal 주변 속도 감소
	}
	
	pKMC->m_RobotPos.setDist(pKMC->m_dpreVel);
	pKMC->m_RobotPos.setPro(pKMC->m_dpreRVel);
	KuLocalPathPlannerPr::getInstance()->setData(pKMC->m_RobotPos, vecWanderObs);

	pKMC->m_TargetPos=KuLocalPathPlannerPr::getInstance()->getTargetPos();
		
	generatedVel = pKMC->m_KanayaMC.generateTRVelocity(pKMC->m_TargetPos, pKMC->m_RobotPos, 100 );
	
	pKMC->m_KuVrWheelActuator.moveTRVelocity(generatedVel.m_nTranslationVel, generatedVel.m_nRotationDegVel);
	pKMC->m_dpreVel=(double)(generatedVel.m_nTranslationVel)/1000.0;
	pKMC->m_dpreRVel=(double)(generatedVel.m_nRotationDegVel);	
	pKMC->drawNaviData();

	vecWanderObs.clear();
}
/**
@brief Korean: 주행 관련 정보를 그려주는 함수.
@brief English: 
*/
void KuVrKanayaMotionControlBh::drawNaviData()
{
	//KuDrawingInfo::getInstance()->setMap(KuLocalPathPlannerPr::getInstance()->getCautionMap(),m_nMapX,m_nMapY);
	//KuDrawingInfo::getInstance()->setpredictedProMap(KuLocalPathPlannerPr::getInstance()->getpredictedProMap(),m_nMapX,m_nMapY);
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos); //로봇 위치 화면에 표시		
	KuDrawingInfo::getInstance()->setTargetPos(m_TargetPos); //타겟 위치 화면에 표시
	KuDrawingInfo::getInstance()->setLocalPath(KuLocalPathPlannerPr::getInstance()->getPath());
	KuDrawingInfo::getInstance()->setLaserData181(m_nLaserData181);

}
KuPose KuVrKanayaMotionControlBh::getStaticTargetPos(KuPose RobotPos,int nDistToTarget)
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
KuPose KuVrKanayaMotionControlBh::getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist)
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
void KuVrKanayaMotionControlBh::terminate()
{

	KuThread::terminate();
	m_bThreadFlag = false;	
	m_KuVrWheelActuator.moveTRVelocity(0,0);
	//KuKanayamaMotionControlBhSupervisor::getInstance()->notifyBehavirTermination();
	cout<<"[KuVrKanayaMotionControlBh]:: Behavior is terminated!!!"<<endl;

}


/**
 @brief Korean: KuVrKanayaMotionControlBh를 실행시키는 함수
 @brief English: 
*/

bool KuVrKanayaMotionControlBh::execute(KuCommandMessage CMessage)
{
	switch(CMessage.getCommandName()){
	case KuCommandMessage::START_THREAD:
		initialize(CMessage);	
		KuThread::start(doThread,this,m_nThreadFuncPeriod); //메인 스레드 시작	
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