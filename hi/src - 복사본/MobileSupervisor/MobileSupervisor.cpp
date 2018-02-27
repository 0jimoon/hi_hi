#include "stdafx.h"
#include "MobileSupervisor.h"

MobileSupervisor::MobileSupervisor()
{
	m_bBehaviorStates=false;
	m_nNum=0;
}

MobileSupervisor::~MobileSupervisor()
{

}

/**
 @brief Korean: 초기지도 정보를 불러오는 함수
 @brief English: 
*/
bool MobileSupervisor::loadMap()
{
	char cMapPathName[200];

	//지도를 불러 온다--------------------------------------------------------------------------
	int nTotalFloorNum=KuRobotParameter::getInstance()->getTotalfloorNum();
	
	string strMapNameNPath = KuRobotParameter::getInstance()->getMapNameNPath();
	int nCurFloor=KuRobotParameter::getInstance()->getCurfloor();
	
	m_nCurFloor=nCurFloor;
	KuPose RobotPos;
	RobotPos.setXm( (double)KuRobotParameter::getInstance()->getMapSizeXm()/2. );
	RobotPos.setYm( (double)KuRobotParameter::getInstance()->getMapSizeYm()/2. );
	RobotPos.setID(nCurFloor);
	KuDrawingInfo::getInstance()->setRobotPos(RobotPos); //지도를 기준 중앙위치를 로봇의 초기위치로 설정
	KuDrawingInfo::getInstance()->setCurFloor(m_nCurFloor);
	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/IH_%dF.png", strMapNameNPath.c_str(),nCurFloor);	
	POIMapPara PIOMapData;
	PIOMapData=KuPOIMapParameter::getInstance()->getPIOMapData(0);
 	bool bRes = kuMapRepository::getInstance()->loadMap(cMapPathName); ///지도 로딩
	if(bRes==true)KuDrawingInfo::getInstance()->setRenderMapflag(true);
	KuDrawingInfo::getInstance()->setPIOMapData(PIOMapData);

	m_KuFloorcheckThread.start(checkFloor,this,100); //메인 스레드 시작		

	return bRes;
}

void MobileSupervisor::checkFloor(void* arg)
{
	if(MobileSupervisor::getInstance()->m_nCurFloor!=KuDrawingInfo::getInstance()->getCurFloor())	
	{
		vector<POIMapPara> vecPIOMapData;
		vecPIOMapData=KuPOIMapParameter::getInstance()->getvecPIOMapData();
		int nSelIDX=0;

		for(int i=0;i<vecPIOMapData.size();i++)
		{
			if(vecPIOMapData[i].Floor==KuDrawingInfo::getInstance()->getCurFloor())
			{
				nSelIDX=i;
			}
		}
		
		MobileSupervisor::getInstance()->setCurIndex(nSelIDX);
	}
}

bool MobileSupervisor::setCurIndex(int nIndex)
{
	char cMapPathName[200];
	POIMapPara PIOMapData;
	PIOMapData=KuPOIMapParameter::getInstance()->getPIOMapData(nIndex);
	string strMapNameNPath = KuRobotParameter::getInstance()->getMapNameNPath();
	int nCurFloor =PIOMapData.Floor;
	m_nCurFloor=nCurFloor;
// 	KuPose RobotPos=KuDrawingInfo::getInstance()->getRobotPos();
// 	RobotPos.setID(nCurFloor);
//	KuDrawingInfo::getInstance()->setRobotPos(RobotPos); //지도를 기준 중앙위치를 로봇의 초기위치로 설정
	KuDrawingInfo::getInstance()->setCurFloor(m_nCurFloor);
	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/IH_%dF.png", strMapNameNPath.c_str(),nCurFloor);	
	KuRobotParameter::getInstance()->setCurfloor(nCurFloor);
	bool bRes = kuMapRepository::getInstance()->loadMap(cMapPathName); ///지도 로딩
	if(bRes==true)KuDrawingInfo::getInstance()->setRenderMapflag(true);
	KuDrawingInfo::getInstance()->setPIOMapData(PIOMapData);
	return bRes;
}

bool MobileSupervisor::loadupdateGridMap()
{
	bool bRes;
	int**  nRefMap = KuDrawingInfo::getInstance()->getMap();
	if(NULL==nRefMap) return false;

	int nMapSizeX= KuDrawingInfo::getInstance()->getMapSizeX();
	int nMapSizeY =KuDrawingInfo::getInstance()->getMapSizeY();

	int nMapGridSizeX=nMapSizeX/10.0;
	int nMapGridSizeY=nMapSizeY/10.0;

	int** nMap;

	nMap = new int*[nMapGridSizeX];

	if(nMap){
		for(int i = 0 ; i < nMapGridSizeX; i++){
			nMap[i] = new int[nMapGridSizeY];
		}
	}

	for(int nSizeX=0; nSizeX<nMapGridSizeX;nSizeX++)
	{
		for(int nSizeY=0; nSizeY<nMapGridSizeY;nSizeY++)
		{
			nMap[nSizeX][nSizeY]=2;
		}
	}


	for(int nSizeX=0; nSizeX<nMapSizeX;nSizeX++)
	{
		for(int nSizeY=0; nSizeY<nMapSizeY;nSizeY++)
		{
			if(nRefMap[nSizeX][nSizeY]==KuMap::OCCUPIED_AREA)
			{
				nMap[(int)(nSizeX/10.0+0.5)][(int)(nSizeY/10.0+0.5)]=KuMap::OCCUPIED_AREA;
			}
			else if(nRefMap[nSizeX][nSizeY]==KuMap::EMPTY_AREA)
			{
				if(nMap[(int)(nSizeX/10.0+0.5)][(int)(nSizeY/10.0+0.5)]!=KuMap::OCCUPIED_AREA)
				{
					nMap[(int)(nSizeX/10.0+0.5)][(int)(nSizeY/10.0+0.5)]=KuMap::EMPTY_AREA;
				}
			}
		}
	}

	nMapSizeX=(int)nMapSizeX/10.0;
	nMapSizeY=(int)nMapSizeX/10.0;


//	KuDrawingInfo::getInstance()->setPathMap(nMap, nMapSizeX/10.0, nMapSizeY/10.0);

	return true;
}


/**
 @brief Korean: 초기지도 정보를 불러오는 함수
 @brief English: 
*/
bool MobileSupervisor::loadCADMap()
{
	//CAD 지도를 불러 온다--------------------------------------------------------------------------
 	string strMapNameNPath = KuRobotParameter::getInstance()->getCadMapNameNPath();
	int nMapX=KuRobotParameter::getInstance()->getMapSizeXm();
	int nMapY=KuRobotParameter::getInstance()->getMapSizeYm();
 	bool bRes =kuMapRepository::getInstance()->loadRefMap(strMapNameNPath,nMapX*10,nMapY*10); ///지도 로딩

	return bRes;
}

/**
@brief Korean: 현재 수행중인  Behavior의 상태를 나타내는 함수
@brief English: 
*/
bool MobileSupervisor::getBehaviorStates(KuCommandMessage CMessage)
{
	bool bBehaviorStates=false;
	int nBehaviorName = CMessage.getBehaviorName(); //행위에 대한 이름을 인덱스로 가지고 온다.
	switch(nBehaviorName)
	{	
	case KuCommandMessage::HYBRID_MAP_BUILDING_BH:
		bBehaviorStates = m_MapBuildingBeh.getBehaviorStates();
		break;
	case KuCommandMessage::GOTOGOAL_BH:
		bBehaviorStates=m_GotoGoalBeh.getBehaviorStates();
		break;

	case KuCommandMessage::VIRTUAL_GOTOGOAL_BH:
		bBehaviorStates=m_VrKanayamaMotionControlBeh.getBehaviorStates();
		break;	

	case KuCommandMessage::AUTONOMOUS_GOTOGOAL:
		bBehaviorStates=getBehaviorStates();
		break;
	default:break;
	}

	return bBehaviorStates;
}

/**
@brief Korean: 사용자의 명령에 따라 behavior에세 명령을 내리는 함수
@brief English: 
*/
void MobileSupervisor::execute(KuCommandMessage CMessage)
{

	int nBehaviorName = CMessage.getBehaviorName(); //행위에 대한 이름을 인덱스로 가지고 온다.
	int nBehaviorPeriod = CMessage.getBehaviorPeriod();
	KuPose GoalPos = CMessage.getGoalPos();
	KuPose RobotPos = CMessage.getRobotPos();
		
	switch(nBehaviorName){	
		case KuCommandMessage::HYBRID_MAP_BUILDING_BH:
			m_MapBuildingBeh.execute(CMessage);	
			break;
		case KuCommandMessage::GOTOGOAL_BH:
			m_GotoGoalBeh.execute(CMessage);
			break;
		case KuCommandMessage::VIRTUAL_GOTOGOAL_BH:
			m_VrKanayamaMotionControlBeh.execute(CMessage);
			break;
		case KuCommandMessage::AUTONOMOUS_GOTOGOAL:
			Autonomousexecute(CMessage);	
			break;
		default:break;
	}
}

/**
@brief Korean: 사용자의 명령에 따라 behavior에세 명령을 내리는 함수
@brief English: 
*/
void MobileSupervisor::executesim(KuCommandMessage CMessage)
{

	int nBehaviorName = CMessage.getBehaviorName(); //행위에 대한 이름을 인덱스로 가지고 온다.
	int nBehaviorPeriod = CMessage.getBehaviorPeriod();
	KuPose GoalPos = CMessage.getGoalPos();
	KuPose RobotPos = CMessage.getRobotPos();

	switch(nBehaviorName){	
	case KuCommandMessage::WANDERING_OBSTACLE_PR:
		KuWanderingObstaclePr::getInstance()->execute(CMessage);
		break;			
	default:break;
	}
}

/**
 @brief Korean: 현재 사용중인  Behavior의 Localizer를 나타내는 함수
 @brief English: 
*/
Localizer* MobileSupervisor::getLocalizer(KuCommandMessage CMessage)
{
	Localizer* pLocalizer=NULL;
	int nBehaviorName = CMessage.getBehaviorName(); //행위에 대한 이름을 인덱스로 가지고 온다.

	switch(nBehaviorName){
		case KuCommandMessage::HYBRID_MAP_BUILDING_BH:
			pLocalizer=m_MapBuildingBeh.getLocalizer();	
			break;
		case KuCommandMessage::GOTOGOAL_BH:
			pLocalizer=m_GotoGoalBeh.getLocalizer();
			break;
		default:break;
	}

	return pLocalizer;
}

/**
 @brief Korean: 현재 수행중인  Behavior의 상태를 나타내는 함수
 @brief English: 
*/
bool MobileSupervisor::getBehaviorStates( )
{
	return m_bBehaviorStates;
}
/**
 @brief Korean: 현재 수행중인  Behavior의 상태를 나타내는 함수
 @brief English: 
*/
void MobileSupervisor::setBehaviorStates( bool bBehaviorStates)
{
	 m_bBehaviorStates=bBehaviorStates;
}

/**
 @brief Korean: Autonomous를 실행시키는 함수
 @brief English: 
*/
bool MobileSupervisor::Autonomousexecute(KuCommandMessage CMessage )
{
	
	switch(CMessage.getCommandName()){

	case KuCommandMessage::START_THREAD:	
		m_bBehaviorStates=true;
		m_KuThread.start(AutonomousSupervisor,this,100); //메인 스레드 시작		
		break;
	case KuCommandMessage::TERMINATE_THREAD:
		m_bBehaviorStates=false;
		m_KuThread.terminate();
		m_VrKanayamaMotionControlBeh.terminate();
		break;
	case KuCommandMessage::SUSPEND_THREAD:
		m_KuThread.suspend();
		break;
	case KuCommandMessage::RESUME_THREAD:
		m_KuThread.resume();
		break;
	default:break;
	}

	return true;
}
/**
	@brief Korean: 정해져 있는 구간간의 이동을 명령하는 함수.
	@brief English: 
*/
void MobileSupervisor::AutonomousSupervisor(void* arg)
{

	MobileSupervisor::getInstance()->setBehaviorStates(true);
	KuCommandMessage CMessage;
	CMessage.setCommandName(KuCommandMessage::START_THREAD);
	CMessage.setBehaviorName(KuCommandMessage::VIRTUAL_GOTOGOAL_BH);
	CMessage.setGoalPos(KuDrawingInfo::getInstance()->getGoalPos());
	CMessage.setRobotPos(KuDrawingInfo::getInstance()->getRobotPos());		 
	MobileSupervisor::getInstance()->execute(CMessage);
	
	if(MobileSupervisor::getInstance()->m_vecGoalPos.size()<1)
	{
		MobileSupervisor::getInstance()->m_vecGoalPos.push_back(KuDrawingInfo::getInstance()->getRobotPos());
		MobileSupervisor::getInstance()->m_vecGoalPos.push_back(KuDrawingInfo::getInstance()->getGoalPos());
	}
	
	//KuPose GoalPos=KuDrawingInfo::getInstance()->getRobotPos();

	Sleep(1000);

	while(MobileSupervisor::getInstance()->getBehaviorStates(CMessage))
	{
		Sleep(1000);
	}	
	KuDrawingInfo::getInstance()->setGoalPos(MobileSupervisor::getInstance()->m_vecGoalPos[MobileSupervisor::getInstance()->m_nNum]);
	MobileSupervisor::getInstance()->m_nNum++;
	if(MobileSupervisor::getInstance()->m_nNum>1)
	{
		MobileSupervisor::getInstance()->m_nNum=0;
	}
	Sleep(200);
	

}