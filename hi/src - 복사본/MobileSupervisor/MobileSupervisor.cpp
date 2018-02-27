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
 @brief Korean: �ʱ����� ������ �ҷ����� �Լ�
 @brief English: 
*/
bool MobileSupervisor::loadMap()
{
	char cMapPathName[200];

	//������ �ҷ� �´�--------------------------------------------------------------------------
	int nTotalFloorNum=KuRobotParameter::getInstance()->getTotalfloorNum();
	
	string strMapNameNPath = KuRobotParameter::getInstance()->getMapNameNPath();
	int nCurFloor=KuRobotParameter::getInstance()->getCurfloor();
	
	m_nCurFloor=nCurFloor;
	KuPose RobotPos;
	RobotPos.setXm( (double)KuRobotParameter::getInstance()->getMapSizeXm()/2. );
	RobotPos.setYm( (double)KuRobotParameter::getInstance()->getMapSizeYm()/2. );
	RobotPos.setID(nCurFloor);
	KuDrawingInfo::getInstance()->setRobotPos(RobotPos); //������ ���� �߾���ġ�� �κ��� �ʱ���ġ�� ����
	KuDrawingInfo::getInstance()->setCurFloor(m_nCurFloor);
	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/IH_%dF.png", strMapNameNPath.c_str(),nCurFloor);	
	POIMapPara PIOMapData;
	PIOMapData=KuPOIMapParameter::getInstance()->getPIOMapData(0);
 	bool bRes = kuMapRepository::getInstance()->loadMap(cMapPathName); ///���� �ε�
	if(bRes==true)KuDrawingInfo::getInstance()->setRenderMapflag(true);
	KuDrawingInfo::getInstance()->setPIOMapData(PIOMapData);

	m_KuFloorcheckThread.start(checkFloor,this,100); //���� ������ ����		

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
//	KuDrawingInfo::getInstance()->setRobotPos(RobotPos); //������ ���� �߾���ġ�� �κ��� �ʱ���ġ�� ����
	KuDrawingInfo::getInstance()->setCurFloor(m_nCurFloor);
	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/IH_%dF.png", strMapNameNPath.c_str(),nCurFloor);	
	KuRobotParameter::getInstance()->setCurfloor(nCurFloor);
	bool bRes = kuMapRepository::getInstance()->loadMap(cMapPathName); ///���� �ε�
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
 @brief Korean: �ʱ����� ������ �ҷ����� �Լ�
 @brief English: 
*/
bool MobileSupervisor::loadCADMap()
{
	//CAD ������ �ҷ� �´�--------------------------------------------------------------------------
 	string strMapNameNPath = KuRobotParameter::getInstance()->getCadMapNameNPath();
	int nMapX=KuRobotParameter::getInstance()->getMapSizeXm();
	int nMapY=KuRobotParameter::getInstance()->getMapSizeYm();
 	bool bRes =kuMapRepository::getInstance()->loadRefMap(strMapNameNPath,nMapX*10,nMapY*10); ///���� �ε�

	return bRes;
}

/**
@brief Korean: ���� ��������  Behavior�� ���¸� ��Ÿ���� �Լ�
@brief English: 
*/
bool MobileSupervisor::getBehaviorStates(KuCommandMessage CMessage)
{
	bool bBehaviorStates=false;
	int nBehaviorName = CMessage.getBehaviorName(); //������ ���� �̸��� �ε����� ������ �´�.
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
@brief Korean: ������� ��ɿ� ���� behavior���� ����� ������ �Լ�
@brief English: 
*/
void MobileSupervisor::execute(KuCommandMessage CMessage)
{

	int nBehaviorName = CMessage.getBehaviorName(); //������ ���� �̸��� �ε����� ������ �´�.
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
@brief Korean: ������� ��ɿ� ���� behavior���� ����� ������ �Լ�
@brief English: 
*/
void MobileSupervisor::executesim(KuCommandMessage CMessage)
{

	int nBehaviorName = CMessage.getBehaviorName(); //������ ���� �̸��� �ε����� ������ �´�.
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
 @brief Korean: ���� �������  Behavior�� Localizer�� ��Ÿ���� �Լ�
 @brief English: 
*/
Localizer* MobileSupervisor::getLocalizer(KuCommandMessage CMessage)
{
	Localizer* pLocalizer=NULL;
	int nBehaviorName = CMessage.getBehaviorName(); //������ ���� �̸��� �ε����� ������ �´�.

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
 @brief Korean: ���� ��������  Behavior�� ���¸� ��Ÿ���� �Լ�
 @brief English: 
*/
bool MobileSupervisor::getBehaviorStates( )
{
	return m_bBehaviorStates;
}
/**
 @brief Korean: ���� ��������  Behavior�� ���¸� ��Ÿ���� �Լ�
 @brief English: 
*/
void MobileSupervisor::setBehaviorStates( bool bBehaviorStates)
{
	 m_bBehaviorStates=bBehaviorStates;
}

/**
 @brief Korean: Autonomous�� �����Ű�� �Լ�
 @brief English: 
*/
bool MobileSupervisor::Autonomousexecute(KuCommandMessage CMessage )
{
	
	switch(CMessage.getCommandName()){

	case KuCommandMessage::START_THREAD:	
		m_bBehaviorStates=true;
		m_KuThread.start(AutonomousSupervisor,this,100); //���� ������ ����		
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
	@brief Korean: ������ �ִ� �������� �̵��� ����ϴ� �Լ�.
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