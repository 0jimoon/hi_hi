#include "stdafx.h"
#include "MapBuildingBehavior.h"

MapBuildingBehavior::MapBuildingBehavior()
{
	initialize(); //초기화 작업
	cout<<"[MapBuildingBehavior]: Instance is created!!!"<<endl;
	m_pMap=NULL;
	m_IplCeilingCamera = cvCreateImage(cvSize( Sensor::CEILING_IMAGE_WIDTH, Sensor::CEILING_IMAGE_HEIGHT),8,1);
	m_IplKinectCamera = cvCreateImage(cvSize( Sensor::CEILING_IMAGE_WIDTH, Sensor::CEILING_IMAGE_HEIGHT),8,3);
}

MapBuildingBehavior::~MapBuildingBehavior()
{
	cout<<"[MapBuildingBehavior]: Instance is destroyed!!!"<<endl;
	if(m_pMap!=NULL){
		delete m_pMap;

	}
}

/**
@brief Korean: 초기화 작업을 수행하는 함수.
@brief English: 
*/
void MapBuildingBehavior::initialize()
{
	m_bThreadFlag = false;

}
/**
@brief Korean: 초기화 작업을 수행하는 함수.
@brief English: 
*/
void MapBuildingBehavior::initialize(KuCommandMessage CMessage)
{
	m_nLaserData = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,3900); //거리센서 정보를 저장하는 변수 초기화

	m_bThreadFlag = true;
	m_RobotPos=CMessage.getRobotPos();
	m_nThreadFuncPeriod = CMessage.getBehaviorPeriod(); //스레드 함수 실행주기 입력.

	m_nMapSizeXm =0,m_nMapSizeYm =0;
	CMessage.getMapSizeXmYm(&m_nMapSizeXm, &m_nMapSizeYm);
	// 	m_RobotPos.setXm(m_nMapSizeXm/2);
	// 	m_RobotPos.setYm(m_nMapSizeYm/2);
	int nMapSizeXAI=0;
	int nMapSizeYAI=0;

	m_Math.setCellSizeMM(Sensor::CELLSIZE);

	nMapSizeXAI=m_Math.MM2AI(m_nMapSizeXm*1000.0);
	nMapSizeYAI=m_Math.MM2AI(m_nMapSizeYm*1000.0);


	if(NULL==m_pMap)
	{
		//initialize map------------------------------------------------------------------
		m_pMap = new KuMap(nMapSizeXAI, nMapSizeYAI); 
		KuDrawingInfo::getInstance()->setMap(m_pMap); 
		//initialize map===================================================
	}

	double dThicknessofWall=50;
	string strUpdateSpeed="no";

	KuMapBuilderParameter InputParam;
	InputParam.setMapSizeXmYm(m_nMapSizeXm, m_nMapSizeYm);
	InputParam.setMapSizeXYAI(nMapSizeXAI, nMapSizeYAI);
	InputParam.setLaserScanIdx(Sensor::URG04LX_DATA_NUM181);
	InputParam.setCellSize(Sensor::CELLSIZE);
	InputParam.setMinDistofSensorData(KuRobotParameter::getInstance()->getURG04LXLaserMinDist()); // unit mm
	InputParam.setMaxDistofSensorData(KuRobotParameter::getInstance()->getURG04LXLaserMaxDist()); // unit mm
	InputParam.setLaserXOffset(KuRobotParameter::getInstance()->getURG04LXLaserXOffset());
	m_LaserMapBuilder.initialize(InputParam);

	initICPProcess();

}

/**
@brief Korean: ICP 프로세스를 수행하기 위해 필요한 초기화 작업을 수행하는 함수.
@brief English: 
*/
void MapBuildingBehavior::initICPProcess()
{
	m_ICPLocalizer.setInitRobotPos(m_RobotPos);
	SensorSupervisor::getInstance()->readSensorData(); //sensor reading
	int_1DArray nLaserData = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,3900);
	KuPose EncoderDelPos;

	nLaserData = SensorSupervisor::getInstance()->getURG04LXLaserData();
	EncoderDelPos =SensorSupervisor::getInstance()->getEncoderDelPos();

}

/**
@brief Korean: 스레드로 돌아가는 함수
@brief English: 
*/
void MapBuildingBehavior::doThread(void* arg)
{
	MapBuildingBehavior* pMBB = (MapBuildingBehavior*)arg;

	KuMapBuilderParameter InputParam; 			


	if(SensorSupervisor::getInstance()->readSensorData()==false) pMBB->terminate();	
	pMBB->m_nLaserData = SensorSupervisor::getInstance()->getURG04LXLaserData();
	pMBB->m_EncoderDelPos = SensorSupervisor::getInstance()->getEncoderDelPos();

	// 여기다 만들다

	pMBB->m_RobotPos = pMBB->m_ICPLocalizer.estimateRobotPos(pMBB->m_nLaserData, pMBB->m_EncoderDelPos);


	InputParam.setDelRobotPos(pMBB->m_EncoderDelPos); 
	InputParam.setRobotPos(pMBB->m_RobotPos);
	InputParam.setLaserData(pMBB->m_nLaserData);
	pMBB->m_LaserMapBuilder.buildMap(InputParam);		


	pMBB->drawNaviData();

}
/**
@brief Korean: 주행 관련 정보를 그려주는 함수.
@brief English: 
*/
void MapBuildingBehavior::drawNaviData()
{
	KuDrawingInfo::getInstance()->setLaserData181(m_nLaserData);
	KuDrawingInfo::getInstance()->setBuildingMap(m_LaserMapBuilder.getProMap());
	KuDrawingInfo::getInstance()->setRobotPos(m_RobotPos);		
}

/**
@brief Korean: 격자 지도를 BMP형태로 생성하는 함수
@brief English: 
*/
void MapBuildingBehavior::generateMapData2BMPFile()
{
	//격자지도를 BMP형태로 생성----------------------------------------------------------------------------
	m_pMap->setMap(m_LaserMapBuilder.getMap());
	//m_pMap->setMap(m_KuSRPMLaserMapBuilder.getMap());
	kuMapRepository::getInstance()->saveMap(m_pMap);
	KuDrawingInfo::getInstance()->setRenderBuildingMapflag(false);
	//-----------------------------------------------------------------------------------------------------
}

/**
@brief Korean: 클래스를 종료한다.
@brief English: 
*/
void MapBuildingBehavior::terminate()
{
	KuThread::terminate();
	if(m_bThreadFlag)generateMapData2BMPFile();
	//
	m_bThreadFlag = false;	

}
/**
@brief Korean: Behavior의 상태를 가져가는 함수
@brief English: 
*/
bool MapBuildingBehavior::getBehaviorStates()
{
	return m_bThreadFlag;
}
/**
@brief Korean: Behavior에서 사용되는 Localizer를 가져간다
@brief English: 
*/
Localizer* MapBuildingBehavior::getLocalizer()
{
	return &m_ICPLocalizer;
}

/**
@brief Korean: MapBuildingBehavior를 실행시키는 함수
@brief English: 
*/
bool MapBuildingBehavior::execute(KuCommandMessage CMessage)
{
	switch(CMessage.getCommandName()){
	case KuCommandMessage::START_THREAD:
		initialize(CMessage);		
		KuThread::start(doThread,this,200); //메인 스레드 시작			
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