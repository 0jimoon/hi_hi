#include "StdAfx.h"
#include "KuObstAvoidancePr.h"

KuObstAvoidancePr::KuObstAvoidancePr()
{
	m_vecPath.clear();
	//m_listPath.clear();
	m_vecWayPoint.clear();
	m_bThreadFlag = false;
	m_nPrePathIndx=-1;
	m_nLocalGoalIndex=3;
	m_nLocalGoalWayPointIndex=-1;
	m_pLocalMap=NULL;
	m_pOriginalMap=NULL;
	m_pMap= NULL;

	m_nCellSize=100;
	m_nDistToTarget=1200;

	m_nLaserDataIDX=181;
	m_dLaserSensorOffsetmm=200;
	m_dLaserMinDistanceMM=50;
	m_dLaserMaxDistanceMM=20000;

	m_nKinectRnageDataIDX=56;
	m_dKinectSensorOffsetmm=200;
	m_dKinectMaxDistanceMM=5000;
	m_dKinectMinDistanceMM=700;

	m_dRobotRadius=500;

	cout<<"[KuObstAvoidancePr]: Instance is created!!!"<<endl;
}

KuObstAvoidancePr::~KuObstAvoidancePr()
{
	if(m_pLocalMap!=NULL)
	{
		delete m_pLocalMap;
		m_pLocalMap=NULL;
	}
	if(m_pOriginalMap!=NULL)
	{
		delete  m_pOriginalMap;
		m_pOriginalMap=NULL;
	}
	if(m_pMap!=NULL)
	{
		delete  m_pMap;
		m_pMap=NULL;
	}

	m_vecPath.clear();
	m_vecLocalPath.clear();
	m_DetourPathList.clear();
	m_vecWayPoint.clear();
	m_vectempPath.clear();

	cout<<"[KuObstAvoidancePr]: Instance is destroyed!!!"<<endl;
}

/**
@brief Korean: 초기화 작업을 수행하는 함수.
@brief English: 
*/
bool KuObstAvoidancePr::initialize( )
{
	printf("!!KuObstAvoidancePr initialize1\n");

	m_bStandbyFlag=false;
	m_nPrePathIndx=-1;
	m_nLocalGoalIndex=3;
	m_nLocalGoalWayPointIndex=-1;
	m_nKinectRnageData.clear();
	m_nLaserData.clear();
	m_TargetPos.init();

	m_nKinectRnageData=m_KuUtil.generateIntType1DArray(m_nKinectRnageDataIDX,0);
	m_nLaserData=m_KuUtil.generateIntType1DArray(m_nLaserDataIDX,0);
	m_math.setCellSizeMM(m_nCellSize);

	//initMapbuildingProcess(true);	
	if(!generateLocalPath(m_RobotPos)) return false;	

	printf("!!KuObstAvoidancePr initialize2\n");

	return true;	

}

void KuObstAvoidancePr::setParameter(int nLaserDataIDX,double dLaserSensorOffsetmm,double dLaserMaxDistanceMM,double dLaserMinDistanceMM,
	int nKinectRnageDataIDX,double dKinectSensorOffsetmm,double dKinectMaxDistanceMM,double dKinectMinDistanceMM,
	int nCellSize,int nDistToTarget,double dRobotRadius)
{
	m_nCellSize=nCellSize;
	m_nDistToTarget=nDistToTarget;

	m_nLaserDataIDX=nLaserDataIDX;
	m_dLaserSensorOffsetmm=dLaserSensorOffsetmm;
	m_dLaserMinDistanceMM=dLaserMinDistanceMM;
	m_dLaserMaxDistanceMM=dLaserMaxDistanceMM;

	m_nKinectRnageDataIDX=nKinectRnageDataIDX;
	m_dKinectSensorOffsetmm=dKinectSensorOffsetmm;
	m_dKinectMaxDistanceMM=dKinectMaxDistanceMM;
	m_dKinectMinDistanceMM=dKinectMinDistanceMM;

	m_dRobotRadius=dRobotRadius;
}
/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
void KuObstAvoidancePr::setPath(list<KuPose> listPath)
{
	m_vecPath.clear();
	m_vecLocalPath.clear();

	//m_CriticalSection.Lock();

	doCriticalSection(0);
	if(listPath.size()>0)
	{
		list<KuPose>::iterator it;
		for(it=listPath.begin(); it!=listPath.end(); it++){
			m_vecPath.push_back(*it);
		}
	}
	doCriticalSection(1);
	//m_CriticalSection.Unlock();


	int nPathSize = m_vecPath.size()-1;
	double dPathX = m_vecPath[0].getX();
	double dPathY = m_vecPath[0].getY();
	m_RobotPos.setX(dPathX);
	m_RobotPos.setY(dPathY);

	dPathX = m_vecPath[nPathSize].getX();
	dPathY = m_vecPath[nPathSize].getY();
	m_GoalPos.setX(dPathX);
	m_GoalPos.setY(dPathY);


}
/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
void KuObstAvoidancePr::setData(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData,int_1DArray nKinectRnageData)
{
	//m_CriticalSection.Lock();

	doCriticalSection(0);
	m_RobotPos=RobotPos;
	m_DelEncoderData=DelEncoderData;
	for (int i=0; i<m_nLaserDataIDX;i++)
	{
		m_nLaserData[i]=nLaserData[i];
	}
	for (int i=0; i<m_nKinectRnageDataIDX;i++)
	{
		m_nKinectRnageData[i]=nKinectRnageData[i];
	}
	doCriticalSection(1);

	//m_CriticalSection.Unlock();

}

void KuObstAvoidancePr::setData(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nKinectRnageData)
{
	//m_CriticalSection.Lock();
	doCriticalSection(0);
	m_RobotPos=RobotPos;
	m_DelEncoderData=DelEncoderData;
	for (int i=0; i<m_nKinectRnageDataIDX;i++)
	{
		m_nKinectRnageData[i]=nKinectRnageData[i];
	}
	//m_CriticalSection.Unlock();
	doCriticalSection(1);
}
/**
@brief Korean: 경로를 넘겨주는 함수.
@brief English: write in English
*/
list<KuPose> KuObstAvoidancePr::getPath()
{
	list<KuPose> listPath;
	doCriticalSection(0);
	list<KuPose>::iterator it;
	for(it=m_DetourPathList.begin(); it!=m_DetourPathList.end(); it++){
		listPath.push_back(*it);
	}
	doCriticalSection(1);
	return listPath;
}

void KuObstAvoidancePr::doCriticalSection(int nID)
{
	static int nWaitingID=0;

	if(nID==1)
	{
		nWaitingID=0;
	}
	else if(nID!=1)
	{
		while (nWaitingID==1){Sleep(1);}
		nWaitingID=1;
	}
}

void KuObstAvoidancePr::calStartPathPoint(KuPose RobotPos )
{
	//KuPose RobotPos=KuDrawingInfo::getInstance()->getRobotPos();
	int nDistToTarget =m_nDistToTarget;

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

KuPose KuObstAvoidancePr::calPathPoint( KuPose RobotPos)
{
	//KuPose RobotPos=KuDrawingInfo::getInstance()->getRobotPos();
	int nDistToTarget =m_nDistToTarget;
	KuPose TargetPos;

	int  nMinIdx=-1;
	double dMinPathX = 0.0;
	double dMinPathY = 0.0;
	double dDist=0;
	bool bcheck= false;
	int nSelectedMinIndx=m_vecPath.size()-1;

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
			nSelectedMinIndx = nMinIdx;
			break;
		}	
		else if(dDist<nDistToTarget)
		{
			bcheck= true;
		}
	}

	TargetPos.setX(m_vecPath[nSelectedMinIndx].getX());
	TargetPos.setY(m_vecPath[nSelectedMinIndx].getY());


	return TargetPos;
}

/**
@brief Korean: 
@brief English: 
*/
void KuObstAvoidancePr::initMapbuildingProcess(bool bLocalMapflag)
{
	double dThicknessofWall=50;
	string strUpdateSpeed="no";

	m_pRefMap = kuMapRepository::getInstance()->getMap(); 
	m_nMapSizeX =m_nGlobalMapX=m_nBuildingMapX=m_pRefMap->getX();
	m_nMapSizeY =m_nGlobalMapY=m_nBuildingMapY=m_pRefMap->getY();


	if(NULL==m_pMap)
	{
		m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
	}

	int** nMap = m_pMap->getMap();
	int** nRefMap = m_pRefMap->getMap();

	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			nMap[i][j] = nRefMap[i][j];
		}
	}
	KuMath m_math;

	KuMapBuilderParameter InputParam;
	InputParam.setMapSizeXYAI(m_nMapSizeX, m_nMapSizeY);
	InputParam.setLaserScanIdx(m_nLaserDataIDX);
	InputParam.setMinDistofSensorData(m_dLaserMinDistanceMM); // unit mm
	InputParam.setMaxDistofSensorData(m_dLaserMaxDistanceMM); // unit mm
	InputParam.setLaserXOffset(m_dLaserSensorOffsetmm);
	InputParam.setRadiusofRobot(m_dRobotRadius);
	InputParam.setCellSize(m_nCellSize);

	InputParam.setRobotPos(m_RobotPos);
	InputParam.setSigma(100);	
	InputParam.setThicknessofWall(dThicknessofWall);
	InputParam.setLaserUpdateSpeedflag(true);
	m_LaserMapBuilder.initialize(InputParam);
	
	if(true==bLocalMapflag)
	{
		//지역 지도 생성
		if(NULL==m_pLocalMap)
		{
			int nLocalMapSize = m_math.MM2AI(LOCALGOAL_DISTANCE*2)+1; //*2는 LOCALGOAL_DISTANCE가 지도의 반이라서, /100은 mm단위는 격자단위로하기 위해
			m_pLocalMap = new KuMap(nLocalMapSize+50, nLocalMapSize+50);  //10은 마진 
			m_nLocalMapX=m_pLocalMap->getX();
			m_nLocalMapY=m_pLocalMap->getY();
			//지역 경로계획
			m_KuLocalPathPlanner.setRadiusofRobot(m_dRobotRadius);
			m_KuLocalPathPlanner.initIntCost(LOCALPATH_COST);
			m_KuLocalPathPlanner.initializeMapSize(m_nLocalMapX,m_nLocalMapY);	

			m_pOriginalMap = new KuMap(m_nLocalMapX, m_nLocalMapY); 
		}
	}

}

void KuObstAvoidancePr::setMap(int **nRefMap,int nMapSizeX,int nMapSizeY)
{
	double dThicknessofWall=50;
	string strUpdateSpeed="no";

	m_nMapSizeX =m_nGlobalMapX=m_nBuildingMapX=nMapSizeX;
	m_nMapSizeY =m_nGlobalMapY=m_nBuildingMapY=nMapSizeY;


	if(NULL==m_pMap)
	{
		m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
		m_pOriginalMap = new KuMap(m_nMapSizeX, m_nMapSizeY);
		m_pRefMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
	}
	else
	{
		if(m_pOriginalMap!=NULL)
		{
			delete m_pOriginalMap;
			m_pOriginalMap=NULL;
		}
		if(m_pMap!=NULL)
		{
			delete m_pMap;
			m_pMap=NULL;
		}

		if(m_pRefMap!=NULL)
		{
			delete m_pRefMap;
			m_pRefMap=NULL;
		}

		m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
		m_pOriginalMap = new KuMap(m_nMapSizeX, m_nMapSizeY);
		m_pRefMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
	}

	int** nMap = m_pMap->getMap();
	int** nOriMap = m_pOriginalMap->getMap();

	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			nMap[i][j] = nRefMap[i][j];
			nOriMap[i][j]= nRefMap[i][j];
		}
	}
	//Map copy  end------------------------------------------------------------------------------------------------------------------------	


	KuMapBuilderParameter InputParam;
	InputParam.setMapSizeXYAI(m_nMapSizeX, m_nMapSizeY);
	InputParam.setLaserScanIdx(m_nLaserDataIDX);
	InputParam.setMinDistofSensorData(m_dLaserMinDistanceMM); // unit mm
	InputParam.setMaxDistofSensorData(m_dLaserMaxDistanceMM); // unit mm
	InputParam.setLaserXOffset(m_dLaserSensorOffsetmm);
	InputParam.setRadiusofRobot(m_dRobotRadius);
	InputParam.setCellSize(m_nCellSize);

	InputParam.setRobotPos(m_RobotPos);
	InputParam.setSigma(100);	
	InputParam.setThicknessofWall(dThicknessofWall);
	InputParam.setLaserUpdateSpeedflag(true);
	m_LaserMapBuilder.initialize(InputParam);

	//지역 지도 생성
	if(NULL==m_pLocalMap)
	{
		int nLocalMapSize = (LOCALGOAL_DISTANCE*2)/(double)m_nCellSize; //*2는 LOCALGOAL_DISTANCE가 지도의 반이라서, /100은 mm단위는 격자단위로하기 위해
		m_pLocalMap = new KuMap(nLocalMapSize+50, nLocalMapSize+50);  //10은 마진 
		m_nLocalMapX=m_pLocalMap->getX();
		m_nLocalMapY=m_pLocalMap->getY();
		//지역 경로계획
		m_KuLocalPathPlanner.setRadiusofRobot(m_dRobotRadius);
		//m_KuLocalPathPlanner.initIntCost(LOCALPATH_COST);
		m_KuLocalPathPlanner.initializeMapSize(m_nLocalMapX,m_nLocalMapY);

		//	m_pOriginalMap = new KuMap(m_nLocalMapX, m_nLocalMapY); 
	}
}
void KuObstAvoidancePr::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}

float KuObstAvoidancePr::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}
/**
@brief Korean: 작성중인 지도를 전역 지도에 복사하는 함수.
@brief English: Copies the building map to global map
*/
void KuObstAvoidancePr::copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap,  KuPose RobotPos)
{
	int nRobotX = (int)( m_math.MM2AI(RobotPos.getX()));
	int nRobotY = (int)( m_math.MM2AI(RobotPos.getY()));
	int nRange= 50;

	for(int i=nRobotX -nRange; i<nRobotX+nRange; i++){
		for(int j=nRobotY -nRange; j<nRobotY+nRange; j++){
			if(m_nMapSizeX-1<i||i<1||m_nMapSizeY-1<j||j<1) continue;
			if(m_nMapSizeX-1<i-nRobotX+nRange||i-nRobotX+nRange<1
				||m_nMapSizeY-1<j-nRobotY+nRange||j-nRobotY+nRange<1) continue;
			if(KuMap::UNKNOWN_AREA == nBuildingMap[i][j]) continue;

			nGlobalMap[i][j] = nBuildingMap[i][j];

		}
	}
}

/**
@brief Korean: 작성중인 지도를 전역 지도에 복사하는 함수.
@brief English: Copies the building map to global map
*/
void KuObstAvoidancePr::copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap, int** nOriginalMap, KuPose RobotPos)
{
	if(nBuildingMap==NULL||nGlobalMap==NULL||nOriginalMap==NULL) return;
	int nRobotX = m_math.MM2AI(RobotPos.getX());
	int nRobotY =m_math.MM2AI(RobotPos.getY());

	int nLocalMapX=m_nLocalMapX/2.0-1;
	int nLocalMapY=m_nLocalMapY/2.0-1;

	for(int i=nRobotX -nLocalMapX; i<nRobotX+nLocalMapX; i++){
		for(int j=nRobotY -nLocalMapY; j<nRobotY+nLocalMapY; j++){
			if(m_nBuildingMapX-1<i||i<1||m_nBuildingMapY-1<j||j<1) continue;
			if(m_nGlobalMapX-1<i||i<1||m_nGlobalMapY-1<j||j<1) continue;
			if(m_nLocalMapX-1<i-nRobotX+nLocalMapX||i-nRobotX+nLocalMapX<1||m_nLocalMapY-1<j-nRobotY+nLocalMapY||j-nRobotY+nLocalMapY<1) continue;

			nOriginalMap[i-nRobotX+nLocalMapX][j-nRobotY+nLocalMapY] =nGlobalMap[i][j]; 

			if(KuMap::UNKNOWN_AREA == nBuildingMap[i][j]) continue;
			if(KuMap::OCCUPIED_AREA == nGlobalMap[i][j]) continue;
			nGlobalMap[i][j] = nBuildingMap[i][j];
		}
	}
}

/**
@brief Korean: 전역 지도를 지역 지도에 복사하는 함수.
@brief English: Copies the global map to local map
*/
void KuObstAvoidancePr::copyGlobalMapToLocalMap(int**nGlobalMap, int** nLocalMap, KuPose RobotPos, int* nLocalMapSPosX, int* nLocalMapSPosY)

{
	//--------초기화----------------------------------//
	for(int i=0;i<m_nLocalMapX; i++){
		for(int j=0; j<m_nLocalMapY; j++){
			nLocalMap[i][j] = KuMap::UNKNOWN_AREA;
		}
	}
	//=================================================

	int nRobotX = (int)( m_math.MM2AI(RobotPos.getX()));
	int nRobotY = (int)( m_math.MM2AI(RobotPos.getY()));
	int nX=0, nY=0;
	int nHalfSizeOfLocalMapX =m_nLocalMapX/2;
	int nHalfSizeOfLocalMapY =m_nLocalMapY/2;

	for(int i=0;i<m_nLocalMapX; i++){
		for(int j=0; j<m_nLocalMapY; j++){
			nX = nRobotX - nHalfSizeOfLocalMapX + i;
			nY = nRobotY - nHalfSizeOfLocalMapY + j;
			if(m_nLocalMapX<i||i<1||m_nLocalMapY<j||j<1) continue;
			if(m_nGlobalMapX<nX||nX<1||m_nGlobalMapY<nY||nY<1) continue;	
			nLocalMap[i][j]=nGlobalMap[nX][nY];

		}
	}
	*nLocalMapSPosX = nRobotX - nHalfSizeOfLocalMapX;
	*nLocalMapSPosY = nRobotY - nHalfSizeOfLocalMapY;
}

/**
@brief Korean: 전역지도를 원래의 상태로 바꾸는 함수
@brief English: Changes the global map to original map
*/
void KuObstAvoidancePr::copyOriginalMapToGlobalMap(int** nOriginalMap, int** nGlobalMap, KuPose RobotPos)
{

	int nRobotX = m_math.MM2AI(RobotPos.getX());
	int nRobotY = m_math.MM2AI(RobotPos.getY());
	int nLocalMapX=m_nLocalMapX/2.0-1;
	int nLocalMapY=m_nLocalMapY/2.0-1;

	for(int i=nRobotX -nLocalMapX; i<nRobotX+nLocalMapX; i++){
		for(int j=nRobotY -nLocalMapY; j<nRobotY+nLocalMapY; j++){
			if(m_nGlobalMapX<i||i<1||m_nGlobalMapY<j||j<1) continue;
			if(m_nLocalMapX<i-nRobotX+nLocalMapX||i-nRobotX+nLocalMapX<1||m_nLocalMapY<j-nRobotY+nLocalMapY||j-nRobotY+nLocalMapY<1) continue;

			nGlobalMap[i][j]=nOriginalMap[i-nRobotX+nLocalMapX][j-nRobotY+nLocalMapY] ; 
		}
	}

}
/**
@brief Korean: 
@brief English: 
*/
int_1DArray KuObstAvoidancePr::combinateLaserNKinect(int_1DArray nLaserData181, int_1DArray nKinectRnageData)
{
	int nLaserMinDis=m_dLaserMinDistanceMM;;
	int nKinectMinDist=m_dKinectMinDistanceMM;;
	int nKinectMaxDist=m_dKinectMaxDistanceMM;;
	int_1DArray nCombinateRangeData=m_KuUtil.generateIntType1DArray(m_nLaserDataIDX,0);
	int nStart=(m_nLaserDataIDX-m_nKinectRnageDataIDX)/2.0;
	int nEnd=(m_nLaserDataIDX+m_nKinectRnageDataIDX)/2.0;

	for(int i=0;i<m_nLaserDataIDX;i++)
	{
		nCombinateRangeData[i]=nLaserData181[i];

		if(i>=nStart&&i<nEnd)
		{
			if((nLaserData181[i]>nKinectRnageData[i-nStart]||nLaserData181[i]==0)
				&& nKinectRnageData[i-nStart]>nKinectMinDist
				&& nKinectRnageData[i-nStart]<nKinectMaxDist)
			{
				nCombinateRangeData[i]=nKinectRnageData[i-nStart];
			}
		}
	}

	return nCombinateRangeData;
}

void KuObstAvoidancePr::generateLocalMap(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData181, int_1DArray nKinectRnageData)
{
	KuMapBuilderParameter InputParam; 		

	int_1DArray nCombinateRangeData;
	nCombinateRangeData =combinateLaserNKinect(nLaserData181, nKinectRnageData);


	//if(DelEncoderData.getThetaDeg()<3.0)
	{

		//지도 작성 시작---------------------------------------------------------------------
		InputParam.setDelRobotPos(DelEncoderData); 
		InputParam.setRobotPos(RobotPos);
		InputParam.setLaserData(nCombinateRangeData);
		InputParam.setLaserUpdateSpeedflag(true);
		m_LaserMapBuilder.buildMap(InputParam);		
		//지도 작성 종료---------------------------------------------------------------------

		//Map copy  start------------------------------------------------------------------------------------------------------------------------	
		copyBuildingMapToGlobalMap(m_LaserMapBuilder.getMap(), m_pMap->getMap(),m_pOriginalMap->getMap(), RobotPos);
		//copyGlobalMapToLocalMap( m_pMap->getMap(), m_pLocalMap->getMap(), m_RobotPos, &m_nLocalMapSPosX, &m_nLocalMapSPosY);
		//Map copy  end------------------------------------------------------------------------------------------------------------------------	
		//KuDrawingInfo::getInstance()->setMap(m_LaserMapBuilder.getpMap());
		//KuDrawingInfo::getInstance()->setMap(m_pMap);

		//KuDrawingInfo::getInstance()->setnMap(m_pMap->getMap(),m_pMap->getX(),m_pMap->getY());
	}

}
/**
@brief Korean: 우회 경로를 생성하기 위한 목적지를 설정하는 함수
@brief English: Sets the goal position to generate detour path
*/
KuPose KuObstAvoidancePr::generateDetourGoalPos(KuPose RobotPos, vector<KuPose> vecPath, int nPathSize)
{
	KuPose LocalGoalPos;	

	if(nPathSize<3) return m_GoalPos;

	for(int i=m_nLocalGoalIndex-3; i<nPathSize; i++)
	{
		if(i<0) continue;

		for(int j=1; j<m_vecWayPoint.size(); j++){

			if(hypot((m_vecWayPoint[j].getX()- vecPath[i].getX()),(m_vecWayPoint[j].getY() - vecPath[i].getY()))< 300
				&&m_vecWayPoint[j].getID()!=1)
			{
				LocalGoalPos.setX(m_vecWayPoint[j].getX());
				LocalGoalPos.setY(m_vecWayPoint[j].getY());
				if(hypot((vecPath[i].getX()- RobotPos.getX()),(vecPath[i].getY() - RobotPos.getY()))< 500)
				{
					m_vecWayPoint[j].setID(1);					
				}
				m_nLocalGoalIndex=i;
				m_nLocalGoalWayPointIndex=j;

				return LocalGoalPos;
			}
		}

		if(hypot((vecPath[i].getX()- RobotPos.getX()),(vecPath[i].getY() - RobotPos.getY()))> LOCALGOAL_DISTANCE){		
			LocalGoalPos.setX(vecPath[i].getX());
			LocalGoalPos.setY(vecPath[i].getY());
			m_nLocalGoalIndex=i;
			return LocalGoalPos;
		}
	}


	return m_GoalPos;
}

/**
@brief Korean: 지역 지도 상의 우회 경로 생성 함수
@brief English: Creates detour path in local map
*/
bool KuObstAvoidancePr::generateDetourPath(KuPose RobotPos, KuPose DetourGoalPos, KuMap* pLocalMap, 
	int nLocalMapSPosX, int nLocalMapSPosY, list<KuPose> *DetourPathList)
{
	KuPose LocalRobotPos, LocalGoalPos;
	LocalRobotPos.setX(RobotPos.getX()- m_math.AI2MM(nLocalMapSPosX));
	LocalRobotPos.setY(RobotPos.getY()- m_math.AI2MM(nLocalMapSPosY));
	LocalGoalPos.setX( DetourGoalPos.getX()- m_math.AI2MM(nLocalMapSPosX));
	LocalGoalPos.setY( DetourGoalPos.getY()- m_math.AI2MM(nLocalMapSPosY));
	m_KuLocalPathPlanner.setMap(pLocalMap->getMap(),m_math.getCellSizeMM());

	int nResult =m_KuLocalPathPlanner.generatePath(LocalGoalPos,LocalRobotPos);//경로를 생성시키는 함수.

	if(nResult==0){ //경로가 생성된 경우.
		*DetourPathList = m_KuLocalPathPlanner.getPath();//실질적으로 경로를 리스트 형태로 받아오는 함수.
		*DetourPathList = m_KuPathSmoothing.smoothingPath(*DetourPathList);
		//cout<<"우회경로 생성 성공"<<endl;
		return true; 
	}
	//cout<<"우회경로 생성 실패~~~~~!!!!!!!!!!"<<endl;
	return false;
}

/**
@brief Korean: 생성된 지역지도 상의 경로를 전역 위치상의 경로로 바꿔주는 함수
@brief English: Changes path created by global position into path created by local map
*/
list<KuPose> KuObstAvoidancePr::transferLocaltoGlobalPath( list<KuPose> LocalPath,KuPose RobotPose,int nLocalMapSPosX, int nLocalMapSPosY)
{
	list<KuPose>  GlobalDetourPathList;
	list<KuPose>::iterator it;
	KuPose GlobalDetourPath;
	if(LocalPath.size()>0){
		it = LocalPath.begin();
		it++;
		while(it != LocalPath.end()){
			GlobalDetourPath.setX(it->getX() + m_math.AI2MM(nLocalMapSPosX));
			GlobalDetourPath.setY(it->getY() + m_math.AI2MM(nLocalMapSPosY));
			GlobalDetourPathList.push_back(GlobalDetourPath);
			it++;
		}

	}
	return GlobalDetourPathList;
}
/**
@brief Korean: 지역 경로 생성 함수
@brief English: Executes local path planning
*/
bool KuObstAvoidancePr::generateLocalPath(KuPose RobotPos)
{
	list<KuPose> DetourPathList;
	copyGlobalMapToLocalMap( m_pMap->getMap(), m_pLocalMap->getMap(), RobotPos, &m_nLocalMapSPosX, &m_nLocalMapSPosY);
	m_LocalGoalPos = generateDetourGoalPos(RobotPos, m_vecPath,m_vecPath.size()-1);
	generateDetourPath(RobotPos, m_LocalGoalPos, m_pLocalMap, m_nLocalMapSPosX, m_nLocalMapSPosY, &DetourPathList);
	DetourPathList= transferLocaltoGlobalPath(DetourPathList,RobotPos,m_nLocalMapSPosX, m_nLocalMapSPosY);
	m_vecLocalPath.clear();

	doCriticalSection(0);
	m_DetourPathList.clear();
	list<KuPose>::iterator it;
	for(it=DetourPathList.begin(); it!=DetourPathList.end(); it++){
		m_vecLocalPath.push_back(*it);
		m_DetourPathList.push_back(*it);
	}
	doCriticalSection(1);

	//KuDrawingInfo::getInstance()->setLocalPath(m_DetourPathList);

	if(m_vecLocalPath.size()==0) return false;	
	return true; 
}

/**
@brief Korean: 경로에 장애물의 존재 여부를 판단하고 장애물의 크기를 확장하는 함수
@brief English: Checks the presence of obstacles in path and extends the size of obstacles
*/
bool KuObstAvoidancePr::existObstaclePath(KuPose RobotPos, vector<KuPose> vecLocalPath, int nPathSize, KuMap* pLocalMap, int nLocalMapSPosX, int nLocalMapSPosY )
{
	int nX=0, nY=1;
	int **nMap=m_pMap->getMap();
	bool bObsCSpaceflag=false;
	int nObsCSpace=1;//2;
	int ninterval=1;//검사할 영역 간격
	int nObnCnt=3;
	int nObstVeticlaBnd=1;//3;//30cm
	int nRX= m_math.M2AI(RobotPos.getXm());
	int nRY=m_math.M2AI(RobotPos.getYm()); 

	if(nPathSize<nObnCnt){return false;}


	for(int i=nObnCnt; i<nPathSize; i++ )
	{
		int nGridPathX = m_math.M2AI(vecLocalPath[i].getXm()) ;
		int nGridPathY = m_math.M2AI(vecLocalPath[i].getYm()) ;

		for(int n=-ninterval;n<(ninterval+1);n+=1){
			for(int m=-ninterval;m<(ninterval+1);m+=1){

				if(nGridPathX+n>m_pMap->getX()-1||nGridPathX+n< 1
					||nGridPathY+m>m_pMap->getY()-1||nGridPathY+m<1){continue;}

				if(m_pMap->getMap()[nGridPathX+n][nGridPathY+m]==KuMap::OCCUPIED_AREA){		

					bObsCSpaceflag=true; 

					for(int q=-nObsCSpace;q<nObsCSpace+1;q++)
					{
						for(int p=-nObsCSpace;p<nObsCSpace+1;p++)
						{

							if(nGridPathX+n+q>m_pMap->getX()-1||nGridPathX+n+q< 1
								||nGridPathY+m+p>m_pMap->getY()-1||nGridPathY+m+p<1){continue;}

							if(m_pMap->getMap()[nGridPathX+n+q][nGridPathY+m+p]==KuMap::OCCUPIED_AREA) {continue;}


							double dDelX=m_math.AI2M((double)(nGridPathX-nRX))/10.0;
							double dDelY=m_math.AI2M((double)(nGridPathY-nRY))/10.0;
							double dAccDelX=dDelX;
							double dAccDelY=dDelY;

							while(hypot(dAccDelX,dAccDelY)<nObstVeticlaBnd&&(dDelX!=0.0&&dDelY!=0.0))
							{
								if(nGridPathX+n+q+(int)dAccDelX>m_pMap->getX()-1||nGridPathX+n+q+(int)dAccDelX<1
									||nGridPathY+m+p+(int)dAccDelY>m_pMap->getY()-1||nGridPathY+m+p+(int)dAccDelY<1){
										break;
								}

								m_pMap->getMap()[nGridPathX+n+q+(int)dAccDelX][nGridPathY+m+p+(int)dAccDelY]=KuMap::WARNING_AREA;

								dAccDelX+=dDelX;
								dAccDelY+=dDelY;
							}
						}
					}

				}
			}
		}

	}
	if(bObsCSpaceflag){
		return true;
	}
	return false;
}
bool KuObstAvoidancePr::generateLocalPathforObsAvoidance(KuPose RobotPos,KuPose DelEncoderData)
{
	bool bDoLocalPathPlanning=false;
	bool bexistObstaclePath=false;
	bool breturn=true;

	if((fabs(RobotPos.getX()-m_LocalGoalPos.getX())<2000) &&( fabs(RobotPos.getY()-m_LocalGoalPos.getY())<2000) &&		
		(hypot(m_GoalPos.getX()-m_LocalGoalPos.getX(),m_GoalPos.getY()-m_LocalGoalPos.getY())>1000))
	{

		bDoLocalPathPlanning = true;
		if(m_nLocalGoalWayPointIndex>0)
			if((fabs(m_vecWayPoint[m_nLocalGoalWayPointIndex].getX()-m_LocalGoalPos.getX())<100) 
				&&( fabs(m_vecWayPoint[m_nLocalGoalWayPointIndex].getY()-m_LocalGoalPos.getY())<100)
				&&hypot((m_LocalGoalPos.getX()- RobotPos.getX()),(m_LocalGoalPos.getY() - RobotPos.getY()))> 500				
				)
			{
				bDoLocalPathPlanning = false;
			}	
	}
	else if((fabs(RobotPos.getX()-m_LocalGoalPos.getX())<2000) &&( fabs(RobotPos.getY()-m_LocalGoalPos.getY())<2000) &&		
		(hypot(m_GoalPos.getX()-m_LocalGoalPos.getX(),m_GoalPos.getY()-m_LocalGoalPos.getY())<=500))
	{
		bDoLocalPathPlanning = true;
	}

	if(existObstaclePath(RobotPos, m_vecLocalPath, m_vecLocalPath.size()-1,  m_pLocalMap,  m_nLocalMapSPosX, m_nLocalMapSPosY ))//로컬패스의 장애물 검사.
	{
		bDoLocalPathPlanning = true;
		bexistObstaclePath =true;
	}
	if(fabs(DelEncoderData.getThetaDeg())>0&&hypot(DelEncoderData.getX(),DelEncoderData.getY())<30)
	{
		bDoLocalPathPlanning = false;
		bexistObstaclePath =false;
	}
	//Path planning-----------------------------------------------------------------------
	if(bDoLocalPathPlanning == true||m_vecLocalPath.size()==0)
	{
		bDoLocalPathPlanning=false;
		breturn=generateLocalPath(RobotPos);
		if(!breturn) bDoLocalPathPlanning=true;
	}

	return breturn;
}

/**
@brief Korean: 센서데이터를 이용하여 타겟점과 로봇 사이의 장애물 유무를 검사하는 함수
@brief English: 
*/
bool KuObstAvoidancePr::checkObstacles(KuPose RobotPos,KuPose TargetPos,int_1DArray nLaserData181, int_1DArray nKinectRnageData ,double dTargetDist)
{

	double dGradientX = TargetPos.getXm() - RobotPos.getXm();
	double dGradientY = TargetPos.getYm() - RobotPos.getYm();
	double dDistofObst=_hypot(TargetPos.getX()- RobotPos.getX(), TargetPos.getY()- RobotPos.getY());
	double dDistObstoRobot=dTargetDist;
	double dTempDistObstoRobot=dTargetDist;
	int nDistIndx=m_math.MM2AI(dDistObstoRobot);
	double dKinectOffset= (double)m_dKinectSensorOffsetmm;
	double dLaserOffset= (double)m_dLaserSensorOffsetmm;
	int nradiusofRobot=m_dRobotRadius;

	for (int ndistance = nDistIndx; ndistance <dDistObstoRobot; ndistance += nDistIndx) {

		double dRayOfX =  RobotPos.getX()+ ndistance*dGradientX;
		double dRayOfY = RobotPos.getY()+ ndistance*dGradientY;

		dTempDistObstoRobot=_hypot(ndistance*dGradientX, ndistance*dGradientY);

		for(int i=0;i<m_nKinectRnageDataIDX;i++){

			if(nLaserData181[i+66]<50)continue;
			if(nKinectRnageData[i] <50)continue;

			if((nKinectRnageData[i] != 1000000&&nKinectRnageData[i] >50)&&nKinectRnageData[i] <dDistofObst){
				double dAngleRad = (double)(i -  (double)m_nKinectRnageDataIDX/2.) * D2R;
				double dX = RobotPos.getX() + ((double)nKinectRnageData[i] * cos(dAngleRad) + dKinectOffset ) * cos(RobotPos.getThetaRad()) + 
					((double)nKinectRnageData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());
				double dY = RobotPos.getY() + ((double)nKinectRnageData[i] * cos(dAngleRad) + dKinectOffset ) * sin(RobotPos.getThetaRad()) + 
					((double)nKinectRnageData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

				double tempD=hypot(dRayOfX-dX, dRayOfY- dY);

				if(tempD<nradiusofRobot&&tempD>0 ){	return true;	}// 거리에거리에 따른 장애물 감지
			}
			else if((nLaserData181[i+66] != 1000000&&nLaserData181[i+66]>50)&&nLaserData181[i+66]<dDistofObst){
				double dAngleRad = (double)(i -  (double)m_nKinectRnageDataIDX/2.) * D2R;
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

void KuObstAvoidancePr::doLocalPathplanningThread(void* arg)
{
	KuObstAvoidancePr* pLPPT = (KuObstAvoidancePr*)arg;
	LARGE_INTEGER present1;		
	pLPPT->startTimeCheck(present1);

	double dTargetDist=0;	
	KuPose RobotPos=pLPPT->m_RobotPos;
	pLPPT->generateLocalMap(RobotPos,pLPPT->m_DelEncoderData,pLPPT->m_nLaserData,  pLPPT->m_nKinectRnageData);
	pLPPT->generateLocalPathforObsAvoidance(RobotPos,pLPPT->m_DelEncoderData);
	pLPPT->m_LaserMapBuilder.initMap();
	pLPPT->m_TargetPos = pLPPT->getTargetPos( RobotPos, pLPPT->m_nDistToTarget, &dTargetDist);
	pLPPT->copyOriginalMapToGlobalMap(pLPPT->m_pOriginalMap->getMap(), pLPPT->m_pMap->getMap(),  RobotPos);
	double dtotalelapsed =(double) pLPPT->finishTimeCheck(present1);
	//printf("doLocalPathplanningThread::%f\n",dtotalelapsed);

}

/**
@brief Korean: 시뮬레이터를 종료한다.
@brief English: 
*/
void KuObstAvoidancePr::terminate()
{
	m_LocalPathplaningThread.terminate();
	m_LocalPathplaningThread.terminate();
	cout<<"[KuObstAvoidancePr]:: Behavior is terminated!!!"<<endl;
	m_bThreadFlag = false;	

}

/**
@brief Korean: 경로에서 타겟점을 선택하는 함수
@brief English: 
*/
KuPose KuObstAvoidancePr::getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist)
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

	if(nPathSize<=1) return calPathPoint(RobotPos );

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
	if(nselectedMinIndx!=-1)	//로봇에서부터의 최소가 되는 지점에서의 법선 벡터
	{
		dMinTangentthetaDeg=atan2(m_vecLocalPath[nselectedMinIndx].getY()-m_vecLocalPath[nselectedMinIndx-1].getY(),m_vecLocalPath[nselectedMinIndx].getX()-m_vecLocalPath[nselectedMinIndx-1].getX());

	}
	else
	{
		return calPathPoint( RobotPos);
	}
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


	if(nselectedTargetIndx!=-1)	//로봇에서부터의 최소가 되는 지점에서의 법선 벡터
	{
		dTargetTangentthetaDeg=atan2(m_vecLocalPath[nselectedTargetIndx].getY()-m_vecLocalPath[nselectedTargetIndx-1].getY(),m_vecLocalPath[nselectedTargetIndx].getX()-m_vecLocalPath[nselectedTargetIndx-1].getX());

	}
	else
	{
		return calPathPoint( RobotPos );
	}
	//목표 지점에서의 법선 벡터


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

KuPose KuObstAvoidancePr::getTargetPos()
{
	return m_TargetPos;
}

/**
@brief Korean: KuObstAvoidancePr를 실행시키는 함수
@brief English: 
*/

bool KuObstAvoidancePr::start(int nTime)
{

	if(initialize( ))
	{
		m_LocalPathplaningThread.start(doLocalPathplanningThread,this,nTime); //메인 스레드 시작	
		return true;	
	}
	else return false;
}
