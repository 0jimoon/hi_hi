#include "stdafx.h"
#include "KuLocalPathPlannerPr.h"
#include "CTracker.h"
#include "KuTracker.h"

Mat fgMaskMOG2;
Ptr<BackgroundSubtractor> pMOG2; 
Scalar Colors[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,0,255),Scalar(255,127,255),Scalar(127,0,255),Scalar(127,0,127)};
//CTracker tracker(0.2,0.5,60000.0,1,50);
KuTracker ktracker(0.2,0.5,60000.0,1,50);

KuLocalPathPlannerPr::KuLocalPathPlannerPr()
{
	m_vecPath.clear();
	m_vecWayPoint.clear();
	m_bThreadFlag = false;
	m_nPrePathIndx=-1;
	m_nLocalGoalIndex=3;
	m_nLocalGoalWayPointIndex=-1;
	m_pLocalMap=NULL;
	m_pOriginalMap=NULL;
	m_pMap= NULL;
	m_pCautionMap= NULL;
	m_nCautionMap=NULL;
	m_dpredictedProMap=NULL;
	m_dtmppredictedProMap=NULL;
	m_dlocalpredictedProMap=NULL;
	//m_bchepredictedProMap=NULL;
	m_nCellSize=100;
	m_nDistToTarget=1200;
	m_bDrawingmapflag=false;
	m_nLaserDataIDX=181;
	m_dLaserSensorOffsetmm=200;
	m_dLaserMinDistanceMM=50;
	m_dLaserMaxDistanceMM=20000;

	m_nKinectRnageDataIDX=56;
	m_dKinectSensorOffsetmm=200;
	m_dKinectMaxDistanceMM=5000;
	m_dKinectMinDistanceMM=700;

	m_dRobotRadius=500;
	m_bsetdataflag=false;

	m_storage = cvCreateMemStorage(0);
	m_matImgSrc.create(Sensor::IMAGE_HEIGHT,Sensor::IMAGE_WIDTH,CV_8UC3);//Result image initialization
	m_cvMatImage.create(Sensor::IMAGE_HEIGHT,Sensor::IMAGE_WIDTH,CV_8UC3);//Result image initialization
	m_cvGrayMatImage.create(Sensor::IMAGE_HEIGHT,Sensor::IMAGE_WIDTH,CV_8UC1);//Result image initialization
	m_IplKinectCamera =  cvCreateImage(cvSize( Sensor::IMAGE_WIDTH, Sensor::IMAGE_HEIGHT),8,3);	

	m_bpredictexitObs=false;
	m_dseltime=0;
	cout<<"[KuLocalPathPlannerPr]: Instance is created!!!"<<endl;
}

KuLocalPathPlannerPr::~KuLocalPathPlannerPr()
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
	if(m_pCautionMap!=NULL)
	{
		delete [] m_pCautionMap;
		m_pCautionMap=NULL;
	}

	m_vecPath.clear();
	m_vecLocalPath.clear();
	m_DetourPathList.clear();
	m_vecWayPoint.clear();
	m_vectempPath.clear();

	cout<<"[KuLocalPathPlannerPr]: Instance is destroyed!!!"<<endl;
}

void KuLocalPathPlannerPr::setParameter(int nLaserDataIDX,double dLaserSensorOffsetmm,double dLaserMaxDistanceMM,double dLaserMinDistanceMM,
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
@brief Korean: 초기화 작업을 수행하는 함수.
@brief English: 
*/
bool KuLocalPathPlannerPr::initialize( )
{
	cout<<"KuLocalPathPlannerPr initialize!!!"<<endl;

	nGroupID=1;
	m_vecWanderObstaclePose.clear();
	m_bStandbyFlag=false;
	m_nPrePathIndx=-1;
	m_nLocalGoalIndex=3;
	m_nLocalGoalWayPointIndex=-1;
	m_nKinectRnageData.clear();
	m_nLaserData.clear();

	m_nKinectRnageData=m_KuUtil.generateIntType1DArray(m_nKinectRnageDataIDX,0);
	m_nLaserData=m_KuUtil.generateIntType1DArray(m_nLaserDataIDX,0);
	m_math.setCellSizeMM(m_nCellSize);

	m_timer.sleepMS(100);

	if(!initializePath()) return false;

	initMapbuildingProcess(true);

	copyGlobalMapToLocalMap( m_pMap->getMap(), m_pLocalMap->getMap(), m_RobotPos, &m_nLocalMapSPosX, &m_nLocalMapSPosY);
	//KuDrawingInfo::getInstance()->setLocalMap(m_pLocalMap->getMap(),m_nLocalMapX,m_nLocalMapY);

	if(!generateLocalPath(m_RobotPos)) return false;

	m_bStandbyFlag=true;

	return true;	
}

bool KuLocalPathPlannerPr::initializePath( )
{
	if(m_vectempPath.size()<1) return false;
	cout<<"initializePath !!!"<<endl;
	m_vecPath.clear();
	m_vecLocalPath.clear();
	m_DetourPathList.clear();

	for(int i=0; i<m_vectempPath.size(); i++)
	{
		m_vecPath.push_back(m_vectempPath[i]);
	}

	int nPathSize = m_vecPath.size()-1;
	double dPathX = m_vecPath[0].getX();
	double dPathY = m_vecPath[0].getY();
	m_RobotPos.setX(dPathX);
	m_RobotPos.setY(dPathY);

	dPathX = m_vecPath[nPathSize].getX();
	dPathY = m_vecPath[nPathSize].getY();
	m_GoalPos.setX(dPathX);
	m_GoalPos.setY(dPathY);


	return true;
}

/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
bool KuLocalPathPlannerPr::setPath(list<KuPose> listPath)
{
	cout<<"setPath !!!"<<endl;

	m_bStandbyFlag=false;
	m_vectempPath.clear();
	double dDist=0;
	KuPose PrePose;
	if(listPath.size()>0)
	{
		list<KuPose>::iterator it;
		for(it=listPath.begin(); it!=listPath.end(); it++){
			//m_listPath.push_back(*it);
			if (PrePose.getID()==1)
			{
				dDist=hypot(it->getXm()-PrePose.getXm(),it->getYm()-PrePose.getYm());
				it->setDist(dDist);
			}		
			else
			{
				it->setDist(0.0);
			}
			m_vectempPath.push_back(*it);		

			PrePose=(*it); PrePose.setID(1);
		}
	}
	else
		return false;

	return true;
}
/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
void KuLocalPathPlannerPr::setData(KuPose RobotPos, vector<KuPose> WanderObstaclePose)
{
	m_bStandbyFlag=false;

	doCriticalSection(0);	
	m_RobotPos=RobotPos;
	m_vecWanderObstaclePose.clear();
	m_dRobotVel=RobotPos.getDist();
	m_dRobotRVel=RobotPos.getPro()*D2R;
	if(m_TrajRobotPos.Tra_x.size()<1)
	{
		m_TrajRobotPos.Tra_x.push_back(RobotPos.getXm());
		m_TrajRobotPos.Tra_y.push_back(RobotPos.getYm());
		m_TrajRobotPos.Tra_th.push_back(RobotPos.getThetaRad());
	}
	else
	{
		int nsize=m_TrajRobotPos.Tra_x.size()-1;
		if(hypot(m_TrajRobotPos.Tra_x[nsize]-RobotPos.getXm(),m_TrajRobotPos.Tra_y[nsize]-RobotPos.getYm())>0.1)
		{
			m_TrajRobotPos.Tra_x.push_back(RobotPos.getXm());
			m_TrajRobotPos.Tra_y.push_back(RobotPos.getYm());
			m_TrajRobotPos.Tra_th.push_back(RobotPos.getThetaRad());
		}

	}
		doCriticalSection(1);	

		doCriticalSection(0);	

	for( size_t i = 0; i < WanderObstaclePose.size(); i++ )
	{
		double dm =RobotPos.getThetaRad()-M_PI_2;
		if(dm > M_PI) dm -=2*M_PI;
		if(dm < M_PI) dm +=2*M_PI;
		dm = atan(dm);
		double dscope= WanderObstaclePose[i].getYm()-RobotPos.getYm() -(dm*WanderObstaclePose[i].getXm()) + (dm*RobotPos.getXm());

		if (dscope < 0 && m_math.calcDistBetweenPosesM(RobotPos, WanderObstaclePose[i]) < 10.0)
		{
			m_vecWanderObstaclePose.push_back(WanderObstaclePose[i]);
		}	
	}
	doCriticalSection(1);	

	m_bsetdataflag=true;
	m_bStandbyFlag=true;
}

/**
@brief Korean: 경로를 넘겨주는 함수.
@brief English: write in English
*/
list<KuPose> KuLocalPathPlannerPr::getPath()
{
	doCriticalSection(0);	
	list<KuPose> listPath;
	list<KuPose>::iterator it;
	for(it=m_DetourPathList.begin(); it!=m_DetourPathList.end(); it++){
		listPath.push_back(*it);
	}
	doCriticalSection(1);

	return listPath;
}

void KuLocalPathPlannerPr::doCriticalSection(int nID)
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


void KuLocalPathPlannerPr::doLocalPathplanningThread(void* arg)
{
	KuLocalPathPlannerPr* pLPPT = (KuLocalPathPlannerPr*)arg;

	if(pLPPT->m_bStandbyFlag==true&&pLPPT->m_bsetdataflag==true)
	{
		KuPose RobotPose = pLPPT->m_RobotPos;

		vector<KuPose> vecWanderObstPose;

		pLPPT->doCriticalSection(0);		
		for( size_t i = 0; i < pLPPT->m_vecWanderObstaclePose.size(); i++ )
		{
			vecWanderObstPose.push_back(pLPPT->m_vecWanderObstaclePose[i]);
		}
		pLPPT->doCriticalSection(1);	


		//cout<<"PathplanningThread!!!"<<endl;
		LARGE_INTEGER present1;
		pLPPT->startTimeCheck(present1);
		double dTargetDist=0;
 		pLPPT->generatePredictionmap(RobotPose, pLPPT->m_Global3DPose,pLPPT->m_IplKinectCamera,vecWanderObstPose );
 		pLPPT->generateLocalMap(RobotPose,pLPPT->m_DelEncoderData,pLPPT->m_nLaserData,  pLPPT->m_nKinectRnageData);

		pLPPT->generateLocalPathforObsAvoidance(RobotPose,pLPPT->m_DelEncoderData, pLPPT->m_nLaserData,  pLPPT->m_nKinectRnageData);

		//pLPPT->m_LaserMapBuilder.initMap();
		pLPPT->m_TargetPos = pLPPT->getTargetPos(RobotPose, pLPPT->m_nDistToTarget, &dTargetDist);
		pLPPT->copyOriginalMapToGlobalMap(pLPPT->m_pOriginalMap->getMap(), pLPPT->m_pMap->getMap(),  RobotPose);


		double dtotalelapsed =(double) pLPPT->finishTimeCheck(present1);
		printf("doLocalPathplanningThread::%f\n",dtotalelapsed);

	}
}

int** KuLocalPathPlannerPr::getCautionMap()
{
	int** nMap = m_pCautionMap->getMap();
	doCriticalSection(0);
	for(int i=0; i<m_nMapSizeX; i++){
		for(int j=0; j<m_nMapSizeY; j++){

			m_nCautionMap[i][j] = nMap[i][j];
		}
	}
	doCriticalSection(1);
	return m_nCautionMap;	
}

void KuLocalPathPlannerPr::generatePredictionmap(KuPose RobotPos, KuPose*pKinectDataPose, IplImage* IplKinectCamera,vector<KuPose> vecWanderObstaclePose)
{
	m_kutimer.checkStartTime();

	vector<Point2f> centers;
	vector<int> theta;
	vector<KuPose> vecWanderObstPose;

	centers.clear();
	theta.clear();
	doCriticalSection(0);		
	for( size_t i = 0; i < vecWanderObstaclePose.size(); i++ )
	{
		vecWanderObstPose.push_back(vecWanderObstaclePose[i]);
	}
	doCriticalSection(1);	

	for( size_t i = 0; i < vecWanderObstPose.size(); i++ )
	{
		Point2f center = Point2f(vecWanderObstPose[i].getXm(), vecWanderObstPose[i].getYm());
		int thetaa = vecWanderObstPose[i].getThetaRad();
		centers.push_back(center);
		theta.push_back(thetaa);
	}

	if(centers.size()>0)
	{ 
		ktracker.Update(centers);
		
		int ndetectedId=-1;

		for(int i=0;i<centers.size();i++)
		{
			if(ktracker.kutracks[i]->trace.size()>1)
			{
				for(int j=0;j<ktracker.kutracks[i]->trace.size()-1;j++)
				{
					if(j==ktracker.kutracks[i]->trace.size()-2)
					{
						KuPose detectedPose;
						detectedPose.setXm(centers[i].x);
						detectedPose.setYm(centers[i].y);
						detectedPose.setThetaRad(theta[i]);
						detectedPose.setObsID(ktracker.kutracks[i]->track_id);
						m_vecObstaclePose.push_back(detectedPose);
					}	
				}
			}
		}
	}

	else
	{
		for(int i=0;i<ktracker.kutracks.size();i++)
		{
			delete ktracker.kutracks[i];
		}
		ktracker.kutracks.clear();
	}

	
	if (m_prevecObstaclePose.size()>0)
	{
		for (int i=0; i<m_vecObstaclePose.size(); i++)
		{
			for (int j=i+1; j<m_vecObstaclePose.size(); j++)
			{
				double dDist = m_math.calcDistBetweenPosesM(m_vecObstaclePose[i], m_vecObstaclePose[j]);
				double dAngle=fabs(m_vecObstaclePose[i].getThetaRad()-m_vecObstaclePose[j].getThetaRad());
				if(dDist < 1 && dAngle < 0.2)
				{
					if (m_prevecObstaclePose[i].getObsID() == m_vecObstaclePose[i].getObsID() && m_prevecObstaclePose[i].getGroup() > 0)
					{
						m_vecObstaclePose[i].setGroup(m_prevecObstaclePose[i].getGroup());
						m_vecObstaclePose[j].setGroup(m_prevecObstaclePose[i].getGroup());
					}
					else
					{
						if (m_vecObstaclePose[i].getGroup() >0)
						{
							m_vecObstaclePose[j].setGroup(m_vecObstaclePose[i].getGroup());
						}
						else
						{
							m_vecObstaclePose[i].setGroup(nGroupID);
							m_vecObstaclePose[j].setGroup(nGroupID);
							nGroupID++;
						}
					}
				}
			}
		}
	}
	

 	if (centers.size()>0)
 	{


		KuDrawingInfo::getInstance()->setTrackedObstacle(m_vecObstaclePose);

		double time=m_kutimer.checkEndTime();

		m_prevecObstaclePose.clear();

		doCriticalSection(0);	
		for (int i=0; i<m_vecObstaclePose.size(); i++){

			m_prevecObstaclePose.push_back(m_vecObstaclePose[i]);		
			saveObstacledata(m_vecObstaclePose[i]);
		}
		doCriticalSection(1);	

		m_preRobotPose=RobotPos;

		m_vecObstaclePose.clear();
		vecWanderObstPose.clear();
		//waitKey(5);
	}
}

void KuLocalPathPlannerPr::saveObstacledata(KuPose ObjPose)
{
	int ndectectedId=-1;
	double dDist=0.0;
	for(int j=0; j<m_vecObstacleTrajectory.size();j++)
	{
		if(ObjPose.getObsID()==m_vecObstacleTrajectory[j].ID)
		{			
			int nsize=m_vecObstacleTrajectory[j].Tra_x.size()-1;
			dDist=hypot(ObjPose.getXm()-m_vecObstacleTrajectory[j].Tra_x[nsize],
				ObjPose.getYm()-m_vecObstacleTrajectory[j].Tra_y[nsize]);
			ndectectedId=j;
			break;
		}
	}

	if (ndectectedId!=-1)
	{
		if(dDist>0.1)
		{
			m_vecObstacleTrajectory[ndectectedId].Tra_x.push_back(ObjPose.getXm());
			m_vecObstacleTrajectory[ndectectedId].Tra_y.push_back(ObjPose.getYm());
			m_vecObstacleTrajectory[ndectectedId].Tra_th.push_back(ObjPose.getThetaRad());
		}

	}
	else
	{
		KuTrajectory singleObjectTrajectory;
		singleObjectTrajectory.ID=ObjPose.getObsID();
		singleObjectTrajectory.Tra_x.push_back(ObjPose.getXm());
		singleObjectTrajectory.Tra_y.push_back(ObjPose.getYm());
		singleObjectTrajectory.Tra_th.push_back(ObjPose.getThetaRad());	
		m_vecObstacleTrajectory.push_back(singleObjectTrajectory);
	}

}


bool KuLocalPathPlannerPr::checkstaticinMap(int** nGlobalMap,  int nObX, int nObY)
{
	int nRange= 5;

	for(int i=nObX -nRange; i<nObX+nRange; i++){
		for(int j=nObY -nRange; j<nObY+nRange; j++){
			if(m_nMapSizeX-1<i||i<1||m_nMapSizeY-1<j||j<1) continue;
			if(m_nMapSizeX-1<i-nObX+nRange||i-nObX+nRange<1
				||m_nMapSizeY-1<j-nObY+nRange||j-nObY+nRange<1) continue;
			if(nGlobalMap[i][j]==KuMap::OCCUPIED_AREA)
			{
				return false;
			}

		}
	}
	return true;
}

bool KuLocalPathPlannerPr::checkEllipse(int nObX, int nObY,double dX, double dY, int nX, int nY)
{

	double dDelataDist=hypot(dX,dY)*30;
	double db2=9.0;
	double da2=(dDelataDist)*0.25;
	double dtheta;


	if(dX==0)
	{
		dtheta=M_PI_2;
	}
	else
	{
		dtheta=atan2(dY,dX);
	}


	double dFunctionResult = pow((nX-(nObX+0.5*dX))*cos(dtheta) + (nY-(nObY+0.5*dY))*sin(dtheta), 2.0)/da2
		+ pow((nX-(nObX+0.5*dX))*sin(dtheta) - (nY-(nObY+0.5*dY))*cos(dtheta), 2.0)/db2;

	if(dFunctionResult < 1.0)
	{
		//점유격자지정
		return true;

	}
	return false;
}


KuPose KuLocalPathPlannerPr::calPathPoint(KuPose RobotPos )
{
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
void KuLocalPathPlannerPr::initMapbuildingProcess(bool bLocalMapflag)
{
	double dThicknessofWall=50;
	string strUpdateSpeed="no";

	m_pRefMap = kuMapRepository::getInstance()->getMap(); 
	m_nMapSizeX =m_nGlobalMapX=m_nBuildingMapX=m_pRefMap->getX();
	m_nMapSizeY =m_nGlobalMapY=m_nBuildingMapY=m_pRefMap->getY();


	if(NULL==m_pMap)
	{
		m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
		m_pOriginalMap = new KuMap(m_nMapSizeX, m_nMapSizeY);
		m_pBuildingMap = new KuMap(m_nMapSizeX, m_nMapSizeY);
		m_pCautionMap = new KuMap(m_nMapSizeX, m_nMapSizeY);

	}	

	if(m_dpredictedProMap==NULL){

		m_dpredictedProMap = new double*[m_nMapSizeX];
		m_dtmppredictedProMap= new double*[m_nMapSizeX];
		if(m_dpredictedProMap){
			for(int i = 0 ; i < m_nMapSizeX ; i++){
				m_dpredictedProMap[i] = new double[m_nMapSizeY];
				m_dtmppredictedProMap[i] = new double[m_nMapSizeY];				
			}
		}
	}

	if(m_dlocalpredictedProMap==NULL){
		m_dlocalpredictedProMap = new double*[LOCAL_PREDICT_MAPSIZE];
	//	m_bchepredictedProMap= new bool*[LOCAL_PREDICT_MAPSIZE];
		if(m_dlocalpredictedProMap){
			for(int i = 0 ; i < LOCAL_PREDICT_MAPSIZE ; i++){
				m_dlocalpredictedProMap[i] = new double[LOCAL_PREDICT_MAPSIZE];
				//m_bchepredictedProMap[i] = new bool[LOCAL_PREDICT_MAPSIZE];
			}
		}
	}
	

	int** nMap = m_pMap->getMap();
	int** nRefMap = m_pRefMap->getMap();
	int** nOriMap = m_pOriginalMap->getMap();
	int** nCautionMap = m_pCautionMap->getMap();
	int nMapX = m_pCautionMap->getX();
	int nMapY = m_pCautionMap->getY();

	if(m_nCautionMap==NULL){
		//처음 m_nMap을 생성할때
		m_nCautionMap = new int*[m_nMapSizeX];
		if(m_nCautionMap){
			for(int i = 0 ; i < m_nMapSizeX ; i++){
				m_nCautionMap[i] = new int[m_nMapSizeY];
			}
		}
	}
	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			nMap[i][j] = nRefMap[i][j];
			nOriMap[i][j]= nRefMap[i][j];
			nCautionMap[i][j]= nRefMap[i][j];
			m_dpredictedProMap[i][j]=0;
		}
	}
	//Map copy  end------------------------------------------------------------------------------------------------------------------------	

	if(true==bLocalMapflag)
	{
		//지역 지도 생성
		if(NULL==m_pLocalMap)
		{
			int nLocalMapSize = (LOCALGOAL_DISTANCE*2)/(double)m_nCellSize; //*2는 LOCALGOAL_DISTANCE가 지도의 반이라서, /100은 mm단위는 격자단위로하기 위해
			m_pLocalMap = new KuMap(nLocalMapSize+50, nLocalMapSize+50);  //10은 마진 
			m_nLocalMapX=m_pLocalMap->getX();
			m_nLocalMapY=m_pLocalMap->getY();
			//지역 경로계획
			m_KuLocalPathPlanner.setRadiusofRobot(m_dRobotRadius);
			m_KuLocalPathPlanner.initIntCost(LOCALPATH_COST);
			m_KuLocalPathPlanner.initializeMapSize(m_nLocalMapX,m_nLocalMapY);

			//	m_pOriginalMap = new KuMap(m_nLocalMapX, m_nLocalMapY); 
		}
	}

}


/**
@brief Korean: 작성중인 지도를 전역 지도에 복사하는 함수.
@brief English: Copies the building map to global map
*/
void KuLocalPathPlannerPr::copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap,  KuPose RobotPos)
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
void KuLocalPathPlannerPr::copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap, int** nOriginalMap, KuPose RobotPos)
{
	int nRobotX = (int)( m_math.MM2AI(RobotPos.getX()));
	int nRobotY = (int)( m_math.MM2AI(RobotPos.getY()));

	int nHalfSizeOfLocalMapX =m_nLocalMapX/2;
	int nHalfSizeOfLocalMapY =m_nLocalMapY/2;

	for(int i=nRobotX -nHalfSizeOfLocalMapX; i<nRobotX+nHalfSizeOfLocalMapX; i++){
		for(int j=nRobotY -nHalfSizeOfLocalMapY; j<nRobotY+nHalfSizeOfLocalMapY; j++){
			if(m_nBuildingMapX-1<i||i<1||m_nBuildingMapY-1<j||j<1) continue;
			if(m_nGlobalMapX-1<i||i<1||m_nGlobalMapY-1<j||j<1) continue;

			if(KuMap::OCCUPIED_AREA != nBuildingMap[i][j]) continue;
			if(KuMap::OCCUPIED_AREA == nGlobalMap[i][j]) continue;
			if(	KuMap::OCCUPIED_AREA == nBuildingMap[i][j])
			{
				nGlobalMap[i][j] = KuMap::OCCUPIED_AREA;
			}
			// 			else
			// 			{
			// 				nGlobalMap[i][j] = nBuildingMap[i][j];
			// 			}
			nBuildingMap[i][j]=KuMap::UNKNOWN_AREA;	

		}
	}
}

/**
@brief Korean: 전역 지도를 지역 지도에 복사하는 함수.
@brief English: Copies the global map to local map
*/
void KuLocalPathPlannerPr::copyGlobalMapToLocalMap(int**nGlobalMap, int** nLocalMap, KuPose RobotPos, int* nLocalMapSPosX, int* nLocalMapSPosY)
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
	int nX=0., nY=0;
	int nHalfSizeOfLocalMapX =m_nLocalMapX/2;
	int nHalfSizeOfLocalMapY =m_nLocalMapY/2;


	for(int i=0;i<m_nLocalMapX; i++){
		for(int j=0; j<m_nLocalMapY; j++){
			nX = nRobotX - nHalfSizeOfLocalMapX + i;
			nY = nRobotY - nHalfSizeOfLocalMapY + j;
			if(m_nLocalMapX-1<i||i<1||m_nLocalMapY-1<j||j<1) continue;
			if(m_nGlobalMapX-1<nX||nX<1||m_nGlobalMapY-1<nY||nY<1) continue;

			nLocalMap[i][j]=nGlobalMap[nX][nY];

		}
	}
	*nLocalMapSPosX = nRobotX - nHalfSizeOfLocalMapX;
	*nLocalMapSPosY = nRobotY - nHalfSizeOfLocalMapY;
}
void KuLocalPathPlannerPr::copyGlobalMapToLocalMap(int**nGlobalMap, int** nLocalMap, KuPose RobotPos,
	double **dGlobalMap, double** dLocalMap,
	int* nLocalMapSPosX, int* nLocalMapSPosY)
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
	int nX=0., nY=0;
	int nHalfSizeOfLocalMapX =m_nLocalMapX/2;
	int nHalfSizeOfLocalMapY =m_nLocalMapY/2;


	for(int i=0;i<m_nLocalMapX; i++){
		for(int j=0; j<m_nLocalMapY; j++){
			nX = nRobotX - nHalfSizeOfLocalMapX + i;
			nY = nRobotY - nHalfSizeOfLocalMapY + j;
			if(m_nLocalMapX-1<i||i<1||m_nLocalMapY-1<j||j<1) continue;
			if(m_nGlobalMapX-1<nX||nX<1||m_nGlobalMapY-1<nY||nY<1) continue;

			dLocalMap[i][j]=dGlobalMap[nX][nY];

		}
	}

	for(int i=0;i<m_nLocalMapX; i++){
		for(int j=0; j<m_nLocalMapY; j++){
			nX = nRobotX - nHalfSizeOfLocalMapX + i;
			nY = nRobotY - nHalfSizeOfLocalMapY + j;
			if(m_nLocalMapX-1<i||i<1||m_nLocalMapY-1<j||j<1) continue;
			if(m_nGlobalMapX-1<nX||nX<1||m_nGlobalMapY-1<nY||nY<1) continue;

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
void KuLocalPathPlannerPr::copyOriginalMapToGlobalMap(int** nOriginalMap, int** nGlobalMap, KuPose RobotPos)
{

	int nRobotX = (int)( m_math.MM2AI(RobotPos.getX()));
	int nRobotY = (int)( m_math.MM2AI(RobotPos.getY()));

	int nHalfSizeOfLocalMapX =m_nLocalMapX/2;
	int nHalfSizeOfLocalMapY =m_nLocalMapY/2;

	for(int i=nRobotX -nHalfSizeOfLocalMapX; i<nRobotX+nHalfSizeOfLocalMapX; i++){
		for(int j=nRobotY -nHalfSizeOfLocalMapY; j<nRobotY+nHalfSizeOfLocalMapY; j++){
			if(m_nGlobalMapX-1<i||i<1||m_nGlobalMapY-1<j||j<1) continue;

			nGlobalMap[i][j]=nOriginalMap[i][j] ;
		}
	}

}
/**
@brief Korean: 
@brief English: 
*/
int_1DArray KuLocalPathPlannerPr::combinateLaserNKinect(int_1DArray nLaserData181, int_1DArray nKinectRnageData)
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

		if(i>nStart&&i<nEnd)
		{
			if(nLaserData181[i]>nKinectRnageData[i-nStart]
			&& nKinectRnageData[i-nStart]>nKinectMinDist
				&& nKinectRnageData[i-nStart]<nKinectMaxDist)
			{
				nCombinateRangeData[i]=nKinectRnageData[i-nStart];
			}
		}
	}

	return nCombinateRangeData;
}

void KuLocalPathPlannerPr::generateLocalMap(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData181, int_1DArray nKinectRnageData)
{
	KuMapBuilderParameter InputParam; 		

	//Map copy  start------------------------------------------------------------------------------------------------------------------------	
	copyBuildingMapToGlobalMap(m_pBuildingMap->getMap(), m_pMap->getMap(),m_pOriginalMap->getMap(), RobotPos);		
	copyGlobalMapToLocalMap( m_pMap->getMap(), m_pLocalMap->getMap(), RobotPos, &m_nLocalMapSPosX, &m_nLocalMapSPosY);
}

/**
@brief Korean: 우회 경로를 생성하기 위한 목적지를 설정하는 함수
@brief English: Sets the goal position to generate detour path
*/
KuPose KuLocalPathPlannerPr::generateDetourGoalPos(KuPose RobotPos, vector<KuPose> vecPath, int nPathSize)
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
bool KuLocalPathPlannerPr::generateDetourPath(KuPose RobotPos, KuPose DetourGoalPos, KuMap* pLocalMap, 
	int nLocalMapSPosX, int nLocalMapSPosY, list<KuPose> *DetourPathList)
{
	int nRobotX = (int)( m_math.MM2AI(RobotPos.getX()));
	int nRobotY = (int)( m_math.MM2AI(RobotPos.getY()));

	KuPose LocalRobotPos, LocalGoalPos;
	LocalRobotPos.setX(RobotPos.getX()- m_math.AI2MM(nLocalMapSPosX));
	LocalRobotPos.setY(RobotPos.getY()- m_math.AI2MM(nLocalMapSPosY));
	LocalGoalPos.setX( DetourGoalPos.getX()- m_math.AI2MM(nLocalMapSPosX));
	LocalGoalPos.setY( DetourGoalPos.getY()- m_math.AI2MM(nLocalMapSPosY));
	m_KuLocalPathPlanner.setMap(pLocalMap->getMap(),m_math.getCellSizeMM());
	m_KuLocalPathPlanner.setProMap(pLocalMap->getProMap());
	int nResult =m_KuLocalPathPlanner.generatePath(LocalGoalPos,LocalRobotPos);//경로를 생성시키는 함수.

	if(nResult==0){ //경로가 생성된 경우.
		*DetourPathList = m_KuLocalPathPlanner.getPath();//실질적으로 경로를 리스트 형태로 받아오는 함수.
		*DetourPathList = m_KuPathSmoothing.smoothingPath(*DetourPathList);
		cout<<"우회경로 생성 성공"<<endl;
		return true; 
	}
	cout<<"우회경로 생성 실패~~~~~!!!!!!!!!!"<<endl;
	return false;
}

/**
@brief Korean: 생성된 지역지도 상의 경로를 전역 위치상의 경로로 바꿔주는 함수
@brief English: Changes path created by global position into path created by local map
*/
list<KuPose> KuLocalPathPlannerPr::transferLocaltoGlobalPath( list<KuPose> LocalPath,KuPose RobotPose,int nLocalMapSPosX, int nLocalMapSPosY)
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
bool KuLocalPathPlannerPr::generateLocalPath(KuPose RobotPos)
{
	list<KuPose> DetourPathList;
//	copyGlobalMapToLocalMap( m_pMap->getMap(), m_pLocalMap->getMap(), RobotPos, &m_nLocalMapSPosX, &m_nLocalMapSPosY);
	copyGlobalMapToLocalMap(m_pMap->getMap(), m_pLocalMap->getMap(), RobotPos,m_dpredictedProMap,m_pLocalMap->getProMap(), &m_nLocalMapSPosX, &m_nLocalMapSPosY);
	m_LocalGoalPos = generateDetourGoalPos(RobotPos, m_vecPath,m_vecPath.size()-1);
	generateDetourPath(RobotPos, m_LocalGoalPos, m_pLocalMap, m_nLocalMapSPosX, m_nLocalMapSPosY, &DetourPathList);
	DetourPathList= transferLocaltoGlobalPath(DetourPathList,RobotPos,m_nLocalMapSPosX, m_nLocalMapSPosY);
	doCriticalSection(0);	
	m_vecLocalPath.clear();    
	m_DetourPathList.clear();
	list<KuPose>::iterator it;
	for(it=DetourPathList.begin(); it!=DetourPathList.end(); it++){
		m_vecLocalPath.push_back(*it);
		m_DetourPathList.push_back(*it);
	}
	doCriticalSection(1);	
	if(m_vecLocalPath.size()==0) return false;	
	return true; 
}

/**
@brief Korean: 경로에 장애물의 존재 여부를 판단하고 장애물의 크기를 확장하는 함수
@brief English: Checks the presence of obstacles in path and extends the size of obstacles
*/
bool KuLocalPathPlannerPr::existObstaclePath(KuPose RobotPos, vector<KuPose> vecLocalPath, int nPathSize, KuMap* pLocalMap, int nLocalMapSPosX, int nLocalMapSPosY )
{
	int nX=0, nY=1;
	int **nMap=m_pMap->getMap();
	bool bObsCSpaceflag=false;
	int nObsCSpace=1;//2;
	int ninterval=2;//검사할 영역 간격
	int nObnCnt=3;
	int nObstVeticlaBnd=1;//3;//30cm

	int nRX = (int)( m_math.MM2AI(RobotPos.getX()));
	int nRY = (int)( m_math.MM2AI(RobotPos.getY()));

	if(nPathSize<nObnCnt){return false;}


	for(int i=nObnCnt; i<nPathSize; i++ )
	{
		int nGridPathX = (int)( m_math.MM2AI(vecLocalPath[i].getX()));
		int nGridPathY =(int)( m_math.MM2AI(vecLocalPath[i].getY()));

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


							double dDelX= ( m_math.AI2MM(nGridPathX-nRX))/1000.0;//((double)(nGridPathX-nRX))/10.0;
							double dDelY=( m_math.AI2MM(nGridPathY-nRY))/1000.0;//((double)(nGridPathY-nRY))/10.0;
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
bool KuLocalPathPlannerPr::generateLocalPathforObsAvoidance(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData181, int_1DArray nKinectRnageData)
{
	bool bDoLocalPathPlanning=false;
	bool bexistObstaclePath=false;
	bool breturn=true;

	if((fabs(RobotPos.getX()-m_LocalGoalPos.getX())<2000) &&( fabs(RobotPos.getY()-m_LocalGoalPos.getY())<2000) &&		
		(hypot(m_GoalPos.getX()-m_LocalGoalPos.getX(),m_GoalPos.getY()-m_LocalGoalPos.getY())>500))
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

// 	if(existObstaclePath(RobotPos, m_vecLocalPath, m_vecLocalPath.size()-1,  m_pLocalMap,  m_nLocalMapSPosX, m_nLocalMapSPosY ))//로컬패스의 장애물 검사.
// 	{
		if(m_bpredictexitObs==true)
		{
			bDoLocalPathPlanning = true;
			bexistObstaclePath =true;
			m_bpredictexitObs=false;	
			int nselObsID=m_mselObsID;
			int nseltime=m_dseltime;
			m_dseltime=-1;	
			m_mselObsID=-1;
			vector<KuTrajectory>	vecTunedTrajectory;
			KuDrawingInfo::getInstance()->getPredictedTraj(&vecTunedTrajectory);


			//RobotPos=calculatepredictRobotpos(RobotPos,m_dRobotVel,m_dRobotRVel,m_dseltime);
			int nObsCSpace=3;

			for(int i=nseltime;i<vecTunedTrajectory[nselObsID].PredictedPt.size();i++)
			{
				int nRX = (int)( m_math.M2AI(vecTunedTrajectory[nselObsID].PredictedPt[i].x));
				int nRY = (int)( m_math.M2AI(vecTunedTrajectory[nselObsID].PredictedPt[i].y));

				for(int q=-nObsCSpace;q<nObsCSpace+1;q++)
				{
					for(int p=-nObsCSpace;p<nObsCSpace+1;p++)
					{

						m_pMap->getMap()[nRX+q][nRY+p]=KuMap::WARNING_AREA;					
					}
				}
			}
				//KuDrawingInfo::getInstance()->setMap(m_pMap->getMap(),m_pMap->getX(),m_pMap->getY());

				CalculateGridPro(vecTunedTrajectory,nselObsID,nseltime,  0.8, 0.7);
				//CalculateGridPro(vecTunedTrajectory, 0.8, 0.7);
		}

	//}


	if(DelEncoderData.getThetaDeg()>0&&hypot(DelEncoderData.getX(),DelEncoderData.getY())<50)
	{
		bDoLocalPathPlanning = false;
		bexistObstaclePath =false;
	}
	//Path planning-----------------------------------------------------------------------
	if(bDoLocalPathPlanning == true||m_vecLocalPath.size()==0)
	{
		bDoLocalPathPlanning=false;
		breturn=generateLocalPath( RobotPos);
		if(!breturn) bDoLocalPathPlanning=true;
	}

	return breturn;
}

/**
@brief Korean: 센서데이터를 이용하여 타겟점과 로봇 사이의 장애물 유무를 검사하는 함수
@brief English: 
*/
bool KuLocalPathPlannerPr::checkObstacles(KuPose RobotPos,KuPose TargetPos,int_1DArray nLaserData181, int_1DArray nKinectRnageData ,double dTargetDist)
{
	double dGradientX = TargetPos.getXm() - RobotPos.getXm();
	double dGradientY = TargetPos.getYm() - RobotPos.getYm();
	double dDistofObst= hypot(TargetPos.getX()- RobotPos.getX(), TargetPos.getY()- RobotPos.getY());
	double dDistObstoRobot=dTargetDist;
	double dTempDistObstoRobot=dTargetDist;
	int nDistIndx=(int)dDistObstoRobot/(double)m_nCellSize;
	double dKinectOffset= (double)m_dKinectSensorOffsetmm;
	double dLaserOffset= (double)m_dLaserSensorOffsetmm;
	int nradiusofRobot=m_dRobotRadius;

	for (int ndistance = nDistIndx; ndistance <dDistObstoRobot; ndistance += nDistIndx) {

		double dRayOfX =  RobotPos.getX()+ ndistance*dGradientX;
		double dRayOfY = RobotPos.getY()+ ndistance*dGradientY;

		dTempDistObstoRobot=hypot(ndistance*dGradientX, ndistance*dGradientY);

		for(int i=0;i<m_nKinectRnageDataIDX;i++){

			if(nLaserData181[i+66]<50)continue;
			if(nKinectRnageData[i] <50)continue;

			if((nKinectRnageData[i] != 1000000&&nKinectRnageData[i] >50)&&nKinectRnageData[i] <dDistofObst){
				double dAngleRad = (double)(i -  m_nKinectRnageDataIDX/2) * D2R;
				double dX = RobotPos.getX() + ((double)nKinectRnageData[i] * cos(dAngleRad) + dKinectOffset ) * cos(RobotPos.getThetaRad()) + 
					((double)nKinectRnageData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());
				double dY = RobotPos.getY() + ((double)nKinectRnageData[i] * cos(dAngleRad) + dKinectOffset ) * sin(RobotPos.getThetaRad()) + 
					((double)nKinectRnageData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

				double tempD=hypot(dRayOfX-dX, dRayOfY- dY);

				if(tempD<nradiusofRobot&&tempD>0 ){	return true;	}// 거리에거리에 따른 장애물 감지
			}
			else if((nLaserData181[i+66] != 1000000&&nLaserData181[i+66]>50)&&nLaserData181[i+66]<dDistofObst){
				double dAngleRad = (double)(i -  m_nKinectRnageDataIDX/2) * D2R;
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
@brief Korean: 소요 시간을 측정하기 위해서 초기화하는 함수
@brief English: Initializes to count the duration
*/
void KuLocalPathPlannerPr::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}
/**
@brief Korean: 측정된 소요 시간을 받아오는 함수
@brief English: Gets the estimated duration
*/
float KuLocalPathPlannerPr::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}
/**
@brief Korean: 시뮬레이터를 종료한다.
@brief English: 
*/
void KuLocalPathPlannerPr::terminate()
{
	cout<<"[KuLocalPathPlannerPr]:: Behavior is terminated!!!"<<endl;
	m_LocalPathplaningThread.terminate();
	m_predictedMappingThread.terminate();
	cout<<"[KuLocalPathPlannerPr]:: Behavior is terminated!!!"<<endl;
	m_bThreadFlag = false;	
	m_bStandbyFlag=false; 

}

/**
@brief Korean: 경로에서 타겟점을 선택하는 함수
@brief English: 
*/
KuPose KuLocalPathPlannerPr::getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist)
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

	if(nPathSize<=1) return calPathPoint( RobotPos);

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
		return calPathPoint(RobotPos );
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
		return calPathPoint( RobotPos);
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


void KuLocalPathPlannerPr::predictObjectpositionPro()
{
	KuPose RobotPose = m_RobotPos;
	vector<KuTrajectory> vecPredictTraj;
	vector<KuTrajectory> vecObstectTrajectory;
	vector<KuPose> GlobalPath;

	doCriticalSection(0);
	for(int i=0; i<m_vecPath.size(); i++)
	{
		GlobalPath.push_back(m_vecPath[i]);
	}
	doCriticalSection(1);

	//first
	doCriticalSection(0);	
	for (int i=0;i<m_vecObstacleTrajectory.size();i++)
	{
		vecObstectTrajectory.push_back(m_vecObstacleTrajectory[i]);
		savedistantObstacledata(m_vecObstacleTrajectory[i]);
	}
	doCriticalSection(1);
	
	if(m_vecdistantObstacleTrajectory.size()>0)
	{
		predictObstaclePath();
	}	

	vecObstectTrajectory.clear();
	GlobalPath.clear();
	m_bDrawingmapflag=true;
}
double**KuLocalPathPlannerPr::getpredictedProMap()
{
	if(m_bDrawingmapflag==false) return m_dtmppredictedProMap;

	doCriticalSection(0);
	for(int i=0; i<m_nMapSizeX; i++){
		for(int j=0; j<m_nMapSizeY; j++){

			m_dtmppredictedProMap[i][j] = m_dpredictedProMap[i][j];
		}
	}
	doCriticalSection(1);

	return m_dtmppredictedProMap;
}

double KuLocalPathPlannerPr::GetGaussianValue2(double sigma, double x, double crx, double y,double cry)
{
	if (sigma==0) return 0;

	return exp(-((x-crx)*(x-crx)/2.0+(y-cry)*(y-cry)/2.0)/sigma/sigma)/2.0;
}

double KuLocalPathPlannerPr::GetGaussianValue(double sigma, double x)
{
	if (sigma==0) return 0;

	return exp(-x*x/2.0/sigma/sigma);
}


void KuLocalPathPlannerPr::dopredictedMappingThread(void* arg)
{
	KuLocalPathPlannerPr* pLPPT = (KuLocalPathPlannerPr*)arg;

	pLPPT->predictObjectpositionPro();
	
	//Sleep(100);

}

KuPose KuLocalPathPlannerPr::getTargetPos()
{
	return m_TargetPos;
}

/**
@brief Korean: KuLocalPathPlannerPr를 실행시키는 함수
@brief English: 
*/

bool KuLocalPathPlannerPr::start(int nTime)
{
	cout<<"initialize!!!"<<endl;
	if(initialize( )==true)
	{
		if(m_bThreadFlag==false)
		{
			m_bThreadFlag = true;
			m_LocalPathplaningThread.start(doLocalPathplanningThread,this,nTime); //메인 스레드 시작
			m_predictedMappingThread.start(dopredictedMappingThread,this,nTime); //메인 스레드 시작
			return true;	
		}
	}
}

void KuLocalPathPlannerPr::savedistantObstacledata(KuTrajectory KuTraj)
{
	int ndectectedId=-1;
	double dDist=0.0;
	int nTrasize=KuTraj.Tra_x.size()-1;

	for(int j=0; j<m_vecdistantObstacleTrajectory.size();j++)
	{
		if(KuTraj.ID==m_vecdistantObstacleTrajectory[j].ID)
		{		
			int nsize=m_vecdistantObstacleTrajectory[j].Tra_x.size()-1;
			dDist=hypot(KuTraj.Tra_x[nTrasize]-m_vecdistantObstacleTrajectory[j].Tra_x[nsize],
				KuTraj.Tra_y[nTrasize]-m_vecdistantObstacleTrajectory[j].Tra_y[nsize]);
			ndectectedId=j;
			break;
		}
	}

	if (ndectectedId!=-1)
	{
		if(dDist>0.5)
		{
			m_vecdistantObstacleTrajectory[ndectectedId].Tra_x.push_back(KuTraj.Tra_x[nTrasize]);
			m_vecdistantObstacleTrajectory[ndectectedId].Tra_y.push_back(KuTraj.Tra_y[nTrasize]);
			m_vecdistantObstacleTrajectory[ndectectedId].Tra_th.push_back(KuTraj.Tra_th[nTrasize]);
		}

	}
	else
	{
		KuTrajectory singleObjectTrajectory;
		singleObjectTrajectory.ID=KuTraj.ID;
		singleObjectTrajectory.Tra_x.push_back(KuTraj.Tra_x[nTrasize]);
		singleObjectTrajectory.Tra_y.push_back(KuTraj.Tra_y[nTrasize]);
		singleObjectTrajectory.Tra_th.push_back(KuTraj.Tra_th[nTrasize]);	
		m_vecdistantObstacleTrajectory.push_back(singleObjectTrajectory);			
	}

}

void KuLocalPathPlannerPr::predictObstaclePath()
{
	vector<KuTrajectory> vecTempTrajectory;
	vector<KuTrajectory> vecTunedTrajectory;
	vector<KuPose> vecRTrajectory;

	list<KuPose> DetourPathList;
	list<KuPose> outDetourPathList;
	KuPose ObjPos;
	int nID=-2;
	for( size_t i = 0; i < m_vecdistantObstacleTrajectory.size(); i++ )
	{
		for( size_t j = 0; j < m_vecdistantObstacleTrajectory[i].Tra_x.size(); j++ )
		{
			ObjPos.setXm(m_vecdistantObstacleTrajectory[i].Tra_x[j]);
			ObjPos.setYm(m_vecdistantObstacleTrajectory[i].Tra_y[j]);
			ObjPos.setThetaRad(m_vecdistantObstacleTrajectory[i].Tra_th[j]);
			nID = m_vecdistantObstacleTrajectory[i].ID;
			DetourPathList.push_back(ObjPos);

		}

		if(DetourPathList.size()>1)
			outDetourPathList = m_KuPathSmoothing.smoothingPathforprediction(DetourPathList);
		DetourPathList.clear();

		list<KuPose>::iterator it;
		size_t ik = 0;
	
		KuTrajectory  tempTrajectory;
		for (it = outDetourPathList.begin(); it != outDetourPathList.end(); it++,ik++)
		{

			tempTrajectory.Tra_x.push_back(it->getXm());
			tempTrajectory.Tra_y.push_back(it->getYm());
			tempTrajectory.Tra_th.push_back(it->getThetaRad());
			tempTrajectory.ID =nID;
		}

		if(tempTrajectory.ID > -2)
		{
			vecTempTrajectory.push_back(tempTrajectory);
		}

	}

	for (int i=0; i <vecTempTrajectory.size(); i++)
	{
		setdistantCubicSplineddata(vecTempTrajectory[i]);
	}

	for (int i=0; i <m_vecCubicSpTrajectory.size(); i++)
	{
		vecTunedTrajectory.push_back(m_vecCubicSpTrajectory[i]);
	}

	if(vecTunedTrajectory.size()>0)
	{

		KuDrawingInfo::getInstance()->setCubicPath(vecTunedTrajectory);


		double dTVel=m_dRobotVel;
		KuPose  RobotPos=m_RobotPos;
		double dRotDegVel=m_dRobotRVel;
		double dTime=30;
		double dSumRDeg=0.0;
		
		for (int k=1; k<dTime; k++)
		{
			RobotPos=calculatepredictRobotpos(RobotPos,dTVel,dRotDegVel,k);

			dSumRDeg+=dRotDegVel;

			if(dSumRDeg<M_PI_2&&dSumRDeg>-M_PI_2)
			{
				vecRTrajectory.push_back(RobotPos);
			}
		}

		double dCriDeg=0;
		for(int i=0; i<vecTunedTrajectory.size(); i++)
		{
			int nsize=vecTunedTrajectory[i].Tra_x.size()-1;	

			
			if(vecTunedTrajectory[i].Tra_x.size()>4) //continue; 
			{
				float fdx = vecTunedTrajectory[i].Tra_x[nsize] - vecTunedTrajectory[i].Tra_x[nsize-1];
				float fdy = vecTunedTrajectory[i].Tra_y[nsize] - vecTunedTrajectory[i].Tra_y[nsize-1];
				float fdx2 = vecTunedTrajectory[i].Tra_x[nsize-1] - vecTunedTrajectory[i].Tra_x[nsize-2];
				float fdy2 = vecTunedTrajectory[i].Tra_y[nsize-1] - vecTunedTrajectory[i].Tra_y[nsize-2];
				float fdx3 = vecTunedTrajectory[i].Tra_x[nsize-2] - vecTunedTrajectory[i].Tra_x[nsize-3];
				float fdy3 = vecTunedTrajectory[i].Tra_y[nsize-2] - vecTunedTrajectory[i].Tra_y[nsize-3];
				float fdx4 = vecTunedTrajectory[i].Tra_x[nsize-3] - vecTunedTrajectory[i].Tra_x[nsize-4];
				float fdy4 = vecTunedTrajectory[i].Tra_y[nsize-3] - vecTunedTrajectory[i].Tra_y[nsize-4];

				float fth = atan2(fdy,fdx);	
				dCriDeg=fth;
				if(fth > M_PI) fth -=2*M_PI;if(fth < -M_PI) fth +=2*M_PI;
				float fth2 = atan2(fdy2,fdx2);
				if(fth2 > M_PI) fth2 -=2*M_PI;if(fth2 < -M_PI) fth2 +=2*M_PI;				
				float fth3 = atan2(fdy3,fdx3);
				if(fth3 > M_PI) fth3 -=2*M_PI;if(fth3 < -M_PI) fth3 +=2*M_PI;
				float fth4 = atan2(fdy4,fdx4);
				if(fth4 > M_PI) fth4 -=2*M_PI;if(fth4 < -M_PI) fth4 +=2*M_PI;

				vecTunedTrajectory[i].Avg_x = 0.9*fdx + 0.09*fdx2 + 0.009*fdx3 + 0.0009*fdx4;
				vecTunedTrajectory[i].Avg_y = 0.9*fdy + 0.09*fdy2 + 0.009*fdy3 + 0.0009*fdy4;
				vecTunedTrajectory[i].Avg_th = atan2(vecTunedTrajectory[i].Avg_y, vecTunedTrajectory[i].Avg_x );
				vecTunedTrajectory[i].Avg_dth = 0.9*(fth - fth2) + 0.09*(fth2 - fth3) + 0.009*(fth3 - fth4);
				if(vecTunedTrajectory[i].Avg_th > M_PI) vecTunedTrajectory[i].Avg_th -=2*M_PI;
				if(vecTunedTrajectory[i].Avg_th < -M_PI) vecTunedTrajectory[i].Avg_th +=2*M_PI;
				if(vecTunedTrajectory[i].Avg_dth > M_PI) vecTunedTrajectory[i].Avg_dth -=2*M_PI;
				if(vecTunedTrajectory[i].Avg_dth < -M_PI) vecTunedTrajectory[i].Avg_dth +=2*M_PI;

				float fDgree= vecTunedTrajectory[i].Avg_dth;
				float fDist= hypot(vecTunedTrajectory[i].Avg_x ,vecTunedTrajectory[i].Avg_y);
				float fDist1= hypot(fdx ,fdy);
				float fDist2= hypot(fdx2 ,fdy2);
				float fDist3= hypot(fdx3 ,fdy3);
				float fDist4= hypot(fdx4 ,fdy4);
				float fDiffDist=((fDist1-fDist)*0.9+(fDist2-fDist)*0.09+(fDist3-fDist)*0.009+(fDist4-fDist)*0.0009);
				
				
				double dDiffdegr1=fabs(fth - fth2);
				double dDiffdegr2=fabs(fth2 - fth3);
				double dDiffdegr3=fabs(fth3 - fth4);
				double dRatio=10.0;

				dDiffdegr1=fabs(dDiffdegr1-vecTunedTrajectory[i].Avg_dth);
				dDiffdegr2=fabs(dDiffdegr2-vecTunedTrajectory[i].Avg_dth);
				dDiffdegr3=fabs(dDiffdegr3-vecTunedTrajectory[i].Avg_dth);
				double dDiffMean=(dDiffdegr1+dDiffdegr2+dDiffdegr3)/3.0*dRatio;
				 
				float x = vecTunedTrajectory[i].Tra_x[nsize] + (fDist*cos(fDgree+dCriDeg));
				float y = vecTunedTrajectory[i].Tra_y[nsize] + (fDist*sin(fDgree+dCriDeg));
					
				
				double dSumDeg=0.0;
				//dTime=dTime*(1-dDiffMean);			
				KuPose outPos;			
				double dMinDistBetRNObs=100000;



				KuDrawingInfo::getInstance()->setPredictedRTraj(vecRTrajectory);

				for (int j=1; j<vecRTrajectory.size(); j++)
				{
					
					double dAngle=fDgree*(1+dDiffMean*j);
					fDist+=fDiffDist;
					dSumDeg+=dAngle;
					if(dAngle > M_PI) dAngle -=2*M_PI;
					if(dAngle < -M_PI) dAngle +=2*M_PI;
					if(dSumDeg>M_PI||dSumDeg<-M_PI) continue;
					if(fDist>1.0) fDist=1.0;
					Point2f pt = Point2f(x,y);//x*cos(nAngle)-y*sin(nAngle), x*sin(nAngle)+y*cos(nAngle));

					if(dSumDeg<M_PI&&dSumDeg>-M_PI)
					{
						vecTunedTrajectory[i].PredictedPt.push_back(pt);
						int nnsize = vecTunedTrajectory[i].PredictedPt.size()-1;

						x = vecTunedTrajectory[i].PredictedPt[nnsize].x + (fDist*cos(dAngle+dCriDeg));
						y = vecTunedTrajectory[i].PredictedPt[nnsize].y + (fDist*sin(dAngle+dCriDeg));
					}



					double dDistBetRNObs=hypot(vecRTrajectory[j].getXm()-x,vecRTrajectory[j].getYm()-y);
					//double dDistBetRNObs=hypot(RobotPos.getXm()-x,RobotPos.getYm()-y);

					if(dDistBetRNObs<dMinDistBetRNObs)
					{
						dMinDistBetRNObs=dDistBetRNObs;
						if(dMinDistBetRNObs<0.5)
						{
							m_dseltime=j;	
							m_mselObsID=i;
							m_bpredictexitObs=true;
							printf("!!!!!!!!!!!!!!!!!  dseltime=%d ",j);
						}
					}


				}				
			}
			
		}
	}

	vecRTrajectory.clear();
	KuDrawingInfo::getInstance()->setPredictedTraj(vecTunedTrajectory);
	

	vecTunedTrajectory.clear();
	vecTempTrajectory.clear();
	vecRTrajectory.clear();


}

void KuLocalPathPlannerPr::CalculateGridPro(vector<KuTrajectory> vecObstacleTraj,int nselObsID,int nseltime,  double dDeviation, double dWeight)
{
	int nObstSizeNum = vecObstacleTraj.size();
	int nTrajSizeNum;
	int nMinGridX, nMinGridY, nMaxGridX, nMaxGridY;
	double dStartXm, dStartYm, dEndXm, dEndYm;
	if(	nselObsID==-1||nseltime==-1) return;

	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			m_dpredictedProMap[i][j]=0;
		}
	}

	///예외처리
	if(nObstSizeNum==0)   return;

	//for(int i=0; i<nObstSizeNum; i++)
	{
		nTrajSizeNum = nseltime+3;

		//예외처리
		//if(nTrajSizeNum==0)   continue;
		if(nTrajSizeNum>=vecObstacleTraj[nselObsID].PredictedPt.size()) nTrajSizeNum=vecObstacleTraj[nselObsID].PredictedPt.size()-1;
		if (nTrajSizeNum<1)	{return;}
		dStartXm = vecObstacleTraj[nselObsID].PredictedPt[0].x;
		dStartYm = vecObstacleTraj[nselObsID].PredictedPt[0].y;
		dEndXm = vecObstacleTraj[nselObsID].PredictedPt[nTrajSizeNum].x;
		dEndYm = vecObstacleTraj[nselObsID].PredictedPt[nTrajSizeNum].y;

		//한 장애물마다 확률 검사 영역 설정
		if(dStartXm>=dEndXm)
		{
			nMinGridX = m_math.M2AI(dEndXm-3.0);
			nMaxGridX = m_math.M2AI(dStartXm+3.0);

			if(dStartYm>=dEndYm)
			{
				nMinGridY = m_math.M2AI(dEndYm-3.0);
				nMaxGridY = m_math.M2AI(dStartYm+3.0);
			}
			else
			{
				nMinGridY = m_math.M2AI(dStartYm-3.0);
				nMaxGridY = m_math.M2AI(dEndYm+3.0);
			}
		}
		else
		{
			nMinGridX = m_math.M2AI(dStartXm-3.0);
			nMaxGridX = m_math.M2AI(dEndXm+3.0);

			if(dStartYm>=dEndYm)
			{
				nMinGridY = m_math.M2AI(dEndYm-3.0);
				nMaxGridY = m_math.M2AI(dStartYm+3.0);
			}
			else
			{
				nMinGridY = m_math.M2AI(dStartYm-3.0);
				nMaxGridY = m_math.M2AI(dEndYm+3.0);
			}
		}

		int nGridX, nGridY;
		double dGridXm, dGridYm;

		for(nGridX = nMinGridX; nGridX<nMaxGridX; nGridX++)
		{
			for(nGridY = nMinGridY; nGridY<nMaxGridY; nGridY++)
			{
				//m_dpredictedProMap[nGridX][nGridY] = 0.0;

				dGridXm = m_math.AI2M(nGridX);
				dGridYm = m_math.AI2M(nGridY);

				for(int j=0; j<nTrajSizeNum; j++)
				{
					double dTempMeanXm=vecObstacleTraj[nselObsID].PredictedPt[j].x;
					double dtempMeanYm=vecObstacleTraj[nselObsID].PredictedPt[j].y;

					double dTempPro = pow(dWeight,nTrajSizeNum-1-j)*GetGaussianValue2(dDeviation,dGridXm, dTempMeanXm, dGridYm, dtempMeanYm);
					m_dpredictedProMap[nGridX][nGridY] += dTempPro;         
				}
			}
		}
	}

// 	KuDrawingInfo::getInstance()->setpredictedProMap(KuLocalPathPlannerPr::getInstance()->getpredictedProMap(),
// 		kuMapRepository::getInstance()->getMap()->getX(),kuMapRepository::getInstance()->getMap()->getY());
}

void KuLocalPathPlannerPr::CalculateGridPro(vector<KuTrajectory> vecObstacleTraj, double dDeviation, double dWeight)
{
	int nObstSizeNum = vecObstacleTraj.size();
	int nTrajSizeNum;
	int nMinGridX, nMinGridY, nMaxGridX, nMaxGridY;
	double dStartXm, dStartYm, dEndXm, dEndYm;

	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			m_dpredictedProMap[i][j]=0;
		}
	}

	///예외처리
	if(nObstSizeNum==0)   return;

	for(int i=0; i<nObstSizeNum; i++)
	{
		nTrajSizeNum = vecObstacleTraj[i].PredictedPt.size();

		//예외처리
		if(nTrajSizeNum==0)   continue;

		dStartXm = vecObstacleTraj[i].PredictedPt[0].x;
		dStartYm = vecObstacleTraj[i].PredictedPt[0].y;
		dEndXm = vecObstacleTraj[i].PredictedPt[nTrajSizeNum-1].x;
		dEndYm = vecObstacleTraj[i].PredictedPt[nTrajSizeNum-1].y;

		//한 장애물마다 확률 검사 영역 설정
		if(dStartXm>=dEndXm)
		{
			nMinGridX = m_math.M2AI(dEndXm-3.0);
			nMaxGridX = m_math.M2AI(dStartXm+3.0);

			if(dStartYm>=dEndYm)
			{
				nMinGridY = m_math.M2AI(dEndYm-3.0);
				nMaxGridY = m_math.M2AI(dStartYm+3.0);
			}
			else
			{
				nMinGridY = m_math.M2AI(dStartYm-3.0);
				nMaxGridY = m_math.M2AI(dEndYm+3.0);
			}
		}
		else
		{
			nMinGridX = m_math.M2AI(dStartXm-3.0);
			nMaxGridX = m_math.M2AI(dEndXm+3.0);

			if(dStartYm>=dEndYm)
			{
				nMinGridY = m_math.M2AI(dEndYm-3.0);
				nMaxGridY = m_math.M2AI(dStartYm+3.0);
			}
			else
			{
				nMinGridY = m_math.M2AI(dStartYm-3.0);
				nMaxGridY = m_math.M2AI(dEndYm+3.0);
			}
		}

		int nGridX, nGridY;
		double dGridXm, dGridYm;

		for(nGridX = nMinGridX; nGridX<nMaxGridX; nGridX++)
		{
			for(nGridY = nMinGridY; nGridY<nMaxGridY; nGridY++)
			{
				//m_dpredictedProMap[nGridX][nGridY] = 0.0;

				dGridXm = m_math.AI2M(nGridX);
				dGridYm = m_math.AI2M(nGridY);

				for(int j=0; j<nTrajSizeNum; j++)
				{
					double dTempMeanXm=vecObstacleTraj[i].PredictedPt[j].x;
					double dtempMeanYm=vecObstacleTraj[i].PredictedPt[j].y;

					double dTempPro = pow(dWeight,nTrajSizeNum-1-j)*GetGaussianValue2(dDeviation,dGridXm, dTempMeanXm, dGridYm, dtempMeanYm);
					m_dpredictedProMap[nGridX][nGridY] += dTempPro;         
				}
			}
		}
	}

	KuDrawingInfo::getInstance()->setpredictedProMap(KuLocalPathPlannerPr::getInstance()->getpredictedProMap(),
		kuMapRepository::getInstance()->getMap()->getX(),kuMapRepository::getInstance()->getMap()->getY());
}

KuPose KuLocalPathPlannerPr::calculatepredictRobotpos(KuPose  RobotPos,double dTVel,double dRotDegVel,double dTime)
{	

	vector<KuPose> vecLocalPath;
	KuPose  outRobotPos;
// 	double dTVel;
// 	double dRotDegVel;

	doCriticalSection(0);
	for (int i=0;i<m_vecLocalPath.size();i++)
	{
		vecLocalPath.push_back(m_vecLocalPath[i]);
	}
	doCriticalSection(1);
	
	static int BETWEEN_WHEEL = 500;

	double dW = dRotDegVel;
	double dRightWheelVel = dTVel + (BETWEEN_WHEEL * dW)/2.;
	double  dLeftWheelVel = dTVel - (BETWEEN_WHEEL * dW)/2.;

	double dAverageWheelDistance = (dLeftWheelVel + dRightWheelVel)/2;

	double  dDeltaT = (dRightWheelVel - dLeftWheelVel) / BETWEEN_WHEEL;
	double  dDeltaX;
	double  dDeltaY;

	if(fabs(dDeltaT) >= 0.0017f ) {
		double dDistance2RobotCenter = dAverageWheelDistance / dDeltaT;
		dDeltaY = dDistance2RobotCenter - ( dDistance2RobotCenter * cos(dDeltaT) );
		dDeltaX = dDistance2RobotCenter * sin(dDeltaT);
	}
	else {
		dDeltaY = 0;
		dDeltaX = dAverageWheelDistance;
	}

	//상대좌표를 절대좌표로 변환하는것.
	outRobotPos.setXm( RobotPos.getXm()+ dDeltaX * cos( RobotPos.getThetaRad() ) + dDeltaY * sin( - RobotPos.getThetaRad() ));
	outRobotPos.setYm( RobotPos.getYm()+dDeltaX * sin(  RobotPos.getThetaRad() ) + dDeltaY * cos(  RobotPos.getThetaRad()  ));
	outRobotPos.setThetaRad(  RobotPos.getThetaRad()+dDeltaT);
	
	return  outRobotPos;
}

void KuLocalPathPlannerPr::setdistantCubicSplineddata(KuTrajectory KuTraj)
{
	int ndectectedId=-1;
	double dDist=0.0;
	double dDegree=0.0;

	int nTrasize=KuTraj.Tra_x.size()-1;

	for(int j=0; j<m_vecCubicSpTrajectory.size();j++)
	{
		if(KuTraj.ID==m_vecCubicSpTrajectory[j].ID)
		{		
			int nsize=m_vecCubicSpTrajectory[j].Tra_x.size()-1;
			dDist=hypot(KuTraj.Tra_x[nTrasize]-m_vecCubicSpTrajectory[j].Tra_x[nsize],
				KuTraj.Tra_y[nTrasize]-m_vecCubicSpTrajectory[j].Tra_y[nsize]);

			if(nTrasize>2)
			{
				float fdx = KuTraj.Tra_x[nTrasize] - KuTraj.Tra_x[nTrasize-1];
				float fdy = KuTraj.Tra_y[nTrasize] - KuTraj.Tra_y[nTrasize-1];

				float fdx2 = KuTraj.Tra_x[nTrasize-1] - KuTraj.Tra_x[nTrasize-2];
				float fdy2 = KuTraj.Tra_y[nTrasize-1] - KuTraj.Tra_y[nTrasize-2];
				
				dDegree=atan2(fdy2,fdx2)-atan2(fdy,fdx);				

			}
			ndectectedId=j;
			break;
		}
	}

	if (ndectectedId!=-1)
	{
		if(dDist>0.3||fabs(dDegree)>3*D2R)
		{
			m_vecCubicSpTrajectory[ndectectedId].Tra_x.push_back(KuTraj.Tra_x[nTrasize]);
			m_vecCubicSpTrajectory[ndectectedId].Tra_y.push_back(KuTraj.Tra_y[nTrasize]);
			m_vecCubicSpTrajectory[ndectectedId].Tra_th.push_back(KuTraj.Tra_th[nTrasize]);
		}

	}
	else
	{
		KuTrajectory singleObjectTrajectory;
		singleObjectTrajectory.ID=KuTraj.ID;
		singleObjectTrajectory.Tra_x.push_back(KuTraj.Tra_x[nTrasize]);
		singleObjectTrajectory.Tra_y.push_back(KuTraj.Tra_y[nTrasize]);
		singleObjectTrajectory.Tra_th.push_back(KuTraj.Tra_th[nTrasize]);	
		m_vecCubicSpTrajectory.push_back(singleObjectTrajectory);			
	}

}