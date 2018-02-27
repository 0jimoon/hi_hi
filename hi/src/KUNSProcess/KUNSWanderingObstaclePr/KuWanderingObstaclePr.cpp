#include "StdAfx.h"
#include "KuWanderingObstaclePr.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#endif
KuWanderingObstaclePr::KuWanderingObstaclePr(void)
{
	m_bIsThreadFuncGenerated = false; //스레드가 생성되지 않았다는 플래그
	m_doThreadFunc = false; //스레드 함수가 실행되도록 하는 플래그
	m_nThreadFuncPeriod = 0; //스레드 함수 실행 주기..단위는ms
	m_bSuspendFlag = false;
	m_ngroupID=0;
	m_nMovingId=0;
	m_pMap=NULL;
	srand((unsigned)time(NULL)); //랜덤 씨드를 항상 바꿔주기 위해서 설정해야한다.
	m_psensor_value=NULL;
}

KuWanderingObstaclePr::~KuWanderingObstaclePr(void)
{
	if(m_pMap!=NULL)
		delete m_pMap;

	if(m_psensor_value){
		delete[] m_psensor_value;
	}
}


void KuWanderingObstaclePr::suspend()
{
	m_bSuspendFlag = true;
}

void KuWanderingObstaclePr::resume()
{
	m_bSuspendFlag = false;
}
void KuWanderingObstaclePr::terminate()
{
	m_bIsThreadFuncGenerated = false; //스레드가 생성되지 않았다는 플래그
	m_doThreadFunc = false; //스레드 함수가 실행되도록 하는 플래그
	m_bSuspendFlag = false;
}


void KuWanderingObstaclePr::doThread(void* arg)
{
	KuWanderingObstaclePr* pWOBh = (KuWanderingObstaclePr*)arg;
	pWOBh->executeBehavior();
}


bool KuWanderingObstaclePr::execute(KuCommandMessage CMessage)
{

	switch(CMessage.getCommandName()){
	case KuCommandMessage::START_THREAD:
		//if(initializeAll(CMessage))	
		if(m_doThreadFunc==false)
			if(initialize(CMessage))	
			{
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
vector<WanderingObs>KuWanderingObstaclePr::getvecMultiObs()
{
	vector<WanderingObs> vecMultiObs;
	m_CriticalSection.Lock();

	for(int i=0; i<m_vecMultiObs.size();i++)
	{
		vecMultiObs.push_back(m_vecMultiObs[i]);
	}
	m_CriticalSection.Unlock();

	return vecMultiObs;
}

void KuWanderingObstaclePr::setvecMultiObs(vector<WanderingObs> vecMultiObs)
{
	m_CriticalSection.Lock();	
	m_vecMultiObs.clear();
	for(int i=0; i<vecMultiObs.size();i++)
	{
		m_vecMultiObs.push_back(vecMultiObs[i]);
	}
	m_CriticalSection.Unlock();

}


vector<KuPose>KuWanderingObstaclePr::getvecMultiWanderingObstacles()
{
	
	vector<KuPose> vecMultiRobot;
	m_CriticalSection.Lock();	
	for(int i=0; i<m_vecMultiObs.size();i++)
	{
		KuPose ObsPos;		
		ObsPos.setID(m_vecMultiObs[i].ID);
		ObsPos.setXm(m_vecMultiObs[i].xm);
		ObsPos.setYm(m_vecMultiObs[i].ym);
		ObsPos.setZm(m_vecMultiObs[i].groupID);
		ObsPos.setDist(m_vecMultiObs[i].vel);
		ObsPos.setThetaRad(m_vecMultiObs[i].tr);
		vecMultiRobot.push_back(ObsPos);
	}
	m_CriticalSection.Unlock();
	return vecMultiRobot;
}
vector<KuPose>KuWanderingObstaclePr::getvecMultiGoalPose()
{

	vector<KuPose> vecMultiRobot;
	m_CriticalSection.Lock();	
	for(int i=0; i<m_vecMultiObs.size();i++)
	{
		KuPose ObsPos;		
		ObsPos.setID(m_vecMultiObs[i].ID);
		ObsPos.setXm(m_vecMultiObs[i].Gxm);
		ObsPos.setYm(m_vecMultiObs[i].Gym);
		ObsPos.setZm(m_vecMultiObs[i].groupID);
		ObsPos.setDist(m_vecMultiObs[i].vel);
		ObsPos.setThetaRad(m_vecMultiObs[i].tr);
		vecMultiRobot.push_back(ObsPos);
	}
	m_CriticalSection.Unlock();
	return vecMultiRobot;
}
void KuWanderingObstaclePr::setvecMultiWanderingObstacles(vector<KuPose> vecMultiRobot)
{
	m_CriticalSection.Lock();	

	m_vecMultiRobot.clear();
	for(int i=0; i<vecMultiRobot.size();i++)
	{
		m_vecMultiRobot.push_back(vecMultiRobot[i]);
	}
	m_CriticalSection.Unlock();

}

bool KuWanderingObstaclePr::checkBoundary(vector<KuPose>vecObstacleAreas,double dRandX,double dRandY,KuPose ObsPose)
{
	int nObstacleAreas=vecObstacleAreas.size();
	if(vecObstacleAreas.size()>0.0)	nObstacleAreas=SAMPLENUM/nObstacleAreas+1;
	else nObstacleAreas=1;
	int nObsAreaDist=3000;

	for(int k=0; k<vecObstacleAreas.size();k++)
		if(sqrt(pow(vecObstacleAreas[k].getX()-dRandX,2)+pow(vecObstacleAreas[k].getY()-dRandY,2))>nObsAreaDist&&ObsPose.getPro( )==vecObstacleAreas[k].getPro()){
			return true;
		}
		return false;
}

void KuWanderingObstaclePr::initObstacles()
{
	vector<KuPose> vecObstacleAreas = KuDrawingInfo::getInstance()->getObstacleAreas();
	KuPose GoalPos=KuDrawingInfo::getInstance()->getObsGoalPos();
	if(GoalPos.getX()==0&&GoalPos.getY()==0) return;
	KuPose initGoalPos;
	KuDrawingInfo::getInstance()->setObsGoalPos(initGoalPos);
	WanderingObs ObsPos;
	vector<WanderingObs> vecMultiObs = getvecMultiObs();

	if(m_vecObstacleAreas.size()<vecObstacleAreas.size())
	{
		m_vecObstacleAreas.push_back(vecObstacleAreas[vecObstacleAreas.size()-1]);
		int nID=vecObstacleAreas[vecObstacleAreas.size()-1].getID();
		if(nID==STOPID)
		{
			ObsPos.xm=vecObstacleAreas[vecObstacleAreas.size()-1].getXm(); 
			ObsPos.ym=vecObstacleAreas[vecObstacleAreas.size()-1].getYm();
			ObsPos.tr=0.0;
			ObsPos.vel=0.0;
			ObsPos.groupID=-1;
			ObsPos.groupNum=1;
			ObsPos.dist=0.3;
			ObsPos.ID=KuMap::OCCUPIED_OBSTACLES_GRID+vecMultiObs.size();
			ObsPos.patternID=STOPID;
			vecMultiObs.push_back(ObsPos);
		}	
		else if(nID==MOVINGID)
		{
			ObsPos.xm=vecObstacleAreas[vecObstacleAreas.size()-1].getXm(); 
			ObsPos.ym=vecObstacleAreas[vecObstacleAreas.size()-1].getYm();
			ObsPos.tr=M_PI*(0.5 -(float)rand()/(float)RAND_MAX);
			ObsPos.vel=0.09;
			ObsPos.dist=0.3;
			ObsPos.ID=KuMap::OCCUPIED_OBSTACLES_GRID+vecMultiObs.size();
			ObsPos.groupID=-1;
			ObsPos.groupNum=1;
			ObsPos.patternID=MOVINGID;
			ObsPos.movingID=m_nMovingId;
	
			while(1)
			{
// 				ObsPos.Gxm=32.6;//100.0*(float)rand()/(float)RAND_MAX;
// 				ObsPos.Gym=33.96;//100.0*(float)rand()/(float)RAND_MAX;
// 				ObsPos.Gxm=55.2;//100.0*(float)rand()/(float)RAND_MAX;
// 				ObsPos.Gym=61.15;//100.0*(float)rand()/(float)RAND_MAX;
				ObsPos.Gxm=GoalPos.getXm();//100.0*(float)rand()/(float)RAND_MAX;
				ObsPos.Gym=GoalPos.getYm();//100.0*(float)rand()/(float)RAND_MAX;
				if (m_math.M2AI(ObsPos.Gxm)<1 || m_math.M2AI(ObsPos.Gxm) >= m_nMapSizeX-1
					|| m_math.M2AI(ObsPos.Gym)< 1 || m_math.M2AI(ObsPos.Gym) >= m_nMapSizeY-1 ){
						continue;
				}
				if ( m_pMap->getMap()[m_math.M2AI(ObsPos.Gxm)][m_math.M2AI(ObsPos.Gym)] == KuMap::EMPTY_AREA )
				{
					KuPose RobotPos; RobotPos.setXm(ObsPos.xm); RobotPos.setYm(ObsPos.ym);
					KuPose GoalPos; GoalPos.setXm(ObsPos.Gxm); GoalPos.setYm(ObsPos.Gym);

					m_KuPathPlanner.generatePath(GoalPos, RobotPos); //경로 생성
					list <KuPose> listPath = m_KuPathPlanner.getPath();
					if(listPath.size()==0){	continue;}//경로생성 실패의 경우

					list<KuPose>::iterator it;
					//Path
					for (it = listPath.begin(); it != listPath.end(); it++) 
					{					
						ObsPos.pathlist.push_back(*it);
					}
			
					KuDrawingInfo::getInstance()->setvecObsLocalPath(listPath);
					break;
				}
			}
			
			KuVFHPlusPr tmpVFHPlusPr;
			tmpVFHPlusPr.init();
			tmpVFHPlusPr.setVelocity(900,10,30);
			tmpVFHPlusPr.setRobotRadius(0.2);
			vecKVFHPPr.push_back(tmpVFHPlusPr);

			vecMultiObs.push_back(ObsPos);
			m_nMovingId++;
		}	
		else if(nID==GROUPID2)
		{			
			ObsPos.xm=vecObstacleAreas[vecObstacleAreas.size()-1].getXm();
			ObsPos.ym=vecObstacleAreas[vecObstacleAreas.size()-1].getYm();
			ObsPos.tr=M_PI*(0.5 -(float)rand()/(float)RAND_MAX);;
			ObsPos.vel=0.08;
			ObsPos.dist=0.5;//GROUPDIST/2.0;
			ObsPos.groupID=m_ngroupID;
			ObsPos.groupNum=2;
			ObsPos.ID=KuMap::OCCUPIED_OBSTACLES_GRID+vecMultiObs.size();
			ObsPos.patternID=GROUPID2;
			ObsPos.movingID=m_nMovingId;	

			while(1)
			{
				ObsPos.Gxm=GoalPos.getXm();//100.0*(float)rand()/(float)RAND_MAX;
				ObsPos.Gym=GoalPos.getYm();//100.0*(float)rand()/(float)RAND_MAX;

				if (m_math.M2AI(ObsPos.Gxm)<1 || m_math.M2AI(ObsPos.Gxm) >= m_nMapSizeX-1
					|| m_math.M2AI(ObsPos.Gym)< 1 || m_math.M2AI(ObsPos.Gym) >= m_nMapSizeY-1 ){
						continue;
				}
				if ( m_pMap->getMap()[m_math.M2AI(ObsPos.Gxm)][m_math.M2AI(ObsPos.Gym)] == KuMap::EMPTY_AREA )
				{
					KuPose RobotPos; RobotPos.setXm(ObsPos.xm); RobotPos.setYm(ObsPos.ym);
					KuPose GoalPos; GoalPos.setXm(ObsPos.Gxm); GoalPos.setYm(ObsPos.Gym);

					m_KuPathPlanner.generatePath(GoalPos, RobotPos); //경로 생성
					list <KuPose> listPath = m_KuPathPlanner.getPath();
					if(listPath.size()==0){	continue;}//경로생성 실패의 경우

					list<KuPose>::iterator it;
					//Path
					for (it = listPath.begin(); it != listPath.end(); it++) 
					{					
						ObsPos.pathlist.push_back(*it);
					}
					break;
				}
			}
			double dvelocity=100+800*((float)rand()/(float)RAND_MAX)*(m_ngroupID%2);
			double dRvelocity=10+50*((float)rand()/(float)RAND_MAX)*(m_ngroupID%2);

			KuVFHPlusPr tmpVFHPlusPr;
			tmpVFHPlusPr.init();
			tmpVFHPlusPr.setRobotRadius((double)0.4);
			tmpVFHPlusPr.setVelocity(dvelocity,10,30);
			vecKVFHPPr.push_back(tmpVFHPlusPr);
			int ncriid=vecMultiObs.size();
			vecMultiObs.push_back(ObsPos);
			double dX=ObsPos.xm;
			double dY=ObsPos.ym;
			double dth=ObsPos.tr;

			for(int i=1;i<2;i++)
			{	
				WanderingObs nextObsPos=getGroupWanderingObs(ObsPos,vecMultiObs);
				vecMultiObs.push_back(nextObsPos);
				dX+=nextObsPos.xm;
				dY+=nextObsPos.ym;
				dth+=nextObsPos.tr;
			}	

			vecMultiObs[ncriid].cri_xm=dX/2.0;
			vecMultiObs[ncriid].cri_ym=dY/2.0;
			vecMultiObs[ncriid].cri_tr=dth/2.0;
		
			m_ngroupID++;
			m_nMovingId++;
		}	
		else if(nID==GROUPID3)
		{			
			ObsPos.xm=vecObstacleAreas[vecObstacleAreas.size()-1].getXm();
			ObsPos.ym=vecObstacleAreas[vecObstacleAreas.size()-1].getYm();
			ObsPos.tr=M_PI*(0.5 -(float)rand()/(float)RAND_MAX);;
			ObsPos.vel=0.08;
			ObsPos.dist=0.5;//GROUPDIST/2.0;
			ObsPos.groupID=m_ngroupID;
			ObsPos.groupNum=3;
			ObsPos.ID=KuMap::OCCUPIED_OBSTACLES_GRID+vecMultiObs.size();
			ObsPos.patternID=GROUPID3;
			ObsPos.movingID=m_nMovingId;	

			while(1)
			{
				ObsPos.Gxm=GoalPos.getXm();//100.0*(float)rand()/(float)RAND_MAX;
				ObsPos.Gym=GoalPos.getYm();//100.0*(float)rand()/(float)RAND_MAX;

				if (m_math.M2AI(ObsPos.Gxm)<1 || m_math.M2AI(ObsPos.Gxm) >= m_nMapSizeX-1
					|| m_math.M2AI(ObsPos.Gym)< 1 || m_math.M2AI(ObsPos.Gym) >= m_nMapSizeY-1 ){
						continue;
				}
				if ( m_pMap->getMap()[m_math.M2AI(ObsPos.Gxm)][m_math.M2AI(ObsPos.Gym)] == KuMap::EMPTY_AREA )
				{
					KuPose RobotPos; RobotPos.setXm(ObsPos.xm); RobotPos.setYm(ObsPos.ym);
					KuPose GoalPos; GoalPos.setXm(ObsPos.Gxm); GoalPos.setYm(ObsPos.Gym);

					m_KuPathPlanner.generatePath(GoalPos, RobotPos); //경로 생성
					list <KuPose> listPath = m_KuPathPlanner.getPath();
					if(listPath.size()==0){	continue;}//경로생성 실패의 경우

					list<KuPose>::iterator it;
					//Path
					for (it = listPath.begin(); it != listPath.end(); it++) 
					{					
						ObsPos.pathlist.push_back(*it);
					}
					break;
				}
			}

			KuVFHPlusPr tmpVFHPlusPr;
			tmpVFHPlusPr.init();
			tmpVFHPlusPr.setRobotRadius((double)0.4);
			tmpVFHPlusPr.setVelocity(700,10,30);
			vecKVFHPPr.push_back(tmpVFHPlusPr);

			vecMultiObs.push_back(ObsPos);

			for(int i=1;i<3;i++)
			{	
				WanderingObs nextObsPos=getGroupWanderingObs(ObsPos,vecMultiObs);
				vecMultiObs.push_back(nextObsPos);
			}		
			m_ngroupID++;
			m_nMovingId++;
		}	
	}

	setvecMultiObs(vecMultiObs);
}

WanderingObs KuWanderingObstaclePr::getGroupWanderingObs(WanderingObs ObsPos,vector<WanderingObs> vecMultiObs)
{
	double dRad=0.0;
	double dDistX=0.3;
	double dDistY=0.3;

	WanderingObs outObsPos;

	while(1)
	{
		dDistX=(0.5 -(float)rand()/(float)RAND_MAX);
		dDistY=(0.5 -(float)rand()/(float)RAND_MAX);
		if(hypot(dDistX,dDistY)<0.7)continue;
		outObsPos.xm=ObsPos.xm+dDistX;	
		outObsPos.ym=ObsPos.ym+dDistY;
		outObsPos.tr=ObsPos.tr+GROUPDEGREE*D2R*(0.5 -(float)rand()/(float)RAND_MAX);;
		outObsPos.vel=ObsPos.vel;
		outObsPos.dist=0.5;
		outObsPos.groupID=ObsPos.groupID;
		outObsPos.groupNum=ObsPos.groupNum;
		outObsPos.ID=KuMap::OCCUPIED_OBSTACLES_GRID+vecMultiObs.size();
		outObsPos.patternID=ObsPos.patternID;

		if(!checkBetweenAllObstacles(outObsPos,vecMultiObs)){break;}
	}

	return outObsPos;
}


WanderingObs KuWanderingObstaclePr::getGroupWanderingObsinBoundary(WanderingObs CriPos,WanderingObs ObsPos,vector<WanderingObs> vecMultiObs,int ngroupID,double *deltax,double *deltay)
{
	double dRad=0.0;
	double dDistX=0.3;
	double dDistY=0.3;

	WanderingObs outObsPos;
	double derr=2.0;
	//int ncnt=0;
	while(1)
	{
		double dTheta=GROUPDEGREE*D2R*(0.5 -(float)rand()/(float)RAND_MAX);
		dDistX=CriPos.vel*2.0*(0.5-(float)rand()/(float)RAND_MAX)*cos(CriPos.tr+dTheta);
		dDistY=CriPos.vel*derr*(0.5-(float)rand()/(float)RAND_MAX)*sin(CriPos.tr+dTheta);
		
		//if(hypot(dDistX,dDistY)>0.05)continue;
		
		outObsPos.xm=ObsPos.xm+dDistX;	
		outObsPos.ym=ObsPos.ym+dDistY;
		outObsPos.tr=ObsPos.tr;//+GROUPDEGREE*D2R*(0.5 -(float)rand()/(float)RAND_MAX);;
		
		if(hypot(CriPos.xm-outObsPos.xm,CriPos.ym-outObsPos.ym)>0.7) continue;

		if(!checkBetweenAllObstacles(outObsPos,vecMultiObs,ngroupID)){break;}
		derr+=0.1;
	}
	(*deltax)=dDistX;
	(*deltay)=dDistY;


	return outObsPos;
}
/*
bool KuWanderingObstaclePr::executeBehavior()
{
	initObstacles();

	KuPose RobotPos;
	vector<WanderingObs> vecMultiObs=getvecMultiObs();
	int ngroupNum=1;
	WanderingObs ObsPos;
	vector<GroupObs> vecGObsDB;
	
	GroupObs singleGObs;

	for(int i=0; i<vecMultiObs.size();i+=ngroupNum)
	{
		if(vecMultiObs[i].patternID==MOVINGID||vecMultiObs[i].patternID==STOPID)
		{
			singleGObs.dist=0.3;
			singleGObs.vecWanderObs.push_back(vecMultiObs[i]);
		}
		else if (vecMultiObs[i].patternID==GROUPID2)
		{
			singleGObs.dist=GROUPDIST;


		}

		vecGObsDB.push_back(singleGObs);
	}


	setvecMultiObs(vecMultiObs);

	KuDrawingInfo::getInstance()->setMultiObstaclePosVector(getvecMultiRobot());

	return true;
}
*/

bool KuWanderingObstaclePr::executeBehavior()
{
	
	initObstacles();
	if(KuDrawingInfo::getInstance()->getWanderingflag()==false)
	{
		KuDrawingInfo::getInstance()->setMultiObstaclePosVector(getvecMultiWanderingObstacles());
		KuDrawingInfo::getInstance()->setMultiObstacleGoalPosVector(getvecMultiGoalPose());
		return false;
	}
	KuPose RobotPos;
	vector<WanderingObs> vecMultiObs=getvecMultiObs();
	vector<int> vecnX; vector<int> vecnY;
	checkVirtualObstacle(m_pMap,vecMultiObs,&vecnX,&vecnY);
	KuVrRangeInterface::getInstance()->connect(m_pMap->getMap(),m_pMap->getX(),m_pMap->getY());

	int ngroupNum=1;
	WanderingObs ObsPos;
	for(int i=0; i<vecMultiObs.size();i+=ngroupNum)
	{
		ngroupNum=1;
		if(vecMultiObs[i].patternID==MOVINGID)
		{
			KuPose singleObs;
			singleObs.setXm(vecMultiObs[i].xm);
			singleObs.setYm(vecMultiObs[i].ym);
			singleObs.setThetaRad(vecMultiObs[i].tr);
			int_1DArray nData= KuVrRangeInterface::getInstance()->getData(singleObs);
			//KuDrawingInfo::getInstance()->setRangeData(nData);
			int nmovingid=vecMultiObs[i].movingID;
			
			double dGXm=0.0;
			double dGYm=0.0;

			list <KuPose > listLocalPath;
			for(int j=0;j<vecMultiObs[i].pathlist.size();j++)
			{	
				double dDist= m_math.calcDistBetweenPosesM(singleObs,vecMultiObs[i].pathlist[j]);
				if(dDist<2.0)
				{
					dGXm=vecMultiObs[i].pathlist[j].getXm();
					dGYm=vecMultiObs[i].pathlist[j].getYm();
				}
				listLocalPath.push_back(vecMultiObs[i].pathlist[j]);			
			}			

			//KuDrawingInfo::getInstance()->setLocalPath(listLocalPath);
			listLocalPath.clear();

			if(dGXm==0||dGYm==0) continue;

			KuPose  TargetPos;
			TargetPos.setXm(dGXm);			TargetPos.setYm(dGYm);
			
			KuVFHPVelocity generatedVel;
		
			generatedVel= vecKVFHPPr[nmovingid].generateTRVelocity(TargetPos, singleObs, nData );	

			double dDistance=generatedVel.dTranslationVel/10.0;
			double dRandThetaRad =vecMultiObs[i].tr+generatedVel.dRotationDegVel/10.0;
		
			if (dRandThetaRad>M_PI) dRandThetaRad -= 2.0*M_PI;
			else if (dRandThetaRad<-M_PI) dRandThetaRad+= 2.0*M_PI;

			double	dRandX =( vecMultiObs[i].xm+dDistance*cos(dRandThetaRad));
			double	dRandY =( vecMultiObs[i].ym+dDistance*sin(dRandThetaRad));

			vecMultiObs[i].xm=dRandX;
			vecMultiObs[i].ym=dRandY;
			vecMultiObs[i].tr=dRandThetaRad;
				
		}
		else if(vecMultiObs[i].patternID==GROUPID2||vecMultiObs[i].patternID==GROUPID3)
		{

			int ngrid=vecMultiObs[i].groupID;
			ngroupNum=vecMultiObs[i].groupNum;
			double dX=0.0;			double dY=0.0;			double dth=0.0;
			KuPose singleObs;
			vector<KuPose> vecObsPos;
			for(int j=0;j<ngroupNum;j++)
			{
// 				dX+=vecMultiObs[i+j].xm;
// 				dY+=vecMultiObs[i+j].ym;
// 				dth+=vecMultiObs[i+j].tr;
				singleObs.setXm(vecMultiObs[i+j].xm);
				singleObs.setYm(vecMultiObs[i+j].ym);
				singleObs.setThetaRad(vecMultiObs[i+j].tr);
				vecObsPos.push_back(singleObs);
				//tempvecMultiObs.push_back(vecMultiObs[i+j]);
			}	
		
			singleObs.setXm(vecMultiObs[i].cri_xm);
			singleObs.setYm(vecMultiObs[i].cri_ym);
		
			double  dRandThetaRad=vecMultiObs[i].cri_tr;//dth/(double)ngroupNum;
			if (dRandThetaRad>M_PI) dRandThetaRad -= 2.0*M_PI;
			else if (dRandThetaRad<-M_PI) dRandThetaRad+= 2.0*M_PI;
			
			singleObs.setThetaRad(dRandThetaRad);

			int_1DArray nData= KuVrRangeInterface::getInstance()->getData(singleObs,vecObsPos);			
			//KuDrawingInfo::getInstance()->setRangeData(nData);

			int nmovingid=vecMultiObs[i].movingID;

			double dGXm=0.0;
			double dGYm=0.0;
			list <KuPose > listLocalPath;
			for(int j=0;j<vecMultiObs[i].pathlist.size();j++)
			{	
				double dDist= m_math.calcDistBetweenPosesM(singleObs,vecMultiObs[i].pathlist[j]);
				if(dDist<2.0)
				{
					dGXm=vecMultiObs[i].pathlist[j].getXm();
					dGYm=vecMultiObs[i].pathlist[j].getYm();
				}
				listLocalPath.push_back(vecMultiObs[i].pathlist[j]);			
			}			
			//KuDrawingInfo::getInstance()->setLocalPath(listLocalPath);
			listLocalPath.clear();
			if(dGXm==0||dGYm==0) continue;
		
			KuPose  TargetPos;
			TargetPos.setXm(dGXm);			TargetPos.setYm(dGYm);
						
// 			KuDrawingInfo::getInstance()->setRobotPos(singleObs);
// 			KuDrawingInfo::getInstance()->setTargetPos(TargetPos);
			
			KuVFHPVelocity generatedVel;

			generatedVel= vecKVFHPPr[nmovingid].generateTRVelocity(TargetPos, singleObs, nData );	

			double dDistance=generatedVel.dTranslationVel/10.0;
			//dRandThetaRad =vecMultiObs[i].tr+generatedVel.dRotationDegVel/10.0;
			dRandThetaRad =singleObs.getThetaRad()+generatedVel.dRotationDegVel/10.0;

			if (dRandThetaRad>M_PI) dRandThetaRad -= 2.0*M_PI;
			else if (dRandThetaRad<-M_PI) dRandThetaRad+= 2.0*M_PI;
			
			WanderingObs CriObs;

			vecMultiObs[i].cri_xm=CriObs.xm=(singleObs.getXm()+dDistance*cos(dRandThetaRad));
			vecMultiObs[i].cri_ym=CriObs.ym=(singleObs.getYm()+dDistance*sin(dRandThetaRad));
			vecMultiObs[i].cri_tr=CriObs.tr=(dRandThetaRad);
			CriObs.vel=(generatedVel.dTranslationVel/10.0);
			CriObs.r_vel=(generatedVel.dRotationDegVel/10.0);

			vector<WanderingObs> vectmpMultiObs;

			for(int j=0;j<ngroupNum;j++)
			{	
				vecMultiObs[i+j].xm=vecMultiObs[i+j].xm+dDistance*cos(dRandThetaRad);
				vecMultiObs[i+j].ym=vecMultiObs[i+j].ym+dDistance*sin(dRandThetaRad);
				vecMultiObs[i+j].tr=(dRandThetaRad);
				
				vectmpMultiObs.push_back(vecMultiObs[i+j]);
				vecMultiObs[i+j].xm=0;
				vecMultiObs[i+j].ym=0;
				vecMultiObs[i+j].tr=0;
			}

	
			double dincreAvgX=0.0;
			double dincreAvgY=0.0;
			for(int j=0;j<ngroupNum;j++)
			{		
				double dOutDistance=dDistance;
				double dOutRandThetaRad=dRandThetaRad;
				double dincreX=0.0;
				double dincreY=0.0;
				WanderingObs outWanderObs=getGroupWanderingObsinBoundary(CriObs,vectmpMultiObs[j],vecMultiObs,vecMultiObs[i+j].groupID,&dincreX,&dincreY);
				dincreAvgX+=dincreX;
				dincreAvgY+=dincreY;

				vecMultiObs[i+j].xm=outWanderObs.xm;
				vecMultiObs[i+j].ym=outWanderObs.ym;
				vecMultiObs[i+j].tr=outWanderObs.tr;
			}		
			dincreAvgX=dincreAvgX/(double)ngroupNum;
			dincreAvgY=dincreAvgY/(double)ngroupNum;
			vecMultiObs[i].cri_xm+=dincreAvgX;
			vecMultiObs[i].cri_ym+=dincreAvgY;
								
		}

	}
	
	setvecMultiObs(vecMultiObs);

	resetMap(m_pMap,vecnX,vecnY);


	KuDrawingInfo::getInstance()->setMultiObstaclePosVector(getvecMultiWanderingObstacles());
	KuDrawingInfo::getInstance()->setMultiObstacleGoalPosVector(getvecMultiGoalPose());
	return true;

}
void KuWanderingObstaclePr::getRandVelocity(double dDistance,double dRandThetaRad,double*dOutDistance,double*dOutRandThetaRad,int nidx)
{
	double dWeight=0.7;
	(*dOutDistance)=dDistance*dWeight+dDistance*(1-dWeight)*(0.5-(float)rand()/(float)RAND_MAX);
	(*dOutRandThetaRad)=dRandThetaRad*dWeight+dRandThetaRad*(1-dWeight)*M_PI/180.0*(0.5-(float)rand()/(float)RAND_MAX);
}

bool KuWanderingObstaclePr::checkBetweenWall(WanderingObs WanderOb)
{
	bool bdetectedObjects=false;
	double dDist=WanderOb.dist;

	for(double i=-dDist; i<dDist+0.1;i+=0.1)
	{
		for(double j=-dDist; j<dDist+0.1;j+=0.1)
		{
			if(m_pMap->getX()-1<m_math.M2AI(WanderOb.xm+i)||m_pMap->getY()-1<m_math.M2AI(WanderOb.ym+j)
				||(int)m_math.M2AI(WanderOb.xm+i)<1||(int)(m_math.M2AI(WanderOb.ym+j))<1){return true;}
			else if(m_pMap->getMap()[m_math.M2AI(WanderOb.xm+i)][m_math.M2AI(WanderOb.ym+j)]!=KuMap::EMPTY_AREA){return true;}	
		}
	}
	
	return false;
}
bool KuWanderingObstaclePr::checkBetweenAllObstacles(WanderingObs WanderOb, vector<WanderingObs> vecObsPos)
{
	for (int i=0;i<vecObsPos.size();i++)
	{
		if(hypot(WanderOb.xm-vecObsPos[i].xm,WanderOb.ym-vecObsPos[i].ym)<0.7)
		{
			return true;
		}
	}
	return false;
}
bool KuWanderingObstaclePr::checkBetweenAllObstacles(WanderingObs WanderOb, vector<WanderingObs> vecObsPos,int ngroupID)
{
	for (int i=0;i<vecObsPos.size();i++)
	{
		if(hypot(WanderOb.xm-vecObsPos[i].xm,WanderOb.ym-vecObsPos[i].ym)<0.7&&vecObsPos[i].groupID!=ngroupID)
		{
			return true;
		}
		else if(hypot(WanderOb.xm-vecObsPos[i].xm,WanderOb.ym-vecObsPos[i].ym)<0.1&&vecObsPos[i].groupID==ngroupID)
		{
			return true;
		}
		else if(hypot(WanderOb.xm-vecObsPos[i].xm,WanderOb.ym-vecObsPos[i].ym)>0.7
			&&vecObsPos[i].xm!=0.0&&vecObsPos[i].ym!=0
			&&vecObsPos[i].groupID==ngroupID)
		{
			return true;
		}
	}
	return false;
}
bool KuWanderingObstaclePr::checkBetweenObstacles(WanderingObs WanderOb, vector<WanderingObs> vecObsPos)
{
	for (int i=0;i<vecObsPos.size();i++)
	{
		if(hypot(WanderOb.xm-vecObsPos[i].xm,WanderOb.ym-vecObsPos[i].ym)<WanderOb.dist+vecObsPos[i].dist
			&&(WanderOb.ID-KuMap::OCCUPIED_OBSTACLES_GRID)!=i
			&&(WanderOb.groupID!=vecObsPos[i].groupID||WanderOb.groupID==-1))
		{
			return true;
		}
	}
	return false;
}

vector<WanderingObs> KuWanderingObstaclePr::selGroupObs( WanderingObs ObsPos, int ngroupnum)
{
	vector<WanderingObs> vecMultiObs;

	int ngroupid=ObsPos.groupID;
	int ngroupNum=ObsPos.groupNum;

	double dCriXm=ObsPos.xm;
	double dCriYm=ObsPos.ym;
	double dCriTr=ObsPos.tr;
	
	vecMultiObs.push_back(ObsPos);


	for (int j=1;j<ngroupNum;j++)
	{
		double dXm=0.0;
		double dYm=0.0;
		double dThr=0.0;
		double dRand=0.0;

		while(1)
		{
			bool bdetectedObjects=false;
			dRand=(0.5 -(float)rand()/(float)RAND_MAX);	if(dRand<0.3&&dRand>=0.0) dRand=0.3;	else if(dRand>-0.3&&dRand<0.0) dRand=-0.3;
			dXm=dCriXm+GROUPDIST*dRand;
			dRand=(0.5 -(float)rand()/(float)RAND_MAX);	if(dRand<0.3&&dRand>=0.0) dRand=0.3;	else if(dRand>-0.3&&dRand<0.0) dRand=-0.3;
			dYm=dCriYm+GROUPDIST*dRand;
			dRand=(0.5 -(float)rand()/(float)RAND_MAX);	if(dRand<0.01&&dRand>=0.0) dRand=0.01;	else if(dRand>-0.01&&dRand<0.0) dRand=-0.01;
			dThr=dCriTr+(GROUPDEGREE*D2R)*dRand;

			for(int m=-1; m<2;m+=1)
			{
				for(int n=-1; n<2;n+=1)
				{
					if(m_pMap->getX()-1<m_math.M2AI(dXm+m)||m_pMap->getY()-1<m_math.M2AI(dYm+n)||(int)m_math.M2AI(dXm+m)<1||(int)m_math.M2AI(dYm+n)<1){bdetectedObjects=true;}
					else if(m_pMap->getMap()[m_math.M2AI(dXm+m)][m_math.M2AI(dYm+n)]!=KuMap::EMPTY_AREA){bdetectedObjects=true;}	
				}
			}

			for (int f=1;f<j;f++)
			{
				if(hypot(vecMultiObs[f].xm-dXm,vecMultiObs[f].ym-dYm)<0.3) bdetectedObjects=true;
			}

			if(bdetectedObjects==false){
				ObsPos.xm=dXm;
				ObsPos.ym=dYm;
				ObsPos.tr=dThr;				
				ObsPos.ID=ObsPos.ID+1;
				break;
			}
		}		
		vecMultiObs.push_back(ObsPos);
	}

	return vecMultiObs;
}
vector<WanderingObs> KuWanderingObstaclePr::movinggroupmotion( vector<WanderingObs> vecGroupObsPos,vector<WanderingObs> vecMultiObs)
{
	WanderingObs  ObsPos;

	vector<WanderingObs>  vecoutObsPos;
	WanderingObs  outObsPos;

	for(int i=0;i<vecGroupObsPos.size();i++)
	{
		vecoutObsPos.push_back(vecGroupObsPos[i]);
	}
	int ngroupNum=vecGroupObsPos.size();
	WanderingObs meanObsPos;
	meanObsPos.xm=0.0;			meanObsPos.ym=0.0;			meanObsPos.tr=0.0;

	for(int i=0;i<vecGroupObsPos.size();i++)
	{
		meanObsPos.xm+=vecGroupObsPos[i].xm;
		meanObsPos.ym+=vecGroupObsPos[i].ym;
		meanObsPos.tr+=vecGroupObsPos[i].tr;
		meanObsPos.vel=0.08;
		meanObsPos.dist=0.5;
		meanObsPos.groupID=vecGroupObsPos[i].groupID;;
		meanObsPos.groupNum=2;
		meanObsPos.ID=vecGroupObsPos[i].ID;
		meanObsPos.patternID=GROUPID2;
	}

	meanObsPos.xm=meanObsPos.xm/(double)ngroupNum;
	meanObsPos.ym=meanObsPos.ym/(double)ngroupNum;
	meanObsPos.tr=meanObsPos.tr/(double)ngroupNum;

	if(movingmotion( meanObsPos,vecMultiObs,&ObsPos))
	{
		//진행
		double dDisttomean=hypot(meanObsPos.xm-ObsPos.xm, meanObsPos.ym-ObsPos.ym);
		double dDegreetomean=fabs(meanObsPos.tr-ObsPos.tr);
		if(dDegreetomean>M_PI)dDegreetomean=dDegreetomean-2*M_PI;
		else if(dDegreetomean<-M_PI)dDegreetomean=dDegreetomean+2*M_PI;


		while(1)
		{
			for(int i=0;i<vecoutObsPos.size();i++)
			{
				vecoutObsPos[i].vel=dDisttomean+dDisttomean/10.0*(0.5-(double)rand()/(double)RAND_MAX);			
				vecoutObsPos[i].tr=ObsPos.tr+(dDegreetomean)*(0.5-(double)rand()/(double)RAND_MAX);
				vecoutObsPos[i].xm=vecGroupObsPos[i].xm+vecoutObsPos[i].vel*cos(vecoutObsPos[i].tr);
				vecoutObsPos[i].ym=vecGroupObsPos[i].ym+vecoutObsPos[i].vel*sin(vecoutObsPos[i].tr);
			}

			meanObsPos.xm=0.0;			meanObsPos.ym=0.0;			meanObsPos.tr=0.0;

			for(int i=0;i<vecoutObsPos.size();i++)
			{
				meanObsPos.xm+=vecoutObsPos[i].xm;
				meanObsPos.ym+=vecoutObsPos[i].ym;
				meanObsPos.tr+=vecoutObsPos[i].tr;
				meanObsPos.vel=0.08;
				meanObsPos.dist=0.5;
				meanObsPos.groupID=vecoutObsPos[i].groupID;;
				meanObsPos.groupNum=2;
				meanObsPos.ID=vecoutObsPos[i].ID;
				meanObsPos.patternID=GROUPID2;
			}


			meanObsPos.xm=meanObsPos.xm/(double)ngroupNum;
			meanObsPos.ym=meanObsPos.ym/(double)ngroupNum;
			meanObsPos.tr=meanObsPos.tr/(double)ngroupNum;

			if(checkBetweenWall(meanObsPos))
			{							
				continue;;
			}
			else if(checkBetweenObstacles(meanObsPos,vecMultiObs))
			{
				continue;;
			}

			bool bgrouping=true;
			for(int i=0;i<vecoutObsPos.size();i++)
			{
				for(int j=i+1;j<vecoutObsPos.size();j++)
				{
					double dDistMeanvalue2Obs=hypot(vecoutObsPos[i].xm-vecoutObsPos[j].xm,vecoutObsPos[i].ym-vecoutObsPos[j].ym);
					if(dDistMeanvalue2Obs>GROUPDIST||fabs(vecoutObsPos[i].tr-vecoutObsPos[j].tr)>GROUPDEGREE*D2R){bgrouping=false;}
				}
			}

			if(bgrouping==true) break;
			
		}
	}
		else
		{
			//회전
			for(int i=0;i<vecoutObsPos.size();i++)
			{
				vecoutObsPos[i]=vecGroupObsPos[i];
				vecoutObsPos[i].tr=vecoutObsPos[i].tr+10*D2R;
			}
		}


	return  vecoutObsPos;
}

bool KuWanderingObstaclePr::movingmotion( WanderingObs ObsPos,vector<WanderingObs> vecObsPos, WanderingObs* outObsPos)
{
	int nDistance=500;
	double dDesiredVel=20;
	KuPose RobotPos;
	KuPose preRobotPos;
	//WanderingObs outObsPos;

	double dDistance=ObsPos.vel*(double)rand()/(double)RAND_MAX;
	double dRandThetaRad =ObsPos.tr+(0.1*D2R)*(double)rand()/(double)RAND_MAX;

	if(dRandThetaRad > M_PI){dRandThetaRad -= 2*M_PI;}
	else if(dRandThetaRad < -M_PI){	dRandThetaRad += 2*M_PI;}			

	double	dRandX =( ObsPos.xm+dDistance*cos(dRandThetaRad));
	double	dRandY =( ObsPos.ym+dDistance*sin(dRandThetaRad));
	
	(*outObsPos)=ObsPos;
	(*outObsPos).xm=dRandX;
	(*outObsPos).ym=dRandY;
	(*outObsPos).tr=dRandThetaRad;

	//벽 감지-----------------------------------------------------------------------------------------------------

	if(checkBetweenWall((*outObsPos)))
	{
		dRandThetaRad=ObsPos.tr+(10*D2R)*(double)rand()/(double)RAND_MAX;		
		if(dRandThetaRad > M_PI){dRandThetaRad -= 2*M_PI;}
		else if(dRandThetaRad < -M_PI){	dRandThetaRad += 2*M_PI;}	
		ObsPos.tr=dRandThetaRad;
		(*outObsPos)=ObsPos;
		return  false;
	}
	else if(checkBetweenObstacles((*outObsPos),vecObsPos))
	{
		dRandThetaRad=ObsPos.tr+(10*D2R)*(double)rand()/(double)RAND_MAX;		
		if(dRandThetaRad > M_PI){dRandThetaRad -= 2*M_PI;}
		else if(dRandThetaRad < -M_PI){	dRandThetaRad += 2*M_PI;}	
		ObsPos.tr=dRandThetaRad;
		(*outObsPos)=ObsPos;
		return  false;

	}

	//벽 감지-----------------------------------------------------------------------------------------------------
	return  true;

}

KuPose KuWanderingObstaclePr::movingmotion( WanderingObs ObsPos)
{
	int nDistance=500;
	double dDesiredVel=20;
	KuPose RobotPos;
	KuPose preRobotPos;

	double dDistance=ObsPos.vel*(double)rand()/(double)RAND_MAX;
	double dRandThetaRad =ObsPos.tr+(0.1*D2R)*(double)rand()/(double)RAND_MAX;

	if(dRandThetaRad > M_PI){dRandThetaRad -= 2*M_PI;}
	else if(dRandThetaRad < -M_PI){	dRandThetaRad += 2*M_PI;}			

	double	dRandX =( ObsPos.xm+dDistance*cos(dRandThetaRad));
	double	dRandY =( ObsPos.ym+dDistance*sin(dRandThetaRad));

	preRobotPos.setXm(ObsPos.xm);		preRobotPos.setYm(ObsPos.ym);			preRobotPos.setThetaRad(ObsPos.tr);
	RobotPos.setXm(dRandX);		RobotPos.setYm(dRandY);			RobotPos.setThetaRad(dRandThetaRad);

	//RobotPos=TempDrive(dRandX,dRandY,dDesiredVel,preRobotPos);		

	//벽 감지-----------------------------------------------------------------------------------------------------
	if(m_pMap->getX()<(int)(m_math.M2AI(RobotPos.getXm()))-1||m_pMap->getY()<(int)m_math.M2AI(RobotPos.getYm())-1
		||(int)m_math.M2AI(RobotPos.getXm())<1||(int)m_math.M2AI(RobotPos.getYm())<1){
			preRobotPos.setThetaDeg(preRobotPos.getThetaDeg()+180);			RobotPos=preRobotPos;
	}
	else if(m_pMap->getMap()[(int)m_math.M2AI(RobotPos.getXm())][(int)m_math.M2AI(RobotPos.getYm())]!=KuMap::EMPTY_AREA){
		preRobotPos.setThetaDeg(preRobotPos.getThetaDeg()+180);
		RobotPos=preRobotPos;
	}
	

	//벽 감지-----------------------------------------------------------------------------------------------------
	return  RobotPos;
}


bool KuWanderingObstaclePr::initialize(KuCommandMessage CMessage)
{

	m_doThreadFunc = true; //스레드 함수가 반복동작 할 수 있도록 플래그 설정.
	m_bIsThreadFuncGenerated = true; //스레드 함수가 생성되었다는 플래그 설정.
	m_nThreadFuncPeriod = CMessage.getBehaviorPeriod(); //스레드 함수 실행주기 입력.
	m_RobotPos = CMessage.getRobotPos();
	m_math.setCellSizeMM(Sensor::CELLSIZE);

	m_pRefMap = kuMapRepository::getInstance()->getMap(); 
	m_nMapSizeX=m_pRefMap->getX();
	m_nMapSizeY=m_pRefMap->getY();

	if(NULL==m_pMap)
	{		
		m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
	}
	
	int** nMap = m_pMap->getMap();
	int** nOriMap = m_pRefMap->getMap();

	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			nMap[i][j]= nOriMap[i][j];
		}
	}

	if(m_psensor_value==NULL)
	{
		m_psensor_value=  new double[361];
	}

	m_KuPathPlanner.setRadiusofRobot(Sensor::ROBOT_RADIUS);
	m_KuPathPlanner.initializeMapSize(kuMapRepository::getInstance()->getMap()->getX(),
		kuMapRepository::getInstance()->getMap()->getY());
	m_KuPathPlanner.setMap(kuMapRepository::getInstance()->getMap()->getMap(),Sensor::CELLSIZE); 

	return true;
}


void KuWanderingObstaclePr::checkVirtualObstacle(KuMap* pMap,vector<WanderingObs> vecMultiObs,vector<int>* vecnX,vector<int>* vecnY)
{
	int** nMap = pMap->getMap();

	for(int i=0;i<vecMultiObs.size();i++)
	{
		int nObsX = m_math.M2AI(vecMultiObs[i].xm);
		int nObsY = m_math.M2AI(vecMultiObs[i].ym);
		if(nObsX==0 && nObsY==0) return;

		int nSize=2;
		for(int i=nObsX-nSize; i<nObsX+nSize+1; i++ ){
			for(int j=nObsY-nSize; j<nObsY+nSize+1; j++){
				if(nMap[i][j] ==KuMap::EMPTY_AREA )	nMap[i][j] = KuMap::WARNING_AREA;	
				vecnX->push_back(i);
				vecnY->push_back(j);
			}
		}
	}
	KuPose robotpos=KuDrawingInfo::getInstance()->getRobotPos();

	int nObsX = m_math.M2AI(robotpos.getXm());
	int nObsY = m_math.M2AI(robotpos.getYm());
	if(nObsX==0 && nObsY==0) return;

	int nSize=2;
	for(int i=nObsX-nSize; i<nObsX+nSize+1; i++ ){
		for(int j=nObsY-nSize; j<nObsY+nSize+1; j++){
			if(nMap[i][j] ==KuMap::EMPTY_AREA )	nMap[i][j] = KuMap::WARNING_AREA;	
			vecnX->push_back(i);
			vecnY->push_back(j);
		}
	}

	//KuVrHokuyoUTM30LXInterface::getInstance()->connect( m_pMap->getMap(),nMapX,nMapY );

}
void KuWanderingObstaclePr::resetMap(KuMap* pMap,vector<int> vecnX,vector<int> vecnY)
{
	int** nMap = pMap->getMap();

	for(int i=0;i<vecnX.size();i++)
	{
		nMap[vecnX[i]][vecnY[i]] = KuMap::EMPTY_AREA;		
	}
}
KuPose KuWanderingObstaclePr::TempDrive(double dTargetX, double dTargetY, double dDesiredVel,KuPose RobotPos)
{
	KuPose TempPos;
	double dMaxV = 500;     // 500mm/s가 로봇이 낼 수 있는 최대 속도
	double dMaxW = 60*D2R; //50degree/s가 로봇이 낼 수 있는 최대 각속도

	double dkx = 0.4;   //로봇의 전진방향 오차를 극복하는 게인. 실제 로봇의 경우 너무 크면 출렁이고, 너무 작으면 원하는 속도보다 느리게 움직임.
	double dky = 1.0;   //로봇의 측면방향 오차를 극복하는 게인. 영향이 크지는 않지만, 크면 불안정하고, 작으면 측면방향 오차를 보정 못함.
	double dkth = 1.2;  //목적지를 향한 로봇의 방향(heading)을 극복하는 게인. 크면 매우 출렁이고(특히 초기에 제자리에서 목적지를 향해 휙 돔), 작으면 딴 방향을 향함.


	// 목표위치와 로봇위치 사이의 에라를 계산.
	// 목표위치와 로봇위치는 절대좌표계 기준. 로봇의 모션은(v,w) 로봇좌표계 기준.
	// 따라서 절대좌표계 상에서의 에라를 상대좌표계로 변환해야 함.

	double derrorX = (dTargetX/1000. - RobotPos.getXm()) * cos(-RobotPos.getThetaRad()) + (dTargetY/1000.-RobotPos.getYm()) * sin(RobotPos.getThetaRad());
	double derrorY = (dTargetX/1000. - RobotPos.getXm()) * sin(-RobotPos.getThetaRad()) + (dTargetY/1000.-RobotPos.getYm()) * cos(-RobotPos.getThetaRad());
	double derrorThRad = atan2(dTargetY/1000.-RobotPos.getYm(), dTargetX/1000. - RobotPos.getXm()) - RobotPos.getThetaRad();
	if(derrorThRad > M_PI ) derrorThRad = derrorThRad - 2*M_PI;
	if(derrorThRad <-M_PI ) derrorThRad = derrorThRad + 2*M_PI;
	//Kanayama의 식을 이용해서 에라를 줄이기 위한, 즉 목표점을 향해 가기 위한 로봇의 속도 계산.
	double dTVel = dDesiredVel/1000.*cos(derrorThRad) + dkx*derrorX;
	double dRotVel = 0 + dDesiredVel/1000.*(dky*derrorY + dkth*sin(derrorThRad));


	if (dTVel>dMaxV) dTVel=dMaxV; 
	if (dRotVel>dMaxW) dRotVel=dMaxW;
	if (dRotVel<-dMaxW) dRotVel=-dMaxW; 
	if(dTVel<0)dTVel=0;
	if(fabs(derrorX*1000)<100){
		//		stop();
		TempPos=RobotPos;//move(dTVel*1000, dRotVel);
	}
	else{
		TempPos=move(dTVel*1000, dRotVel);
	}
	return TempPos;
}

KuPose KuWanderingObstaclePr::move(double V, double W)
{
	double dLeftEncData=0;
	double dRightEncData=0;
	KuPose TempPos;

	dLeftEncData = V - (BETWEEN_WHEEL * W)/2.;
	dRightEncData =  V + (BETWEEN_WHEEL * W)/2.;

	TempPos=calcEncoderData( dLeftEncData, dRightEncData);

	return TempPos;
}

KuPose KuWanderingObstaclePr::calcEncoderData(double dLeftEncData,double dRightEncData)
{
	KuPose TempPos;

	double dAverageWheelDistance = (dLeftEncData + dRightEncData)/2;

	double dDeltaT = (dRightEncData - dLeftEncData) / BETWEEN_WHEEL;

	double dDistance2RobotCenter;

	double dDeltaX=0;
	double dDeltaY=0;

	if(fabs(dDeltaT) >= 0.0017f ) {
		dDistance2RobotCenter =dAverageWheelDistance / dDeltaT;
		dDeltaY = dDistance2RobotCenter - ( dDistance2RobotCenter * cos(dDeltaT) );
		dDeltaX = dDistance2RobotCenter * sin(dDeltaT);
	}
	else {
		dDistance2RobotCenter = 0.0f;
		dDeltaY = 0;
		dDeltaX = dAverageWheelDistance;
	}



	//상대좌표를 절대좌표로 변환하는것.
	m_dReferenceX +=  dDeltaX * cos( m_dReferenceT ) + dDeltaY * sin( -m_dReferenceT +dDeltaT/2.0);
	m_dReferenceY +=  dDeltaX * sin( m_dReferenceT) + dDeltaY * cos( m_dReferenceT  +dDeltaT/2.0);
	m_dReferenceT =  m_dReferenceT+dDeltaT;


	if(m_dReferenceT > M_PI){
		m_dReferenceT -= 2*M_PI;
	}
	else if(m_dReferenceT < -M_PI){
		m_dReferenceT += 2*M_PI;
	}

	TempPos.setX(m_dReferenceX);
	TempPos.setY(m_dReferenceY);
	TempPos.setThetaRad(m_dReferenceT);


	return TempPos;
}