#include "stdafx.h"
#include "KuDrawingInfo.h"


KuDrawingInfo::KuDrawingInfo()
{
	initVariable(); //변수 초기화 함수.
	cout<<"[KuDrawingInfo]: Singletone type instance is created!!!"<<endl;
	m_nMap=NULL;
	m_dProMap=NULL;
	m_nCADMap=NULL;
	m_dProObMap=NULL;
	m_dPathMap=NULL;
	m_nLocalMap=NULL;
	m_dpredictedProMap=NULL;
	m_bRenderCADMapflag=false;
	m_bDirectionofPathflag=false;
	m_bRenderKinect3DCloudFlag=false;
	m_nPathMapSizeX=0;
	m_nPathMapSizeY=0;
	m_nCurFloor=-1;
	m_nPatternID=-1;
	m_bWanderingstart=false;
}

KuDrawingInfo::~KuDrawingInfo()
{

	if(m_nMap!=NULL)
	{
		for(int i = 0 ; i < m_nMapX ; i++){
			delete[] m_nMap[i];
			delete[] m_dProMap[i];
			delete[] m_dProObMap[i];

			m_nMap[i] = NULL;
			m_dProMap[i] = NULL;
			m_dProObMap[i] = NULL;

		}
		delete[] m_nMap;
		delete[] m_dProMap;
		delete[] m_dProObMap;


	}
	if(m_nCADMap!=NULL)
	{
		for(int i = 0 ; i < m_nMapX ; i++){
			delete[] m_nCADMap[i];
			m_nCADMap[i] = NULL;
		}
		delete[] m_nCADMap;

	}

	cout<<"[KuDrawingInfo]: Singletone type instance is destroyed!!!"<<endl;
}

/**
@brief Korean: 변수들을 초기화 한다. 
@brief English: write in English
*/
void KuDrawingInfo::initVariable()
{
	m_nLaserData181 = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

	m_nRplidarData181= m_KuUtil.generateIntType1DArray(Sensor::RPLIDAR_DATA_NUM181,0);
	
	m_nKinectRangeData = m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,0);

	m_IplKinectImg = cvCreateImage(cvSize( Sensor::IMAGE_WIDTH, Sensor::IMAGE_HEIGHT),8,3);

	m_IplCeilingImage = cvCreateImage(cvSize( Sensor::CEILING_IMAGE_WIDTH, Sensor::CEILING_IMAGE_HEIGHT),8,1);

	memset(m_Global3DPose,0,sizeof(m_Global3DPose));

	m_nRangeData= m_KuUtil.generateIntType1DArray(361,0);

	m_nTVByKeyEvt=m_nRVByKeyEvt=0; //키보드 이벤트를 통해 받은 로봇 속도를 저장하는 변수.
	
	m_bRenderMapflag=false;//지도 그리기로 결정

	m_bRenderBuildingMapflag=false;
	
	m_bRenderCeilingImagflag=false;


	m_PIOMapData.Floor=-1;
	m_PIOMapData.POIIDNum=-1;
	m_PIOMapData.POIID.clear();
	m_PIOMapData.POIName.clear();
	m_PIOMapData.ROBTh.clear();
	m_PIOMapData.ROBX.clear();
	m_PIOMapData.ROBY.clear();
}

/**
@brief Korean: 지도정보를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setMap(KuMap *pMap)
{
	m_CriticalSection.Lock();

	if(m_nMap==NULL){
		//처음 m_nMap을 생성할때
		m_nMapX = pMap->getX();
		m_nMapY = pMap->getY();

		m_nMap = new int*[m_nMapX];
		m_dProMap = new double*[m_nMapX];

		if(m_nMap){
			for(int i = 0 ; i < m_nMapX ; i++){
				m_nMap[i] = new int[m_nMapY];
				m_dProMap[i] = new double[m_nMapY];

			}
		}
	}
	else if(m_nMap!=NULL&&(m_nMapX != pMap->getX()||m_nMapY != pMap->getY()))
	{
		for(int i = 0 ; i < m_nMapX ; i++){
			delete[] m_nMap[i];
			delete[] m_dProMap[i];

			m_nMap[i] = NULL;
			m_dProMap[i] = NULL;
		}
		delete[] m_nMap;
		delete[] m_dProMap;

		m_nMapX = pMap->getX();
		m_nMapY = pMap->getY();

		m_nMap = new int*[m_nMapX];
		m_dProMap = new double*[m_nMapX];

		if(m_nMap){
			for(int i = 0 ; i < m_nMapX ; i++){
				m_nMap[i] = new int[m_nMapY];
				m_dProMap[i] = new double[m_nMapY];

			}
		}
	}
	//지도 데이터 복사
	int** nMap = pMap->getMap();
	double** dProMap = pMap->getProMap();


	for(int i=0; i<m_nMapX; i++){
		for(int j=0; j<m_nMapY; j++){

			m_nMap[i][j] = nMap[i][j];

			m_dProMap[i][j] = dProMap[i][j]; 
		}
	}

	m_CriticalSection.Unlock();
}

void KuDrawingInfo::setMap(int** nMap,int nMapSizeX,int nMapSizeY)
{
	if(nMapSizeX!=m_nMapX||nMapSizeY!=m_nMapY) return;

	m_CriticalSection.Lock();

	for(int i=0; i<nMapSizeX; i++){
		for(int j=0; j<nMapSizeY; j++){

			m_nMap[i][j] = nMap[i][j];
		}
	}
	m_CriticalSection.Unlock();
}
/**
 @brief Korean: 저장된 지도정보를 얻어가는 함수. 
 @brief English: write in English
*/
int** KuDrawingInfo::getMap()
{
	return m_nMap;
}



/**
@brief Korean: 지도정보를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setCADMap(int** nCADMap,int nMapSizeX,int nMapSizeY)
{
	if(nMapSizeX==0||nMapSizeY==0) return;
	m_nCADMapSizeX=nMapSizeX;
	m_nCADMapSizeY=nMapSizeY;

	m_CriticalSection.Lock();

	if(m_nCADMap==NULL){
	
		m_nCADMap = new int*[nMapSizeX];

		if(m_nMap){
			for(int i = 0 ; i < nMapSizeX ; i++){
				m_nCADMap[i] = new int[nMapSizeY];
			}
		}
	}
	
	for(int i=0; i<nMapSizeX; i++){
		for(int j=0; j<nMapSizeY; j++){

			m_nCADMap[i][j] = nCADMap[i][j];
		}
	}

	m_CriticalSection.Unlock();
}
/**
 @brief Korean: 저장된 지도정보를 얻어가는 함수. 
 @brief English: write in English
*/
int** KuDrawingInfo::getCADMap(int* nCADMapSizeX,int* nCADMapSizeY)
{
	(*nCADMapSizeX)=m_nCADMapSizeX;
	(*nCADMapSizeY)=m_nCADMapSizeY;

	return m_nCADMap;
}



/**
 @brief Korean: 저장된 지도의 X 방향의 크기를 가져가는 함수 (단위: 10cm)
 @brief English: write in English
*/
int KuDrawingInfo::getMapSizeX()
{
	return m_nMapX;
}
/**
 @brief Korean: 저장된 지도의 Y 방향의 크기를 가져가는 함수 (단위: 10cm)
 @brief English: write in English
*/
int  KuDrawingInfo::getMapSizeY()
{
	return m_nMapY;
}


/**
 @brief Korean: 레이저 데이터를 저장하는 함수.
 @brief English: write in English
*/
void KuDrawingInfo::setRangeData(int_1DArray nRangeData)
{
	m_CriticalSection.Lock();
	//레이저 데이터 복사과정-------------------------------------------------------
	for(int i=0; i<361; i++){
			m_nRangeData[i] =  nRangeData[i]; 
	}
	//*******************************************************************
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 저장된 레이저 데이터를 넘겨주는 함수.
 @brief English: write in English
*/
int_1DArray KuDrawingInfo::getRangeData()
{
	return m_nRangeData;
}
/**
 @brief Korean: 레이저 데이터를 저장하는 함수.
 @brief English: write in English
*/
void KuDrawingInfo::setLaserData181(int_1DArray nLaserData181)
{
	m_CriticalSection.Lock();
	//레이저 데이터 복사과정-------------------------------------------------------
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181; i++){
			m_nLaserData181[i] =  nLaserData181[i]; 
	}
	//*******************************************************************
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 저장된 레이저 데이터를 넘겨주는 함수.
 @brief English: write in English
*/
int_1DArray KuDrawingInfo::getLaserData181()
{
	return m_nRplidarData181;
}
/**
 @brief Korean: 레이저 데이터를 저장하는 함수.
 @brief English: write in English
*/
void KuDrawingInfo::setRplidarData181(int_1DArray nLaserData181)
{
	m_CriticalSection.Lock();
	//레이저 데이터 복사과정-------------------------------------------------------
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181; i++){
			m_nRplidarData181[i] =  nLaserData181[i]; 
	}
	//*******************************************************************
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 저장된 레이저 데이터를 넘겨주는 함수.
 @brief English: write in English
*/
int_1DArray KuDrawingInfo::getRplidarData181()
{
	return m_nLaserData181;
}

/**
 @brief Korean: 키넥트 데이터를 저장하는 함수.
 @brief English: write in English
*/
void KuDrawingInfo::setKinectRangeData(int_1DArray nKinectRangeData)
{
	m_CriticalSection.Lock();
	//레이저 데이터 복사과정-------------------------------------------------------
	for(int i=0; i<Sensor::KINECT_SENSOR_FOV; i++){
			m_nKinectRangeData[i] =  nKinectRangeData[i]; 
	}
	//*******************************************************************
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 저장된 키게트 데이터를 넘겨주는 함수.
 @brief English: write in English
*/
int_1DArray KuDrawingInfo::getKinectRangeData()
{
	return m_nKinectRangeData;
}
/**
 @brief Korean: 저장된 로봇 위치를 넘겨주는 함수.
 @brief English: write in English
*/
KuPose  KuDrawingInfo::getRobotPos()
{
	return m_RobotPos;
}

/**
 @brief Korean: 로봇 위치를 저장하는 함수
 @brief English: write in English
*/
void  KuDrawingInfo::setRobotPos(KuPose RobotPos)
{
	m_RobotPos=RobotPos;
}

/**
 @brief Korean: 실험 목적의 로봇 위치를 넘겨주는 함수.
 @brief English: write in English
*/
KuPose KuDrawingInfo::getAuxiliaryRobotPos()
{
	
	KuPose AuxiliaryRobotPos;
	m_CriticalSection.Lock();
	AuxiliaryRobotPos = m_AuxiliaryRobotPos;
	m_CriticalSection.Unlock();

	return AuxiliaryRobotPos;
}

/**
 @brief Korean: 실험 목적의 로봇 위치를 저장하는 함수
 @brief English: write in English
*/
void KuDrawingInfo::setAuxiliaryRobotPos(KuPose AuxiliaryRobotPos)
{
	m_CriticalSection.Lock();
	m_AuxiliaryRobotPos = AuxiliaryRobotPos;
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 실험 목적의 이상적인 로봇 위치를 넘겨주는 함수.
 @brief English: write in English
*/
KuPose KuDrawingInfo::getIdealRobotPos()
{
	KuPose IdealRobotPos;
	m_CriticalSection.Lock();
	IdealRobotPos = m_IdealRobotPos;
	m_CriticalSection.Unlock();

	return IdealRobotPos;

}
/**
 @brief Korean: //실험 목적의 이상적인 로봇 위치를 저장하는 함수
 @brief English: write in English
*/
void KuDrawingInfo::setIdealRobotPos(KuPose IdealRobotPos)
{

	m_CriticalSection.Lock();
	m_IdealRobotPos = IdealRobotPos;
	m_CriticalSection.Unlock();

}

/**
 @brief Korean: 저장된 골 위치를 넘겨주는 함수.
 @brief English: write in English
*/
KuPose KuDrawingInfo::getGoalPos()
{
	KuPose GoalPos;
	m_CriticalSection.Lock();
	GoalPos = m_GoalPos;
	m_CriticalSection.Unlock();

	return GoalPos;
}

/**
 @brief Korean: 골 위치를 저장하는 함수
 @brief English: write in English
*/
void KuDrawingInfo::setGoalPos(KuPose GoalPos)
{
	m_CriticalSection.Lock();
	m_GoalPos=GoalPos;
	m_CriticalSection.Unlock();

} 

/**
 @brief Korean: 로봇 속도를 저장하는 함수
 @brief English: write in English
*/
void KuDrawingInfo::setRobotTRVel(int nTVel, int nRVel)
{
	m_CriticalSection.Lock();
	if(0==nTVel && 0 == nRVel){
		m_nTVByKeyEvt = 0;
		m_nRVByKeyEvt = 0;

	}else{
		m_nTVByKeyEvt += nTVel;
		m_nRVByKeyEvt += nRVel;
	}
	m_CriticalSection.Unlock();
	
}

/**
 @brief Korean: 로봇 속도를 넘겨주는 함수
 @brief English: write in English
*/
void KuDrawingInfo::getRobotTRVel(int* nTVel, int* nRVel)
{

	m_CriticalSection.Lock();
	*nTVel = m_nTVByKeyEvt;
	*nRVel = m_nRVByKeyEvt;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setPath(list<KuPose> listPath)
{

	m_CriticalSection.Lock();
	m_listPath = listPath;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 경로를 넘겨주는 함수.
@brief English: write in English
*/
list<KuPose> KuDrawingInfo::getPath()
{
	list<KuPose> listPath;
	m_CriticalSection.Lock();
	listPath = m_listPath;
	m_CriticalSection.Unlock();

	return listPath;
}

/**
@brief Korean: //target pos를 저장하는 함수
@brief English: write in English
*/
void KuDrawingInfo::setTargetPos(KuPose TargetPos)
{
	m_CriticalSection.Lock();
	m_TargetPos =TargetPos;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: //target pos를 넘겨주는 함수.
@brief English: write in English
*/
KuPose KuDrawingInfo::getTargetPos()
{
	KuPose TargetPos;
	m_CriticalSection.Lock();
	TargetPos = m_TargetPos;
	m_CriticalSection.Unlock();
	return TargetPos;
}
/**
@brief Korean: 작성되는 지도의 확률적인 값을 얻어가는 부분
@brief English: write in English
*/
double ** KuDrawingInfo::getBuildingMap( int* nX, int* nY)
{	

	m_CriticalSection.Lock();
	*nX = m_nMapX;
	*nY = m_nMapY;
	double ** dMap;

	if(NULL != m_dProMap){ //2차 배열이 할당된 적이 없는 상태이다.
		dMap = m_dProMap;
	}
	m_CriticalSection.Unlock();

	return dMap;

}
/**
@brief Korean: 작성되는 지도의 확률적인 값을 얻어가는 부분
@brief English: write in English
*/
void KuDrawingInfo::setBuildingMap( double**  dMap )
{
	m_CriticalSection.Lock();
	m_dProMap = dMap  ;
	m_CriticalSection.Unlock();
}
/**
@brief Korean: 다수의 골지점을 얻어가는 부분
@brief English: write in English
*/
list<KuPose> KuDrawingInfo::getGoalPosList()
{
	list<KuPose> GoalPosList;
	m_CriticalSection.Lock();
	GoalPosList = m_GoalPosList;
	m_CriticalSection.Unlock();
	return GoalPosList;
}
/**
@brief Korean: 다수의 골지점을 저장하는 부분
@brief English: write in English
*/
void KuDrawingInfo::setGoalPosList(list<KuPose> GoalPosList)
{
	m_CriticalSection.Lock();
	m_GoalPosList = GoalPosList;
	m_CriticalSection.Unlock();
}
/**
@brief Korean: 지도의 그리기 여부를 가져오는 함수
@brief English: write in English
*/
void KuDrawingInfo::setRenderMapflag(bool bRenderMapflag)
{
	m_bRenderMapflag=bRenderMapflag;
}
/**
@brief Korean:  지도의 그리기 여부를  정하는 함수
@brief English: write in English
*/
bool KuDrawingInfo::getRenderMapflag()
{
	return m_bRenderMapflag;
}
/**
	@brief Korean:  레이저의 그리기 여부를 정하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setRenderLaserflag(bool bRendeLaserflag)
{
	m_bRendeLaserflag=bRendeLaserflag;
}
/**
	@brief Korean:  레이저의 그리기 여부를 가져오는 함수
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderLaserflag()
{
	return m_bRendeLaserflag;
}
/**
	@brief Korean: 키넥트의 그리기 여부를 정하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setRenderKinectflag(bool bRenderKinectflag)
{
	m_bRenderKinectflag=bRenderKinectflag;
}
/**
	@brief Korean:  키넥트의 그리기 여부를 가져오는 함수
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderKinectflag()
{
	return m_bRenderKinectflag;
}
/**
	@brief Korean: 경로의 그리기 여부를 정하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setRenderPathflag(bool bRenderPathflag)
{
	m_bRenderPathflag=bRenderPathflag;
}
/**
	@brief Korean:  경로의 그리기 여부를 가져오는 함수
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderPathflag()
{
	return m_bRenderPathflag;
}

/**
	@brief Korean: 키넥트의 이미지 데이터를 저장하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setKinectImageData(IplImage* IplKinectImg)
{
	if(IplKinectImg!=NULL){
		m_CriticalSection.Lock();
		//cvCopy(IplKinectImg,m_IplKinectImg);
		for(int i=0; i<320; i++){
			for(int j=0; j<240; j++){
				m_IplKinectImg->imageData[(j*320+i)*3] = IplKinectImg->imageData[(j*320+i)*3];
				m_IplKinectImg->imageData[(j*320+i)*3+1] = IplKinectImg->imageData[(j*320+i)*3+1];
				m_IplKinectImg->imageData[(j*320+i)*3+2] = IplKinectImg->imageData[(j*320+i)*3+2];
			}
		}
		m_CriticalSection.Unlock();
	}
}

/**
	@brief Korean: 키넥트의 이미지 데이터를 가져가는 함수
	@brief English: write in English
*/
void KuDrawingInfo::getKinectImageData(IplImage* IplKinectImg)
{
	m_CriticalSection.Lock();
	for(int i=0; i<320; i++){
		for(int j=0; j<240; j++){
			IplKinectImg->imageData[(j*320+i)*3] = m_IplKinectImg->imageData[(j*320+i)*3];
			IplKinectImg->imageData[(j*320+i)*3+1] = m_IplKinectImg->imageData[(j*320+i)*3+1];
			IplKinectImg->imageData[(j*320+i)*3+2] = m_IplKinectImg->imageData[(j*320+i)*3+2];
		}
	}
	m_CriticalSection.Unlock();

}

/**
	@brief Korean: 작성중인 지도의 그리기 여부를 결정하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setRenderBuildingMapflag(bool bRenderBuildingMapflag)
{
	m_bRenderBuildingMapflag=bRenderBuildingMapflag;
}
/**
	@brief Korean: 작성중인 지도의 그리기 여부를 가져가는 함순
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderBuildingMapflag()
{
	return m_bRenderBuildingMapflag;
}
/**
	@brief Korean: 경로의 방향을 정하는 함수
	@brief English: write in English
*/
void KuDrawingInfo::setDirectionofPathflag(bool bDirectionofPathflag)
{
	m_bDirectionofPathflag=bDirectionofPathflag;
}

/**
	@brief Korean: 경로의 방향을  가져오는 함수
	@brief English: write in English
*/
bool KuDrawingInfo::getDirectionofPathflag()
{
	return m_bDirectionofPathflag;
}

void KuDrawingInfo::setvecLandmarkPos(vector<KuPose> vecLandmark)
{
	m_CriticalSection.Lock();
	m_vecLandmark = vecLandmark;
	m_CriticalSection.Unlock();
}
vector<KuPose> KuDrawingInfo::getvecLandmarkPos()
{
	vector<KuPose> vecLandmark;
	m_CriticalSection.Lock();
	vecLandmark = m_vecLandmark;
	m_CriticalSection.Unlock();
	return vecLandmark;

}
/**
	@brief Korean: 
	@brief English: write in English
*/
void KuDrawingInfo::setRenderCeilingImageflag(bool bRenderCeilingImagflag)
{
	m_bRenderCeilingImagflag=bRenderCeilingImagflag;
}
/**
	@brief Korean:  
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderCeilingImageflag()
{
	return m_bRenderCeilingImagflag;
}
/**
@brief Korean: 경유점 list를  저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setWayPointList(list<KuPose> listWayPoint)
{

	m_CriticalSection.Lock();
	m_listWayPoint = listWayPoint;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 경유점 list 를 넘겨주는 함수.
@brief English: write in English
*/
list<KuPose> KuDrawingInfo::getWayPointList()
{
	list<KuPose> listWayPoint;
	m_CriticalSection.Lock();
	listWayPoint = m_listWayPoint;
	m_CriticalSection.Unlock();

	return listWayPoint;
}
/**
	@brief Korean: 
	@brief English: write in English
*/
void KuDrawingInfo::setWayPointflag(bool bWayPointflag)
{
	m_bWayPointflag=bWayPointflag;
}
/**
	@brief Korean:  
	@brief English: write in English
*/
bool KuDrawingInfo::getWayPointflag()
{
	return m_bWayPointflag;
}

void KuDrawingInfo::setRenderCADMapflag(bool bRenderCADMapflag)
{
	m_bRenderCADMapflag=bRenderCADMapflag;
}
/**
	@brief Korean:  
	@brief English: write in English
*/
bool KuDrawingInfo::getRenderCADMapflag()
{
	return m_bRenderCADMapflag;
}

/**
 @brief Korean: 파티클에 대한 정보를 저장하는 함수
 @brief English: write in English
*/
void KuDrawingInfo::setParticle(vector<Sample> vecParticle)
{
	m_CriticalSection.Lock();
	m_vecParticle = vecParticle;
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 파티클에 대한 정보를 넘겨주는 함수
@brief English: write in English
*/
vector<Sample> KuDrawingInfo::getParticle()
{
	vector<Sample> vecParticle;
	m_CriticalSection.Lock();
	vecParticle = m_vecParticle;
	m_CriticalSection.Unlock();

	return vecParticle;
}


void KuDrawingInfo::setRenderKinect3DCloudFlag(bool bFlag)
{
	m_CriticalSection.Lock();
	m_bRenderKinect3DCloudFlag = bFlag;
	m_CriticalSection.Unlock();
}

/**
@brief Korean: Xtion센서의 3D 정보의 그리기 여부를 넘겨주는 함수
@brief English: Gets whether the Xtion sensor 3D range data are to be rendered or not
*/
bool KuDrawingInfo::getRenderKinect3DCloudFlag()
{
	bool bFlag;
	m_CriticalSection.Lock();
	bFlag = m_bRenderKinect3DCloudFlag;
	m_CriticalSection.Unlock();
	return bFlag;
}

/**
@brief Korean: Xtion센서의 3차원 거리 정보를 받아오는 함수
@brief English: Sets 3D range data of Xtion sensor
*/
void KuDrawingInfo::setKinectGlobal3DPos(KuPose* pGlobal3DPose)
{
	m_CriticalSection.Lock();
	int nSize = Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT;
	for(int i=0; i<nSize; i++){
		m_Global3DPose[i] = pGlobal3DPose[i];
	}
	m_CriticalSection.Unlock();
}

/**
@brief Korean: Xtion센서의 3차원 거리 정보를 넘겨주는 함수
@brief English: Gets 3D range data of Xtion sensor
*/
KuPose*  KuDrawingInfo::getKinectGlobal3DPos()
{
	return m_Global3DPose;
}

/**
@brief Korean: 
@brief English: 
*/

void  KuDrawingInfo::getMultiObstaclePosVector(vector<KuPose>* VecMultiObstaclePos)
{
	m_CriticalSection.Lock();
	for(int i=0; i<m_VecMultiObstaclePos.size();i++)
	{
		(*VecMultiObstaclePos).push_back(m_VecMultiObstaclePos[i]);
	}
	m_CriticalSection.Unlock();
}

void KuDrawingInfo::setMultiObstaclePosVector(vector<KuPose> VecMultiObstaclePos)
{
	m_CriticalSection.Lock();
	m_VecMultiObstaclePos.clear();
	for(int i=0;i<VecMultiObstaclePos.size();i++)
	{
		m_VecMultiObstaclePos.push_back(VecMultiObstaclePos[i]);
	}
	m_CriticalSection.Unlock();

}

void  KuDrawingInfo::getMultiObstacleGoalPosVector(vector<KuPose>* VecMultiObstacleGoalPos)
{
	m_CriticalSection.Lock();
	for(int i=0; i<m_VecMultiObstacleGoalPos.size();i++)
	{
		(*VecMultiObstacleGoalPos).push_back(m_VecMultiObstacleGoalPos[i]);
	}
	m_CriticalSection.Unlock();
}

void KuDrawingInfo::setMultiObstacleGoalPosVector(vector<KuPose> VecMultiObstacleGoalPos)
{
	m_CriticalSection.Lock();
	m_VecMultiObstacleGoalPos.clear();
	for(int i=0;i<VecMultiObstacleGoalPos.size();i++)
	{
		m_VecMultiObstacleGoalPos.push_back(VecMultiObstacleGoalPos[i]);
	}
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 
@brief English: 
*/
void  KuDrawingInfo::getTrackedObstacle(vector<KuPose>* VecTrackedObstacle)
{
	m_CriticalSection.Lock();
	for(int i=0; i<m_VecTrackedObstacle.size();i++)
	{
		(*VecTrackedObstacle).push_back(m_VecTrackedObstacle[i]);
	}
	m_CriticalSection.Unlock();
}

void KuDrawingInfo::setTrackedObstacle(vector<KuPose> VecTrackedObstacle)
{
	m_CriticalSection.Lock();
	m_VecTrackedObstacle.clear();
	for(int i=0;i<VecTrackedObstacle.size();i++)
	{
		m_VecTrackedObstacle.push_back(VecTrackedObstacle[i]);
	}
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 작성되는 지도의 확률적인 값을 얻어가는 부분
@brief English: write in English
*/
double ** KuDrawingInfo::getProObBuildingMap( int* nX, int* nY)
{	

	m_CriticalSection.Lock();
	*nX = m_nMapX;
	*nY = m_nMapY;
	double ** dMap=NULL;

	if(NULL != m_dProObMap){ //2차 배열이 할당된 적이 없는 상태이다.
		dMap = m_dProObMap;
	}
	m_CriticalSection.Unlock();

	return dMap;

}
/**
@brief Korean: 작성되는 지도의 확률적인 값을 얻어가는 부분
@brief English: write in English
*/
void KuDrawingInfo::setProObBuildingMap( double**  dMap )
{
	m_CriticalSection.Lock();
	m_dProObMap = dMap  ;
	m_CriticalSection.Unlock();
}

/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setLocalPath(list<KuPose> listPath)
{
	m_CriticalSection.Lock();
	m_listLocalPath.clear();
	list<KuPose>::iterator it;
	for(it=listPath.begin(); it!=listPath.end(); it++){
		m_listLocalPath.push_back(*it);
	}
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 경로를 넘겨주는 함수.
@brief English: write in English
*/
list<KuPose> KuDrawingInfo::getLocalPath()
{
	list<KuPose> listPath;
	list<KuPose>::iterator it;

	m_CriticalSection.Lock();
	for(it=m_listLocalPath.begin(); it!=m_listLocalPath.end(); it++){
		listPath.push_back(*it);
	}
	m_CriticalSection.Unlock();

	return listPath;
}

/**
@brief Korean: 
@brief English: 
*/
void KuDrawingInfo::setObstacleAreasFlag(bool bFlag)
{
	m_CriticalSection.Lock();
	m_bObstacleAreasFlag = bFlag;
	m_CriticalSection.Unlock();
}

/**
@brief Korean: 
@brief English: 
*/
bool KuDrawingInfo::getObstacleAreasFlag()
{
	bool bFlag;
	m_CriticalSection.Lock();
	bFlag = m_bObstacleAreasFlag;
	m_CriticalSection.Unlock();
	return bFlag;
}


/**
@brief Korean: 
@brief English: 
*/
void KuDrawingInfo::setObstacleAreas(KuPose ObstacleAreas)
{


	m_CriticalSection.Lock();
	m_vecObstacleAreas.push_back(ObstacleAreas);
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 
@brief English: 
*/
vector<KuPose>  KuDrawingInfo::getObstacleAreas()
{
	vector<KuPose> vecObstacleAreas;

	m_CriticalSection.Lock();
	for(int i=0; i<m_vecObstacleAreas.size();i++)
	{
		m_vecObstacleAreas[i].setPro(i);
		vecObstacleAreas.push_back(m_vecObstacleAreas[i]);
	}
	m_CriticalSection.Unlock();
	
	return vecObstacleAreas;
}

/**
@brief Korean: 
@brief English: 
*/
void KuDrawingInfo::clearObstacleAreas()
{
	m_CriticalSection.Lock();
	m_vecObstacleAreas.clear();
	m_CriticalSection.Unlock();
}



/**
@brief Korean: 지도정보를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setPathMap(double **dMap,int nMapSizeX,int nMapSizeY)
{
	if(m_dPathMap==NULL){

		m_dPathMap = new double*[nMapSizeX];

		if(m_dPathMap){
			for(int i = 0 ; i < nMapSizeX ; i++){
				m_dPathMap[i] = new double[nMapSizeY];
			}
		}
	}
	
	m_nPathMapSizeX=nMapSizeX;
	m_nPathMapSizeY=nMapSizeY;
	
	m_CriticalSection.Lock();
	//지도 데이터 복사
	for(int i=0; i<nMapSizeX; i++){
		for(int j=0; j<nMapSizeY; j++){
			m_dPathMap[i][j] = dMap[i][j];
		}
	}

	m_CriticalSection.Unlock();
}
/**
 @brief Korean: 저장된 지도정보를 얻어가는 함수. 
 @brief English: write in English
*/
double** KuDrawingInfo::getPathMap(int*nPathMapSizeX,int* nPathMapSizeY)
{
	(*nPathMapSizeX)=m_nPathMapSizeX;
	(*nPathMapSizeY)=m_nPathMapSizeY;

	return m_dPathMap;
}

void KuDrawingInfo::setPIOMapData(POIMapPara PIOMapData)
{
	POIMapPara tempPIOMapData;

	m_PIOMapData.Floor=-1;
	m_PIOMapData.POIIDNum=-1;
	m_PIOMapData.POIID.clear();
	m_PIOMapData.POIName.clear();
	m_PIOMapData.ROBTh.clear();
	m_PIOMapData.ROBX.clear();
	m_PIOMapData.ROBY.clear();

	tempPIOMapData.Floor=PIOMapData.Floor;
	tempPIOMapData.POIIDNum=PIOMapData.POIIDNum;

	for(int i=0; i<PIOMapData.POIIDNum;i++)
	{
		tempPIOMapData.POIID.push_back(PIOMapData.POIID[i]);
		tempPIOMapData.POIName.push_back(PIOMapData.POIName[i]);
		tempPIOMapData.ROBTh.push_back(PIOMapData.ROBTh[i]);
		tempPIOMapData.ROBX.push_back(PIOMapData.ROBX[i]);
		tempPIOMapData.ROBY.push_back(PIOMapData.ROBY[i]);
	}

	m_CriticalSection.Lock();
	m_PIOMapData=tempPIOMapData;
	m_CriticalSection.Unlock();
	
}
POIMapPara KuDrawingInfo::getPIOMapData( )
{
	return m_PIOMapData;
}


void KuDrawingInfo::setCurFloor(int nCurFloor)
{
	m_nCurFloor=nCurFloor;
}

int  KuDrawingInfo::getCurFloor()
{
	return m_nCurFloor;
}




/**
@brief Korean: 지도정보를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setLocalMap(int **nMap,int nMapSizeX,int nMapSizeY)
{
	if(m_nLocalMap==NULL){

		m_nLocalMap = new int*[nMapSizeX];

		if(m_nLocalMap){
			for(int i = 0 ; i < nMapSizeX ; i++){
				m_nLocalMap[i] = new int[nMapSizeY];
			}
		}
	}
	
	m_nLocalMapSizeX=nMapSizeX;
	m_nLocalMapSizeY=nMapSizeY;
	
	m_CriticalSection.Lock();
	//지도 데이터 복사
	for(int i=0; i<nMapSizeX; i++){
		for(int j=0; j<nMapSizeY; j++){
			m_nLocalMap[i][j] = nMap[i][j];
		}
	}

	m_CriticalSection.Unlock();
}
/**
 @brief Korean: 저장된 지도정보를 얻어가는 함수. 
 @brief English: write in English
*/
int** KuDrawingInfo::getLocalMap(int*nPathMapSizeX,int* nPathMapSizeY)
{
	(*nPathMapSizeX)=m_nLocalMapSizeX;
	(*nPathMapSizeY)=m_nLocalMapSizeY;

	return m_nLocalMap;
}

int KuDrawingInfo::getPatternID()
{
	 return m_nPatternID;
}

void KuDrawingInfo::setPatternID(int nPatternID)
{
	m_nPatternID=nPatternID;
}





/**
@brief Korean: 지도정보를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setpredictedProMap(double **dMap,int nMapSizeX,int nMapSizeY)
{
	if (dMap==NULL) return;
	if(m_dpredictedProMap==NULL){

		m_dpredictedProMap = new double*[nMapSizeX];

		if(m_dpredictedProMap){
			for(int i = 0 ; i < nMapSizeX ; i++){
				m_dpredictedProMap[i] = new double[nMapSizeY];
			}
		}
	}
	
	m_npredictedMapSizeX=nMapSizeX;
	m_npredictedMapSizeY=nMapSizeY;
	
	m_CriticalSection.Lock();
	//지도 데이터 복사
	for(int i=0; i<nMapSizeX; i++){
		for(int j=0; j<nMapSizeY; j++){
			m_dpredictedProMap[i][j] = dMap[i][j];
		}
	}

	m_CriticalSection.Unlock();
}
/**
 @brief Korean: 저장된 지도정보를 얻어가는 함수. 
 @brief English: write in English
*/
double** KuDrawingInfo::getpredictedProMap(int*npredictedMapSizeX,int* npredictedMapSizeY)
{
	(*npredictedMapSizeX)=m_npredictedMapSizeX;
	(*npredictedMapSizeY)=m_npredictedMapSizeY;

	return 	m_dpredictedProMap;
}

void KuDrawingInfo::setPredictedTraj(vector<KuTrajectory> VecTrajectory)
{
	m_CriticalSection.Lock();

	m_vecPredictedTraj.clear();

	for(int i=0;i<VecTrajectory.size();i++)
	{
		m_vecPredictedTraj.push_back(VecTrajectory[i]);
	}

	m_CriticalSection.Unlock();
}

void KuDrawingInfo::getPredictedTraj(vector<KuTrajectory>* VecTrajectory)
{
	m_CriticalSection.Lock();

	for(int i=0;i<m_vecPredictedTraj.size();i++)
	{
		(*VecTrajectory).push_back( m_vecPredictedTraj[i]);
	}

	m_CriticalSection.Unlock();
}

void KuDrawingInfo::setCubicPath(vector<KuTrajectory> VecPath)
{
	m_CriticalSection.Lock();

	m_vecCubicPath.clear();

	for(int i=0;i<VecPath.size();i++)
	{
		m_vecCubicPath.push_back(VecPath[i]);
	}

	m_CriticalSection.Unlock();
}

void KuDrawingInfo::getCubicPath(vector<KuTrajectory>* VecPath)
{
	m_CriticalSection.Lock();

	for(int i=0;i<m_vecCubicPath.size();i++)
	{
		(*VecPath).push_back( m_vecCubicPath[i]);
	}

	m_CriticalSection.Unlock();
}


void KuDrawingInfo::setPredictedRTraj(vector<KuPose> VecTrajectory)
{
	m_CriticalSection.Lock();

	m_vecPredictedRTraj.clear();

	for(int i=0;i<VecTrajectory.size();i++)
	{
		m_vecPredictedRTraj.push_back(VecTrajectory[i]);
	}

	m_CriticalSection.Unlock();
}

void KuDrawingInfo::getPredictedRTraj(vector<KuPose>* VecTrajectory)
{
	m_CriticalSection.Lock();

	for(int i=0;i<m_vecPredictedRTraj.size();i++)
	{
		(*VecTrajectory).push_back( m_vecPredictedRTraj[i]);
	}

	m_CriticalSection.Unlock();
}



/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setObsLocalPath(list<KuPose> listPath)
{
	m_CriticalSection.Lock();
	m_listObsLocalPath.clear();
	list<KuPose>::iterator it;
	for(it=listPath.begin(); it!=listPath.end(); it++){
		m_listObsLocalPath.push_back(*it);
	}
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 경로를 넘겨주는 함수.
@brief English: write in English
*/
list<KuPose> KuDrawingInfo::getObsLocalPath()
{
	list<KuPose> listPath;
	list<KuPose>::iterator it;

	m_CriticalSection.Lock();
	for(it=m_listObsLocalPath.begin(); it!=m_listObsLocalPath.end(); it++){
		listPath.push_back(*it);
	}
	m_CriticalSection.Unlock();

	return listPath;
}


/**
 @brief Korean: 저장된 골 위치를 넘겨주는 함수.
 @brief English: write in English
*/
KuPose KuDrawingInfo::getObsGoalPos()
{
	KuPose GoalPos;
	m_CriticalSection.Lock();
	GoalPos = m_ObsGoalPos;
	m_CriticalSection.Unlock();

	return GoalPos;
}

/**
 @brief Korean: 골 위치를 저장하는 함수
 @brief English: write in English
*/
void KuDrawingInfo::setObsGoalPos(KuPose GoalPos)
{
	m_CriticalSection.Lock();
	m_ObsGoalPos=GoalPos;
	m_CriticalSection.Unlock();

} 

bool KuDrawingInfo::getWanderingflag()
{
	return m_bWanderingstart;
}

void KuDrawingInfo::setWanderingflag(bool bWanderingstart)
{
	m_bWanderingstart=bWanderingstart;

} 


/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
void KuDrawingInfo::setvecObsLocalPath(list<KuPose> listPath)
{
	m_CriticalSection.Lock();
	m_veclistObsLocalPath.push_back(listPath);
	m_CriticalSection.Unlock();

}

/**
@brief Korean: 경로를 넘겨주는 함수.
@brief English: write in English
*/
vector<list<KuPose>> KuDrawingInfo::getvecObsLocalPath()
{
	vector<list<KuPose>> veclistPath;
	list<KuPose> listPath;
	list<KuPose>::iterator it;

	m_CriticalSection.Lock();
	for (int i=0;i<m_veclistObsLocalPath.size();i++)
	{
		listPath.clear();
		for(it=m_veclistObsLocalPath[i].begin(); it!=m_veclistObsLocalPath[i].end(); it++){
			listPath.push_back(*it);
		}
		veclistPath.push_back(listPath);
	}
	m_CriticalSection.Unlock();

	return veclistPath;
}