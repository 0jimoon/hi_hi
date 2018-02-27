#include "stdafx.h"
#include "KuBuildingPOMapPr.h"

KuBuildingPOMapPr::KuBuildingPOMapPr()
{
	m_bThreadFlag = false;
	m_pLocalMap=NULL;
	m_pOriginalMap=NULL;
	m_pMap= NULL;
	m_nCellSize = 1000;				// 1cell 1000mm;
	MIN_PROBABILITY = 0.2;		// 센서 모델이 갖는 최소 확률 값
	MAX_PROBABILITY = 0.8;		// 센서 모델이 갖는 최대 확률 값
	INITIAL_PROBABILITY = 0.5;	// 초기 확률: 0.5 (unknown region)

	m_dThicknessofWall= 50;  // 벽의 두께 (50mm)

	m_dRadiusofRobot = 400; //로봇반지름 400(mm)

	GAUSSIAN_SD = 5000;			// 50mm; 센서 모델의 가우시안 표준편차 값, 점유 영역의 폭을 조절//0.4

	m_pMap =  NULL;
	m_nRefMap=NULL;
	m_LaserscannerConfiguration = NULL; //Laser위치 정보
	//m_pINIReaderWriter=NULL;

	m_nScanIdX = -1;
	m_nLaserMinDist = -1;
	m_nLaserMaXDist = -1; 
	m_nMapSizeX=0, m_nMapSizeY=0;
	m_nMapTimeNum=0;
	m_nMapNum=0;

	m_nDay=0;
	m_nHour=0;
	m_nDataNum=0;

	m_nRangeData=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	 m_ntempRangeData=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	cout<<"[KuProBuildingMapPr]: Instance is created!!!"<<endl;
}

KuBuildingPOMapPr::~KuBuildingPOMapPr()
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
	cout<<"[KuProBuildingMapPr]: Instance is destroyed!!!"<<endl;
}

/**
@brief Korean: 초기화 작업을 수행하는 함수.
@brief English: 
*/
bool KuBuildingPOMapPr::initialize(KuBuildingPOMapParameter InputParam)
{
	//m_bThreadFlag = false;
	m_bStandbyFlag=false;
	//int  nMapSizeX, nMapSizeY;
	m_timer.sleepMS(100);
	m_vecProbabilityMap.clear();
	m_nScanIdX=Sensor::URG04LX_DATA_NUM181;
	m_nRangeData=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

	double dThicknessofWall=InputParam.getThicknessofWall();
	InputParam.getMapSizeXmYm(&m_nMapSizeX, &m_nMapSizeY);
	m_nCellSize=InputParam.getCellSize();
	m_nLaserMinDist = InputParam.getMinDistofSensorData();//센서 측정가능 최소거리
	m_nLaserMaXDist = InputParam.getMaxDistofSensorData();//센서 측정가능 최대거리
	m_strMapFilePath=InputParam.getPath();


	if(loadProMap(m_strMapFilePath))
	{
		if(NULL==m_pMap)
		{
			m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 

			m_dProMap = m_pMap->getProMap();
			m_nMap = m_pMap->getMap();

			for(int i=0; i<m_nMapSizeX;i++)
			{
				for(int j=0; j<m_nMapSizeY;j++)
				{
					m_nMap[i][j] = 0;
					m_dProMap[i][j] = 0.5;
				}
			}
		}
	}

	for(int i=0; i<m_nMapSizeX;i++)
	{
		for(int j=0; j<m_nMapSizeY;j++)
		{
			m_nMap[i][j] = 0;
			m_dProMap[i][j] = 0.5;
		}
	}

	if(m_LaserscannerConfiguration==NULL)
	{
		//실제 레이저의 위치를 반영하기 위해서 값을 넣어 준다.------------------------------------
		m_LaserscannerConfiguration = new KuPose[m_nScanIdX];
	}
	else
	{
		if(NULL!=m_LaserscannerConfiguration)
		{	
			delete [] m_LaserscannerConfiguration;	
		}		
		//실제 레이저의 위치를 반영하기 위해서 값을 넣어 준다.------------------------------------
		m_LaserscannerConfiguration = new KuPose[m_nScanIdX];
	}

	for (int i=0; i<m_nScanIdX; i++) 
	{
		double dAngleDeg = (double)(i - (double)m_nScanIdX/2.0);
		m_LaserscannerConfiguration[i].setX(InputParam.getLaserXOffset());
		m_LaserscannerConfiguration[i].setY(0.0);
		m_LaserscannerConfiguration[i].setThetaDeg(dAngleDeg);
	}
	m_bStandbyFlag=true;
	
	return true;	
}

bool KuBuildingPOMapPr::loadProMap(string strMapFilePath)
{

	char cMapPathName[200];
	m_vecProbabilityMap.clear();
	string strMapFile;

	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/ProbablityMapData.txt", strMapFilePath.c_str());

	KuINIReadWriter* pINIReaderWriter;
	pINIReaderWriter = new KuINIReadWriter(cMapPathName); //설정파일 읽기

	if (pINIReaderWriter->ParseError() < 0) { //파일을 읽지 못한 경우 프로그램을 종료시킨다.
		cout << "Can't load "<<strMapFilePath<<endl;;
		//_getch();
	}
	else
	{
		int nMapSizeX = pINIReaderWriter->getIntValue("MAP","MAP_SIZE_X",-1);
		int nMapSizeY = pINIReaderWriter->getIntValue("MAP","MAP_SIZE_Y",-1);
		int nCellSize = pINIReaderWriter->getIntValue("MAP","CELL_SIZE",-1);

		if(nMapSizeX!=-1){m_nMapSizeX=nMapSizeX;}
		if(nMapSizeY!=-1){m_nMapSizeY=nMapSizeY;}
		if(nCellSize!=-1){m_nCellSize=nCellSize;}
		else{m_nCellSize=m_nCellSize*10;}

		m_math.setCellSizeMM(m_nCellSize);
	}
	
	delete pINIReaderWriter;

	///-----------------------------------------------------------------------------------------------///
	SYSTEMTIME stTime;
	GetLocalTime(&stTime);
	m_nDay = stTime.wDay;
	m_nHour =stTime.wHour;

	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/Day%d/Hour%d/ProbablityMapData.txt", strMapFilePath.c_str(),m_nDay,m_nHour);

	KuINIReadWriter* pINIReaderWriterA;
	pINIReaderWriterA = new KuINIReadWriter(cMapPathName); //설정파일 읽기

	if (pINIReaderWriterA->ParseError() < 0) { //파일을 읽지 못한 경우 프로그램을 종료시킨다.
		cout << "Can't load "<<strMapFilePath<<endl;;
		//_getch();
	}
	else
	{
		m_nDataNum = pINIReaderWriterA->getIntValue("PATH","DATANUMBER",0);
	}

	delete pINIReaderWriterA;

	return true;
}

void KuBuildingPOMapPr::saveProMap(string strMapFilePath,int nMapSizeX,int nMapSizeY, double** dMap)
{
	int i = 0;
	int j = 0;

	ofstream data(strMapFilePath);

	for(i=0; i<nMapSizeX; i++){

		for(j=0; j<nMapSizeY; j++){

			data<<dMap[i][j]<<" ";
		}
		data<<endl;
	}

	data.close();

}

bool KuBuildingPOMapPr::loadProMap(string strMapFilePath,int nMapSizeX,int nMapSizeY, double** dMap)
{	
	char cData;

	double t;

	ifstream file_in;
	file_in.open(strMapFilePath);

	if(!file_in.is_open())
	{
		return false;
	}

	for(int i = 0 ; i< nMapSizeX; i++)
	{
		for(int j = 0; j< nMapSizeY ; j++) 
		{
			file_in >> t;
			dMap[i][j] = t;
		}
	}

	file_in.close();
}
bool KuBuildingPOMapPr::saveProMap(string strMapFilePath)
{
	CString strTemp;
	CString strPath;
	char cMapPathName[200];
	CString strMapName;

	strPath=strMapFilePath.c_str();

	m_nDataNum=m_nDataNum+1;

	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/Day%d/Hour%d/ProbablityMapData.txt", strMapFilePath.c_str(),m_nDay,m_nHour);
	strPath=cMapPathName;

	strTemp.Format(L"%d",m_nDataNum);
	WritePrivateProfileString(_T("PATH"), _T("DATANUMBER"), strTemp,strPath);

	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"PATH&NAME%d",m_nDataNum);

	strMapName=cMapPathName;

	memset(cMapPathName,0,sizeof(cMapPathName));
	sprintf_s(cMapPathName,"%s/Day%d/Hour%d/Map%d.txt",strMapFilePath.c_str(),m_nDay,m_nHour, m_nDataNum);
	strTemp=cMapPathName;

	WritePrivateProfileString(_T("PATH"), strMapName, strTemp,strPath);

	saveProMap(cMapPathName,m_nMapSizeX,m_nMapSizeY, m_dProMap);

	return true;
}

/**
@brief Korean: ///경로를 저장하는 함수.
@brief English: write in English
*/
void KuBuildingPOMapPr::setData(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nRangeData)
{
	KuCriticalSection::getInstance()->Lock();
	m_RobotPos=RobotPos;
	m_DelEncoderData=DelEncoderData;
	for (int i=0; i<Sensor::URG04LX_DATA_NUM181;i++)
	{
		m_nRangeData[i]=nRangeData[i];
	}
	KuCriticalSection::getInstance()->Unlock();
}

void KuBuildingPOMapPr::updateBayesianformula(KuPose RobotPos,KuPose DelEncoderData,int nRangeIdx, int_1DArray nRangeData)
{
	double dCellSize = m_math.getCellSizeMM();			// 격자지도의 격자 사이즈(10cm X 10cm)		
	double dPro;							// 가우시안분포를 갖는 확률밀도함수의계산값
	double dInitPro = INITIAL_PROBABILITY;	// 초기 확률: 0.5 (unknown region)
	double dMinPro = MIN_PROBABILITY;		// 센서 모델이 갖는 최소 확률 값
	double dMaxPro = MAX_PROBABILITY;		// 센서 모델이 갖는 최대 확률 값
	double dSigma = GAUSSIAN_SD;			// 50mm; 가우시안 모델의 표준편차 값, 점유 영역의 폭을 조절할 수 있음//0.4
	double dThicknessofWall = m_dThicknessofWall;     // 지도의 두께를 나타내는 변수 mm
	double dRadiusofRobot_cell =m_dRadiusofRobot/dCellSize;
	double dLold, dLnew, dLsensor;							// Log odds form 값
	double dLo = (double)log(dInitPro/(1.0-dInitPro));		// 초기 log값
	int nIntervalDistance=m_math.getCellSizeMM()/10.0; 


	for (int nSensorNO=0; nSensorNO<nRangeIdx; nSensorNO++) 
	{
		double dData = (double)nRangeData[nSensorNO];

		if (dData > m_nLaserMaXDist) {dData = m_nLaserMaXDist;}//LRF최대측정가능거리보다 큰데이터는 최대 측정거리로 맞춰줌
		if (dData < m_nLaserMinDist) {continue;}//LRF최소측정가능거리보다 작은데이터는 사용하지 않음

		// 센서의 위치
		KuPose dSensorPose = m_math.getTransformedPose(0.0, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//실제 센서위치
		KuPose dDetectPose = m_math.getTransformedPose(dData, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//레이저데이터가 검출된 곳의 위치	

		double dGradientX = m_math.MM2AI((dDetectPose.getX() - dSensorPose.getX())*1000.0)/(10.0*1000.0);			//센서로부터 레이저데이터가 검출된 위치까지 X거리
		double dGradientY = m_math.MM2AI((dDetectPose.getY() - dSensorPose.getY())*1000.0)/(10.0*1000.0);			//센서로부터 레이저데이터가 검출된 위치까지 Y거리

		int nRobotX=m_math.MM2AI(RobotPos.getX());
		int nRobotY=m_math.MM2AI(RobotPos.getY());

		// 		for(int i=-dRadiusofRobot_cell; i<dRadiusofRobot_cell;i++)
		// 		{
		// 			for(int j=-dRadiusofRobot_cell; j<dRadiusofRobot_cell;j++)
		// 			{
		// 				if (nRobotX+i<0 || nRobotY+j<0 || nRobotX+i>=m_pMap->getX() || nRobotY+j>=m_pMap->getY()) {continue;}	// 맵사이즈를 벗어날 경우 예외 처리
		// 				m_dProMap[nRobotX+i][nRobotY+j] =0.01;//로봇위치 주변 반경 4x4맵을 비점유영역으로 설정
		// 				m_nMap[nRobotX+i][nRobotY+j] = KuMap::EMPTY_AREA;	
		// 			}
		// 		}
		// 	
		double dRay = dData;
		int nCheckX=0, nCheckY=0;
		double dGridDist = 0;
		for (int nDistance=0; dGridDist<dData+dThicknessofWall; nDistance+=dCellSize) //ndistance는 증가시킬 간격
		{
			double dRayOfX = nDistance*dGradientX;		//레이저의 X방향으로 간격만큼 증가 시킴
			double dRayOfY = nDistance*dGradientY;		//레이저의 Y방향으로 간격만큼 증가 시킴
			dGridDist=hypot(dRayOfX, dRayOfY);
			dRay = dData-dGridDist;		//레이저 데이터로부터 내부영역 확률 계산을위해 센서로부터 레이저데이터가 검출된 위치까지 거리에 nDistance수치를 곱한 값을 뺀거리

			if(dGridDist >= m_nLaserMaXDist*0.95) {continue;}	//받아들인 레이저가 최대값이상 일때//0.8

			// 			int nX = (int)( 0.5 + (dSensorPose.getX()+dRayOfX)/dCellSize );	//nX를 확률지도상의 센서데이터의 X좌표 인덱스로사용(0.5는 반올림)
			// 			int nY = (int)( 0.5 + (dSensorPose.getY()+dRayOfY)/dCellSize );	//nY를 확률지도상의 센서데이터의 Y좌표 인덱스로사용(0.5는 반올림)

			int nX = m_math.MM2AI(dSensorPose.getX()+dRayOfX);	//nX를 확률지도상의 센서데이터의 X좌표 인덱스로사용
			int nY = m_math.MM2AI(dSensorPose.getY()+dRayOfY);	//nY를 확률지도상의 센서데이터의 Y좌표 인덱스로사용

			if (nX<0 || nY<0 || nX>=m_pMap->getX() || nY>=m_pMap->getY()) {continue;}	// 맵사이즈를 벗어날 경우 예외 처리
			if (nX==nCheckX && nY==nCheckY) {continue;}									// 중복 격자 예외 처리

			else 
			{
				// Bayesian update formula===========================================================
				dPro = 0.52* (exp( -pow( (double)((dRay)/dSigma) , 2) / 2)) / (sqrt(2.0*M_PI)*dSigma) + dMinPro;


				if(dData<dGridDist){dPro=0.9;}
				if(dPro<0.5){ dPro=0.4999;}
				//if(dPro>0.5){dPro=0.52;	}

				dLsensor = (double)log(dPro/(1.0-dPro));						//inverse sensor model
				dLold = (double)log(m_dProMap[nX][nY]/(1.0-m_dProMap[nX][nY]));	//이전 맵확률의 log odds form				                                                                
				dLnew = dLold + dLsensor - dLo;//점유 확률을 log odds 형태로 계산

				m_dProMap[nX][nY] = (double)(1.0 - 1.0/ (1.0 + exp(dLnew)));//log odds 형태를 확률값으로 변환

				if (m_dProMap[nX][nY]>0.999) //최대확률 값을 벗어날때 0.99로 제한
				{ 
					m_dProMap[nX][nY]=0.999; 
				}
				else if (m_dProMap[nX][nY]<0.01) //최소확률 값을 벗어날때 0.01로 제한
				{ 
					m_dProMap[nX][nY]=0.00000001;
				}
				//===================================================================================
			}			

			//if(m_nRefMap[nX][nY] == KuMap::FIXED_CAD_AREA) continue;

			//격자의 점유상태를 int 형 지도에 저장-----------------------------------------------
			if (m_dProMap[nX][nY] > MAX_PROBABILITY){ 
				m_nMap[nX][nY] = KuMap::OCCUPIED_AREA; }		//최대확률보다 크다면 점유 영역
			else if (m_dProMap[nX][nY] < MIN_PROBABILITY){ 
				m_nMap[nX][nY] = KuMap::EMPTY_AREA;	}		//최소확률보다 작다면 비점유 영역
			else{ 
				m_nMap[nX][nY] = KuMap::UNKNOWN_AREA;}		// 미지 영역
			//------------------------------------------------------------------------------------

			//중복격자인지 검사하기 위해
			nCheckX = nX;
			nCheckY = nY;
		} // for문 끝		
	} // for문 끝

}

void KuBuildingPOMapPr::doProBuilingdThread(void* arg)
{
	KuBuildingPOMapPr* pPBMT = (KuBuildingPOMapPr*)arg;

	if(pPBMT->m_bStandbyFlag==true)
	{
		LARGE_INTEGER present1;		
		pPBMT->startTimeCheck(present1);

		double dTargetDist=0;	

		KuCriticalSection::getInstance()->Lock();
		KuPose RobotPos =pPBMT->m_RobotPos;
		KuPose DelEncoderData =pPBMT->m_DelEncoderData;
		int nRangeIdx =pPBMT->m_nScanIdX;
		//int_1DArray nRangeData=pPBMT->m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
		for(int i=0;i<Sensor::URG04LX_DATA_NUM181;i++)	pPBMT->m_ntempRangeData[i]=pPBMT->m_nRangeData[i];
		KuCriticalSection::getInstance()->Unlock();

		pPBMT->updateBayesianformula(RobotPos,DelEncoderData,nRangeIdx, pPBMT->m_ntempRangeData);


		double dtotalelapsed =(double) pPBMT->finishTimeCheck(present1);
		printf("doProBuilingdThread::%f\n",dtotalelapsed);
	}
}

void KuBuildingPOMapPr::startTimeCheck(LARGE_INTEGER& nStart)
{
	QueryPerformanceCounter(&nStart);
}

float KuBuildingPOMapPr::finishTimeCheck(const LARGE_INTEGER nStart)
{
	LARGE_INTEGER nFinish, freq;

	QueryPerformanceFrequency(&freq);

	QueryPerformanceCounter(&nFinish);

	return (float)(1000.0 * (nFinish.QuadPart - nStart.QuadPart) / freq.QuadPart);
}
bool KuBuildingPOMapPr::saveProMap( )
{
	saveProMap(m_strMapFilePath);
	return true;
}

/**
@brief Korean: 시뮬레이터를 종료한다.
@brief English: 
*/
void KuBuildingPOMapPr::terminate()
{
	saveProMap(m_strMapFilePath);
	m_BuildingProMapThread.terminate();
	cout<<"[KuProBuildingMapPr]:: Behavior is terminated!!!"<<endl;
	m_bThreadFlag = false;	

}
/**
@brief Korean: KuProBuildingMapPr를 실행시키는 함수
@brief English: 
*/

bool KuBuildingPOMapPr::start(KuBuildingPOMapParameter InputParam)
{
	if(initialize(InputParam )==true)
	{
		if(m_bThreadFlag==false)
		{
			m_bThreadFlag = true;

			m_BuildingProMapThread.start(doProBuilingdThread,this,100); //메인 스레드 시작	
			return true;
		}
	}
	else return false;
}

/**
@brief Korean: 확률 맵데이터를 가져가는 함수
@brief English: 
*/
double** KuBuildingPOMapPr::getProMap()
{
	return m_dProMap;
}
