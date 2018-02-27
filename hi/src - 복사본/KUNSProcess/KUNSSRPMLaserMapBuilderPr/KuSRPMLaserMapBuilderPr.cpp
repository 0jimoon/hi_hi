#include "stdafx.h"
#include "KuSRPMLaserMapBuilderPr.h"

KuSRPMLaserMapBuilderPr::KuSRPMLaserMapBuilderPr(void)
{

	m_nCellSize = 100;				// 1cell 100mm;
	MIN_PROBABILITY = 0.2;		// 센서 모델이 갖는 최소 확률 값
	MAX_PROBABILITY = 0.8;		// 센서 모델이 갖는 최대 확률 값
	INITIAL_PROBABILITY = 0.5;	// 초기 확률: 0.5 (unknown region)

	m_dThicknessofWall= 50;  // 벽의 두께 (50mm)

	m_dRadiusofRobot = 400; //로봇반지름 400(mm)

	m_bLocalMappingflag=false;

	GAUSSIAN_SD = 50;			// 50mm; 센서 모델의 가우시안 표준편차 값, 점유 영역의 폭을 조절//0.4

	m_pMap =  NULL;
	m_nRefMap=NULL;
	m_nRefOriMap=NULL;
	m_LaserscannerConfiguration = NULL; //Laser위치 정보

	m_nScanIdX = -1;
	m_nLaserMinDist = -1;
	m_nLaserMaXDist = -1; 
	m_nMapSizeX=0, m_nMapSizeY=0;

}

KuSRPMLaserMapBuilderPr::~KuSRPMLaserMapBuilderPr(void)
{
	if(NULL!=m_pMap){
		delete m_pMap;		
		m_pMap=NULL;
	}
	if(NULL!=m_nRefMap){
		for(int i=0; i<m_nMapSizeX; i++)       
			delete []m_nRefMap[i];  //동적으로 생성된 객체 메모리 해제  
		delete []m_nRefMap;  
	}
	if(NULL!=m_nRefOriMap){
		for(int i=0; i<m_nMapSizeX; i++)       
			delete []m_nRefOriMap[i];  //동적으로 생성된 객체 메모리 해제 
		delete []m_nRefOriMap;
	}
	
	if(NULL!=m_LaserscannerConfiguration){		
		delete [] m_LaserscannerConfiguration;
	}
}

/**
@brief Korean: 초기화 함수
@brief English: 
*/
void KuSRPMLaserMapBuilderPr::initialize(KuSRPMMapBuilderParameter InputParam)
{
	int nMapSizeXm=0, nMapSizeYm=0;

	m_dRadiusofRobot = InputParam.getRadiusofRobot();
	m_dThicknessofWall =InputParam.getThicknessofWall();
	m_nCellSize= InputParam.getCellSize();
	m_nScanIdX = InputParam.getLaserScanIdx();	//LRF 데이터 개수
	m_nLaserMinDist = InputParam.getMinDistofSensorData();//센서 측정가능 최소거리
	m_nLaserMaXDist = InputParam.getMaxDistofSensorData();//센서 측정가능 최대거리
	GAUSSIAN_SD = InputParam.getSigma();//센서 측정가능 최대거
	double dCellSize=m_nCellSize;
	m_dLaserSensorXOffset=InputParam.getLaserXOffset();
	m_bLocalMappingflag=InputParam.getLocalMappingflag();

	InputParam.getMapSizeXmYm(&nMapSizeXm, &nMapSizeYm);
	m_nMapSizeX = nMapSizeXm * M2MM * 1/dCellSize; //m -> mm 변환하고 mm-> 한격자가 100mm인 배열 형태로 변환
	m_nMapSizeY = nMapSizeYm * M2MM * 1/dCellSize; //m -> mm 변환하고 mm-> 한격자가 100mm인 배열 형태로 변환

	if(m_pMap==NULL)
	{
		m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
		m_dProMap = m_pMap->getProMap();
		m_nMap = m_pMap->getMap();
		
		if(NULL!=m_nRefMap){
			for(int i=0; i<m_nMapSizeX; i++)       
				delete []m_nRefMap[i];  //동적으로 생성된 객체 메모리 해제   
			delete []m_nRefMap; 
		}
		if(NULL!=m_nRefOriMap){
			for(int i=0; i<m_nMapSizeX; i++)       
				delete []m_nRefOriMap[i];  //동적으로 생성된 객체 메모리 해제   
			delete []m_nRefOriMap; 
		}

		m_nRefMap = new int*[m_nMapSizeX];
		m_nRefOriMap = new int*[m_nMapSizeX];

		if(m_nRefMap){
			for(int i = 0 ; i < m_nMapSizeX ; i++){
				m_nRefMap[i] = new int[m_nMapSizeY];
				m_nRefOriMap[i] = new int[m_nMapSizeY];
			}
		}		
	}
	else
	{
		if(NULL!=m_pMap){delete m_pMap;}
		m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
		m_dProMap = m_pMap->getProMap();
		m_nMap = m_pMap->getMap();
	
		if(NULL!=m_nRefMap){
			for(int i=0; i<m_nMapSizeX; i++)       
				delete []m_nRefMap[i];  //동적으로 생성된 객체 메모리 해제 46   
			delete []m_nRefMap;  
		}
		if(NULL!=m_nRefOriMap){			
			for(int i=0; i<m_nMapSizeX; i++)       
				delete []m_nRefOriMap[i];  //동적으로 생성된 객체 메모리 해제 46   
			delete []m_nRefOriMap;
		}

		m_nRefMap = new int*[m_nMapSizeX];
		m_nRefOriMap = new int*[m_nMapSizeX];

		if(m_nRefMap){
			for(int i = 0 ; i < m_nMapSizeX ; i++){
				m_nRefMap[i] = new int[m_nMapSizeY];
				m_nRefOriMap[i] = new int[m_nMapSizeY];

			}
		}
	}

	// 확률 맵 초기화
	for(int i = 0; i<m_nMapSizeX; i++){
		for(int j = 0; j< m_nMapSizeY;j++){
			m_dProMap[i][j] = INITIAL_PROBABILITY;
			m_nMap[i][j] = KuMap::UNKNOWN_AREA;
		}
	}

	m_nLaserData = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

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
		m_LaserscannerConfiguration[i].setX(m_dLaserSensorXOffset);
		m_LaserscannerConfiguration[i].setY(0.0);
		m_LaserscannerConfiguration[i].setThetaDeg(dAngleDeg);
	}
}
/**
@brief Korean: 모든 지도정보를 초기화 해주는 함수
@brief English: 
*/
void KuSRPMLaserMapBuilderPr::initMap()
{
	// 확률 맵 초기화
	for(int i = 0; i<m_nMapSizeX; i++){
		for(int j = 0; j< m_nMapSizeY;j++){
			m_dProMap[i][j] = INITIAL_PROBABILITY;
			m_nMap[i][j] = KuMap::UNKNOWN_AREA;
		}
	}
}
inline void KuSRPMLaserMapBuilderPr::copyRangeData(int_1DArray  nData)
{
	KuCartesianCoordinate2D CartesianCoordiNewRangeData;
	m_CartesianCoordiNewRangeData.clear();
	for(int i=0; i<m_vetnIndex.size(); i++){
		CartesianCoordiNewRangeData= m_Math.transfromPolar2CartesianMM( nData[m_vetnIndex[i]], (m_vetnIndex[i]-90), m_dLaserSensorXOffset );
		m_CartesianCoordiNewRangeData.push_back(CartesianCoordiNewRangeData);
	}
	//m_vetnIndex.clear();
}
inline void KuSRPMLaserMapBuilderPr::copyRangeDatafromMap(	vector<int> nLaserData)
{
	KuCartesianCoordinate2D CartesianCoordiRangeData;
	m_CartesianCoordiLastRangeData.clear();
	double dInter =1;
	for(int i=0; i<nLaserData.size(); i++){
		CartesianCoordiRangeData= m_Math.transfromPolar2CartesianMM( nLaserData[i], (1*i-90), m_dLaserSensorXOffset );
		m_CartesianCoordiLastRangeData.push_back(CartesianCoordiRangeData);
	}
	nLaserData.clear();
}


/**
@brief Korean: 추정된 로봇의 지도 정보를 바탕으로 가상의 레이저 데이터를 받아온다
@brief English: 
*/
void  KuSRPMLaserMapBuilderPr::calculateLaserDatafromMap( KuPose RobotPos,int** nMap)
{
	double dCellSize=m_nCellSize;
	int nScanIdx = m_nScanIdX;
	double dX_LASER_OFFSET =m_dLaserSensorXOffset;
	int nMapSizeX=m_nMapSizeX;
	int nMapSizeY=m_nMapSizeY;

	int nRobotX = (int)RobotPos.getX()/100;
	int nRobotY = (int)RobotPos.getY()/100;
	int nX=0., nY=0;
	double dTempX;
	double dTempY;

	double dX,dY,dZ;

	vector<int> nLaserData;
	double dDist = m_nLaserMaXDist;
	double dInter =1;
	int nDetectingRange=3;

	KuCartesianCoordinate2D CartesianCoordiRangeData;
	m_CartesianCoordiLastRangeData.clear();


	for (double i=0; i<nScanIdx; i+=dInter)
	{
		bool bflag=false;

		for(int n=0; n<m_vetnIndex.size();n++)
		{
			if(m_vetnIndex[n]-(nDetectingRange+1)<i<m_vetnIndex[n]+(nDetectingRange+1))
			{
				bflag=true;
			}
		}

		if(bflag==true){
			for(int j = 0; j < m_nLaserMaXDist; j += 10) // 3 cm step
			{
				dTempX = j * cos((i - 90) * D2R) + dX_LASER_OFFSET;
				dTempY = j * sin((i - 90) * D2R);

				
				nX = nRobotX + (int)((dTempX * cos(RobotPos.getThetaRad()) - dTempY * sin(RobotPos.getThetaRad())) /100);
				nY = nRobotY + (int)((dTempX * sin(RobotPos.getThetaRad()) + dTempY * cos(RobotPos.getThetaRad())) /100);

				if(nX < 1 || nX >= m_nMapSizeX|| nY < 1 || nY >= m_nMapSizeY) continue;
				if( nMap[nX][nY]!= KuMap::EMPTY_AREA)
				{
					CartesianCoordiRangeData= m_Math.transfromPolar2CartesianMM( j, (dInter*i-90), m_dLaserSensorXOffset );
					m_CartesianCoordiLastRangeData.push_back(CartesianCoordiRangeData);
					break;
				}

			}
		}
	}

}
/**
@brief Korean: 거리정보를 이용하여 로봇의 위치를 추정하는 함수
@brief English: 
*/
double KuSRPMLaserMapBuilderPr::generateRobotPose( )
{
	bool bReturn = false;
	int NoNewData, NoPreData;
	
	double *PreData;
	double *NewData;
	PreData = new double[m_CartesianCoordiLastRangeData.size()*2];
	NewData = new double[m_CartesianCoordiNewRangeData.size()*2];	

	double Tmatrix[3], Rmatrix[3][3];
	double RMSError=-1.0;
	
	// -------------------------------------------------------- ICP ---------------------------------------------------------------- //			
	
		// 형식에 맞춰서 센서값 입력
		NoPreData = 0;
		for (int i=0; i<m_CartesianCoordiLastRangeData.size(); i++) {
			if ( sqrt(m_CartesianCoordiLastRangeData[i].getXm() * m_CartesianCoordiLastRangeData[i].getXm() + 
						  m_CartesianCoordiLastRangeData[i].getYm() * m_CartesianCoordiLastRangeData[i].getYm() ) <= m_nLaserMaXDist*MM2M) {
					PreData[NoPreData*2+0] = m_CartesianCoordiLastRangeData[i].getXm();
					PreData[NoPreData*2+1] = m_CartesianCoordiLastRangeData[i].getYm();
					NoPreData++;
			}
		}
		NoNewData = 0;

		for (int i=0; i<m_CartesianCoordiNewRangeData.size(); i++) {
			if ( sqrt( m_CartesianCoordiNewRangeData[i].getXm() * m_CartesianCoordiNewRangeData[i].getXm() + 
				m_CartesianCoordiNewRangeData[i].getYm() * m_CartesianCoordiNewRangeData[i].getYm() ) <= m_nLaserMaXDist*MM2M) {
					NewData[NoNewData*2+0] = m_CartesianCoordiNewRangeData[i].getXm();
					NewData[NoNewData*2+1] = m_CartesianCoordiNewRangeData[i].getYm();
					NoNewData++;
			}
		}

		
	// ICP 수행
	RMSError = m_KuICP.icp(Rmatrix, Tmatrix,PreData, NoPreData, NewData, NoNewData, 30, 200, 0.1, 0.01, 2.0);
	printf("RMSError=%f \n",RMSError);

	if(RMSError!=-1&&RMSError<0.15&&fabs(Tmatrix[0])<0.5&&fabs(Tmatrix[1])<0.5)
		computeRobotPoseByICPFor2D(Tmatrix[0], Tmatrix[1], Rmatrix[0][0]);

	if(NULL!=PreData){	delete [] PreData;	}	
	if(NULL!=NewData){	delete [] NewData;	}	


	return RMSError;
}
void KuSRPMLaserMapBuilderPr::computeRobotPoseByICPFor2D(double dX, double dY, double dTheta)
{
	double s1=0.,c1=0.;

	// 반복되는 싸인,코싸인값 미리 계산
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad());

	
	// localizer를 수행하여 보정하는 위치
	double dXm = (c1*dX + -s1*dY);
	double dYm = (s1*dX + c1*dY);

	printf("x=%f, y=%f,Deg=%f \n",dX,dY,dTheta*R2D);

	m_RobotPos.setXm( m_RobotPos.getXm() + dXm);
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm);
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + dTheta);
}

/**
@brief Korean: KuSRPMLaserMapBuilderPr의 시작 함수
@brief English: 
*/
void KuSRPMLaserMapBuilderPr::buildMap(KuSRPMMapBuilderParameter InputParam)
{

	KuPose DelRobotPos = InputParam.getDelRobotPos();//엔코더 데이터의 로봇위치
	m_RobotPos = InputParam.getRobotPos();//로봇 위치
	int_1DArray nData = InputParam.getLaserData();	//레이저 데이터 받아옴
	for(int i=0; i<m_nScanIdX; i++){
		m_nLaserData[i] = nData[i];
	}
	
	if(m_bLocalMappingflag)
	{

// 		calculateDatafromMap(m_RobotPos,m_nLaserData, m_nScanIdX);
// 
// 		if(m_vetnIndex.size()>30)
// 		{
// 			copyRangeData(m_nLaserData);
// 			calculateLaserDatafromMap( m_RobotPos, m_nRefOriMap);
// 			generateRobotPose( );
// 		}
	}

	buildGridmapByBayesUpdate(m_RobotPos,m_nLaserData, m_nScanIdX, InputParam.getLaserUpdateSpeedflag());//베이즈룰에 따른 확률값 업데이트 함수 호출
}

/**
@brief Korean: 베이즈룰에 따른 확률값 업데이트 함수
@brief English: 
*/
void KuSRPMLaserMapBuilderPr::buildGridmapByBayesUpdate(KuPose RobotPos, int_1DArray nLaserData, int nLaserIdx, bool bupdateSpeedflag)
{
	double dCellSize = m_nCellSize;			// 격자지도의 격자 사이즈(10cm X 10cm)
	double dPro;							// 가우시안분포를 갖는 확률밀도함수의계산값
	double dInitPro = INITIAL_PROBABILITY;	// 초기 확률: 0.5 (unknown region)
	double dMinPro = MIN_PROBABILITY;		// 센서 모델이 갖는 최소 확률 값
	double dMaxPro = MAX_PROBABILITY;		// 센서 모델이 갖는 최대 확률 값
	double dSigma = GAUSSIAN_SD;			// 50mm; 가우시안 모델의 표준편차 값, 점유 영역의 폭을 조절할 수 있음//0.4
	double dThicknessofWall = m_dThicknessofWall;     // 지도의 두께를 나타내는 변수 mm
	double dRadiusofRobot_cell =m_dRadiusofRobot/dCellSize;
	double dLold, dLnew, dLsensor;							// Log odds form 값
	double dLo = (double)log(dInitPro/(1.0-dInitPro));		// 초기 log값
	int nIntervalDistance=dCellSize/10.0;

	for (int nSensorNO=0; nSensorNO<nLaserIdx; nSensorNO++) 
	{
		double dData = (double)nLaserData[nSensorNO];

		if (dData > m_nLaserMaXDist) {dData = m_nLaserMaXDist;}//LRF최대측정가능거리보다 큰데이터는 최대 측정거리로 맞춰줌
		if (dData < m_nLaserMinDist) {continue;}//LRF최소측정가능거리보다 작은데이터는 사용하지 않음

		// 센서의 위치
		KuPose dSensorPose = m_Math.getTransformedPose(0.0, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//실제 센서위치
		KuPose dDetectPose = m_Math.getTransformedPose(dData, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//레이저데이터가 검출된 곳의 위치	

		double dGradientX = (dDetectPose.getX() - dSensorPose.getX())/(dCellSize*1000);		//센서로부터 레이저데이터가 검출된 위치까지 X거리
		double dGradientY = (dDetectPose.getY() - dSensorPose.getY())/(dCellSize*1000);		//센서로부터 레이저데이터가 검출된 위치까지 Y거리

		int nRobotX=RobotPos.getX()/dCellSize;
		int nRobotY=RobotPos.getY()/dCellSize;

		for(int i=-dRadiusofRobot_cell; i<dRadiusofRobot_cell;i++)
		{
			for(int j=-dRadiusofRobot_cell; j<dRadiusofRobot_cell;j++)
			{
				if (nRobotX+i<0 || nRobotY+j<0 || nRobotX+i>=m_pMap->getX() || nRobotY+j>=m_pMap->getY()) {continue;}	// 맵사이즈를 벗어날 경우 예외 처리
				if(m_nRefMap[nRobotX+i][nRobotY+j] != KuMap::DYNAMIC_ENVIRONMENT_AREA
					&&m_nRefMap[nRobotX+i][nRobotY+j] != KuMap::FIXED_CAD_AREA)
				{
					m_dProMap[nRobotX+i][nRobotY+j] =0.01;//로봇위치 주변 반경 4x4맵을 비점유영역으로 설정
					m_nMap[nRobotX+i][nRobotY+j] = KuMap::EMPTY_AREA;	
				}
			}
		}

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

			int nX = (int)( 0.5 + (dSensorPose.getX()+dRayOfX)/dCellSize );	//nX를 확률지도상의 센서데이터의 X좌표 인덱스로사용(0.5는 반올림)
			int nY = (int)( 0.5 + (dSensorPose.getY()+dRayOfY)/dCellSize );	//nY를 확률지도상의 센서데이터의 Y좌표 인덱스로사용(0.5는 반올림)
			if (nX<0 || nY<0 || nX>=m_pMap->getX() || nY>=m_pMap->getY()) {continue;}	// 맵사이즈를 벗어날 경우 예외 처리
			if (nX==nCheckX && nY==nCheckY) {continue;}									// 중복 격자 예외 처리

			if (bupdateSpeedflag) //로컬 맵빌딩시 즉시 점유 시키기위해 레이저의 끝에 높은 확률값을줌
			{
				int nEndGridX = (int)( 0.5 + dDetectPose.getX()/dCellSize );
				int nEndGridY = (int)( 0.5 + dDetectPose.getY()/dCellSize );

				if (nX==nEndGridX && nY==nEndGridY) {
					m_dProMap[nX][nY] = 0.95;
				}
				else {
					m_dProMap[nX][nY] = 0.05;
				}
			}
			else 
			{
	
				// Bayesian update formula===========================================================
				dPro = 1.6*dSigma * (exp( -pow( (double)((dRay)/dSigma) , 2) / 2)) / (sqrt(2.0*M_PI)*dSigma) + dMinPro;
				if(dData<dGridDist)dPro=0.9;
				dLsensor = (double)log(dPro/(1.0-dPro));						//inverse sensor model
				dLold = (double)log(m_dProMap[nX][nY]/(1.0-m_dProMap[nX][nY]));	//이전 맵확률의 log odds form				                                                                
				dLnew = dLold + dLsensor - dLo;//점유 확률을 log odds 형태로 계산

				m_dProMap[nX][nY] = (double)(1.0 - 1.0/ (1.0 + exp(dLnew)));//log odds 형태를 확률값으로 변환

				if (m_dProMap[nX][nY]>0.99) //최대확률 값을 벗어날때 0.99로 제한
				{ 
					m_dProMap[nX][nY]=0.99; 
				}
				else if (m_dProMap[nX][nY]<0.01) //최소확률 값을 벗어날때 0.01로 제한
				{ 
					m_dProMap[nX][nY]=0.01;
				}
				//===================================================================================
			}			
			
			if(compareMapwithCADMap(nX, nY ))
			{	
				nCheckX=nX;	nCheckX=nY;
				continue;
			}

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
bool  KuSRPMLaserMapBuilderPr::compareMapwithCADMap(int nX, int nY )
{

	if(m_nRefMap[nX][nY] == KuMap::FIXED_CAD_AREA||m_nRefMap[nX][nY] == KuMap::TEMP_CAD_AREA)
	{
		if(m_bLocalMappingflag)
		{
			if(m_dProMap[nX][nY] > MAX_PROBABILITY){ 
				m_nMap[nX][nY] = KuMap::FIXED_CAD_AREA;
			}	
			else
				m_nMap[nX][nY] = KuMap::EMPTY_AREA;
			return true;
		}
		else{
			if(m_dProMap[nX][nY] > MAX_PROBABILITY){ 
				m_nMap[nX][nY] = KuMap::FIXED_CAD_AREA;
			}	
			else
				m_nMap[nX][nY] = KuMap::EMPTY_AREA;
			return true;
		}
	}
	else if(m_nRefMap[nX][nY] == KuMap::DYNAMIC_ENVIRONMENT_AREA)
	{
		if(m_bLocalMappingflag)
		{
			if(m_dProMap[nX][nY] > MAX_PROBABILITY){ 
				m_nMap[nX][nY] = KuMap::LUGGAGE_AREA;
			}	
			else
				m_nMap[nX][nY] = KuMap::DYNAMIC_ENVIRONMENT_AREA;
			return true;
		}
		else
		{
			if(m_dProMap[nX][nY] > MAX_PROBABILITY){ 
				m_nMap[nX][nY] = KuMap::LUGGAGE_AREA;
			}	
			else
				m_nMap[nX][nY] = KuMap::DYNAMIC_ENVIRONMENT_AREA;
			return true;
		}
	}

	return false;
}
/**
@brief Korean: 추정된 로봇의 지도 정보를 바탕으로 가상의 레이저 데이터를 받아온다
@brief English: 
*/
void  KuSRPMLaserMapBuilderPr::calculateDatafromMap( KuPose RobotPos, int_1DArray nLaserData, int nLaserIdx)
{

	double dCellSize = m_nCellSize;			// 격자지도의 격자 사이즈(10cm X 10cm)
	double dPro;							// 가우시안분포를 갖는 확률밀도함수의계산값
	double dInitPro = INITIAL_PROBABILITY;	// 초기 확률: 0.5 (unknown region)
	double dMinPro = MIN_PROBABILITY;		// 센서 모델이 갖는 최소 확률 값
	double dMaxPro = MAX_PROBABILITY;		// 센서 모델이 갖는 최대 확률 값
	double dSigma = GAUSSIAN_SD;			// 50mm; 가우시안 모델의 표준편차 값, 점유 영역의 폭을 조절할 수 있음//0.4
	int nIntervalDistance=100;
	m_vetnIndex.clear();

	for (int nSensorNO=0; nSensorNO<nLaserIdx; nSensorNO++) 
	{
		double dData = (double)nLaserData[nSensorNO];

		if (dData > m_nLaserMaXDist) {dData = m_nLaserMaXDist;}//LRF최대측정가능거리보다 큰데이터는 최대 측정거리로 맞춰줌
		if (dData < m_nLaserMinDist) {continue;}//LRF최소측정가능거리보다 작은데이터는 사용하지 않음

		// 센서의 위치
		KuPose dSensorPose = m_Math.getTransformedPose(0.0, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//실제 센서위치
		KuPose dDetectPose = m_Math.getTransformedPose(dData, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//레이저데이터가 검출된 곳의 위치	

		double dGradientX = (dDetectPose.getX() - dSensorPose.getX())/(dCellSize*1000);		//센서로부터 레이저데이터가 검출된 위치까지 X거리
		double dGradientY = (dDetectPose.getY() - dSensorPose.getY())/(dCellSize*1000);		//센서로부터 레이저데이터가 검출된 위치까지 Y거리

		int nRobotX=RobotPos.getX()/dCellSize;
		int nRobotY=RobotPos.getY()/dCellSize;


		double dRay = dData;
		int nCheckX=0, nCheckY=0;
		double dGridDist = 0;
		for (int nDistance=0; dGridDist<dData+100; nDistance+=nIntervalDistance) //ndistance는 증가시킬 간격
		{
			double dRayOfX = nDistance*dGradientX;		//레이저의 X방향으로 간격만큼 증가 시킴
			double dRayOfY = nDistance*dGradientY;		//레이저의 Y방향으로 간격만큼 증가 시킴
			dGridDist=hypot(dRayOfX, dRayOfY);
			dRay = dData-dGridDist;		//레이저 데이터로부터 내부영역 확률 계산을위해 센서로부터 레이저데이터가 검출된 위치까지 거리에 nDistance수치를 곱한 값을 뺀거리

			if(dGridDist >= m_nLaserMaXDist*0.95) {continue;}	//받아들인 레이저가 최대값이상 일때//0.8

			int nX = (int)( 0.5 + (dSensorPose.getX()+dRayOfX)/dCellSize );	//nX를 확률지도상의 센서데이터의 X좌표 인덱스로사용(0.5는 반올림)
			int nY = (int)( 0.5 + (dSensorPose.getY()+dRayOfY)/dCellSize );	//nY를 확률지도상의 센서데이터의 Y좌표 인덱스로사용(0.5는 반올림)
			if (nX<0 || nY<0 || nX>=m_pMap->getX() || nY>=m_pMap->getY()) {continue;}	// 맵사이즈를 벗어날 경우 예외 처리
			if (nX==nCheckX && nY==nCheckY) {continue;}									// 중복 격자 예외 처리
		

			if(m_nRefMap[nX][nY]== KuMap::TEMP_CAD_AREA||m_nRefMap[nX][nY]== KuMap::FIXED_CAD_AREA||m_nRefMap[nX][nY]== KuMap::OCCUPIED_AREA)
			{
				m_vetnIndex.push_back(nSensorNO);
				break;
			}
			else if( m_nRefMap[nX][nY]!= KuMap::EMPTY_AREA)
			{
				break;
			}
			//중복격자인지 검사하기 위해
			nCheckX = nX;
			nCheckY = nY;
		} // for문 끝		
	} // for문 끝

}
/**
@brief Korean: 확률 맵데이터를 가져가는 함수
@brief English: 
*/
double** KuSRPMLaserMapBuilderPr::getProMap()
{
	return m_dProMap;
}

/**
@brief Korean: 맵데이터를 가져가는 함수
@brief English: 
*/
int** KuSRPMLaserMapBuilderPr::getMap()
{
	return m_nMap;
}
/**
@brief Korean: 맵데이터를 가져가는 함수
@brief English: 
*/
KuMap* KuSRPMLaserMapBuilderPr::getpMap()
{
	return m_pMap;
}

/**
@brief Korean: 센서 모델의 가우시안 표준편차값을 설정하는 함수
@brief English: 
*/
void KuSRPMLaserMapBuilderPr::setSigma(double dSigma)
{
	GAUSSIAN_SD = dSigma;	
}

void KuSRPMLaserMapBuilderPr::setReferenceCADMap(int** nMap )
{
	// 확률 맵 초기화
	for(int i = 0; i<m_nMapSizeX; i++){
		for(int j = 0; j< m_nMapSizeY;j++){

			if(i<1 || i > m_nMapSizeX-2 || j<1 || j> m_nMapSizeY-2){
				continue;
			}

			m_nRefMap[i][j] = nMap[i][j];		

			if(nMap[i][j] ==KuMap::FIXED_CAD_AREA) {
				if( nMap[i-1][j] != KuMap::FIXED_CAD_AREA ){ //상 검사...
					m_nRefMap[i-1][j] = KuMap::TEMP_CAD_AREA;
				}
				if( nMap[i+1][j] != KuMap::FIXED_CAD_AREA){ //하 검사.
					m_nRefMap[i+1][j] = KuMap::TEMP_CAD_AREA;
				}
				if( nMap[i][j-1]!= KuMap::FIXED_CAD_AREA){ //좌 검사.
					m_nRefMap[i][j-1] = KuMap::TEMP_CAD_AREA;
				}
				if( nMap[i][j+1]!= KuMap::FIXED_CAD_AREA ){  //우 검사.
					m_nRefMap[i][j+1] = KuMap::TEMP_CAD_AREA;
				}
			}

		}
	}

	for(int i=0;i<m_nMapSizeX;i++){
		for(int j=0;j<m_nMapSizeY;j++){
			if(m_nRefMap[i][j] == KuMap::TEMP_CAD_AREA ){
				m_nRefMap[i][j] = KuMap::FIXED_CAD_AREA;
			}
		}
	}




	if(m_bLocalMappingflag)
	{
		for(int i=0;i<5;i++)
		{
			doMorphologyCloseForOccupiedGrid(m_nRefMap);
		}
	}
	else
	{
		// 확률 맵 초기화
		for(int i = 0; i<m_nMapSizeX; i++){
			for(int j = 0; j< m_nMapSizeY;j++){
				m_nMap[i][j] = m_nRefMap[i][j];
			}
		}
	}

}


void KuSRPMLaserMapBuilderPr::doMorphologyCloseForOccupiedGrid(int** nMap)
{

	int nTmpVal=12345678;

	for(int i=0;i<m_nMapSizeX;i++){
		for(int j=0;j<m_nMapSizeY;j++){
			if(i<1 || i > m_nMapSizeX-2 || j<1 || j> m_nMapSizeY-2){
				continue;
			}

			if(nMap[i][j] ==KuMap::FIXED_CAD_AREA) {
				if( nMap[i-1][j] !=KuMap::FIXED_CAD_AREA ){ //상 검사...
					nMap[i-1][j] = nTmpVal;
				}
				if( nMap[i+1][j] !=KuMap::FIXED_CAD_AREA ){ //하 검사.
					nMap[i+1][j] = nTmpVal;
				}
				if( nMap[i][j-1]!=KuMap::FIXED_CAD_AREA ){ //좌 검사.
					nMap[i][j-1] = nTmpVal;
				}
				if( nMap[i][j+1]!=KuMap::FIXED_CAD_AREA  ){  //우 검사.
					nMap[i][j+1] = nTmpVal;
				}
			}
		}
	}

	for(int i=0;i<m_nMapSizeX;i++){
		for(int j=0;j<m_nMapSizeY;j++){
			if(nMap[i][j] == nTmpVal ){
				nMap[i][j] = KuMap::FIXED_CAD_AREA;
			}
		}
	}


}

void KuSRPMLaserMapBuilderPr::setReferenceOriMap(int** nMap )
{
	// 확률 맵 초기화
	for(int i = 0; i<m_nMapSizeX; i++){
		for(int j = 0; j< m_nMapSizeY;j++){

			if(i<1 || i > m_nMapSizeX-2 || j<1 || j> m_nMapSizeY-2){
				continue;
			}
			m_nRefOriMap[i][j] = nMap[i][j];		

		}
	}

}
