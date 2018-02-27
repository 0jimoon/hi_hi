#include "stdafx.h"
#include "KuSRPMLaserMapBuilderPr.h"

KuSRPMLaserMapBuilderPr::KuSRPMLaserMapBuilderPr(void)
{

	m_nCellSize = 100;				// 1cell 100mm;
	MIN_PROBABILITY = 0.2;		// ���� ���� ���� �ּ� Ȯ�� ��
	MAX_PROBABILITY = 0.8;		// ���� ���� ���� �ִ� Ȯ�� ��
	INITIAL_PROBABILITY = 0.5;	// �ʱ� Ȯ��: 0.5 (unknown region)

	m_dThicknessofWall= 50;  // ���� �β� (50mm)

	m_dRadiusofRobot = 400; //�κ������� 400(mm)

	m_bLocalMappingflag=false;

	GAUSSIAN_SD = 50;			// 50mm; ���� ���� ����þ� ǥ������ ��, ���� ������ ���� ����//0.4

	m_pMap =  NULL;
	m_nRefMap=NULL;
	m_nRefOriMap=NULL;
	m_LaserscannerConfiguration = NULL; //Laser��ġ ����

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
			delete []m_nRefMap[i];  //�������� ������ ��ü �޸� ����  
		delete []m_nRefMap;  
	}
	if(NULL!=m_nRefOriMap){
		for(int i=0; i<m_nMapSizeX; i++)       
			delete []m_nRefOriMap[i];  //�������� ������ ��ü �޸� ���� 
		delete []m_nRefOriMap;
	}
	
	if(NULL!=m_LaserscannerConfiguration){		
		delete [] m_LaserscannerConfiguration;
	}
}

/**
@brief Korean: �ʱ�ȭ �Լ�
@brief English: 
*/
void KuSRPMLaserMapBuilderPr::initialize(KuSRPMMapBuilderParameter InputParam)
{
	int nMapSizeXm=0, nMapSizeYm=0;

	m_dRadiusofRobot = InputParam.getRadiusofRobot();
	m_dThicknessofWall =InputParam.getThicknessofWall();
	m_nCellSize= InputParam.getCellSize();
	m_nScanIdX = InputParam.getLaserScanIdx();	//LRF ������ ����
	m_nLaserMinDist = InputParam.getMinDistofSensorData();//���� �������� �ּҰŸ�
	m_nLaserMaXDist = InputParam.getMaxDistofSensorData();//���� �������� �ִ�Ÿ�
	GAUSSIAN_SD = InputParam.getSigma();//���� �������� �ִ��
	double dCellSize=m_nCellSize;
	m_dLaserSensorXOffset=InputParam.getLaserXOffset();
	m_bLocalMappingflag=InputParam.getLocalMappingflag();

	InputParam.getMapSizeXmYm(&nMapSizeXm, &nMapSizeYm);
	m_nMapSizeX = nMapSizeXm * M2MM * 1/dCellSize; //m -> mm ��ȯ�ϰ� mm-> �Ѱ��ڰ� 100mm�� �迭 ���·� ��ȯ
	m_nMapSizeY = nMapSizeYm * M2MM * 1/dCellSize; //m -> mm ��ȯ�ϰ� mm-> �Ѱ��ڰ� 100mm�� �迭 ���·� ��ȯ

	if(m_pMap==NULL)
	{
		m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY); 
		m_dProMap = m_pMap->getProMap();
		m_nMap = m_pMap->getMap();
		
		if(NULL!=m_nRefMap){
			for(int i=0; i<m_nMapSizeX; i++)       
				delete []m_nRefMap[i];  //�������� ������ ��ü �޸� ����   
			delete []m_nRefMap; 
		}
		if(NULL!=m_nRefOriMap){
			for(int i=0; i<m_nMapSizeX; i++)       
				delete []m_nRefOriMap[i];  //�������� ������ ��ü �޸� ����   
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
				delete []m_nRefMap[i];  //�������� ������ ��ü �޸� ���� 46   
			delete []m_nRefMap;  
		}
		if(NULL!=m_nRefOriMap){			
			for(int i=0; i<m_nMapSizeX; i++)       
				delete []m_nRefOriMap[i];  //�������� ������ ��ü �޸� ���� 46   
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

	// Ȯ�� �� �ʱ�ȭ
	for(int i = 0; i<m_nMapSizeX; i++){
		for(int j = 0; j< m_nMapSizeY;j++){
			m_dProMap[i][j] = INITIAL_PROBABILITY;
			m_nMap[i][j] = KuMap::UNKNOWN_AREA;
		}
	}

	m_nLaserData = m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);

	if(m_LaserscannerConfiguration==NULL)
	{
		//���� �������� ��ġ�� �ݿ��ϱ� ���ؼ� ���� �־� �ش�.------------------------------------
		m_LaserscannerConfiguration = new KuPose[m_nScanIdX];
	}
	else
	{
		if(NULL!=m_LaserscannerConfiguration)
		{	
			delete [] m_LaserscannerConfiguration;	
		}		
		//���� �������� ��ġ�� �ݿ��ϱ� ���ؼ� ���� �־� �ش�.------------------------------------
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
@brief Korean: ��� ���������� �ʱ�ȭ ���ִ� �Լ�
@brief English: 
*/
void KuSRPMLaserMapBuilderPr::initMap()
{
	// Ȯ�� �� �ʱ�ȭ
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
@brief Korean: ������ �κ��� ���� ������ �������� ������ ������ �����͸� �޾ƿ´�
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
@brief Korean: �Ÿ������� �̿��Ͽ� �κ��� ��ġ�� �����ϴ� �Լ�
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
	
		// ���Ŀ� ���缭 ������ �Է�
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

		
	// ICP ����
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

	// �ݺ��Ǵ� ����,�ڽ��ΰ� �̸� ���
	s1 = sin(m_RobotPos.getThetaRad());	// yaw
	c1 = cos(m_RobotPos.getThetaRad());

	
	// localizer�� �����Ͽ� �����ϴ� ��ġ
	double dXm = (c1*dX + -s1*dY);
	double dYm = (s1*dX + c1*dY);

	printf("x=%f, y=%f,Deg=%f \n",dX,dY,dTheta*R2D);

	m_RobotPos.setXm( m_RobotPos.getXm() + dXm);
	m_RobotPos.setYm( m_RobotPos.getYm() + dYm);
	m_RobotPos.setThetaRad( m_RobotPos.getThetaRad() + dTheta);
}

/**
@brief Korean: KuSRPMLaserMapBuilderPr�� ���� �Լ�
@brief English: 
*/
void KuSRPMLaserMapBuilderPr::buildMap(KuSRPMMapBuilderParameter InputParam)
{

	KuPose DelRobotPos = InputParam.getDelRobotPos();//���ڴ� �������� �κ���ġ
	m_RobotPos = InputParam.getRobotPos();//�κ� ��ġ
	int_1DArray nData = InputParam.getLaserData();	//������ ������ �޾ƿ�
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

	buildGridmapByBayesUpdate(m_RobotPos,m_nLaserData, m_nScanIdX, InputParam.getLaserUpdateSpeedflag());//������꿡 ���� Ȯ���� ������Ʈ �Լ� ȣ��
}

/**
@brief Korean: ������꿡 ���� Ȯ���� ������Ʈ �Լ�
@brief English: 
*/
void KuSRPMLaserMapBuilderPr::buildGridmapByBayesUpdate(KuPose RobotPos, int_1DArray nLaserData, int nLaserIdx, bool bupdateSpeedflag)
{
	double dCellSize = m_nCellSize;			// ���������� ���� ������(10cm X 10cm)
	double dPro;							// ����þȺ����� ���� Ȯ���е��Լ��ǰ�갪
	double dInitPro = INITIAL_PROBABILITY;	// �ʱ� Ȯ��: 0.5 (unknown region)
	double dMinPro = MIN_PROBABILITY;		// ���� ���� ���� �ּ� Ȯ�� ��
	double dMaxPro = MAX_PROBABILITY;		// ���� ���� ���� �ִ� Ȯ�� ��
	double dSigma = GAUSSIAN_SD;			// 50mm; ����þ� ���� ǥ������ ��, ���� ������ ���� ������ �� ����//0.4
	double dThicknessofWall = m_dThicknessofWall;     // ������ �β��� ��Ÿ���� ���� mm
	double dRadiusofRobot_cell =m_dRadiusofRobot/dCellSize;
	double dLold, dLnew, dLsensor;							// Log odds form ��
	double dLo = (double)log(dInitPro/(1.0-dInitPro));		// �ʱ� log��
	int nIntervalDistance=dCellSize/10.0;

	for (int nSensorNO=0; nSensorNO<nLaserIdx; nSensorNO++) 
	{
		double dData = (double)nLaserData[nSensorNO];

		if (dData > m_nLaserMaXDist) {dData = m_nLaserMaXDist;}//LRF�ִ��������ɰŸ����� ū�����ʹ� �ִ� �����Ÿ��� ������
		if (dData < m_nLaserMinDist) {continue;}//LRF�ּ��������ɰŸ����� ���������ʹ� ������� ����

		// ������ ��ġ
		KuPose dSensorPose = m_Math.getTransformedPose(0.0, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//���� ������ġ
		KuPose dDetectPose = m_Math.getTransformedPose(dData, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//�����������Ͱ� ����� ���� ��ġ	

		double dGradientX = (dDetectPose.getX() - dSensorPose.getX())/(dCellSize*1000);		//�����κ��� �����������Ͱ� ����� ��ġ���� X�Ÿ�
		double dGradientY = (dDetectPose.getY() - dSensorPose.getY())/(dCellSize*1000);		//�����κ��� �����������Ͱ� ����� ��ġ���� Y�Ÿ�

		int nRobotX=RobotPos.getX()/dCellSize;
		int nRobotY=RobotPos.getY()/dCellSize;

		for(int i=-dRadiusofRobot_cell; i<dRadiusofRobot_cell;i++)
		{
			for(int j=-dRadiusofRobot_cell; j<dRadiusofRobot_cell;j++)
			{
				if (nRobotX+i<0 || nRobotY+j<0 || nRobotX+i>=m_pMap->getX() || nRobotY+j>=m_pMap->getY()) {continue;}	// �ʻ���� ��� ��� ���� ó��
				if(m_nRefMap[nRobotX+i][nRobotY+j] != KuMap::DYNAMIC_ENVIRONMENT_AREA
					&&m_nRefMap[nRobotX+i][nRobotY+j] != KuMap::FIXED_CAD_AREA)
				{
					m_dProMap[nRobotX+i][nRobotY+j] =0.01;//�κ���ġ �ֺ� �ݰ� 4x4���� �������������� ����
					m_nMap[nRobotX+i][nRobotY+j] = KuMap::EMPTY_AREA;	
				}
			}
		}

		double dRay = dData;
		int nCheckX=0, nCheckY=0;
		double dGridDist = 0;
		for (int nDistance=0; dGridDist<dData+dThicknessofWall; nDistance+=dCellSize) //ndistance�� ������ų ����
		{
			double dRayOfX = nDistance*dGradientX;		//�������� X�������� ���ݸ�ŭ ���� ��Ŵ
			double dRayOfY = nDistance*dGradientY;		//�������� Y�������� ���ݸ�ŭ ���� ��Ŵ
			dGridDist=hypot(dRayOfX, dRayOfY);
			dRay = dData-dGridDist;		//������ �����ͷκ��� ���ο��� Ȯ�� ��������� �����κ��� �����������Ͱ� ����� ��ġ���� �Ÿ��� nDistance��ġ�� ���� ���� ���Ÿ�

			if(dGridDist >= m_nLaserMaXDist*0.95) {continue;}	//�޾Ƶ��� �������� �ִ밪�̻� �϶�//0.8

			int nX = (int)( 0.5 + (dSensorPose.getX()+dRayOfX)/dCellSize );	//nX�� Ȯ���������� ������������ X��ǥ �ε����λ��(0.5�� �ݿø�)
			int nY = (int)( 0.5 + (dSensorPose.getY()+dRayOfY)/dCellSize );	//nY�� Ȯ���������� ������������ Y��ǥ �ε����λ��(0.5�� �ݿø�)
			if (nX<0 || nY<0 || nX>=m_pMap->getX() || nY>=m_pMap->getY()) {continue;}	// �ʻ���� ��� ��� ���� ó��
			if (nX==nCheckX && nY==nCheckY) {continue;}									// �ߺ� ���� ���� ó��

			if (bupdateSpeedflag) //���� �ʺ����� ��� ���� ��Ű������ �������� ���� ���� Ȯ��������
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
				dLold = (double)log(m_dProMap[nX][nY]/(1.0-m_dProMap[nX][nY]));	//���� ��Ȯ���� log odds form				                                                                
				dLnew = dLold + dLsensor - dLo;//���� Ȯ���� log odds ���·� ���

				m_dProMap[nX][nY] = (double)(1.0 - 1.0/ (1.0 + exp(dLnew)));//log odds ���¸� Ȯ�������� ��ȯ

				if (m_dProMap[nX][nY]>0.99) //�ִ�Ȯ�� ���� ����� 0.99�� ����
				{ 
					m_dProMap[nX][nY]=0.99; 
				}
				else if (m_dProMap[nX][nY]<0.01) //�ּ�Ȯ�� ���� ����� 0.01�� ����
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

			//������ �������¸� int �� ������ ����-----------------------------------------------
			if (m_dProMap[nX][nY] > MAX_PROBABILITY){ 
				m_nMap[nX][nY] = KuMap::OCCUPIED_AREA; }		//�ִ�Ȯ������ ũ�ٸ� ���� ����
			else if (m_dProMap[nX][nY] < MIN_PROBABILITY){ 
				m_nMap[nX][nY] = KuMap::EMPTY_AREA;	}		//�ּ�Ȯ������ �۴ٸ� ������ ����
			else{ 
				m_nMap[nX][nY] = KuMap::UNKNOWN_AREA;}		// ���� ����
			//------------------------------------------------------------------------------------

			//�ߺ��������� �˻��ϱ� ����
			nCheckX = nX;
			nCheckY = nY;
		} // for�� ��		
	} // for�� ��

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
@brief Korean: ������ �κ��� ���� ������ �������� ������ ������ �����͸� �޾ƿ´�
@brief English: 
*/
void  KuSRPMLaserMapBuilderPr::calculateDatafromMap( KuPose RobotPos, int_1DArray nLaserData, int nLaserIdx)
{

	double dCellSize = m_nCellSize;			// ���������� ���� ������(10cm X 10cm)
	double dPro;							// ����þȺ����� ���� Ȯ���е��Լ��ǰ�갪
	double dInitPro = INITIAL_PROBABILITY;	// �ʱ� Ȯ��: 0.5 (unknown region)
	double dMinPro = MIN_PROBABILITY;		// ���� ���� ���� �ּ� Ȯ�� ��
	double dMaxPro = MAX_PROBABILITY;		// ���� ���� ���� �ִ� Ȯ�� ��
	double dSigma = GAUSSIAN_SD;			// 50mm; ����þ� ���� ǥ������ ��, ���� ������ ���� ������ �� ����//0.4
	int nIntervalDistance=100;
	m_vetnIndex.clear();

	for (int nSensorNO=0; nSensorNO<nLaserIdx; nSensorNO++) 
	{
		double dData = (double)nLaserData[nSensorNO];

		if (dData > m_nLaserMaXDist) {dData = m_nLaserMaXDist;}//LRF�ִ��������ɰŸ����� ū�����ʹ� �ִ� �����Ÿ��� ������
		if (dData < m_nLaserMinDist) {continue;}//LRF�ּ��������ɰŸ����� ���������ʹ� ������� ����

		// ������ ��ġ
		KuPose dSensorPose = m_Math.getTransformedPose(0.0, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//���� ������ġ
		KuPose dDetectPose = m_Math.getTransformedPose(dData, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//�����������Ͱ� ����� ���� ��ġ	

		double dGradientX = (dDetectPose.getX() - dSensorPose.getX())/(dCellSize*1000);		//�����κ��� �����������Ͱ� ����� ��ġ���� X�Ÿ�
		double dGradientY = (dDetectPose.getY() - dSensorPose.getY())/(dCellSize*1000);		//�����κ��� �����������Ͱ� ����� ��ġ���� Y�Ÿ�

		int nRobotX=RobotPos.getX()/dCellSize;
		int nRobotY=RobotPos.getY()/dCellSize;


		double dRay = dData;
		int nCheckX=0, nCheckY=0;
		double dGridDist = 0;
		for (int nDistance=0; dGridDist<dData+100; nDistance+=nIntervalDistance) //ndistance�� ������ų ����
		{
			double dRayOfX = nDistance*dGradientX;		//�������� X�������� ���ݸ�ŭ ���� ��Ŵ
			double dRayOfY = nDistance*dGradientY;		//�������� Y�������� ���ݸ�ŭ ���� ��Ŵ
			dGridDist=hypot(dRayOfX, dRayOfY);
			dRay = dData-dGridDist;		//������ �����ͷκ��� ���ο��� Ȯ�� ��������� �����κ��� �����������Ͱ� ����� ��ġ���� �Ÿ��� nDistance��ġ�� ���� ���� ���Ÿ�

			if(dGridDist >= m_nLaserMaXDist*0.95) {continue;}	//�޾Ƶ��� �������� �ִ밪�̻� �϶�//0.8

			int nX = (int)( 0.5 + (dSensorPose.getX()+dRayOfX)/dCellSize );	//nX�� Ȯ���������� ������������ X��ǥ �ε����λ��(0.5�� �ݿø�)
			int nY = (int)( 0.5 + (dSensorPose.getY()+dRayOfY)/dCellSize );	//nY�� Ȯ���������� ������������ Y��ǥ �ε����λ��(0.5�� �ݿø�)
			if (nX<0 || nY<0 || nX>=m_pMap->getX() || nY>=m_pMap->getY()) {continue;}	// �ʻ���� ��� ��� ���� ó��
			if (nX==nCheckX && nY==nCheckY) {continue;}									// �ߺ� ���� ���� ó��
		

			if(m_nRefMap[nX][nY]== KuMap::TEMP_CAD_AREA||m_nRefMap[nX][nY]== KuMap::FIXED_CAD_AREA||m_nRefMap[nX][nY]== KuMap::OCCUPIED_AREA)
			{
				m_vetnIndex.push_back(nSensorNO);
				break;
			}
			else if( m_nRefMap[nX][nY]!= KuMap::EMPTY_AREA)
			{
				break;
			}
			//�ߺ��������� �˻��ϱ� ����
			nCheckX = nX;
			nCheckY = nY;
		} // for�� ��		
	} // for�� ��

}
/**
@brief Korean: Ȯ�� �ʵ����͸� �������� �Լ�
@brief English: 
*/
double** KuSRPMLaserMapBuilderPr::getProMap()
{
	return m_dProMap;
}

/**
@brief Korean: �ʵ����͸� �������� �Լ�
@brief English: 
*/
int** KuSRPMLaserMapBuilderPr::getMap()
{
	return m_nMap;
}
/**
@brief Korean: �ʵ����͸� �������� �Լ�
@brief English: 
*/
KuMap* KuSRPMLaserMapBuilderPr::getpMap()
{
	return m_pMap;
}

/**
@brief Korean: ���� ���� ����þ� ǥ���������� �����ϴ� �Լ�
@brief English: 
*/
void KuSRPMLaserMapBuilderPr::setSigma(double dSigma)
{
	GAUSSIAN_SD = dSigma;	
}

void KuSRPMLaserMapBuilderPr::setReferenceCADMap(int** nMap )
{
	// Ȯ�� �� �ʱ�ȭ
	for(int i = 0; i<m_nMapSizeX; i++){
		for(int j = 0; j< m_nMapSizeY;j++){

			if(i<1 || i > m_nMapSizeX-2 || j<1 || j> m_nMapSizeY-2){
				continue;
			}

			m_nRefMap[i][j] = nMap[i][j];		

			if(nMap[i][j] ==KuMap::FIXED_CAD_AREA) {
				if( nMap[i-1][j] != KuMap::FIXED_CAD_AREA ){ //�� �˻�...
					m_nRefMap[i-1][j] = KuMap::TEMP_CAD_AREA;
				}
				if( nMap[i+1][j] != KuMap::FIXED_CAD_AREA){ //�� �˻�.
					m_nRefMap[i+1][j] = KuMap::TEMP_CAD_AREA;
				}
				if( nMap[i][j-1]!= KuMap::FIXED_CAD_AREA){ //�� �˻�.
					m_nRefMap[i][j-1] = KuMap::TEMP_CAD_AREA;
				}
				if( nMap[i][j+1]!= KuMap::FIXED_CAD_AREA ){  //�� �˻�.
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
		// Ȯ�� �� �ʱ�ȭ
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
				if( nMap[i-1][j] !=KuMap::FIXED_CAD_AREA ){ //�� �˻�...
					nMap[i-1][j] = nTmpVal;
				}
				if( nMap[i+1][j] !=KuMap::FIXED_CAD_AREA ){ //�� �˻�.
					nMap[i+1][j] = nTmpVal;
				}
				if( nMap[i][j-1]!=KuMap::FIXED_CAD_AREA ){ //�� �˻�.
					nMap[i][j-1] = nTmpVal;
				}
				if( nMap[i][j+1]!=KuMap::FIXED_CAD_AREA  ){  //�� �˻�.
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
	// Ȯ�� �� �ʱ�ȭ
	for(int i = 0; i<m_nMapSizeX; i++){
		for(int j = 0; j< m_nMapSizeY;j++){

			if(i<1 || i > m_nMapSizeX-2 || j<1 || j> m_nMapSizeY-2){
				continue;
			}
			m_nRefOriMap[i][j] = nMap[i][j];		

		}
	}

}
