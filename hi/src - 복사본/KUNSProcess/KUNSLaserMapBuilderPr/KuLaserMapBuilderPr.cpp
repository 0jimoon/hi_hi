#include "stdafx.h"
#include "KuLaserMapBuilderPr.h"

KuLaserMapBuilderPr::KuLaserMapBuilderPr(void)
{

    m_nCellSize = 100;				// 1cell 100mm;
    MIN_PROBABILITY = 0.2;		// ���� ���� ���� �ּ� Ȯ�� ��
    MAX_PROBABILITY = 0.8;		// ���� ���� ���� �ִ� Ȯ�� ��
    INITIAL_PROBABILITY = 0.5;	// �ʱ� Ȯ��: 0.5 (unknown region)

    m_dThicknessofWall= 50;  // ���� �β� (50mm)

    m_dRadiusofRobot = 400; //�κ������� 400(mm)

    GAUSSIAN_SD = 50;			// 50mm; ���� ���� ����þ� ǥ������ ��, ���� ������ ���� ����//0.4

    m_pMap =  NULL;
    m_nRefMap=NULL;
    m_LaserscannerConfiguration = NULL; //Laser��ġ ����

    m_nScanIdX = -1;
    m_nLaserMinDist = -1;
    m_nLaserMaXDist = -1;
    m_nMapSizeX=0, m_nMapSizeY=0;

}

KuLaserMapBuilderPr::~KuLaserMapBuilderPr(void)
{
    if(NULL!=m_pMap){
        delete m_pMap;
    }
    if(NULL!=m_nRefMap){
        delete []m_nRefMap;
    }
    if(NULL!=m_LaserscannerConfiguration){
        delete [] m_LaserscannerConfiguration;
    }
}

/**
@brief Korean: �ʱ�ȭ �Լ�
@brief English:
*/
void KuLaserMapBuilderPr::initialize(KuMapBuilderParameter InputParam)
{
    int nMapSizeXAI=0, nMapSizeYAI=0;
    m_dRadiusofRobot = InputParam.getRadiusofRobot();
    m_dThicknessofWall =InputParam.getThicknessofWall();
    m_nCellSize= InputParam.getCellSize();
    m_nScanIdX = InputParam.getLaserScanIdx();	//LRF ������ ����
    m_nLaserMinDist = InputParam.getMinDistofSensorData();//���� �������� �ּҰŸ�
    m_nLaserMaXDist = InputParam.getMaxDistofSensorData();//���� �������� �ִ�Ÿ�
    GAUSSIAN_SD = InputParam.getSigma();//���� �������� �ִ��
    double dCellSize=m_nCellSize;
    m_Math.setCellSizeMM(dCellSize);

//     InputParam.getMapSizeXmYm(&nMapSizeXm, &nMapSizeYm);
//     m_nMapSizeX = m_Math.MM2AI(nMapSizeXm * M2MM ); //m -> mm ��ȯ�ϰ� mm-> �Ѱ��ڰ� 100mm�� �迭 ���·� ��ȯ
//     m_nMapSizeY = m_Math.MM2AI(nMapSizeYm * M2MM ); //m -> mm ��ȯ�ϰ� mm-> �Ѱ��ڰ� 100mm�� �迭 ���·� ��ȯ
	InputParam.getMapSizeXYAI(&nMapSizeXAI, &nMapSizeYAI);
	m_nMapSizeX =nMapSizeXAI; //m -> mm ��ȯ�ϰ� mm-> �Ѱ��ڰ� 100mm�� �迭 ���·� ��ȯ
	m_nMapSizeY =nMapSizeYAI; //m -> mm ��ȯ�ϰ� mm-> �Ѱ��ڰ� 100mm�� �迭 ���·� ��ȯ

    if(m_pMap==NULL)
    {
        m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY);
        m_dProMap = m_pMap->getProMap();
        m_nMap = m_pMap->getMap();

        if(NULL!=m_nRefMap){delete []m_nRefMap;}
        m_nRefMap = new int*[m_nMapSizeX];
        if(m_nRefMap){
            for(int i = 0 ; i < m_nMapSizeX ; i++){
                m_nRefMap[i] = new int[m_nMapSizeY];
            }
        }

    }
    else
    {
        if(NULL!=m_pMap){delete m_pMap;}
        m_pMap = new KuMap(m_nMapSizeX, m_nMapSizeY);
        m_dProMap = m_pMap->getProMap();
        m_nMap = m_pMap->getMap();

        if(NULL!=m_nRefMap){delete []m_nRefMap;}
        m_nRefMap = new int*[m_nMapSizeX];
        if(m_nRefMap){
            for(int i = 0 ; i < m_nMapSizeX ; i++){
                m_nRefMap[i] = new int[m_nMapSizeY];
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
        m_LaserscannerConfiguration[i].setX(InputParam.getLaserXOffset());
        m_LaserscannerConfiguration[i].setY(0.0);
        m_LaserscannerConfiguration[i].setThetaDeg(dAngleDeg);
    }
}
/**
@brief Korean: ��� ���������� �ʱ�ȭ ���ִ� �Լ�
@brief English:
*/
void KuLaserMapBuilderPr::initMap()
{
    // Ȯ�� �� �ʱ�ȭ
    for(int i = 0; i<m_nMapSizeX; i++){
        for(int j = 0; j< m_nMapSizeY;j++){
            m_dProMap[i][j] = INITIAL_PROBABILITY;
            m_nMap[i][j] = KuMap::UNKNOWN_AREA;
        }
    }
}
/**
@brief Korean: KuLaserMapBuilderPr�� ���� �Լ�
@brief English:
*/
void KuLaserMapBuilderPr::buildMap(KuMapBuilderParameter InputParam)
{
    KuPose RobotPos = InputParam.getRobotPos();//�κ� ��ġ
    KuPose DelRobotPos = InputParam.getDelRobotPos();//���ڴ� �������� �κ���ġ

    int_1DArray nData = InputParam.getLaserData();	//������ ������ �޾ƿ�
    for(int i=0; i<m_nScanIdX; i++){
        m_nLaserData[i] = nData[i];
    }
    buildGridmapByBayesUpdate(RobotPos,m_nLaserData, m_nScanIdX, InputParam.getLaserUpdateSpeedflag());//������꿡 ���� Ȯ���� ������Ʈ �Լ� ȣ��
}

/**
@brief Korean: ������꿡 ���� Ȯ���� ������Ʈ �Լ�
@brief English:
*/
void KuLaserMapBuilderPr::buildGridmapByBayesUpdate(KuPose RobotPos, int_1DArray nLaserData, int nLaserIdx, bool bupdateSpeedflag)
{
    double dCellSize = m_Math.getCellSizeMM();			// ���������� ���� ������(10cm X 10cm)
    double dPro;							// ����þȺ����� ���� Ȯ���е��Լ��ǰ�갪
    double dInitPro = INITIAL_PROBABILITY;	// �ʱ� Ȯ��: 0.5 (unknown region)
    double dMinPro = MIN_PROBABILITY;		// ���� ���� ���� �ּ� Ȯ�� ��
    double dMaxPro = MAX_PROBABILITY;		// ���� ���� ���� �ִ� Ȯ�� ��
    double dSigma = GAUSSIAN_SD;			// 50mm; ����þ� ���� ǥ������ ��, ���� ������ ���� ������ �� ����//0.4
    double dThicknessofWall = m_dThicknessofWall;     // ������ �β��� ��Ÿ���� ���� mm
    double dRadiusofRobot_cell =m_Math.MM2AI(m_dRadiusofRobot);
    double dLold, dLnew, dLsensor;							// Log odds form ��
    double dLo = (double)log(dInitPro/(1.0-dInitPro));		// �ʱ� log��
    int nIntervalDistance=m_Math.getCellSizeMM()/10.0;

    for (int nSensorNO=0; nSensorNO<nLaserIdx; nSensorNO++)
    {
        double dData = (double)nLaserData[nSensorNO];

        if (dData > m_nLaserMaXDist) {dData = m_nLaserMaXDist;}//LRF�ִ��������ɰŸ����� ū�����ʹ� �ִ� �����Ÿ��� ������
        if (dData < m_nLaserMinDist) {continue;}//LRF�ּ��������ɰŸ����� ���������ʹ� ������� ����

        // ������ ��ġ
        KuPose dSensorPose = m_Math.getTransformedPose(0.0, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//���� ������ġ
        KuPose dDetectPose = m_Math.getTransformedPose(dData, m_LaserscannerConfiguration[nSensorNO], RobotPos);	//�����������Ͱ� ����� ���� ��ġ

        double dGradientX = m_Math.MM2AI(dDetectPose.getX() - dSensorPose.getX())/10.0;		//�����κ��� �����������Ͱ� ����� ��ġ���� X�Ÿ�
        double dGradientY = m_Math.MM2AI(dDetectPose.getY() - dSensorPose.getY())/10.0;		//�����κ��� �����������Ͱ� ����� ��ġ���� Y�Ÿ�

        int nRobotX=m_Math.MM2AI(RobotPos.getX());
        int nRobotY=m_Math.MM2AI(RobotPos.getY());

        for(int i=-dRadiusofRobot_cell; i<dRadiusofRobot_cell;i++)
        {
            for(int j=-dRadiusofRobot_cell; j<dRadiusofRobot_cell;j++)
            {
                if (nRobotX+i<0 || nRobotY+j<0 || nRobotX+i>=m_pMap->getX() || nRobotY+j>=m_pMap->getY()) {continue;}	// �ʻ���� ��� ��� ���� ó��
                m_dProMap[nRobotX+i][nRobotY+j] =0.01;//�κ���ġ �ֺ� �ݰ� 4x4���� �������������� ����
                m_nMap[nRobotX+i][nRobotY+j] = KuMap::EMPTY_AREA;
            }
        }

        double dRay = dData;
        int nCheckX=0, nCheckY=0;
        double dGridDist = 0;
        for (int nDistance=0; dGridDist<dData+dThicknessofWall; nDistance+=nIntervalDistance) //ndistance�� ������ų ����
        {
            double dRayOfX = nDistance*dGradientX;		//�������� X�������� ���ݸ�ŭ ���� ��Ŵ
            double dRayOfY = nDistance*dGradientY;		//�������� Y�������� ���ݸ�ŭ ���� ��Ŵ
            dGridDist=hypot(dRayOfX, dRayOfY);
            dRay = dData-dGridDist;		//������ �����ͷκ��� ���ο��� Ȯ�� ��������� �����κ��� �����������Ͱ� ����� ��ġ���� �Ÿ��� nDistance��ġ�� ���� ���� ���Ÿ�

            if(dGridDist >= m_nLaserMaXDist*0.95) {continue;}	//�޾Ƶ��� �������� �ִ밪�̻� �϶�//0.8

            int nX = m_Math.MM2AI(dSensorPose.getX()+dRayOfX);	//nX�� Ȯ���������� ������������ X��ǥ �ε����λ��
            int nY = m_Math.MM2AI(dSensorPose.getY()+dRayOfY);	//nY�� Ȯ���������� ������������ Y��ǥ �ε����λ��
            if (nX<0 || nY<0 || nX>=m_pMap->getX() || nY>=m_pMap->getY()) {continue;}	// �ʻ���� ��� ��� ���� ó��
            if (nX==nCheckX && nY==nCheckY) {continue;}									// �ߺ� ���� ���� ó��

            if (bupdateSpeedflag) //���� �ʺ����� ��� ���� ��Ű������ �������� ���� ���� Ȯ��������
            {
                int nEndGridX = m_Math.MM2AI(dDetectPose.getX());
                int nEndGridY = m_Math.MM2AI(dDetectPose.getY());

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

            if(m_nRefMap[nX][nY] == KuMap::FIXED_CAD_AREA) continue;

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

/**
@brief Korean: Ȯ�� �ʵ����͸� �������� �Լ�
@brief English:
*/
double** KuLaserMapBuilderPr::getProMap()
{
    return m_dProMap;
}

/**
@brief Korean: �ʵ����͸� �������� �Լ�
@brief English:
*/
int** KuLaserMapBuilderPr::getMap()
{
    return m_nMap;
}
/**
@brief Korean: �ʵ����͸� �������� �Լ�
@brief English:
*/
KuMap* KuLaserMapBuilderPr::getpMap()
{
    return m_pMap;
}

/**
@brief Korean: ���� ���� ����þ� ǥ���������� �����ϴ� �Լ�
@brief English:
*/
void KuLaserMapBuilderPr::setSigma(double dSigma)
{
    GAUSSIAN_SD = dSigma;
}

void KuLaserMapBuilderPr::setReferenceCADMap(int** nMap )
{

    // Ȯ�� �� �ʱ�ȭ
    for(int i = 0; i<m_nMapSizeX; i++){
        for(int j = 0; j< m_nMapSizeY;j++){
            m_nRefMap[i][j] = nMap[i][j];
        }
    }
}
