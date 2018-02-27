#include <stdafx.h>
#include "SensorSupervisor.h"

SensorSupervisor::SensorSupervisor()
{
	HokuyoURG04LXInterface::getInstance();
	KinectSensorInterface::getInstance();
	init();
}

SensorSupervisor::~SensorSupervisor()
{
	if(m_f3DX!=NULL)
	delete [] m_f3DX;
	if(m_f3DY!=NULL)
	delete [] m_f3DY;
	if(m_f3DZ!=NULL)
	delete [] m_f3DZ;
	if(m_bFlag3D!=NULL)
	delete [] m_bFlag3D;
// 	if(m_pKinectDataPos!=NULL)
// 	delete [] m_pKinectDataPos;	
	if(m_d3DKinectX!=NULL)
	delete [] m_d3DKinectX;
	if(m_d3DKinectY!=NULL)
	delete [] m_d3DKinectY;
	if(m_d3DKinectZ!=NULL)
	delete [] m_d3DKinectZ;
	if(m_n3DKinectPixX!=NULL)
	delete [] m_n3DKinectPixX;
	if(m_n3DKinectPixY!=NULL)
	delete [] m_n3DKinectPixY;
	if(m_n3DKinectDataValidation!=NULL)
	delete [] m_n3DKinectDataValidation;
}

/**
@brief Korean: ����Ÿ�� �ʱ�ȭ
@brief English: write in English
*/
void SensorSupervisor::init()
{
	m_nURGLaserData=m_KuUtil.generateIntType1DArray(Sensor::URG04LX_DATA_NUM181,0);
	m_nKinectRangeData=m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,0);
	m_nHightDataOfRangeData=m_KuUtil.generateIntType1DArray(Sensor::KINECT_SENSOR_FOV,0);
	m_nRplidarData=m_KuUtil.generateIntType1DArray(Sensor::RPLIDAR_DATA_NUM181,0);


	//Ű��Ʈ ���� ������ �ʱ�ȭ
	m_IplKinectImage = NULL;
	m_IplKinectDepthImg = NULL;
	m_d3DKinectX = new double[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_d3DKinectY = new double[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_d3DKinectZ = new double[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_n3DKinectPixX = new int[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_n3DKinectPixY = new int[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_n3DKinectDataValidation = new int[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_KinectLogIdx = 0;
	m_pKinectDataPos = new KuPose[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];

	//���� �������ʱ�ȭ
	m_f3DX = new float[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_f3DY = new float[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_f3DZ = new float[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_bFlag3D = new bool[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];
	m_fKinectDistanceImage = new float[Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT];

	m_KinectLogIdx = 0;
	m_bDataRecoding=false;
	m_bDataPlay=false;
	m_EncoderDelPos.init();
	m_tempEncoderDelPos.init();
}
/**
@brief Korean: ������ ���� �����͸� Ȱ�뿩�θ� �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::DataRecodingNPlay()
{

	string strDataPath =KuRobotParameter::getInstance()->getDataPath();
	string strDataRecoding =KuRobotParameter::getInstance()->getDataRecoding();
	
	if( strDataRecoding=="yes" ){ //������ ���ڵ��� �����ϴ� ����̴�.

		m_bDataRecoding = true;
		m_DataLog.open(strDataPath);

	}
	else if(strDataRecoding=="no" ){ //������ ���ڵ��� �������� �ʴ� ����̴�.
		
			string strDataPlay = KuRobotParameter::getInstance()->getDataPlay();

		if(strDataPlay=="yes" ){ //���ڵ��� �����͸� �о�� play�ϴ� ����̴�.
			m_bDataPlay = true;
			m_PlayLog.open(strDataPath);
		}
	}
}
/**
@brief Korean: ������������ �����ϴ� �Լ�
@brief English: write in English
*/
bool SensorSupervisor::connectionHokuyoutm()
{
	
	char  HokuyoURGCom[10];
	KuRobotParameter::getInstance()->getURG04LXLaserComport(HokuyoURGCom);	
	HokuyoURG04LXInterface::getInstance()->setComPort(HokuyoURGCom);			
	HokuyoURG04LXInterface::getInstance()->setMaxDistance(KuRobotParameter::getInstance()->getURG04LXLaserMaxDist());
	HokuyoURG04LXInterface::getInstance()->setMinDistance(KuRobotParameter::getInstance()->getURG04LXLaserMinDist());
	if(HokuyoURG04LXInterface::getInstance()->connectLaserScanner()) 	
			return true;
	return false;
}
/**
@brief Korean: ������������ �����ϴ� �Լ�
@brief English: write in English
*/
bool SensorSupervisor::connectionRplidar()
{

	char  RplidarCom[10];
	KuRobotParameter::getInstance()->getRplidarComport(RplidarCom);	
	RplidarInterface::getInstance()->setComPort(RplidarCom);			
	RplidarInterface::getInstance()->setMaxDistance(KuRobotParameter::getInstance()->getRplidarMaxDist());
	RplidarInterface::getInstance()->setMinDistance(KuRobotParameter::getInstance()->getRplidarMinDist());
	RplidarInterface::getInstance()->connectLaserScanner();
	return true;
}

/**
@brief Korean: Ű��Ʈ ������ �����ϴ� �Լ�
@brief English: write in English
*/
bool SensorSupervisor::connectionKinect()
{
	
	if(KinectSensorInterface::getInstance()->connect())	
	{	
		KinectSensorInterface::getInstance()->start();
		return true;			
	}
	return false;
}

/**
@brief Korean: ���ڴ���  �����ϴ� �Լ�
@brief English: write in English
*/
bool SensorSupervisor::connectionWheelactuator()
{
	char  WheelCom[10];
	KuRobotParameter::getInstance()->getWheelComport(WheelCom);	
	if(PioneerWheelActuatorInterface::getInstance()->connect(WheelCom))
		return true;
	return false;
}
/**
@brief Korean: ��� ������ ������
@brief English: write in English
*/
void SensorSupervisor::stopAllSensor()
{
	HokuyoURG04LXInterface::getInstance()->disconnectLaserScanner();
	KinectSensorInterface::getInstance()->terminate();
}
/**
@brief Korean: ���ڴ� �����͸� ������
@brief English: write in English
*/
KuPose SensorSupervisor::getEncoderDelPos()
{
	return m_EncoderDelPos;
}

/**
@brief Korean: �������� �Ÿ��� �����͸� ������
@brief English: write in English
*/
int_1DArray SensorSupervisor::getURG04LXLaserData()
{
	return m_nURGLaserData;
}
/**
@brief Korean: Ű��Ʈ�� 2D �Ÿ��� �����͸� ������
@brief English: write in English
*/
int_1DArray SensorSupervisor::getKinectRangeData()
{
	return m_nKinectRangeData;
}
/**
@brief Korean: Ű��Ʈ�� 2D �Ÿ����� ���� ���� �����͸� ������
@brief English: write in English
*/
int_1DArray SensorSupervisor::getHeightDataOfKinectRangeData()
{
	return m_nHightDataOfRangeData;
}
/**
@brief Korean: Ű��Ʈ�� ���� �����͸� ������
@brief English: write in English
*/
IplImage* SensorSupervisor::getKinectImageData()
{
	return m_IplKinectImage;
}
/**
@brief Korean: Ű��Ʈ�� ���� �����͸� ������
@brief English: write in English
*/
IplImage* SensorSupervisor::getKinectDepthImageData()
{
	return m_IplKinectDepthImg;
}
/**
@brief Korean: Ű��Ʈ�� 3D �Ÿ� �����͸� ������
@brief English: write in English
*/
KuPose* SensorSupervisor::getKinectDataPos()
{
	return m_pKinectDataPos;
}
/**
@brief Korean: Ű��Ʈ�� 3D �Ÿ� �����͸� ������
@brief English: write in English
*/
float* SensorSupervisor::getKinectDistanceImage()
{
	return m_fKinectDistanceImage;
}

/**
@brief Korean: ��� ������ ���� ���� ������ �޾ƿ�
@brief English: write in English
*/
bool SensorSupervisor::readSensorData()
{	
	
	if(m_bDataPlay == true && m_PlayLog.eof()==true){ return false; } //�ùķ��̼� �����͸� �� �о���.

	//���ڴ� ������ ������ ����----------------------------------------------------
	if( m_bDataPlay == false){ //���� ����Ÿ ���
		
		m_EncoderDelPos= PioneerWheelActuatorInterface::getInstance()->getDelEncoderData();
		
		if(m_bDataRecoding == true){ //���� �����͸� ���ڵ��Ѵ�.
			recordEncoderDelPosData(m_EncoderDelPos);
		}
	}
	else{
		//���ڵ� �� �����͸� ���Ϸκ��� �о�ͼ� �����Ѵ�.
		m_EncoderDelPos = playEncoderDelPosData();
	}

	//-------------------------------------------------------------------------------��

	//������ ������ ������ ����----------------------------------------------------
		if( m_bDataPlay == false){ //���� ����Ÿ ���
			int_1DArray nLaserData = HokuyoURG04LXInterface::getInstance()->getData();
			for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++){
				m_nURGLaserData[i] = nLaserData[i]; //������ ������ ����	
			}
			if(m_bDataRecoding == true){ //���� �����͸� ���ڵ��Ѵ�.
				recordLaserData(m_nURGLaserData);
			}

		}
		else{ //���ڵ� �� �����͸� ���Ϸκ��� �о�ͼ� �����Ѵ�.
			playLaserData(m_nURGLaserData);	
		}
	//������ ������ ������ ����----------------------------------------------------��		
		
		if( m_bDataPlay == false  ){ //���� ����Ÿ ���		

			//������ ������ ������ ����----------------------------------------------------
			int_1DArray nRplidarData = RplidarInterface::getInstance()->getData();
			for(int i=0; i<Sensor::RPLIDAR_DATA_NUM181;i++){
				m_nRplidarData[i] = nRplidarData[i]; //������ ������ ����	
			}
			//������ ������ ������ ����----------------------------------------------------��
		}

		//kinect ���� ������ ������ ����------------------------------------------------------------
		if( m_bDataPlay == false  ){ //���� ����Ÿ ���		

			m_IplKinectImage = KinectSensorInterface::getInstance()->get320ColorImage();
			m_IplKinectDepthImg = KinectSensorInterface::getInstance()->get320DepthImage();
			m_pKinectDataPos = KinectSensorInterface::getInstance()->getGlobal3DPose();
			int_1DArray nKinectRangeData = KinectSensorInterface::getInstance()->getRangeData();
			for(int i=0; i<Sensor::KINECT_SENSOR_FOV;i++){
				m_nKinectRangeData[i] = nKinectRangeData[i]; //kinect range data ����
			}

			int_1DArray nHightDataOfRangeData = KinectSensorInterface::getInstance()->getHightDataOfRangeData(); //�Ÿ����� ���� ���̰� 
			for(int i=0; i<Sensor::KINECT_SENSOR_FOV;i++){
				m_nHightDataOfRangeData[i] = nHightDataOfRangeData[i]; //�Ÿ����� ���� ���̰� ����
			}
			if(m_bDataRecoding == true){ //���� �����͸� ���ڵ��Ѵ�.
				recordKinectRangeData(m_nKinectRangeData);
				recordHightDataOfKinectRangeData(m_nHightDataOfRangeData);
				recordKinectData(m_IplKinectImage, m_IplKinectDepthImg, m_pKinectDataPos);
			}
		}
		else{
			//	m_IplXtionImage = m_IplXtionDepthImg = NULL;
			playKinectData();
			PlayKinectRangeData(m_nKinectRangeData);
			PlayHIghtDataOfKinectRangeData(m_nHightDataOfRangeData);
		}
		//kinect ���� ������ ������ ����------------------------------------------------------------��


	return true;
}

/**
@brief Korean: ��� ������ ���� ���� ������ �޾ƿ�(������ ������ �޾ƿ�)
@brief English: write in English
*/
bool SensorSupervisor::readOnlySensorData()
{	
	//���ڴ� ������ ������ ����----------------------------------------------------
	m_EncoderDelPos= PioneerWheelActuatorInterface::getInstance()->getDelEncoderData();
	//-------------------------------------------------------------------------------��

	//������ ������ ������ ����----------------------------------------------------
		int_1DArray nLaserData = HokuyoURG04LXInterface::getInstance()->getData();
		for(int i=0; i<Sensor::URG04LX_DATA_NUM181;i++){
			m_nURGLaserData[i] = nLaserData[i]; //������ ������ ����	
		}
	//������ ������ ������ ����----------------------------------------------------��

		m_IplKinectImage = KinectSensorInterface::getInstance()->get320ColorImage();
		m_IplKinectDepthImg = KinectSensorInterface::getInstance()->get320DepthImage();
		m_pKinectDataPos = KinectSensorInterface::getInstance()->getGlobal3DPose();
		int_1DArray nKinectRangeData = KinectSensorInterface::getInstance()->getRangeData();
		for(int i=0; i<Sensor::KINECT_SENSOR_FOV;i++){
			m_nKinectRangeData[i] = nKinectRangeData[i]; //kinect range data ����
		}

		int_1DArray nHightDataOfRangeData = KinectSensorInterface::getInstance()->getHightDataOfRangeData(); //�Ÿ����� ���� ���̰� 
		for(int i=0; i<Sensor::KINECT_SENSOR_FOV;i++){
			m_nHightDataOfRangeData[i] = nHightDataOfRangeData[i]; //�Ÿ����� ���� ���̰� ����
		}

	return true;
}

/**
@brief Korean: log ���� �κ��� ���ϴ� �κ��� ���ö� ���� Parsing ���ִ�  �Լ�
@brief English: write in English
*/
void SensorSupervisor::dataParsing()
{
	char cData;	
	while(true){
		m_PlayLog >> cData;	
		if( cData ==':' ){ //������ �Ľ� ����
			break;
		}
	}

}

/**
@brief Korean: log ���Ͽ� ���ڴ� ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordEncoderDelPosData(KuPose EncoderDelPos)
{
	m_DataLog<<endl;
	m_DataLog<<"EncoderDelPos(x,y,deg unit/mm): "<<EncoderDelPos.getX()<<" "<<EncoderDelPos.getY()<<" "<<EncoderDelPos.getThetaDeg()<<endl;
}

/**
@brief Korean: log ���Ϸκ���  ���ڴ� ������ �ҷ����� �Լ�
@brief English: write in English
*/
KuPose SensorSupervisor::playEncoderDelPosData()
{
	KuPose DelEncoderPos; 
	double dX=0.0, dY=0.0,dDeg=0.0;	

	dataParsing();
	m_PlayLog >> dX >> dY >> dDeg;
	DelEncoderPos.setX( dX); 
	DelEncoderPos.setY( dY); 
	DelEncoderPos.setThetaDeg(dDeg);
	return DelEncoderPos;
}
/**
@brief Korean: log ���Ͽ� ������ ������ ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordLaserData(int_1DArray nLaserData)
{
	int i=0;
	m_DataLog<<"Laser data("<<Sensor::URG04LX_DATA_NUM181<<" scan idx unit/mm): ";
	for(i=0; i<Sensor::URG04LX_DATA_NUM181 -1 ; i++){
		m_DataLog<<nLaserData[i]<<" ";
	}
	m_DataLog<<nLaserData[i]<<endl;
}
/**
@brief Korean: log ���Ϸκ���  ������ ������ ������ �ҷ����� �Լ�
@brief English: write in English
*/
void SensorSupervisor::playLaserData(int_1DArray nData)
{
	dataParsing();
	for(int i=0; i<Sensor::URG04LX_DATA_NUM181; i++){
		m_PlayLog >> m_nURGLaserData[i];
	}
}
/**
@brief Korean: log ���Ͽ� Ű��Ʈ ������ ������ ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordKinectRangeData(int_1DArray nKinectRangeData)
{
	int i=0;
	m_DataLog<<"Kinect range data("<<Sensor::KINECT_SENSOR_FOV<<" scan idx unit/mm): ";
	for(i=0; i<Sensor::KINECT_SENSOR_FOV-1; i++){
		m_DataLog<<nKinectRangeData[i]<<" ";
	}
	m_DataLog<<nKinectRangeData[i]<<endl;
}
/**
@brief Korean: log ���Ͽ� Ű��Ʈ ������ �������� ���� ���� ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordHightDataOfKinectRangeData(int_1DArray nHightDataOfKinectRangeData)
{
	int i=0;
	m_DataLog<<"Kinect Hight data("<<Sensor::KINECT_SENSOR_FOV<<" scan idx unit/mm): ";
	for(i=0; i<Sensor::KINECT_SENSOR_FOV-1; i++){
		m_DataLog<<nHightDataOfKinectRangeData[i]<<" ";
	}
	m_DataLog<<nHightDataOfKinectRangeData[i];
}

/**
@brief Korean: Ű��Ʈ ������  ������ �����ϴ� �Լ�
@brief English: write in English
*/
void SensorSupervisor::recordKinectData(IplImage* IplKinectImage, IplImage* IplKinectDepthImg, KuPose* pKinectDataPos)
{
	
	char cFilePathName[100];
	char cFilePathName2[100];
	memset(cFilePathName,0,sizeof(cFilePathName));
	memset(cFilePathName2,0,sizeof(cFilePathName2));
	sprintf_s(cFilePathName,"./Data/log/kinect/Image/%d.jpg",m_KinectLogIdx);
//	sprintf_s(cFilePathName2,"./Data/log/kinect/DisparityImage/%d.jpg",m_KinectLogIdx);
	cvSaveImage(cFilePathName,IplKinectImage);
	//cvSaveImage(cFilePathName2,IplKinectDepthImg);


	// 	//////////////////////////////////////////////////////////////////////////
	// 	//3���� ������ ����
	// 	//////////////////////////////////////////////////////////////////////////
	char c1[50], c2[50], c3[50], c4[100], c5[100], c6[100];
	sprintf_s(c1, "./Data/log/kinect/3D/x/%d.log", m_KinectLogIdx);
	sprintf_s(c2, "./Data/log/kinect/3D/y/%d.log", m_KinectLogIdx);
	sprintf_s(c3, "./Data/log/kinect/3D/z/%d.log", m_KinectLogIdx);
	sprintf_s(c4, "./Data/log/kinect/3D/pix_x/%d.log", m_KinectLogIdx);
	sprintf_s(c5, "./Data/log/kinect/3D/pix_y/%d.log", m_KinectLogIdx);
	sprintf_s(c6, "./Data/log/kinect/3D/validation/%d.log", m_KinectLogIdx);

	int nSize = Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT;
	for(int i=0; i< nSize; i++){
		m_d3DKinectX[i] = pKinectDataPos[i].getX();
		m_d3DKinectY[i] = pKinectDataPos[i].getY();
		m_d3DKinectZ[i] = pKinectDataPos[i].getZ();
		m_n3DKinectPixX[i] = pKinectDataPos[i].getPixX();
		m_n3DKinectPixY[i] = pKinectDataPos[i].getPixY();
		m_n3DKinectDataValidation[i] = pKinectDataPos[i].getID(); 

	}

	FILE *fp_3D_X, *fp_3D_Y, *fp_3D_Z, *fp_3D_Pix_X, *fp_3D_Pix_Y, *fp_3D_Validation;
	fp_3D_X = fopen (c1,"wb");
	fp_3D_Y = fopen (c2,"wb");
	fp_3D_Z = fopen (c3,"wb");
	fp_3D_Pix_X = fopen (c4,"wb");
	fp_3D_Pix_Y = fopen (c5,"wb");
	fp_3D_Validation = fopen (c6,"wb");

	fwrite(m_d3DKinectX,sizeof(double),nSize,fp_3D_X);
	fwrite(m_d3DKinectY,sizeof(double),nSize,fp_3D_Y);
	fwrite(m_d3DKinectZ,sizeof(double),nSize,fp_3D_Z);
	fwrite(m_n3DKinectPixX,sizeof(int),nSize,fp_3D_Pix_X);
	fwrite(m_n3DKinectPixY,sizeof(int),nSize,fp_3D_Pix_Y);
	fwrite(m_n3DKinectDataValidation,sizeof(int),nSize,fp_3D_Validation);


	//////////////////////////////////////////////////////////////////////////
	fclose	(fp_3D_X); fclose (fp_3D_Y); fclose (fp_3D_Z); 
	fclose(fp_3D_Pix_X); fclose(fp_3D_Pix_Y); fclose(fp_3D_Validation);

	m_KinectLogIdx++;
}

/**
@brief Korean:  Ű��Ʈ ������ ������ �ҷ����� �Լ�
@brief English: write in English
*/
void SensorSupervisor::playKinectData()
{
	char cFilePathName[100];
	char cFilePathName2[100];
	memset(cFilePathName,0,sizeof(cFilePathName));
	memset(cFilePathName2,0,sizeof(cFilePathName2));
	sprintf_s(cFilePathName,"./Data/log/kinect/Image/%d.jpg",m_KinectLogIdx);
	sprintf_s(cFilePathName2,"./Data/log//kinect/DisparityImage/%d.jpg",m_KinectLogIdx);

	m_IplKinectImage = cvLoadImage(cFilePathName, CV_LOAD_IMAGE_COLOR);
	m_IplKinectDepthImg = cvLoadImage(cFilePathName2, CV_LOAD_IMAGE_COLOR);
	cvFlip(m_IplKinectImage,m_IplKinectImage,-1);	
	cvFlip(m_IplKinectImage,m_IplKinectImage,1);	

	//////////////////////////////////////////////////////////////////////////
	//3���� ������ �ε�
	//////////////////////////////////////////////////////////////////////////

	char c1[50], c2[50], c3[50], c4[100], c5[100], c6[100];
	sprintf_s(c1, "./Data/log/kinect/3D/x/%d.log", m_KinectLogIdx);
	sprintf_s(c2, "./Data/log/kinect/3D/y/%d.log", m_KinectLogIdx);
	sprintf_s(c3, "./Data/log/kinect/3D/z/%d.log", m_KinectLogIdx);
	sprintf_s(c4, "./Data/log/kinect/3D/pix_x/%d.log", m_KinectLogIdx);
	sprintf_s(c5, "./Data/log/kinect/3D/pix_y/%d.log", m_KinectLogIdx);
	sprintf_s(c6, "./Data/log/kinect/3D/validation/%d.log", m_KinectLogIdx);


	FILE *fp_3D_X, *fp_3D_Y, *fp_3D_Z, *fp_3D_Pix_X, *fp_3D_Pix_Y, *fp_3D_Validation;
	fp_3D_X = fopen (c1,"rb");
	fp_3D_Y = fopen (c2,"rb");
	fp_3D_Z = fopen (c3,"rb");
	fp_3D_Pix_X = fopen (c4,"rb");
	fp_3D_Pix_Y = fopen (c5,"rb");
	fp_3D_Validation = fopen (c6,"rb");

	int nSize = Sensor::IMAGE_WIDTH*Sensor::IMAGE_HEIGHT;
	fread(m_d3DKinectX,sizeof(double),nSize,fp_3D_X);
	fread(m_d3DKinectY,sizeof(double),nSize,fp_3D_Y);
	fread(m_d3DKinectZ,sizeof(double),nSize,fp_3D_Z);
	fread(m_n3DKinectPixX,sizeof(int),nSize,fp_3D_Pix_X);
	fread(m_n3DKinectPixY,sizeof(int),nSize,fp_3D_Pix_Y);
	fread(m_n3DKinectDataValidation,sizeof(int),nSize,fp_3D_Validation);

	for(int i=0; i< nSize; i++){
		m_pKinectDataPos[i].setX(m_d3DKinectX[i]);
		m_pKinectDataPos[i].setY(m_d3DKinectY[i]);
		m_pKinectDataPos[i].setZ(m_d3DKinectZ[i]);
		m_pKinectDataPos[i].setPixX(m_n3DKinectPixX[i]);
		m_pKinectDataPos[i].setPixY(m_n3DKinectPixY[i]);
		m_pKinectDataPos[i].setID(m_n3DKinectDataValidation[i]);
		m_fKinectDistanceImage[i]=pow(m_d3DKinectX[i]*m_d3DKinectX[i]+m_d3DKinectY[i]*m_d3DKinectY[i]+m_d3DKinectZ[i]*m_d3DKinectZ[i] , 1/3.0 )/100;
	}

	/////////////////////////////////////////////////////////////////////
	fclose(fp_3D_X);
	fclose(fp_3D_Y);
	fclose(fp_3D_Z);
	fclose(fp_3D_Pix_X);
	fclose(fp_3D_Pix_Y); 
	fclose(fp_3D_Validation); 

	m_KinectLogIdx++;
}
/**
@brief Korean: log ���Ϸκ���  Ű��Ʈ ������ �������� ���� ���� ������ �ҷ����� �Լ�
@brief English: write in English
*/
void SensorSupervisor::PlayHIghtDataOfKinectRangeData(int_1DArray nHightDataOfKinectRangeData)
{
	dataParsing();
	for(int i=0; i<Sensor::KINECT_SENSOR_FOV; i++){
		m_PlayLog >> nHightDataOfKinectRangeData[i];
	}
}
/**
@brief Korean: log ���Ϸκ���  Ű��Ʈ ������ ������ ������ �ҷ����� �Լ�
@brief English: write in English
*/
void SensorSupervisor::PlayKinectRangeData(int_1DArray nKinectRangeData)
{
	dataParsing();
	for(int i=0; i<Sensor::KINECT_SENSOR_FOV; i++){
		m_PlayLog >> nKinectRangeData[i];
	}
}

/**
@brief Korean: �������� �Ÿ��� �����͸� ������
@brief English: write in English
*/
int_1DArray SensorSupervisor::getRplidarData()
{
	return m_nRplidarData;
}