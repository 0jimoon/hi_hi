/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2011 KUME Intelligent Robotics Lab.                                             
All rights reserved.

$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park, jtpark1114@gmail.com                                                                  
$Description : ����ý����� ��� parameter���� ����ϴ� ������Ʈ. �̱������� �ۼ��Ǿ� ����.
$Data: 2011/10                                   
______________________________________________________________________________________________*/

#ifndef KUNS_ROBOT_PARAMETER_H
#define KUNS_ROBOT_PARAMETER_H

#include <conio.h>
#include <iostream>
#include "../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "KuCommandMessage.h"

using namespace std;
class KuRobotParameter : public KuSingletone <KuRobotParameter>
{

private:
	KuINIReadWriter* m_pINIReaderWriter;

	string m_strMapNameNPath ;
	string m_strVelocityMapNameNPath ;
	string m_strCeilingMapNameNPath ;
	string m_strCadMapNameNPath ;
	string m_strZoneMapNameNPath ;

	string m_strMovieNameNPath ;
	string m_strProMapNameNPath ;

	string m_strTeachingPathNameNPath ;
	string m_strImagePathNameNPath;
	string m_strTeachingWayPointNameNPath;
	string m_strOutlineWayPointNameNPath;
	string m_strPathteaching;

	string m_strGlobalLocalization;
	string m_strVelocityMap;	
	int m_nMapSizeXm ; //���� x,y ũ�� ����: m�� ����.
	int m_nMapSizeYm ;
	double m_dHeight;
	int m_nTotalfloorNum ;
	int m_nCurfloor;


	int m_nRobotID ;
	int m_nRadiusofRobot; //�κ� �ݰ�, ��ֹ� ȸ�ǵ ���ȴ�. ����: mm 
	int m_nWheelBaseofRobot ;
	int m_nMaxRobotVelocity ;//�κ��� �ִ� �ӵ�, �κ��� �ּ� �ӵ�.
	int m_nMinRobotVelocity ;
	KuPose m_initRobotPos;
	int m_nLocalization;
	int m_nObstacleDetectionTime;
	string m_strLastRobotPoseNameNPath;

	string m_strDataPath ;
	string m_strDataRecoding ;
	string m_strDataPlay ;

	char m_cWheelCom[10];
	char  m_cHokuyoURG04LXCom[10];
	char  m_cCommunicationCom[10];
	char  m_cRplidarCom[10];

	string m_strdoSonar ;
	int m_nLaserTCPPort;

	//INI ���Ϸκ��� kanayama motion contro���� �����ϱ� ���� �ʿ��� ������ ���´�.------------------------
	int m_nDistToTarget;
	int m_nDesiredVel ;
	int m_nGoalArea;
	double m_dKX ;
	double m_dKY ;
	double m_dKT ;
	int m_nDirectionofRotation;
	//========================================================================================


	//INI ���Ϸκ��� Particle filter�� �����ϱ� ���� �ʿ��� ������ ���´�.------------------------------------
	int m_nMaxParticleNum;  //�ִ�, �ּ� particle ����. �κ��� ��ġ������ ���ȴ�.
	int m_nMinParticleNum ; 
	double m_dDevationforTrans ;
	double m_dDevationforRotate ;
	double m_dDevationforTransRotate ;
	//=========================================================================================

	//INI ���Ϸκ���------------------------------------
	int m_nFeatureTh; // ��Ī�� �˻��ϴ� �ּҰ� . ���� ���� ����Ȯ����. ���� ���� ����//80
	int m_nSiftMatchingTh;// ��Ī�Ǵ� Ư¡���� �ּ� ����//5
	double m_dNumSIFTFeatureTh;//Ư¡���� ������ ��������ִ� ���� �������� ���� ������  ��������
	int m_nNumSURFFeatureTh;//SURF Ư¡���� ������ ��������ִ� ���� �������� ���� ������ ��������
	int m_nInteractionPointTh;// �� �̹����� �� �̹��� ���� ���絵 ����
	double m_dMatchingAngleTh;// ���� ����� ������ ������ ��Ī�Ǵ� Ư¡������ ����.
	string m_strSIFTDataNamePath;
	string m_strSURFDataNamePath;
	int m_nDistnaceFromPath;
	double m_dEllipseHeight;
	double m_dEllipseWidth;
	//=========================================================================================

	//laser ���� �Ķ����----------------------------------------------------------------------------------
	int m_nURG04LX_LaserMaxDist; //��� �������� �ִ� Ž���Ÿ��� �����ϴ� ����. mm����
	int m_nURG04LX_LaserMinDist; //��� �������� �ּ� Ž���Ÿ��� �����ϴ� ����. mm����
	int m_nURG04LX_LaserHeight; //ƿƮ ������ ���� ��� ������������ ���̰��� �����ϴ� ����. mm����
	int m_nURG04LX_LaserXOffset; //ƿƮ ������ ���� ��� ������������ x offset�� �����ϴ� ���� mm����
	int m_nURG04LX_LaserYOffset; //ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� ���� mm����
	//laser ���� �Ķ���� ���� ��===========================================================
	int m_nRplidar_MaxDist; //��� �������� �ִ� Ž���Ÿ��� �����ϴ� ����. mm����
	int m_nRplidar_MinDist; //��� �������� �ּ� Ž���Ÿ��� �����ϴ� ����. mm����
	int m_nRplida_Height; //ƿƮ ������ ���� ��� ������������ ���̰��� �����ϴ� ����. mm����
	int m_nRplida_XOffset; //ƿƮ ������ ���� ��� ������������ x offset�� �����ϴ� ���� mm����
	int m_nnRplida_YOffset; //ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� ���� mm����


	//Kinect ���� �Ķ����----------------------------------------------------------------------------------
	int m_nKinectMaxDist; //Ű��Ʈ�� �ִ� Ž���Ÿ��� �����ϴ� ����. mm����
	int m_nKinectMinDist; //Ű��Ʈ�� �ּ� Ž���Ÿ��� �����ϴ� ����. mm����
	int m_nKinectHeight; //�ٴڿ��� Ű��Ʈ������ ���̰��� �����ϴ� ����. mm����
	int m_nKinectXOffset; //�κ��� �߽ɿ��� ���� Ű��Ʈ������ x offset�� �����ϴ� ���� mm����
	int m_nKinectYOffset; //�κ��� �߽ɿ��� ���� Ű��Ʈ������ y offset�� �����ϴ� ���� mm����
	int m_nKinectMaxHeightDist;
	int m_nKinectMinHeightDist;
	//Kinect ���� �Ķ���� ���� ��===========================================================

	//ALRecognizer ���� �Ķ����------------------------------------------------------------------
	string m_strAlFeatureMapNameNPath ;
	int m_nLandMarkNum; //���帶ũ�� ����
	int m_nHeight_Camera2Mark; //ī�޶�κ��� ���帶ũ������ ����
	//ALRecognizer ���� �Ķ���� ���� ��===========================================================

public:
	bool initialize(); //�ý��� ������ ���õ� ��� �������� �ʱ�ȭ �����ִ� �Լ�.
	void saveParameter();

	//set �Լ����-----------------------------------------------------------------------------------------------------------------------
	//robot ���� �Ķ����
	void setRobotID(int nID); //�κ��� ID�� �������ִ� �Լ�.
	void setRobotRadius(int nRadius); //�κ� �ݰ��� �������ִ� �Լ�. ������ mm
	void setMaxRobotVelocity(int nMaxVelocity); //�κ��� �ִ�ӵ��� �����ϴ� �Լ�.
	void setMinRobotVelocity(int nMinVelocity); //�κ��� �ּҼӵ��� �����ϴ� �Լ�.
	void setGlobalLocalization(string sGlobalLocalization);
	void setInitRobotPose(KuPose initRobotPos);
	void setWheelBaseofRobot(int nWheelBaseofRobot);
	void setLocalization(int nLocalization);
	void setLastRobotPoseNameNPath(string strLastRobotPoseNameNPath);
	void setObstacleDetectionTime(int nObstacleDetectionTime);

	//Map ���� �Ķ����
	void setMapSize(int nSizeXm, int nSizeYm); //����ũ�⸦ �����ϴ� �Լ�.
	void setMapNameNPath(string sMapName) ;//�ۼ��� ������ �̸�.
	void setVelocityMapNameNPath(string sMapName); 
	void setCeilingMapNameNPath(string sMapName) ;
	void setCadMapNameNPath(string sMapName) ;
	void setZoneMapNameNPath(string sMapName); 
	void setTotalfloorNum(int nTotalfloorNum);
	void setCurfloor(int nCurfloor);

	void setHeight(double dHeight );

	void setMovieNameNPath(string sMapName);
		void setProMapNameNPath(string sMapName); 

	void setTeachingPathNameNPath(string sMapName);
	void setImagePathNameNPath(string sMapName) ;
	void setTeachingWayPointNameNPath(string sMapName); 
	void setOutlineWayPointNameNPath(string sMapName); 
	void setPathteaching(string strPathteaching); 


	//Sensor ���� �Ķ����
	void setDataPath(string sDataPath) ;
	void setDataRecoding(string sDataRecoding) ;
	void setDataPlay(string sDataPlay) ;


	void setURG04LXLaserComport(char cComport[10]); //URG04LX laser comport�� �����ϴ� �Լ�.
	void setWheelComport(char cComport[10]); //wheel comport�� �����ϴ� �Լ�.
	void setCommunicationComport(char cComport[10]);
	void setdoSonar(string strdoSonar);
	void setLaserTCPPort(int nLaserTCPPort );


	//laser ���� �Ķ����
	void setURG04LXLaserMaxDist(int nMaxDist); // �������� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	void setURG04LXLaserMinDist(int nMinDist); // �������� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	void setURG04LXLaserHeight(int nHeight); // �ٴڿ��� ������������ ���̰��� �����ϴ� �Լ�.
	void setURG04LXLaserXOffset(int nXOffset); // �κ��� �߽ɿ��� ������������ x offset�� �����ϴ� �Լ�.
	void setURG04LXLaserYOffset(int nYOffset); //�κ��� �߽ɿ��� ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� �Լ�.


	//���̴�
	void setRplidarComport(char cComport[10]);
	void setRplidarMaxDist(int nMaxDist);
	void setRplidarMinDist(int nMinDist);
	void setRplidarHeight(int nHeight);
	void setRplidarXOffset(int nXOffset);
	void setRplidarYOffset(int nYOffset);


	//Kinect ���� �Ķ����
	void setKinectMaxDist(int nMaxDist); //Ű��Ʈ�� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	void setKinectMinDist(int nMinDist); //Ű��Ʈ�� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	void setKinectHeight(int nHeight); //�ٴڿ��� Ű��Ʈ������ ���̰��� �����ϴ� �Լ�.
	void setKinectXOffset(int nXOffset); //�κ��� �߽ɿ��� Ű��Ʈ������ x offset�� �����ϴ� �Լ�.
	void setKinectYOffset(int nYOffset); //�κ��� �߽ɿ��� Ű��Ʈ������ y offset�� �����ϴ� �Լ�.	
	void setKinectMaxHeightDist(int nMaxDist); //Ű��Ʈ�� �ִ� ���̰Ÿ��� �����ϴ� �Լ�.
	void setKinectMinHeightDist(int nMinDist); //Ű��Ʈ�� �ּ� ���̰Ÿ��� �����ϴ� �Լ�.

	//õ��ī�޶� ���� �Ķ����
	void setCeilingCameraPrameterFx(double dFx);
	void setCeilingCameraPrameterFy(double dFy);
	void setCeilingCameraPrameterCx(double dCx);
	void setCeilingCameraPrameterCy(double dCy);
	void setCeilingCameraPrameterD1(double dD1);
	void setCeilingCameraPrameterD2(double dD2);
	void setCeilingCameraPrameterD3(double dD3);
	void setCeilingCameraPrameterD4(double dD4);
	void setCeilingCameraPrameterOffsetX(double doffsetx);


	//���� ���ɰ� ������ parameter ���� �Լ���-------------------------------------------
	void setTargetDistance(int nDistToTarget);
	void setDesiedVel(int nDesiredVel);
	void setGoalArea(int nGoalArea);
	void setdKX(double  dKX);
	void setdKY(double  dKY);
	void setdKT(double  dKT);
	void setDirectionofRotation(int nDirectionofRotation);

	void setMaxParticleNum(int nMaxNum); //�ִ� particle ������ �����ϴ� �Լ�.
	void setMinParticleNUm(int nMinNUm); //�ּ� particle ������ �����ϴ� �Լ�.
	void setDeviationforTrans(double  dDevationforTrans);
	void setDeviationforRotae(double  dDevationforRotate);
	void setDeviationforTransRotae(double  dDevationforTransRotate);
	void setFeatureTh(int nFeatureTh);
	void setMatchingTh(int nMatchingTh);
	void setNumSIFTFeatureTh(double dNuMFeatureTh);
	void setInteractionPointTh(int nInteractionPointTh);
	void setMatchingAngleTh(double dMatchingAngleTh);

	void setSIFTDataNamePath(string strSIFTDataNamePath) ;
	void setSURFDataNamePath(string strSURFDataNamePath) ;
	void setDistanceFromPath(int nDistnaceFromPath);
	void setNumSURFFeatureTh(int nSURFKeypointTh);

	void setEllipseWidth(double dEllipseWidth);
	void setEllipseHeight(double dEllipseHeight);


	//ALRecognizer ���� �Ķ����
	void setAlFeatureMapNameNPath(string sAlFeatureMapNameNPath) ;
	void setLandMarkNum(int nLandMarkNum);
	void setHeight_Camera2Mark(int nHeight_Camera2Mark);

	//=====================================================================================================================================
	//=====================================================================================================================================


	//get �Լ����--------------------------------------------------------------------------------------------------------------------------

	//robot ���� �Ķ����
	int getRobotID(); //�κ��� ID�� �������ִ� �Լ�.
	int getRobotRadius(); //�κ� �ݰ��� �������ִ� �Լ�. ������ mm
	int getMaxRobotVelocity(); //�κ��� �ִ�ӵ��� �����ϴ� �Լ�.
	int getMinRobotVelocity(); //�κ��� �ּҼӵ��� �����ϴ� �Լ�.
	string getGlobalLocalization( );
	KuPose getInitRobotPose();
	int getWheelBaseofRobot();
	int getLocalization();
	string getLastRobotPoseNameNPath( );
	int getObstacleDetectionTime();
	int getTotalfloorNum();
	int getCurfloor();

	//Map ���� �Ķ����
	int getMapSizeXm(); //����ũ�⸦ �����ϴ� �Լ�.
	int getMapSizeYm(); //����ũ�⸦ �����ϴ� �Լ�.
	string getMapNameNPath( ); //�强�� ������ �̸��� �Ѱ��ش�.
	string getVelocityMapNameNPath(	);
	string getCeilingMapNameNPath(	);
	string getCadMapNameNPath( ); 
	string getZoneMapNameNPath( ); 
	string getProMapNameNPath(	); 

	string getTeachingPathNameNPath() ;
	string getImagePathNameNPath() ;
	string getTeachingWayPointNameNPath(); 
	string getOutlineWayPointNameNPath(	); 
	string getPathteaching(); 

	double getHeight();

	string getMovieNameNPath();

	//Sensor ���� �Ķ����
	string getDataPath( ) ;
	string getDataRecoding( ) ;
	string getDataPlay( ) ;

	void getURG04LXLaserComport(char wcComport[10]); // laser comport�� �Ѱ��ش�.  
	void getWheelComport(char cComport[10]); //wheel comport�� �Ѱ��ش�. 
	void getCommunicationComport(char cComport[10]);
	string getdoSonar( );
	int getLaserTCPPort();


	//laser ���� �Ķ����
	int getURG04LXLaserMaxDist(); //��� �������� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	int getURG04LXLaserMinDist(); //��� �������� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	int getURG04LXLaserHeight(); //ƿƮ ������ ���� ��� ������������ ���̰��� �����ϴ� �Լ�.
	int getURG04LXLaserXOffset(); //ƿƮ ������ ���� ��� ������������ x offset�� �����ϴ� �Լ�.
	int getURG04LXLaserYOffset(); //ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� �Լ�.	


	//laser ���� �Ķ����
	void getRplidarComport(char wcComport[10]); // laser comport�� �Ѱ��ش�.  
	int getRplidarMaxDist(); //��� �������� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	int getRplidarMinDist(); //��� �������� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	int getRplidaHeight(); //ƿƮ ������ ���� ��� ������������ ���̰��� �����ϴ� �Լ�.
	int getRplidarXOffset(); //ƿƮ ������ ���� ��� ������������ x offset�� �����ϴ� �Լ�.
	int getRplidaYOffset(); //ƿƮ ������ ���� ��� ������������ y offset�� �����ϴ� �Լ�.	

	//Kinect ���� �Ķ����
	int getKinectMaxDist(); //Ű��Ʈ�� �ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	int getKinectMinDist(); //Ű��Ʈ�� �ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	int getKinectHeight(); //�ٴڿ��� Ű��Ʈ������ ���̰��� �����ϴ� �Լ�.
	int getKinectXOffset(); //�κ��� �߽ɿ��� Ű��Ʈ������ x offset�� �����ϴ� �Լ�.
	int getKinectYOffset(); //�κ��� �߽ɿ��� Ű��Ʈ������ y offset�� �����ϴ� �Լ�.	
	int getKinectMaxHeightDist();
	int getKinectMinHeightDist( );

	//���� ���ɰ� ������ parameter ���� �Լ���-------------------------------------------

	int getTargetDistance();
	int getDesiedVel( );
	int getGoalArea( );
	double getdKX(  );
	double getdKY(  );
	double getdKT(  );
	int getDirectionofRotation();

	int getMaxParticleNum(); //�ִ� particle ������ �����ϴ� �Լ�.
	int getMinParticleNUm(); //�ּ� particle ������ �����ϴ� �Լ�.
	double getDeviationforTrans(  );
	double getDeviationforRotate(  );
	double getDeviationforTransRotate(  );
	int getFeatureTh( );
	int getMatchingTh( );
	double getNumSIFTFeatureTh( );
	int getNumSURFFeatureTh();
	int getInteractionPointTh( );
	double getMatchingAngleTh( );	
	string getSIFTDataNamePath();
	string getSURFDataNamePath();
	int getDistanceFromPath();

	double getEllipseWidth( );
	double getEllipseHeight( );

	//ALRecognizer ���� �Ķ���� �Լ���
	int getLandMarkNum();
	int getHeightCamera2Mark();
	string getAlFeatureMapNameNPath( ) ;

	//======================================================================================================================================
	//======================================================================================================================================


public:
	KuRobotParameter();
	virtual ~KuRobotParameter();
};

#endif

