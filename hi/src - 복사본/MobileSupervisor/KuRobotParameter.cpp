#include "stdafx.h"
#include "KuRobotParameter.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#endif

KuRobotParameter::KuRobotParameter()
{

}

KuRobotParameter::~KuRobotParameter()
{

}

bool  KuRobotParameter::initialize()
{	
	string strInIFileName = "./ini/KUNS.ini";  	//기본지도 이름은 kuns.ini파일에 명시되어 있다. 
	m_pINIReaderWriter = new KuINIReadWriter(strInIFileName); //설정파일 읽기

	if (m_pINIReaderWriter->ParseError() < 0) { //파일을 읽지 못한 경우 프로그램을 종료시킨다.
		cout << "Can't load "<<strInIFileName<<endl;;
		_getch();
		return false;
	}

	const char* constch;	
	m_nTotalfloorNum = m_pINIReaderWriter->getIntValue("MAP", "FLOOR", 1);
	m_nCurfloor= m_pINIReaderWriter->getIntValue("MAP", "CURRENT_FLOOR", 1);
	m_strMapNameNPath = m_pINIReaderWriter->getStringValue("MAP", "PATH&NAME", "no");
	m_strVelocityMapNameNPath = m_pINIReaderWriter->getStringValue("MAP", "VELOCITY_PATH&NAME", "no");
	m_strCeilingMapNameNPath = m_pINIReaderWriter->getStringValue("MAP", "CEILINGMAP_PATH&NAME", "no");
	m_strCadMapNameNPath = m_pINIReaderWriter->getStringValue("MAP", "CADMAP_PATH&NAME", "no");
	m_strZoneMapNameNPath = m_pINIReaderWriter->getStringValue("MAP", "ZONEMAP_PATH&NAME", "no");
	m_strProMapNameNPath = m_pINIReaderWriter->getStringValue("MAP", "PATH&PRONAME", "no");

	m_nMapSizeXm =	m_pINIReaderWriter->getIntValue("MAP","MAP_SIZE_X",0);
	m_nMapSizeYm =	m_pINIReaderWriter->getIntValue("MAP","MAP_SIZE_Y",0);
	m_dHeight = m_pINIReaderWriter->getDoubleValue("MAP","HEIGHT",1.75);

	m_strMovieNameNPath = m_pINIReaderWriter->getStringValue("MOVIE", "MOVIE&PATH&NAME", "no");
	m_strTeachingPathNameNPath = m_pINIReaderWriter->getStringValue("PATH", "TEACHINGPATH_PATH&NAME", "no");
	m_strImagePathNameNPath = m_pINIReaderWriter->getStringValue("PATH", "IMAGEPATH_PATH&NAME", "no");
	m_strTeachingWayPointNameNPath = m_pINIReaderWriter->getStringValue("PATH", "TEACHINGWAYPOINT_PATH&NAME", "no");
	m_strOutlineWayPointNameNPath = m_pINIReaderWriter->getStringValue("PATH", "OUTLINEWAYPOINT_PATH&NAME", "no");
	m_strPathteaching = m_pINIReaderWriter->getStringValue("PATH", "PATHTEACHING", "no");

	m_nRobotID = m_pINIReaderWriter->getIntValue("ROBOT","ROBOT_ID",0);
	m_nRadiusofRobot = m_pINIReaderWriter->getIntValue("ROBOT","ROBOT_RADIUS",0);
	m_nWheelBaseofRobot = m_pINIReaderWriter->getIntValue("ROBOT","ROBOT_WHEEL_BASE",0);
	m_nMaxRobotVelocity = m_pINIReaderWriter->getIntValue("ROBOT","MAX_VELOCITY",0);
	m_nMinRobotVelocity = m_pINIReaderWriter->getIntValue("ROBOT","MIN_VELOCITY",0);
	m_strGlobalLocalization = m_pINIReaderWriter->getStringValue("ROBOT", "GLOBAL_LOCALIZATION", "no");
	m_initRobotPos.setX(m_pINIReaderWriter->getDoubleValue("ROBOT","INIT_XPOSE",0.0));
	m_initRobotPos.setY(m_pINIReaderWriter->getDoubleValue("ROBOT","INIT_YPOSE",0.0));
	m_strLastRobotPoseNameNPath = m_pINIReaderWriter->getStringValue("ROBOT", "LASTROBOTPOSE_PATH&NAME", "no");
	m_initRobotPos.setThetaDeg(m_pINIReaderWriter->getDoubleValue("ROBOT","INIT_THETADEG",0.0));
	m_nObstacleDetectionTime = m_pINIReaderWriter->getIntValue("ROBOT","OBSTACLEDETECTION_TIME",3);
	m_nLocalization = m_pINIReaderWriter->getIntValue("ROBOT","LOCALIZATION",0);




	m_strDataPath = m_pINIReaderWriter->getStringValue("SENSOR", "DATA_PATH", "no");
	m_strDataRecoding = m_pINIReaderWriter->getStringValue("SENSOR", "DATA_RECODING", "no");
	m_strDataPlay = m_pINIReaderWriter->getStringValue("SENSOR", "DATA_PLAY", "no");

	string strWheelCom= m_pINIReaderWriter->getStringValue("SENSOR", "WHEEL_ACTUATOR", "no");
	constch=strWheelCom.c_str();
	strcpy(m_cWheelCom,constch);

	string strHokuyoURG04LXCom= m_pINIReaderWriter->getStringValue("SENSOR", "URG04LX_LASER", "no");
	constch=strHokuyoURG04LXCom.c_str();
	strcpy(m_cHokuyoURG04LXCom,constch);

	m_nLaserTCPPort = m_pINIReaderWriter->getIntValue("SENSOR","LASER_CONNECTION_PORT",0);


	string strCommunicationCom= m_pINIReaderWriter->getStringValue("SENSOR", "COMMUNICATION", "no");
	constch=strCommunicationCom.c_str();
	strcpy(m_cCommunicationCom,constch);

	m_strdoSonar = m_pINIReaderWriter->getStringValue("SENSOR", "SONAR", "no");


	m_nURG04LX_LaserMaxDist = m_pINIReaderWriter->getIntValue("SENSOR","URG04LX_LASER_MAX_DISTANCE",0);
	m_nURG04LX_LaserMinDist = m_pINIReaderWriter->getIntValue("SENSOR","URG04LX_LASER_MIN_DISTANCE",0); 
	m_nURG04LX_LaserHeight = m_pINIReaderWriter->getIntValue("SENSOR","URG04LX_LASER_HEIGHT",0);
	m_nURG04LX_LaserXOffset = m_pINIReaderWriter->getIntValue("SENSOR","URG04LX_LASER_XOFFSET",0);
	m_nURG04LX_LaserYOffset = m_pINIReaderWriter->getIntValue("SENSOR","URG04LX_LASER_YOFFSET",0); 

	m_nKinectMaxDist = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_MAX_DISTANCE",0);
	m_nKinectMinDist = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_MIN_DISTANCE",0); 
	m_nKinectHeight = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_HEIGHT",0);
	m_nKinectXOffset = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_XOFFSET",0);
	m_nKinectYOffset = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_YOFFSET",0); 
	m_nKinectMaxHeightDist = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_MAX_HEIGHT_DISTANCE",0);
	m_nKinectMinHeightDist = m_pINIReaderWriter->getIntValue("SENSOR","KINECT_MIN_HEIGHT_DISTANCE",0); 
	
	string strRplidarCom= m_pINIReaderWriter->getStringValue("SENSOR", "RPLIDAR", "no");
	constch=strRplidarCom.c_str();
	strcpy(m_cRplidarCom,constch);

	m_nRplidar_MaxDist = m_pINIReaderWriter->getIntValue("SENSOR","RPLIDAR_MAX_DISTANCE",0);
	m_nRplidar_MinDist = m_pINIReaderWriter->getIntValue("SENSOR","RPLIDAR_MIN_DISTANCE",0); 
	m_nRplida_Height = m_pINIReaderWriter->getIntValue("SENSOR","RPLIDAR_HEIGHT",0);
	m_nRplida_XOffset = m_pINIReaderWriter->getIntValue("SENSOR","RPLIDAR_XOFFSET",0);
	m_nnRplida_YOffset = m_pINIReaderWriter->getIntValue("SENSOR","RPLIDAR_YOFFSET",0); 


	//INI 파일로부터 kanayama motion controㅣ을 수행하기 위해 필요한 정보를 얻어온다.------------------------------------
	m_nDistToTarget = m_pINIReaderWriter->getIntValue("KANAYAMA_MOTION_CONTROL","DISTANCE_TO_TARGET",0);
	m_nDesiredVel = m_pINIReaderWriter->getIntValue("KANAYAMA_MOTION_CONTROL","DESIRED_VELOCITY",0); 
	m_nGoalArea = m_pINIReaderWriter->getIntValue("KANAYAMA_MOTION_CONTROL","GOAL_AREA",0);
	m_dKX = m_pINIReaderWriter->getDoubleValue("KANAYAMA_MOTION_CONTROL","X_GAIN");
	m_dKY = m_pINIReaderWriter->getDoubleValue("KANAYAMA_MOTION_CONTROL","Y_GAIN");
	m_dKT = m_pINIReaderWriter->getDoubleValue("KANAYAMA_MOTION_CONTROL","T_GAIN");
	m_nDirectionofRotation = m_pINIReaderWriter->getIntValue("KANAYAMA_MOTION_CONTROL","DIRECTION_OF_ROTATION",0);	
	//==================================================================================================================

	//INI 파일로부터 Particle filter을 수행하기 위해 필요한 정보를 얻어온다.------------------------------------
	m_nMaxParticleNum = m_pINIReaderWriter->getIntValue("PARTICLE_FILTER","MAX_SAMPLE_NUM",0);
	m_nMinParticleNum = m_pINIReaderWriter->getIntValue("PARTICLE_FILTER","MIN_SAMPLE_NUM",0); 
	m_dDevationforTrans = m_pINIReaderWriter->getDoubleValue("PARTICLE_FILTER","DEVATION_TRANS");
	m_dDevationforRotate = m_pINIReaderWriter->getDoubleValue("PARTICLE_FILTER","DEVATION_ROTATE");
	m_dDevationforTransRotate = m_pINIReaderWriter->getDoubleValue("PARTICLE_FILTER","DEVATION_TRANSROTATE");
	//==================================================================================================================


	//INI 파일로부터 FEATURES를 수행하기 위해 필요한 정보를 얻어온다.------------------------------------
	m_nFeatureTh = m_pINIReaderWriter->getIntValue("FEATURES","MIN_FEATURE_TH", 160);
	m_nSiftMatchingTh = m_pINIReaderWriter->getIntValue("FEATURES","MATCHING_TH", 20); 
	m_nInteractionPointTh = m_pINIReaderWriter->getIntValue("FEATURES","INTERACTION_POINT_TH", 15);
	m_dNumSIFTFeatureTh = m_pINIReaderWriter->getDoubleValue("FEATURES","NUM_SIFTFEATURE_TH", 0.01);
	m_nNumSURFFeatureTh = m_pINIReaderWriter->getIntValue("FEATURES","NUM_SURFFEATURE_TH",400);
	m_dMatchingAngleTh = m_pINIReaderWriter->getDoubleValue("FEATURES","MATHING_ANGLE_TH", 0.15);
	m_strSIFTDataNamePath = m_pINIReaderWriter->getStringValue("FEATURES", "SIFTDATA_PATH&NAME", "no");
	m_strSURFDataNamePath = m_pINIReaderWriter->getStringValue("FEATURES", "SURFDATA_PATH&NAME", "no");
	m_nDistnaceFromPath = m_pINIReaderWriter->getDoubleValue("FEATURES","DISTANCE_FROM_PATH", 1000);
	m_dEllipseWidth = m_pINIReaderWriter->getDoubleValue("FEATURES","ELLIPSE_WIDTH", 100.0);
	m_dEllipseHeight = m_pINIReaderWriter->getDoubleValue("FEATURES","ELLIPSE_HEIGHT", 50.0);

	//==================================================================================================================


	//INI 파일로부터 Recognizer를 수행하기 위해 필요한 정보를 얻어온다.------------------------------------
	m_strAlFeatureMapNameNPath = m_pINIReaderWriter->getStringValue("RECOGNIZER", "AL_FEATURE_PATH&NAME", "no");
	m_nLandMarkNum = m_pINIReaderWriter->getIntValue("RECOGNIZER","TOTAL_FIDUCIAL_MARK_NUM", 1);
	m_nHeight_Camera2Mark = m_pINIReaderWriter->getIntValue("RECOGNIZER","HEIGHT_CAMERA_MARK", 2000); 
	//==================================================================================================================
	//saveParameter();

	return true;

}
void KuRobotParameter::saveParameter()
{

	string strInIFileName = "./ini/KUNS.ini";  	//기본지도 이름은 kuns.ini파일에 명시되어 있다. 

	CString strTemp;

	
		
	strTemp.Format(L"%d",m_nTotalfloorNum);
	WritePrivateProfileString(_T("MAP"), _T("FLOOR"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nCurfloor);
	WritePrivateProfileString(_T("MAP"), _T("CURRENT_FLOOR"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_strMapNameNPath.c_str();
	WritePrivateProfileString(_T("MAP"), _T("PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strVelocityMapNameNPath.c_str();
	WritePrivateProfileString(_T("MAP"), _T("VELOCITY_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strCeilingMapNameNPath.c_str();
	WritePrivateProfileString(_T("MAP"), _T("CEILINGMAP_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strCadMapNameNPath.c_str();
	WritePrivateProfileString(_T("MAP"), _T("CADMAP_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strZoneMapNameNPath.c_str();
	WritePrivateProfileString(_T("MAP"), _T("ZONEMAP_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nMapSizeXm);
	WritePrivateProfileString(_T("MAP"), _T("MAP_SIZE_X"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nMapSizeYm);
	WritePrivateProfileString(_T("MAP"), _T("MAP_SIZE_Y"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_dHeight);
	WritePrivateProfileString(_T("MAP"), _T("HEIGHT"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_strMovieNameNPath.c_str();
	WritePrivateProfileString(_T("MOVIE"), _T("MOVIE&PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_strTeachingPathNameNPath.c_str();
	WritePrivateProfileString(_T("PATH"), _T("TEACHINGPATH_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strImagePathNameNPath.c_str();
	WritePrivateProfileString(_T("PATH"), _T("IMAGEPATH_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strTeachingWayPointNameNPath.c_str();
	WritePrivateProfileString(_T("PATH"), _T("TEACHINGWAYPOINT_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strOutlineWayPointNameNPath.c_str();
	WritePrivateProfileString(_T("PATH"), _T("OUTLINEWAYPOINT_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strPathteaching.c_str();
	WritePrivateProfileString(_T("PATH"), _T("PATHTEACHING"), strTemp,_T("./ini/KUNS.ini"));


	strTemp.Format(L"%d",m_nRobotID);
	WritePrivateProfileString(_T("ROBOT"), _T("ROBOT_ID"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nRadiusofRobot);
	WritePrivateProfileString(_T("ROBOT"), _T("ROBOT_RADIUS"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nWheelBaseofRobot);
	WritePrivateProfileString(_T("ROBOT"), _T("ROBOT_WHEEL_BASE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nMaxRobotVelocity);
	WritePrivateProfileString(_T("ROBOT"), _T("MAX_VELOCITY"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nMinRobotVelocity);
	WritePrivateProfileString(_T("ROBOT"), _T("MIN_VELOCITY"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%f",m_initRobotPos.getX());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_XPOSE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_initRobotPos.getY());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_YPOSE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_initRobotPos.getThetaDeg());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_THETADEG"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strLastRobotPoseNameNPath.c_str();
	WritePrivateProfileString(_T("ROBOT"), _T("LASTROBOTPOSE_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strGlobalLocalization.c_str();
	WritePrivateProfileString(_T("ROBOT"), _T("GLOBAL_LOCALIZATION"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nObstacleDetectionTime);
	WritePrivateProfileString(_T("ROBOT"), _T("OBSTACLEDETECTION_TIME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nLocalization);
	WritePrivateProfileString(_T("ROBOT"), _T("LOCALIZATION"), strTemp,_T("./ini/KUNS.ini"));


	strTemp=m_strDataPath.c_str();
	WritePrivateProfileString(_T("SENSOR"), _T("DATA_PATH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strDataRecoding.c_str();
	WritePrivateProfileString(_T("SENSOR"), _T("DATA_RECODING"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strDataPlay.c_str();
	WritePrivateProfileString(_T("SENSOR"), _T("DATA_PLAY"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_cWheelCom;
	WritePrivateProfileString(_T("SENSOR"), _T("WHEEL_ACTUATOR"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_cHokuyoURG04LXCom;
	WritePrivateProfileString(_T("SENSOR"), _T("URG04LX_LASER"), strTemp,_T("./ini/KUNS.ini"));
	
	strTemp.Format(L"%d",m_nLaserTCPPort);
	WritePrivateProfileString(_T("SENSOR"), _T("LASER_CONNECTION_PORT"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_cCommunicationCom;
	WritePrivateProfileString(_T("SENSOR"), _T("COMMUNICATION"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_strdoSonar.c_str();
	WritePrivateProfileString(_T("SENSOR"), _T("SONAR"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nURG04LX_LaserMaxDist);
	WritePrivateProfileString(_T("SENSOR"), _T("URG04LX_LASER_MAX_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nURG04LX_LaserMinDist);
	WritePrivateProfileString(_T("SENSOR"), _T("URG04LX_LASER_MIN_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nURG04LX_LaserHeight);
	WritePrivateProfileString(_T("SENSOR"), _T("URG04LX_LASER_HEIGHT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nURG04LX_LaserXOffset);
	WritePrivateProfileString(_T("SENSOR"), _T("URG04LX_LASER_XOFFSET"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nURG04LX_LaserYOffset);
	WritePrivateProfileString(_T("SENSOR"), _T("URG04LX_LASER_YOFFSET"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_cRplidarCom;
	WritePrivateProfileString(_T("SENSOR"), _T("RPLIDAR"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nRplidar_MaxDist);
	WritePrivateProfileString(_T("SENSOR"), _T("RPLIDAR_MAX_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nRplidar_MinDist  );
	WritePrivateProfileString(_T("SENSOR"), _T("RPLIDAR_MIN_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nRplida_Height );
	WritePrivateProfileString(_T("SENSOR"), _T("RPLIDAR_HEIGHT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nRplida_XOffset );
	WritePrivateProfileString(_T("SENSOR"), _T("RPLIDAR_XOFFSET"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nnRplida_YOffset );
	WritePrivateProfileString(_T("SENSOR"), _T("RPLIDAR_YOFFSET"), strTemp,_T("./ini/KUNS.ini"));


	strTemp.Format(L"%d",m_nKinectMaxDist);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_MAX_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectMinDist);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_MIN_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectHeight);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_HEIGHT"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectXOffset);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_XOFFSET"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectYOffset);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_YOFFSET"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectMaxHeightDist);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_MAX_HEIGHT_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nKinectMinHeightDist);
	WritePrivateProfileString(_T("SENSOR"), _T("KINECT_MIN_HEIGHT_DISTANCE"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nDistToTarget);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("DISTANCE_TO_TARGET"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nDesiredVel);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("DESIRED_VELOCITY"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nGoalArea);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("GOAL_AREA"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_dKX);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("X_GAIN"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_dKY);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("Y_GAIN"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_dKT);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("T_GAIN"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nDirectionofRotation);
	WritePrivateProfileString(_T("KANAYAMA_MOTION_CONTROL"), _T("DIRECTION_OF_ROTATION"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nMaxParticleNum);
	WritePrivateProfileString(_T("PARTICLE_FILTER"), _T("MAX_SAMPLE_NUM"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nMinParticleNum);
	WritePrivateProfileString(_T("PARTICLE_FILTER"), _T("MIN_SAMPLE_NUM"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dDevationforTrans);
	WritePrivateProfileString(_T("PARTICLE_FILTER"), _T("DEVATION_TRANS"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dDevationforRotate);
	WritePrivateProfileString(_T("PARTICLE_FILTER"), _T("DEVATION_ROTATE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dDevationforTransRotate);
	WritePrivateProfileString(_T("PARTICLE_FILTER"), _T("DEVATION_TRANSROTATE"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%d",m_nFeatureTh);
	WritePrivateProfileString(_T("FEATURES"), _T("MIN_FEATURE_TH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nSiftMatchingTh);
	WritePrivateProfileString(_T("FEATURES"), _T("MATCHING_TH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nInteractionPointTh);
	WritePrivateProfileString(_T("FEATURES"), _T("INTERACTION_POINT_TH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dNumSIFTFeatureTh);
	WritePrivateProfileString(_T("FEATURES"), _T("NUM_SIFTFEATURE_TH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nNumSURFFeatureTh);
	WritePrivateProfileString(_T("FEATURES"), _T("NUM_SURFFEATURE_TH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dMatchingAngleTh);
	WritePrivateProfileString(_T("FEATURES"), _T("MATHING_ANGLE_TH"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_strSIFTDataNamePath.c_str();
	WritePrivateProfileString(_T("FEATURES"), _T("SIFTDATA_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp=m_strSURFDataNamePath.c_str();
	WritePrivateProfileString(_T("FEATURES"), _T("SURFDATA_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nDistnaceFromPath);
	WritePrivateProfileString(_T("FEATURES"), _T("DISTANCE_FROM_PATH"), strTemp,_T("./ini/KUNS.ini"));

	strTemp.Format(L"%0.4f",m_dEllipseWidth);
	WritePrivateProfileString(_T("FEATURES"), _T("ELLIPSE_WIDTH"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.4f",m_dEllipseHeight);
	WritePrivateProfileString(_T("FEATURES"), _T("ELLIPSE_HEIGHT"), strTemp,_T("./ini/KUNS.ini"));

	strTemp=m_strAlFeatureMapNameNPath.c_str();
	WritePrivateProfileString(_T("RECOGNIZER"), _T("AL_FEATURE_PATH&NAME"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nLandMarkNum );
	WritePrivateProfileString(_T("RECOGNIZER"), _T("TOTAL_FIDUCIAL_MARK_NUM"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nHeight_Camera2Mark  );
	WritePrivateProfileString(_T("RECOGNIZER"), _T("HEIGHT_CAMERA_MARK"), strTemp,_T("./ini/KUNS.ini"));
}
void KuRobotParameter::setRobotID(int nID)
{
	//로봇의 ID를 설정해주는 함수.
	m_nRobotID = nID;
}

void KuRobotParameter::setLocalization(int nLocalization)
{
	//로봇의 ID를 설정해주는 함수.
	m_nLocalization = nLocalization;
}
void KuRobotParameter::setObstacleDetectionTime(int nObstacleDetectionTime)
{
	//로봇의 ID를 설정해주는 함수.
	m_nObstacleDetectionTime = nObstacleDetectionTime;
}

void KuRobotParameter::setInitRobotPose(KuPose initRobotPos)
{
	m_initRobotPos=initRobotPos;
}

void KuRobotParameter::setRobotRadius(int nRadius)
{
	//로봇 반경을 설정해주는 함수. 단위는 mm
	m_nRadiusofRobot = nRadius;
}

void KuRobotParameter::setWheelBaseofRobot(int nWheelBaseofRobot)
{
	//로봇 반경을 설정해주는 함수. 단위는 mm
	m_nWheelBaseofRobot = nWheelBaseofRobot;
}

void KuRobotParameter::setMaxRobotVelocity(int nMaxVelocity)
{
	//로봇의 최대속도를 입력받는다.
	m_nMaxRobotVelocity = nMaxVelocity;
}
void KuRobotParameter::setMinRobotVelocity(int nMinVelocity)
{
	//로봇의 최소속도를 입력받는다.
	m_nMinRobotVelocity = nMinVelocity;
}
void KuRobotParameter::setLastRobotPoseNameNPath(string strLastRobotPoseNameNPath)
{
	m_strLastRobotPoseNameNPath=strLastRobotPoseNameNPath;
}
void KuRobotParameter::setTotalfloorNum(int nTotalfloorNum)
{
	m_nTotalfloorNum = nTotalfloorNum; 
}

void KuRobotParameter::setCurfloor(int nCurfloor)
{
	m_nCurfloor = nCurfloor; 
}

void KuRobotParameter::setMapSize(int nSizeXm, int nSizeYm)
{
	//작성될 지도의 크기를 설정한다. 
	m_nMapSizeXm = nSizeXm; //지도 x,y 크기 단위: m로 설정
	m_nMapSizeYm = nSizeYm; //지도 x,y 크기 단위: m로 설정

}

void KuRobotParameter::setHeight(double dHeight )
{
	m_dHeight = dHeight; // m로 설정

}
void KuRobotParameter::setVelocityMapNameNPath(string sMapName) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strVelocityMapNameNPath=sMapName;
}
void KuRobotParameter::setTeachingPathNameNPath(string sMapName) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strTeachingPathNameNPath=sMapName;
}
void KuRobotParameter::setTeachingWayPointNameNPath(string sMapName) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strTeachingWayPointNameNPath=sMapName;
}
void KuRobotParameter::setOutlineWayPointNameNPath(string sMapName) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strOutlineWayPointNameNPath=sMapName;
}
void KuRobotParameter::setPathteaching(string strPathteaching) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strPathteaching=strPathteaching;
}
void KuRobotParameter::setImagePathNameNPath(string sMapName) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strImagePathNameNPath=sMapName;
}
void KuRobotParameter::setMapNameNPath(string sMapName) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strMapNameNPath=sMapName;
}
void KuRobotParameter::setCeilingMapNameNPath(string sMapName) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strCeilingMapNameNPath=sMapName;
}

void KuRobotParameter::setCadMapNameNPath(string sMapName) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strCadMapNameNPath=sMapName;
}
void KuRobotParameter::setZoneMapNameNPath(string sMapName) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strZoneMapNameNPath=sMapName;
}
void KuRobotParameter::setMovieNameNPath(string sMapName) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strMovieNameNPath=sMapName;
}
void KuRobotParameter::setProMapNameNPath(string sMapName) 
{
	//장성할 지도의 이름을 설정한다. .	
	m_strProMapNameNPath=sMapName;
}
void KuRobotParameter::setDataPath(string sDataPath) 
{
	m_strDataPath=sDataPath;
}
void KuRobotParameter::setDataRecoding(string sDataRecoding)
{
	m_strDataRecoding=sDataRecoding;
}
void KuRobotParameter::setGlobalLocalization(string sGlobalLocalization)
{
	m_strGlobalLocalization=sGlobalLocalization;
}

void KuRobotParameter::setDataPlay(string sDataPlay) 
{
	m_strDataPlay=sDataPlay;
}

void KuRobotParameter::setWheelComport(char cComport[10])
{
	//Wheel comport를 설정하는 함수.
	strcpy(m_cWheelCom, cComport); //문자열 복사 함수.
}

void KuRobotParameter::setURG04LXLaserComport(char cComport[10])
{
	//URG04LX laser comport를 설정하는 함수.
	strcpy(m_cHokuyoURG04LXCom, cComport); //문자열 복사 함수.

}

void KuRobotParameter::setLaserTCPPort(int nLaserTCPPort )
{
	m_nLaserTCPPort = nLaserTCPPort; // m로 설정

}

void KuRobotParameter::setCommunicationComport(char cComport[10])
{
	//URG04LX laser comport를 설정하는 함수.
	strcpy(m_cCommunicationCom, cComport); //문자열 복사 함수.

}

void KuRobotParameter::setdoSonar(string strdoSonar)
{
	m_strdoSonar =strdoSonar ;
}

//laser 관련 파라미터---------------------------------------------------------------------------------
void KuRobotParameter::setURG04LXLaserMaxDist(int nMaxDist)
{
	//상단 레이저의 최대 탐지거리를 설정하는 함수.
	m_nURG04LX_LaserMaxDist = nMaxDist;
}

void KuRobotParameter::setURG04LXLaserMinDist(int nMinDist)
{
	//상단 레이저의 최소 탐지거리를 설정하는 함수.
	m_nURG04LX_LaserMinDist = nMinDist;
}

void KuRobotParameter::setURG04LXLaserHeight(int nHeight)
{
	//틸트 축으로 부터 상단 레이저까지의 높이값을 설정하는 함수.
	m_nURG04LX_LaserHeight = nHeight;
}

void KuRobotParameter::setURG04LXLaserXOffset(int nXOffset)
{
	//틸트 축으로 부터 상단 레이저까지의 x offset을 설정하는 함수.
	m_nURG04LX_LaserXOffset = nXOffset;
}

void KuRobotParameter::setURG04LXLaserYOffset(int nYOffset)
{
	//틸트 축으로 부터 상단 레이저까지의 y offset을 설정하는 함수.
	m_nURG04LX_LaserYOffset = nYOffset;
}
///====
void KuRobotParameter::setRplidarComport(char cComport[10])
{
	//URG04LX laser comport를 설정하는 함수.
	strcpy(m_cRplidarCom, cComport); //문자열 복사 함수.

}
//laser 관련 파라미터---------------------------------------------------------------------------------
void KuRobotParameter::setRplidarMaxDist(int nMaxDist)
{
	//상단 레이저의 최대 탐지거리를 설정하는 함수.
	m_nRplidar_MaxDist   = nMaxDist;
}

void KuRobotParameter::setRplidarMinDist(int nMinDist)
{
	//상단 레이저의 최소 탐지거리를 설정하는 함수.
	m_nRplidar_MinDist   = nMinDist;
}

void KuRobotParameter::setRplidarHeight(int nHeight)
{
	//틸트 축으로 부터 상단 레이저까지의 높이값을 설정하는 함수.
	m_nRplida_Height  = nHeight;
}

void KuRobotParameter::setRplidarXOffset(int nXOffset)
{
	//틸트 축으로 부터 상단 레이저까지의 x offset을 설정하는 함수.
	m_nRplida_XOffset  = nXOffset;
}

void KuRobotParameter::setRplidarYOffset(int nYOffset)
{
	//틸트 축으로 부터 상단 레이저까지의 y offset을 설정하는 함수.
	m_nnRplida_YOffset  = nYOffset;
}
//==
//Kinect 관련 파라미터---------------------------------------------------------------------------------
void KuRobotParameter::setKinectMaxDist(int nMaxDist)
{
	//상단 레이저의 최대 탐지거리를 설정하는 함수.
	m_nKinectMaxDist = nMaxDist;
}

void KuRobotParameter::setKinectMinDist(int nMinDist)
{
	//상단 레이저의 최소 탐지거리를 설정하는 함수.
	m_nKinectMinDist = nMinDist;
}

void KuRobotParameter::setKinectHeight(int nHeight)
{
	//틸트 축으로 부터 상단 레이저까지의 높이값을 설정하는 함수.
	m_nKinectHeight = nHeight;
}

void KuRobotParameter::setKinectXOffset(int nXOffset)
{
	//틸트 축으로 부터 상단 레이저까지의 x offset을 설정하는 함수.
	m_nKinectXOffset = nXOffset;
}

void KuRobotParameter::setKinectYOffset(int nYOffset)
{
	//틸트 축으로 부터 상단 레이저까지의 y offset을 설정하는 함수.
	m_nKinectYOffset = nYOffset;
}
void KuRobotParameter::setKinectMaxHeightDist(int nMaxDist)
{
	//상단 레이저의 최대 탐지거리를 설정하는 함수.
	m_nKinectMaxHeightDist = nMaxDist;
}

void KuRobotParameter::setKinectMinHeightDist(int nMinDist)
{
	//상단 레이저의 최소 탐지거리를 설정하는 함수.
	m_nKinectMinHeightDist = nMinDist;
}

//주행 성능과 연관된 parameter 설정 함수들-------------------------------------------
void KuRobotParameter::setTargetDistance(int nDistToTarget)
{
	m_nDistToTarget=nDistToTarget;
}
void KuRobotParameter::setDesiedVel(int nDesiredVel)
{
	m_nDesiredVel=nDesiredVel;
}
void KuRobotParameter::setGoalArea(int nGoalArea)
{
	m_nGoalArea=nGoalArea;
}
void KuRobotParameter::setdKX(double  dKX)
{
	m_dKX=dKX;
}
void KuRobotParameter::setdKY(double  dKY)
{
	m_dKY=dKY;
}
void KuRobotParameter::setdKT(double  dKT)
{
	m_dKT=dKT;
}
void KuRobotParameter::setDirectionofRotation(int nDirectionofRotation)
{
	m_nDirectionofRotation=nDirectionofRotation;
}

void KuRobotParameter::setMaxParticleNum(int nMaxNum)
{
	m_nMaxParticleNum=nMaxNum;
}//최대 particle 갯수를 설정하는 함수.
void KuRobotParameter::setMinParticleNUm(int nMinNUm)
{
	m_nMinParticleNum=nMinNUm;
} //최소 particle 갯수를 설정하는 함수.
void KuRobotParameter::setDeviationforTrans(double  dDevationforTrans)
{
	m_dDevationforTrans=dDevationforTrans;
}
void KuRobotParameter::setDeviationforRotae(double  dDevationforRotate)
{
	m_dDevationforRotate=dDevationforRotate;
}
void KuRobotParameter::setDeviationforTransRotae(double  dDevationforTransRotate)
{
	m_dDevationforTransRotate=dDevationforTransRotate;
}

void KuRobotParameter::setFeatureTh(int nFeatureTh)
{
	m_nFeatureTh =nFeatureTh;
}
void KuRobotParameter::setMatchingTh(int nMatchingTh)
{
	m_nSiftMatchingTh =nMatchingTh;
}
void KuRobotParameter::setNumSIFTFeatureTh(double dNuMFeatureTh)
{
	m_dNumSIFTFeatureTh =dNuMFeatureTh;
}
void KuRobotParameter::setNumSURFFeatureTh(int nSURFKeypointTh)
{
	m_nNumSURFFeatureTh = nSURFKeypointTh;
}
void KuRobotParameter::setInteractionPointTh(int nInteractionPointTh)
{
	m_nInteractionPointTh =nInteractionPointTh;
}
void KuRobotParameter::setMatchingAngleTh(double dMatchingAngleTh)
{
	m_dMatchingAngleTh =dMatchingAngleTh;
}

void KuRobotParameter::setSIFTDataNamePath(string strSIFTDataNamePath) 
{
	//장성할 지도의 이름을 설정한다. .
	m_strSIFTDataNamePath=strSIFTDataNamePath;
}
void KuRobotParameter::setSURFDataNamePath(string strSURFDataNamePath) 
{
	//장성할 지도의 이름을 설정한다. .
	m_strSURFDataNamePath=strSURFDataNamePath;
}
void KuRobotParameter::setDistanceFromPath(int nDistnaceFromPath)
{
	m_nDistnaceFromPath=nDistnaceFromPath;
}


void KuRobotParameter::setEllipseWidth(double dEllipseWidth)
{
	m_dEllipseWidth =dEllipseWidth;
}
void KuRobotParameter::setEllipseHeight(double dEllipseHeight)
{
	m_dEllipseHeight =dEllipseHeight;
}


/**
@brief Korean: Land mark 갯수를 설정하는 함수.
@brief English: 
*/
void KuRobotParameter::setLandMarkNum(int nLandMarkNum)
{
	m_nLandMarkNum = nLandMarkNum;
}

/**
@brief Korean: 카메라의 위치로부터 fidutial mark까지의 높이를 설정하는 함수.
@brief English: 
*/
void KuRobotParameter::setHeight_Camera2Mark(int nHeight_Camera2Mark)
{
	m_nHeight_Camera2Mark = nHeight_Camera2Mark;
}
/**
@brief Korean: 작성할 지도의 이름과 저장 주소 설정 함수
@brief English: 
*/
void KuRobotParameter::setAlFeatureMapNameNPath(string sAlFeatureMapNameNPath) 
{
	m_strAlFeatureMapNameNPath=sAlFeatureMapNameNPath;
}

//=============================================================================================================================


//get 함수목록--------------------------------------------------------------------------------------------------------------------------
//Robot 파라미터---------------------------------------
int KuRobotParameter::getRobotID()
{
	//총 사용될 RobotID 갯수를 넘겨준다.
	return m_nRobotID;
}
int KuRobotParameter::getLocalization()
{
	//총 사용될 RobotID 갯수를 넘겨준다.
	return m_nLocalization;
}
int KuRobotParameter::getObstacleDetectionTime()
{
	//총 사용될 RobotID 갯수를 넘겨준다.
	return m_nObstacleDetectionTime;
}
KuPose KuRobotParameter::getInitRobotPose()
{
	return m_initRobotPos;
}

int KuRobotParameter::getRobotRadius()
{
	//로봇의 반지름을 넘겨준다.
	return m_nRadiusofRobot;
}

int KuRobotParameter::getWheelBaseofRobot()
{
	//로봇의 반지름을 넘겨준다.
	return m_nWheelBaseofRobot;
}

int KuRobotParameter::getMaxRobotVelocity()
{
	//로봇의 최대 속도를 넘겨준다.
	return m_nMaxRobotVelocity;
}
int KuRobotParameter::getMinRobotVelocity()
{
	//로봇의 최소 속도를 넘겨준다.
	return m_nMinRobotVelocity;
}
string KuRobotParameter::getLastRobotPoseNameNPath( )
{
	return m_strLastRobotPoseNameNPath;
}

int KuRobotParameter::getTotalfloorNum()
{
	return m_nTotalfloorNum;
}
int KuRobotParameter::getCurfloor()
{
	return m_nCurfloor;
}


int KuRobotParameter::getMapSizeXm()
{
	//지도 x,y 크기를 넘겨준다.
	return m_nMapSizeXm;
}
int KuRobotParameter::getMapSizeYm()
{
	//지도 x,y 크기를 넘겨준다.
	return m_nMapSizeYm;
}

double KuRobotParameter::getHeight()
{
	return m_dHeight;
}
string KuRobotParameter::getMapNameNPath(	) 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strMapNameNPath;
}

string KuRobotParameter::getProMapNameNPath(	) 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strProMapNameNPath;
}

string KuRobotParameter::getCadMapNameNPath( ) 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strCadMapNameNPath;
}
string KuRobotParameter::getZoneMapNameNPath( ) 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strZoneMapNameNPath;
}
string KuRobotParameter::getCeilingMapNameNPath( ) 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strCeilingMapNameNPath;
}
string KuRobotParameter::getVelocityMapNameNPath(	) 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strVelocityMapNameNPath;
}
string KuRobotParameter::getMovieNameNPath(	) 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strMovieNameNPath;
}
string KuRobotParameter::getTeachingPathNameNPath(	) 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strTeachingPathNameNPath;
}
string KuRobotParameter::getTeachingWayPointNameNPath(	) 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strTeachingWayPointNameNPath;
}
string KuRobotParameter::getOutlineWayPointNameNPath(	) 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strOutlineWayPointNameNPath;
}
string KuRobotParameter::getPathteaching() 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strPathteaching;
}
string KuRobotParameter::getImagePathNameNPath(	) 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strImagePathNameNPath;
}
string KuRobotParameter::getDataPath( )
{
	return m_strDataPath;
}
string KuRobotParameter::getDataRecoding( )
{
	return m_strDataRecoding;
}
string KuRobotParameter::getGlobalLocalization( )
{
	return m_strGlobalLocalization;
}

string KuRobotParameter::getDataPlay( )
{
	return m_strDataPlay;
}

void KuRobotParameter::getWheelComport(char cComport[10])
{
	//Wheel comport를 설정하는 함수.
	strcpy(cComport, m_cWheelCom); //문자열 복사 함수.
}

void KuRobotParameter::getURG04LXLaserComport(char cComport[10])
{
	//URG04LX laser comport를 설정하는 함수.
	strcpy( cComport,m_cHokuyoURG04LXCom); //문자열 복사 함수.
}

int KuRobotParameter::getLaserTCPPort()
{
	//지도 x,y 크기를 넘겨준다.
	return m_nLaserTCPPort;
}
void KuRobotParameter::getCommunicationComport(char cComport[10])
{
	//URG04LX laser comport를 설정하는 함수.
	strcpy( cComport,m_cCommunicationCom); //문자열 복사 함수.
}

string KuRobotParameter::getdoSonar( )
{
	return m_strdoSonar;
}

//laser 관련 파라미터
int KuRobotParameter:: getURG04LXLaserMaxDist()
{ 
	//상단 레이저의 최대 탐지거리를 받아오는 함수.
	return m_nURG04LX_LaserMaxDist; 
}
int KuRobotParameter:: getURG04LXLaserMinDist()
{ 
	//상단 레이저의 최소 탐지거리를 받아오는 함수.
	return m_nURG04LX_LaserMinDist; 

}
int KuRobotParameter::getURG04LXLaserHeight()
{ 
	//틸트 축으로 부터 상단 레이저까지의 높이값을 받아오는 함수.
	return m_nURG04LX_LaserHeight; 

}
int KuRobotParameter:: getURG04LXLaserXOffset()
{ 
	//틸트 축으로 부터 상단 레이저까지의 x offset을 받아오는 함수.
	return m_nURG04LX_LaserXOffset; 

}
int KuRobotParameter:: getURG04LXLaserYOffset()
{
	//틸트 축으로 부터 상단 레이저까지의 y offset을 받아오는 함수.
	return m_nURG04LX_LaserYOffset; 
}
//라이다

void KuRobotParameter::getRplidarComport(char cComport[10])
{
	//URG04LX laser comport를 설정하는 함수.
	strcpy( cComport,m_cRplidarCom); //문자열 복사 함수.
}
//laser 관련 파라미터
int KuRobotParameter:: getRplidarMaxDist()
{ 
	//상단 레이저의 최대 탐지거리를 받아오는 함수.
	return m_nRplidar_MaxDist  ; 
}
int KuRobotParameter:: getRplidarMinDist()
{ 
	//상단 레이저의 최소 탐지거리를 받아오는 함수.
	return m_nRplidar_MinDist  ; 

}
int KuRobotParameter::getRplidaHeight()
{ 
	//틸트 축으로 부터 상단 레이저까지의 높이값을 받아오는 함수.
	return m_nRplida_Height ; 

}
int KuRobotParameter:: getRplidarXOffset()
{ 
	//틸트 축으로 부터 상단 레이저까지의 x offset을 받아오는 함수.
	return m_nRplida_XOffset ; 

}
int KuRobotParameter:: getRplidaYOffset()
{
	//틸트 축으로 부터 상단 레이저까지의 y offset을 받아오는 함수.
	return m_nnRplida_YOffset ; 
}



//Kinect 관련 파라미터
int KuRobotParameter::getKinectMaxDist()
{
	return m_nKinectMaxDist;
	//키넥트의 최대 탐지거리를 설정하는 함수.
}
int KuRobotParameter::getKinectMinDist( )
{
	return m_nKinectMinDist;
	//키넥트의 최소 탐지거리를 설정하는 함수.
}
int KuRobotParameter::getKinectHeight( )
{
	return m_nKinectHeight;
	//바닥에서 키넥트까지의 높이값을 설정하는 함수.
}
int KuRobotParameter::getKinectXOffset( )
{
	return m_nKinectXOffset;
	//로봇의 중심에서 키넥트까지의 x offset을 설정하는 함수.
}
int KuRobotParameter::getKinectYOffset( )
{
	return m_nKinectYOffset;
	//로봇의 중심에서 키넥트까지의 y offset을 설정하는 함수.	
}
int KuRobotParameter::getKinectMaxHeightDist()
{
	return m_nKinectMaxHeightDist;
	//키넥트의 최대 탐지거리를 설정하는 함수.
}
int KuRobotParameter::getKinectMinHeightDist( )
{
	return m_nKinectMinHeightDist;
	//키넥트의 최소 탐지거리를 설정하는 함수.
}

//주행 성능과 연관된 parameter get 함수----------------------------------------------------------------

int KuRobotParameter::getTargetDistance()
{
	return m_nDistToTarget;
}
int KuRobotParameter::getDesiedVel( )
{
	return m_nDesiredVel;
}
int KuRobotParameter::getGoalArea( )
{
	return m_nGoalArea;
}
double KuRobotParameter::getdKX(  )
{
	return m_dKX;
}
double KuRobotParameter::getdKY(  )
{
	return m_dKY;
}
double KuRobotParameter::getdKT(  )
{
	return m_dKT;
}
int KuRobotParameter::getDirectionofRotation()
{
	return m_nDirectionofRotation;
}

int KuRobotParameter::getMaxParticleNum()
{
	return m_nMaxParticleNum;
}//최대 particle 갯수를 설정하는 함수.
int KuRobotParameter::getMinParticleNUm()
{
	return m_nMinParticleNum;
} //최소 particle 갯수를 설정하는 함수.
double KuRobotParameter::getDeviationforTrans(  )
{
	return m_dDevationforTrans;
}
double KuRobotParameter::getDeviationforRotate(  )
{
	return m_dDevationforRotate;
}
double KuRobotParameter::getDeviationforTransRotate(  )
{
	return m_dDevationforTransRotate;
}
int KuRobotParameter::getFeatureTh( )
{
	return m_nFeatureTh ;
}
int KuRobotParameter::getMatchingTh( )
{
	return m_nSiftMatchingTh ;
}
double KuRobotParameter::getNumSIFTFeatureTh( )
{
	return m_dNumSIFTFeatureTh ;
}
int KuRobotParameter::getNumSURFFeatureTh()
{
	return m_nNumSURFFeatureTh;
}
int KuRobotParameter::getInteractionPointTh( )
{
	return m_nInteractionPointTh ;
}
double KuRobotParameter::getMatchingAngleTh( )
{
	return  m_dMatchingAngleTh ;
}
string KuRobotParameter::getSIFTDataNamePath() 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strSIFTDataNamePath;
}
string KuRobotParameter::getSURFDataNamePath() 
{
	//장성할 지도의 이름을 설정한다. .
	return m_strSURFDataNamePath;
}
int KuRobotParameter::getDistanceFromPath()
{
	return m_nDistnaceFromPath;
}

/**
@brief Korean: Land mark 갯수를 넘겨주는 함수
@brief English: 
*/
int KuRobotParameter::getLandMarkNum()
{
	return m_nLandMarkNum;
}

/**
@brief Korean: 카메라의 위치로부터 fidutial mark까지의 높이를 넘겨주는 함수
@brief English: 
*/
int KuRobotParameter::getHeightCamera2Mark()
{
	return m_nHeight_Camera2Mark;
}

/**
@brief Korean: 작성할 지도의 이름과 저장 주소 설정 함수
@brief English: 
*/
string KuRobotParameter::getAlFeatureMapNameNPath() 
{
	return m_strAlFeatureMapNameNPath;
}

double KuRobotParameter::getEllipseWidth( )
{
	return  m_dEllipseWidth ;
}
double KuRobotParameter::getEllipseHeight( )
{
	return  m_dEllipseHeight;
}
//======================================================================================================================================

