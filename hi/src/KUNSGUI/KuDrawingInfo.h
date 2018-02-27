/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description :그림 그릴 각종 정보들을 저장하고 있는 싱글톤 타입의 클래스.
$Created on: 2012. 6. 4.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/

#ifndef KUNS_DRAWING_INFO_H
#define KUNS_DRAWING_INFO_H


#include <iostream>
#include <vector>
#include <list>
#include "../KUNSMap/KuMap.h"
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../sensor/Sensor.h"
#include "../KUNSPose/KuPose.h"
#include "../Algorithm/ParticleFilter/Sample.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "../MobileSupervisor/KuPOIMapParameter.h"
#include "../KUNSProcess/KUNSLocalPathPlnnerPr/KuTrajectory.h"


using namespace std;
class KuDrawingInfo : public KuSingletone <KuDrawingInfo>
{
	// CSLAM
private:
	CCriticalSection m_CriticalSection;
private:
	KuUtil m_KuUtil;

	///map 관련 변수
	///map 관련 변수
	int m_nMapX;
	int m_nMapY;
	int **  m_nMap; /// 환경에 대한 정보를 정수 형태로 저장하는 변수
	double ** m_dProMap;/// 환경에 대한  확률적인 정보로 저장하는 변수
	double **m_dProObMap;
	double **  m_dPathMap; 
	int** m_nLocalMap;

	double ** m_dpredictedProMap;/// 환경에 대한  확률적인 정보로 저장하는 변수

	//sensor 관련
	int_1DArray m_nLaserData181; ///181개의 레이저 데이터를 저장하는 변수.
	int_1DArray m_nRplidarData181; ///181개의 레이저 데이터를 저장하는 변수.
	int_1DArray m_nKinectRangeData; ///57개의 키넥트 데이터를 저장하는 변수.
	IplImage *m_IplCeilingImage;// 천장 이미지 변수
	IplImage *m_IplKinectImg;// 천장 이미지 변수
	KuPose m_Global3DPose[Sensor::IMAGE_WIDTH* Sensor::IMAGE_HEIGHT];
	int_1DArray m_nRangeData;

	KuPose m_RobotPos; //로봇 위치값을 저장하는 변수
	KuPose m_AuxiliaryRobotPos; //실험 목적의 로봇 위치를 저장하는 변수 예) 엔코더 기반의 로봇 위치, 가상로봇등
	KuPose m_IdealRobotPos; //위치오차가 전혀없는 로봇 위치를 저장하는 변수. 실험목적으로 사용가능함.
	KuPose m_GoalPos; //골 위치값을 저장하는 변수
	KuPose m_TargetPos; //target pose를 저장하는 변수
	int m_nTVByKeyEvt, m_nRVByKeyEvt; //키보드 이벤트를 통해 받은 로봇 속도를 저장하는 변수.
	KuPose m_ObsGoalPos; //골 위치값을 저장하는 변수

	//경로관련 변수
	list<KuPose> m_listPath;
	list<KuPose> m_listWayPoint;

	//particle 관련 변수
	vector<Sample> m_vecParticle;

	//골 위치관련 변수
	list<KuPose> m_GoalPosList;

	//지도 그리기 관련 변수
	bool m_bRenderMapflag;
	bool m_bRenderBuildingMapflag;
	//레이저 그리기 관련 변수
	bool m_bRendeLaserflag;
	//키넥트 그리기 관련 변수
	bool m_bRenderKinectflag;
	// 경로 그리기 관련 변수
	bool m_bRenderPathflag;


	bool m_bDirectionofPathflag;

	vector<KuPose> m_vecLandmark;

	bool m_bRenderCeilingImagflag;

	bool m_bWayPointflag;

	int** m_nCADMap;

	bool m_bRenderCADMapflag;

	int m_nCADMapSizeX,m_nCADMapSizeY;

	bool m_bRenderKinect3DCloudFlag;
	bool m_bObstacleAreasFlag;

	list<KuPose> m_listLocalPath;
	list<KuPose> m_listObsLocalPath;
	vector<list<KuPose>> m_veclistObsLocalPath;


	vector<KuPose> m_VecMultiObstaclePos;
	vector<KuPose>m_VecMultiObstacleGoalPos;
	vector<KuPose> m_VecTrackedObstacle;
	vector<KuPose> m_vecObstacleAreas;
	int m_nPathMapSizeX;
	int m_nPathMapSizeY;
	POIMapPara m_PIOMapData;
	int m_nCurFloor;
	int m_nLocalMapSizeX;
	int m_nLocalMapSizeY;
	int m_nPatternID;

	int m_npredictedMapSizeX;
	int m_npredictedMapSizeY;


	vector<KuTrajectory> m_vecPredictedTraj;
	vector<KuTrajectory> m_vecCubicPath;
	vector<KuPose> m_vecPredictedRTraj;

	bool m_bWanderingstart;


public:

	void initVariable(); ///변수들을 초기화 한다. 

	///map 관련 함수
	void setMap(KuMap *pMap); //지도정보를 저장하는 함수.
	int** getMap(); /// 저장된 지도정보를 얻어가는 함수.
	double** getBuildingMap( int* nX, int* nY);// 작성중인 지도의 정보를 얻어가는 함수
	void setBuildingMap( double** dMap );
	int getMapSizeX();
	int getMapSizeY();
	void setMap(int** nMap,int nMapSizeX,int nMapSizeY);


	//센서관련 함수
	void setLaserData181(int_1DArray nLaserData181); /// 레이저 데이터를 저장하는 함수.
	int_1DArray getLaserData181(); ///저장된 레이저 데이터를 넘겨주는 함수

	void setRangeData(int_1DArray nRansgeData); /// 레이저 데이터를 저장하는 함수.
	int_1DArray getRangeData(); ///저장된 레이저 데이터를 넘겨주는 함수

	void setKinectRangeData(int_1DArray nKinectRangeData); /// 키넥트 데이터를 저장하는 함수.
	int_1DArray getKinectRangeData();///키넥트 데이터를 넘겨주는 함수

	void getKinectImageData(IplImage* IplKinectImg);
	void setKinectImageData(IplImage* IplKinectImg);


	//파티클 관련 함수
	void setParticle(vector<Sample> vecParticle); ///파티클에 대한 정보를 저장하는 함수
	vector<Sample> getParticle(); ///파티클에 대한 정보를 넘겨주는 함수


	KuPose  getRobotPos(); ///저장된 로봇 위치를 넘겨주는 함수.
	void setRobotPos(KuPose RobotPos); //로봇 위치를 저장하는 함수

	KuPose  getAuxiliaryRobotPos(); /////실험 목적의 로봇 위치를 넘겨주는 함수.
	void setAuxiliaryRobotPos(KuPose auxiliaryRobotPos); //실험 목적의 로봇 위치를 저장하는 함수

	KuPose  getIdealRobotPos(); /////실험 목적의 이상적인 로봇 위치를 넘겨주는 함수.
	void setIdealRobotPos(KuPose IdealRobotPos); //실험 목적의 로봇 위치를 저장하는 함수

	KuPose  getGoalPos(); ///저장된 골 위치를 넘겨주는 함수.
	void setGoalPos(KuPose GoalPos); //골 위치를 저장하는 함수

	list<KuPose> getGoalPosList(); ///저장된 골 위치를 넘겨주는 함수.
	void setGoalPosList(list<KuPose> GoalPosList); //골 위치를 저장하는 함수

	//그리기 관련 함수
	void setRenderMapflag(bool bRenderMapflag);
	bool getRenderMapflag(); 
	void setRenderBuildingMapflag(bool bRendbRenderBuildingMapflagerMapflag);
	bool getRenderBuildingMapflag();
	void setRenderLaserflag(bool bRendeLaserflag);
	bool getRenderLaserflag();
	void setRenderKinectflag(bool bRenderKinectflag);
	bool getRenderKinectflag();
	void setRenderPathflag(bool bRenderPathflag);
	bool getRenderPathflag();


	//가상로봇 제어관련 함수
	void setRobotTRVel(int nTVel, int nRVel);
	void getRobotTRVel(int* nTVel, int* nRVel);

	//경로관련 함수
	void setPath(list<KuPose> vecPathPos); ///경로를 저장하는 함수.
	list<KuPose> getPath(); ///경로를 넘겨주는 함수.

	//target pos관련 함수
	void setTargetPos(KuPose TargetPos); //target pos를 저장하는 함수
	KuPose getTargetPos(); //target pos를 넘겨주는 함수.


	void setDirectionofPathflag(bool bDirectionofPathflag);
	bool getDirectionofPathflag();

	vector<KuPose> getvecLandmarkPos();
	void setvecLandmarkPos(vector<KuPose> vecLandmark);

	void setRenderCeilingImageflag(bool bRenderCeilingImagflag);
	bool getRenderCeilingImageflag();

	void setWayPointList(list<KuPose> listWayPoint);
	list<KuPose> getWayPointList();

	void setWayPointflag(bool bWayPointflag);
	bool getWayPointflag();
	
	void setCADMap(int** nCADMap,int nMapSizeX,int nMapSizeY);

	int** getCADMap(int* nCADMapSizeX,int* nCADMapSizeY);

	void setRenderCADMapflag(bool bRenderCADMapflag);
	bool getRenderCADMapflag();


	void setRenderKinect3DCloudFlag(bool bFlag);
	bool getRenderKinect3DCloudFlag();

	void setKinectGlobal3DPos(KuPose* pGlobal3DPose);
	KuPose* getKinectGlobal3DPos();


	void setLocalPath(list<KuPose> listPath);
	list<KuPose> getLocalPath();

	void  getMultiObstaclePosVector(vector<KuPose>* VecMultiObstaclePos);
	void setMultiObstaclePosVector(vector<KuPose> VecMultiObstaclePos);

	void  getMultiObstacleGoalPosVector(vector<KuPose>* VecMultiObstacleGoalPos);
	void setMultiObstacleGoalPosVector(vector<KuPose> VecMultiObstacleGoalPos);

	double ** getProObBuildingMap( int* nX, int* nY);
	void setProObBuildingMap( double**  dMap );

	void setObstacleAreasFlag(bool bFlag);
	bool  getObstacleAreasFlag();
	void setObstacleAreas(KuPose ObstacleAreas);
	vector<KuPose>  getObstacleAreas();
	void clearObstacleAreas();



	void setPathMap(double **dMap,int nMapSizeX,int nMapSizeY);
	double** getPathMap(int*nPathMapSizeX,int* nPathMapSizeY);

	void setPIOMapData(POIMapPara PIOMapData);  
	POIMapPara getPIOMapData( );  

	void setCurFloor(int nCurFloor);
	int  getCurFloor();

	void setRplidarData181(int_1DArray nLaserData181);
	int_1DArray getRplidarData181();

	void setLocalMap(int **nMap,int nMapSizeX,int nMapSizeY);
	int** getLocalMap(int*nPathMapSizeX,int* nPathMapSizeY);

	int getPatternID();
	void setPatternID(int nPatternID);

	void setTrackedObstacle(vector<KuPose> VecTrackedObstacle);
	void getTrackedObstacle(vector<KuPose>* VecTrackedObstacle);
	
	void setpredictedProMap(double **dMap,int nMapSizeX,int nMapSizeY);
	double** getpredictedProMap(int*npredictedMapSizeX,int* npredictedMapSizeY);

	void setPredictedTraj(vector<KuTrajectory> VecTrajectory);
	void getPredictedTraj(vector<KuTrajectory>* VecTrajectory);

	void setCubicPath(vector<KuTrajectory> VecPath);
	void getCubicPath(vector<KuTrajectory>* VecPath);

	void setPredictedRTraj(vector<KuPose> VecTrajectory);
	void getPredictedRTraj(vector<KuPose>* VecTrajectory);

	void setObsLocalPath(list<KuPose> listPath);
	list<KuPose> getObsLocalPath();


	KuPose  getObsGoalPos(); ///저장된 골 위치를 넘겨주는 함수.
	void setObsGoalPos(KuPose GoalPos); //골 위치를 저장하는 함수

	bool getWanderingflag();
	void setWanderingflag(bool bWanderingstart);

	void setvecObsLocalPath(list<KuPose> listPath);
	vector<list<KuPose>> getvecObsLocalPath();

public:
	KuDrawingInfo();
	~KuDrawingInfo();
};

#endif /*DRAWING_INFO_H*/