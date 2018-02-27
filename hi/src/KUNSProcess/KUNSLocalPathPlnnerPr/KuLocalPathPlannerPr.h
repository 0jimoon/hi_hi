#ifndef KUNS_LOCAL_PATHPLANNER_PROCESSE_H
#define KUNS_LOCAL_PATHPLANNER_PROCESSE_H

#include <iostream>
#include <conio.h>
#include <math.h>
#include <cv.h>
#include <cstdlib>
#include <highgui.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/video/video.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/videostab/videostab.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/flann/miniflann.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/ml/ml.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "opencv2/ts/ts.hpp"
#include "opencv2/stitching/stitcher.hpp"
#include "opencv2/legacy/legacy.hpp"

#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../KUNSMap/KuMap.h"
#include "../../KUNSMap/KuMapRepository.h"
#include "../KUNSPathPlannerPr/KuGradientPathPlannerPr.h"
#include "../KUNSPathPlannerPr/KuPathSmoothingPr.h"
#include "../KUNSLaserMapBuilderPr/KuLaserMapBuilderPr.h"
#include "../KUNSLaserMapBuilderPr/KuMapBuilderParameter.h"
#include "../../KUNSUtil/KUNSTimer/KuTimer.h"
#include "../../KUNSProcess/KUNSWanderingObstaclePr/KuWanderingObstaclePr.h"
#include "../../KUNSProcess/KUNSLocalPathPlnnerPr/KuTrajectory.h"
#include "../../KUNSProcess/KUNSLocalPathPlnnerPr/KuKalman.h"
#include "KuGradientPathPlannerbasedPIPr.h"

// typedef struct _tagObstaclePos {
// 	double x,y,th;
// 	double prex,prey,preth;
// 	double vectorx,vectory,vectorth;
// 	int ID;
// }OPose;


using namespace cv;
using namespace std;

class KuLocalPathPlannerPr :public KuSingletone <KuLocalPathPlannerPr>
{
	static const int INFINITY_VALUE = 100000;
	static const int LOCALGOAL_DISTANCE = 4000;
	static const int LOCALPATH_COST = 5;
	static const int LOCAL_PREDICT_MAPSIZE = 400;

private:
	int m_nThreadFuncPeriod;

	/* Class */
	KuGradientPathPlannerPr m_KuPathPlanner; //경로계획기 인스턴스
	KuPathSmoothingPr m_KuPathSmoothing;
	KuMath m_math;
	KuLaserMapBuilderPr m_LaserMapBuilder;
	KuGradientPathPlannerbasedPIPr m_KuLocalPathPlanner; //지역 경로 계획기 생성.
	KuThread m_LocalPathplaningThread;
	KuTimer m_timer;
	KuThread m_predictedMappingThread;
	KuUtil m_KuUtil;
	KuTimer m_kutimer;

	/* Flag */
	bool m_bsetdataflag;
	bool m_bDrawingmapflag;
	bool m_bThreadFlag; 
	bool m_bStandbyFlag;

	/* Robot */
	double m_dRobotRadius;
	KuPose m_preRobotPose;
	KuPose m_RobotPos;
	KuTrajectory m_TrajRobotPos;
	double m_dRobotVel;
	double m_dRobotRVel;

	KuPose m_DelEncoderData;

	/* Map */
	KuMap* m_pBuildingMap;
	KuMap *m_pMap;
	KuMap* m_pCautionMap;
	int **  m_nCautionMap; /// 환경에 대한 정보를 정수 형태로 저장하는 변수
	double ** m_dpredictedProMap;/// 환경에 대한  확률적인 정보로 저장하는 변수
	double ** m_dlocalpredictedProMap;/// 환경에 대한  확률적인 정보로 저장하는 변수
	double ** m_dtmppredictedProMap;/// 환경에 대한  확률적인 정보로 저장하는 변수
	//bool ** m_bchepredictedProMap;/// 환경에 대한  확률적인 정보로 저장하는 변수
	KuMap* m_pRefMap;
	KuMap *m_pLocalMap;
	KuMap *m_pOriginalMap;
	int m_nLocalMapSPosX;
	int m_nLocalMapSPosY;
	int m_nGlobalMapX;
	int m_nGlobalMapY;
	int m_nLocalMapX;
	int m_nLocalMapY;
	int m_nBuildingMapX;
	int m_nBuildingMapY;
	bool m_bAvoidModeflag;
	int m_nMapSizeX;
	int m_nMapSizeY;
	int m_nCellSize;
	
	/* Sensor */
	/* Laser */
	int m_nLaserDataIDX;
	double m_dLaserSensorOffsetmm;
	double m_dLaserMinDistanceMM;
	double m_dLaserMaxDistanceMM;
	int_1DArray m_nLaserData;

	/* Kinect */
	int m_nKinectRnageDataIDX;
	double m_dKinectSensorOffsetmm;
	double m_dKinectMaxDistanceMM;
	double m_dKinectMinDistanceMM;
	int_1DArray m_nKinectRnageData;
	IplImage* m_IplKinectCamera;
	KuPose m_Global3DPose[Sensor::IMAGE_WIDTH* Sensor::IMAGE_HEIGHT];

	/* Path planning*/
	list<KuPose>  m_DetourPathList;
	KuPose m_LocalGoalPos;
	int m_nLocalGoalIndex;
	int m_nLocalGoalWayPointIndex;
	int m_nDistToTarget;
	vector<KuPose> m_vecWayPoint;
	vector<KuPose> m_vecLocalPath;
	vector<KuPose> m_vecPath;
	vector<KuPose> m_vectempPath;
	KuPose m_GoalPos;
	KuPose m_TargetPos;	
	int m_nPrePathIndx;
	int m_nSelectedMinIndx;

	/* Local Path Planning */
	CvMemStorage* m_storage;
	Mat m_matImgSrc;
	Mat m_cvMatImage;
	Mat m_cvGrayMatImage;

	vector<KuPose> m_prevecObstaclePose;
	vector<KuPose> m_vecObstaclePose;
	int nGroupID;

	/* Simulation */
	vector<KuPose> m_vecWanderObstaclePose;

	/* Trajectory */
	vector<KuTrajectory> m_vecObstacleTrajectory;
	vector<KuTrajectory> m_vecdistantObstacleTrajectory;
	vector<KuTrajectory> m_vecCubicSpTrajectory;
	bool m_bpredictexitObs;
	double m_dseltime;
	int m_mselObsID;
public:
	void terminate();
	bool start(int nTime);

	/* Get & Set */
	void setData(KuPose RobotPos, vector<KuPose> WanderObstaclePose);
	bool setPath(list<KuPose> listPath);
	list<KuPose> getPath();
	void setParameter(int nLaserDataIDX,double dLaserSensorOffsetmm,double dLaserMaxDistanceMM,double dLaserMinDistanceMM,
		int nKinectRnageDataIDX,double dKinectSensorOffsetmm,double dKinectMaxDistanceMM,double dKinectMinDistanceMM,
		int nCellSize,int nDistToTarget,double dRobotRadius);
	KuPose getTargetPos();
	int** getCautionMap();
	double** getpredictedProMap();
	KuPose getTargetPos(KuPose RobotPos,int nDistToTarget, double*dTargetDist);

	/* Thread */
	static void doLocalPathplanningThread(void* arg);
	static void dopredictedMappingThread(void* arg);

private:
	void startTimeCheck(LARGE_INTEGER& nStart);
	float finishTimeCheck(const LARGE_INTEGER nStart);
	void doCriticalSection(int nID);

	/* initialize */
	bool initialize();
	bool initializePath( );
	bool initPathplanningProcess();
	void initMapbuildingProcess(bool bLocalMapflag);

	/* Sensor */
	int_1DArray combinateLaserNKinect(int_1DArray nLaserData181, int_1DArray nKinectRnageData);

	/* Path planning */
	bool generateLocalPathforObsAvoidance(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData181, int_1DArray nKinectRnageData);
	bool existObstaclePath(KuPose RobotPos, vector<KuPose> vecLocalPath, int nPathSize, KuMap* pLocalMap, int nLocalMapSPosX, int nLocalMapSPosY );
	bool generateLocalPath(KuPose RobotPos);
	bool checkObstacles(KuPose RobotPos,KuPose TargetPos, int_1DArray nLaserData181, int_1DArray nKinectRnageData ,double dTargetDist);
	KuPose generateDetourGoalPos(KuPose RobotPos, vector<KuPose> vecPath, int nPathSize);
	bool generateDetourPath(KuPose RobotPos, KuPose DetourGoalPos, KuMap* pLocalMap, 
		int nLocalMapSPosX, int nLocalMapSPosY, list<KuPose> *DetourPathList);
	list<KuPose> transferLocaltoGlobalPath( list<KuPose> LocalPath,KuPose RobotPose,int nLocalMapSPosX, int nLocalMapSPosY);
	void calStartPathPoint(KuPose RobotPos );
	KuPose calPathPoint( KuPose RobotPos );
	bool checkstaticinMap(int** nGlobalMap,  int nObX, int nObY);

	/* Copy Map */
	void copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap, int** nOriginalMap, KuPose RobotPos);
	void copyGlobalMapToLocalMap(int**nGlobalMap, int** nLocalMap, KuPose RobotPos, int* nLocalMapSPosX, int* nLocalMapSPosY);
	void copyOriginalMapToGlobalMap(int** nOriginalMap, int** nGlobalMap, KuPose RobotPos);
	void copyBuildingMapToGlobalMap(int** nBuildingMap, int** nGlobalMap,  KuPose RobotPos);

	/* Local path planning */
	bool checkEllipse(int nObX, int nObY,double dX, double dY, int nX, int nY);
	void generateLocalMap(KuPose RobotPos,KuPose DelEncoderData,int_1DArray nLaserData181, int_1DArray nKinectRnageData);	
	void generatePredictionmap(KuPose RobotPos, KuPose* pKinectDataPose, IplImage* IplKinectCamera, vector<KuPose> vecWanderObstaclePose);
	void predictObjectpositionPro();
	inline double GetGaussianValue(double sigma, double x);
	inline double GetGaussianValue2(double sigma, double x, double crx, double y,double cry);
	void saveObstacledata(KuPose ObjPose);
	void savedistantObstacledata(KuTrajectory KuTraj);
	void setdistantCubicSplineddata(KuTrajectory KuTraj);
	void predictObstaclePath();
	//KuPose calculatepredictRobotpos(double dTime);
	KuPose calculatepredictRobotpos(KuPose  RobotPos,double dTVel,double dRotDegVel,double dTime);
	void CalculateGridPro(vector<KuTrajectory> vecObstacleTraj, double dDeviation, double dWeight);
	void CalculateGridPro(vector<KuTrajectory> vecObstacleTraj,int nselObsID,int nseltime,  double dDeviation, double dWeight);
	void copyGlobalMapToLocalMap(int**nGlobalMap, int** nLocalMap, KuPose RobotPos,
		double **dGlobalMap, double** dLocalMap,
		int* nLocalMapSPosX, int* nLocalMapSPosY);
public:
	KuLocalPathPlannerPr();
	~KuLocalPathPlannerPr();

};

#endif
