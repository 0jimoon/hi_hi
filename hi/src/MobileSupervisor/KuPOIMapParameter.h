
#ifndef KUNS_POI_MAP_PARAMETER_H
#define KUNS_POI_MAP_PARAMETER_H

#include <conio.h>
#include <iostream>
#include "../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "KuCommandMessage.h"


struct POIMapPara
{
	int Floor;
	int POIIDNum;
	vector<string> POIName;
	vector<int> POIID;
	vector<float> ROBX;
	vector<float> ROBY; 
	vector<float> ROBTh; 
};


using namespace std;
class KuPOIMapParameter : public KuSingletone <KuPOIMapParameter>
{

private:
	KuINIReadWriter* m_pINIReaderWriter;


	int m_nMapSizeXm ; //지도 x,y 크기 단위: m로 설정.
	int m_nMapSizeYm ;
	int m_nTotalfloorNum ;
	int m_nCurfloor;

	KuPose m_initRobotPos;

	vector<POIMapPara> m_vecPOIMapPara;


public:
	bool initialize(); //시스템 설정에 관련된 모든 변수들을 초기화 시켜주는 함수.
	void saveParameter();

	//set 함수목록-----------------------------------------------------------------------------------------------------------------------
	void setInitRobotPose(KuPose initRobotPos);

	//Map 관련 파라미터
	void setMapSize(int nSizeXm, int nSizeYm); //지도크기를 설정하는 함수.

	void setTotalfloorNum(int nTotalfloorNum);
	void setCurfloor(int nCurfloor);

	void setPIOMapData(POIMapPara PIOMapData);  
	//=====================================================================================================================================
	//=====================================================================================================================================


	//get 함수목록--------------------------------------------------------------------------------------------------------------------------


	KuPose getInitRobotPose();
	int getTotalfloorNum();
	int getCurfloor();

	//Map 관련 파라미터
	int getMapSizeXm(); //지도크기를 설정하는 함수.
	int getMapSizeYm(); //지도크기를 설정하는 함수.
	
	POIMapPara getPIOMapData(int nIndex);  

	vector<POIMapPara> getvecPIOMapData();

	//======================================================================================================================================
	//======================================================================================================================================


public:
	KuPOIMapParameter();
	virtual ~KuPOIMapParameter();
};

#endif

