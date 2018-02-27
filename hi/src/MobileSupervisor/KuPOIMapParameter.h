
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


	int m_nMapSizeXm ; //���� x,y ũ�� ����: m�� ����.
	int m_nMapSizeYm ;
	int m_nTotalfloorNum ;
	int m_nCurfloor;

	KuPose m_initRobotPos;

	vector<POIMapPara> m_vecPOIMapPara;


public:
	bool initialize(); //�ý��� ������ ���õ� ��� �������� �ʱ�ȭ �����ִ� �Լ�.
	void saveParameter();

	//set �Լ����-----------------------------------------------------------------------------------------------------------------------
	void setInitRobotPose(KuPose initRobotPos);

	//Map ���� �Ķ����
	void setMapSize(int nSizeXm, int nSizeYm); //����ũ�⸦ �����ϴ� �Լ�.

	void setTotalfloorNum(int nTotalfloorNum);
	void setCurfloor(int nCurfloor);

	void setPIOMapData(POIMapPara PIOMapData);  
	//=====================================================================================================================================
	//=====================================================================================================================================


	//get �Լ����--------------------------------------------------------------------------------------------------------------------------


	KuPose getInitRobotPose();
	int getTotalfloorNum();
	int getCurfloor();

	//Map ���� �Ķ����
	int getMapSizeXm(); //����ũ�⸦ �����ϴ� �Լ�.
	int getMapSizeYm(); //����ũ�⸦ �����ϴ� �Լ�.
	
	POIMapPara getPIOMapData(int nIndex);  

	vector<POIMapPara> getvecPIOMapData();

	//======================================================================================================================================
	//======================================================================================================================================


public:
	KuPOIMapParameter();
	virtual ~KuPOIMapParameter();
};

#endif

