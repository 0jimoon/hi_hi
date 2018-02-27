#ifndef KUNS_VIRTUAL_RANGE_INTERFACE_H
#define KUNS_VIRTUAL_RANGE_INTERFACE_H

#include <iostream>
#include "../../Sensor/Sensor.h"

#include "../../KUNSMap/KuMap.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../../KUNSPose/KuPose.h"
#include "../../KUNSMath/KuMath.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"


using namespace std;
class  KuVrRangeInterface : public KuSingletone <KuVrRangeInterface>
{
private:
	static const int MAX_LASER_DISTANCE = 2000;
	static const int MAX_LASER_DISTANCE_181 = 20000;

	static const int MAX_LASER_IDX = 361;
	static const int MAX_LASER_IDX_181 = 181;

private:
	KuUtil m_KuUtil;
	int** m_nMap;
	int m_nMapX;
	int m_nMapY;
	int_1DArray m_nLaserData360;
	int_1DArray m_nLaserData181;
	KuMath m_math;

public:
	bool connect(int** nMap,int nMapX,int nMapY);
	int_1DArray getData(KuPose RobotPos);
	int_1DArray getData(KuPose RobotPos,vector<KuPose> vecRobotPos );
	int_1DArray getData181(KuPose RobotPos);

	KuVrRangeInterface();
	virtual ~KuVrRangeInterface();

};

#endif /*KUNS_VIRTUAL_HOKUYO_UTM30LX_INTERFACE_H*/
