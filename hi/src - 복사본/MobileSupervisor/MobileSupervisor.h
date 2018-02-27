#ifndef MOBILE_SUPERVISOR_H
#define MOBILE_SUPERVISOR_H

#include <conio.h>
#include <iostream>
#include "../KUNSUtil/KUNSINIReadWriter/KuINIReadWriter.h"
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSMap/KuMapRepository.h"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../KUNSBehavior/MapBuildingBehavior/MapBuildingBehavior.h"
#include "../KUNSBehavior/GotoGoalBehavior/GotoGoalBehavior.h"
#include "KuCommandMessage.h"
#include "KuRobotParameter.h"

#include "../KUNSBehavior/VirtualBehavior/KuVrKanayaMotionControlBh.h"
#include "../KUNSProcess/KUNSWanderingObstaclePr/KuWanderingObstaclePr.h"



using namespace std;

class MobileSupervisor: public KuSingletone <MobileSupervisor>
{
private:
	MapBuildingBehavior m_MapBuildingBeh;
	GotoGoalBehavior m_GotoGoalBeh;
	KuINIReadWriter* m_pINIReaderWriter;
	int m_nSetGoalPosID,m_nSetRobotPosID;
	int m_nBhID;
	KuThread m_KuThread;
	KuThread m_KuFloorcheckThread;
	bool m_bBehaviorStates;
	vector<KuPose> m_vecWayPoint;

	KuVrKanayaMotionControlBh m_VrKanayamaMotionControlBeh;
	vector<KuPose> m_vecGoalPos;
	int m_nNum;
	int m_nCurFloor;
public:
	void execute(KuCommandMessage CMessage);
	bool loadMap();
	bool getBehaviorStates(KuCommandMessage CMessage);
	Localizer* getLocalizer(KuCommandMessage CMessage);
	void setBehaviorStates(bool  bBehaviorStates);
	bool getBehaviorStates( );
	bool loadCADMap();
	void executesim(KuCommandMessage CMessage);
	static void AutonomousSupervisor(void* arg);
	bool Autonomousexecute(KuCommandMessage CMessage );
	bool loadupdateGridMap();
	bool generatePath();
	bool setCurIndex(int nIndex);
	static void checkFloor(void* arg);

public:
	MobileSupervisor();
	virtual ~MobileSupervisor();


};

#endif