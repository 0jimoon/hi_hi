/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mecanical Engineering Korea University                                    
(c) Copyright 2007 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : .
$Data: 2007/09                                                                           
$Author:                                                                      
______________________________________________________________________________________________*/

#include <iostream>
#include "../../KUNSUtil/KUNSThread/KuThread.h"
#include "../../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../../KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "../../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../../KUNSUtil/KuUtil.h"
#include "../Sensor.h"
#include "RPLidarProxy.h"

using namespace std;

class RplidarInterface : public KuSingletone <RplidarInterface>
{
private:
		KuUtil m_KuUtil;
		CCriticalSection m_CriticalSection;

public:
	static const int MAX_DISTANCE = 5000; //최대탐지거리 단위 mm
	static const int MIN_DISTANCE = 100; //최소탐지거리 단위 mm
	static const int GET_181 = 0;
	static const int GET_241 = 1;
	static const int GET_481= 2;
	static const int GET_961 = 3;

private:
	KuThread m_Thread;
	
	int_1DArray m_LRFRangeData_181;// 181개 레이저 데이터
	int_1DArray m_LRFRangeData_241;// 241개 레이저 데이터
	int_1DArray m_LRFRangeData_481;// 481개 레이저 데이터
	int_1DArray m_LRFRangeData_961;// 961개 레이저 데이터

	
	int m_nMaxDistance, m_nMinDistance; //레이저 스캐너의 최대값과 최소값 단위 mm

	char* m_ComPort;
	RPLidarProxy RPLidar;
	bool m_bConnected;
	bool m_doThreadFunc;
	bool m_bCaptureEndFlag;
	long m_timestamp ;
	vector<long> m_data;


public:
	static void Thread_function(LPVOID arg);
	void start();
	void terminate();
	void suspend();
	void resume();

	int_1DArray getData();

	void setComPort(char* ComPort);
	char* getComPort();
	void capture();
	void setData(int_1DArray LRFRangeData_181);
	bool connectLaserScanner();
	void disconnectLaserScanner();
	void setMaxDistance(int nDistance); //최대 탐지거리를 설정하는 함수.
	void setMinDistance(int nDistance); //최소 탐지거리를 설정하는 함수.

	RplidarInterface();
	~RplidarInterface();

};
