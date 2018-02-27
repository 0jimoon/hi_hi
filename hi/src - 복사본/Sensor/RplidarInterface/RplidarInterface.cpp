
#include "stdafx.h"
#include "RplidarInterface.h"

RplidarInterface::RplidarInterface()
{
	m_doThreadFunc=false;
	m_timestamp = 0;
	m_bConnected = false;
	m_bCaptureEndFlag=false;

	m_LRFRangeData_181=	 m_KuUtil.generateIntType1DArray(Sensor::RPLIDAR_DATA_NUM181,0);

	m_nMaxDistance = MAX_DISTANCE; //���� mm
	m_nMinDistance = MIN_DISTANCE; //������ ��ĳ���� �ִ밪�� �ּҰ�
}
RplidarInterface::~RplidarInterface()
{
	RPLidar.close();
}
/**
 @brief Korean: Laser scanner�� �ִ� ���� �����Ѵ�. 
 @brief English: 
*/
void RplidarInterface::setMaxDistance(int nDistance)
{
	//�ִ� Ž���Ÿ��� �����ϴ� �Լ�.
	if(nDistance > MAX_DISTANCE ){
		m_nMaxDistance = MAX_DISTANCE;
	}else{
		m_nMaxDistance = nDistance;
	}
}
/**
 @brief Korean: Laser scanner�� �ּ� ���� �����Ѵ�. 
 @brief English: 
*/
void RplidarInterface::setMinDistance(int nDistance)
{
	//�ּ� Ž���Ÿ��� �����ϴ� �Լ�.
	if(nDistance < MIN_DISTANCE ){
		m_nMinDistance = MIN_DISTANCE;
	}else{
		m_nMinDistance = nDistance;
	}

}
/**
 @brief Korean: Thread_function ���̷� ������ ���� �ֱ������� �簻���Ѵ�. 
 @brief English: 
*/
void RplidarInterface::Thread_function(LPVOID arg)
{
	RplidarInterface *pHLURGI = (RplidarInterface *) arg;	

	KuTimer Ktimer;
	while(pHLURGI->m_doThreadFunc){
		pHLURGI->m_bCaptureEndFlag= false;
		pHLURGI->capture();
		pHLURGI->m_bCaptureEndFlag= true;
		Ktimer.sleepMS(10);
	}

	printf("RplidarInterface is terminate!!!\n");

}
/**
 @brief Korean: �������� �Ÿ� ���� ������������ ���� �޾ƿ´�. 
 @brief English: 
*/
void RplidarInterface::capture()
{

	int nCnt = 0;
	
	int_1DArray LRFRangeData_181;
	int_1DArray ReverseLRFRangeData_181;

	LRFRangeData_181=	 m_KuUtil.generateIntType1DArray(Sensor::RPLIDAR_DATA_NUM181,0);
	ReverseLRFRangeData_181=	 m_KuUtil.generateIntType1DArray(Sensor::RPLIDAR_DATA_NUM181,0);
	
	int nIndex=0;

	if(m_bConnected ==true){
		RPLidarData data;
		RPLidar.readData(data);
		
		for (int j = 0; j < Sensor::RPLIDAR_DATA_NUM181; ++j) 
		{
			if(data.getDistance()[j]>=0)
			{
				nIndex=(int)(data.getAngle()[j]+0.5);
				if(nIndex<=180&&nIndex>=0)
				LRFRangeData_181[nIndex] = data.getDistance()[j];
			}			
		}

		for(int i = 0 ; i < Sensor::RPLIDAR_DATA_NUM181 ; i++){
			if(LRFRangeData_181[i]==0){
				LRFRangeData_181[i]=-1;
			}
			else if(LRFRangeData_181[i] > m_nMaxDistance){
				LRFRangeData_181[i]=-1;
			}	
			else if(LRFRangeData_181[i]<m_nMinDistance){
				LRFRangeData_181[i]=-1;
			}	
		}

		for(int i = 0 ; i < Sensor::RPLIDAR_DATA_NUM181 ; i++)
		{
			ReverseLRFRangeData_181[i] = LRFRangeData_181[180-i];
		}

		setData(ReverseLRFRangeData_181);
	}

}
/**
 @brief Korean: ������ ������ �޾ƿ��� �κ��� ������. 
 @brief English: 
*/
void RplidarInterface::terminate()
{
	m_Thread.terminate();
}
/**
 @brief Korean: ������ ������ �޾ƿ��� �κ��� ��� �����. 
 @brief English: 
*/
void RplidarInterface::suspend()
{
	m_Thread.suspend();
}
/**
 @brief Korean: ������ ������ �޾ƿ��� �κ��� �ٽ� �簳�Ѵ�. 
 @brief English: 
*/
void RplidarInterface::resume()
{
	m_Thread.resume();

}
/**
 @brief Korean:  �ٸ� Ŭ�������� Laser�� �Ÿ� ��(181 ��)�� ��������. 
 @brief English: 
*/
int_1DArray RplidarInterface::getData()
{
	int_1DArray pLRFRangeData_181;
	//m_CriticalSection.Lock();
	m_CriticalSection.Lock();
	pLRFRangeData_181 = m_LRFRangeData_181;
	m_CriticalSection.Unlock();

	return pLRFRangeData_181;
}
/**
 @brief Korean:  �ٸ� Ŭ�������� Laser�� �Ÿ� ��(181 ��)�� �޾ƿ´�. 
 @brief English: 
*/
void RplidarInterface::setData(int_1DArray LRFRangeData_181)
{
	//m_CriticalSection.Lock();
	m_CriticalSection.Lock();
	for(int i=0;i <Sensor::RPLIDAR_DATA_NUM181;i++)
	{
		m_LRFRangeData_181[i]=LRFRangeData_181[i];
	}

	m_CriticalSection.Unlock();
}
/**
 @brief Korean: ������������ ����Ǵ� ��Ʈ�� �����Ѵ�.  
 @brief English: 
*/
void RplidarInterface::setComPort(char* ComPort)
{
	m_ComPort = ComPort;
}
/**
 @brief Korean: ������������ ����Ǵ� ��Ʈ�� �޾ƿ´�.. 
 @brief English: 
*/
char* RplidarInterface::getComPort()
{
	return m_ComPort;
}
/**
 @brief Korean: ������������ �����Ѵ�.
 @brief English: 
*/
bool RplidarInterface::connectLaserScanner()
{
	char* device = getComPort();
	if (RPLidar.open(device)) {
		printf("RPLidar::connect: Fail\n");
		
		return false;
		//exit(0);
	}else{
		m_bConnected = true;
		m_doThreadFunc=true;
		m_Thread.start(&Thread_function ,this, 100);
		return true;
	}

}
/**
 @brief Korean: ������������ ������ ���� ������ ������ Ŭ������ ���� ��Ų��.
 @brief English: 
*/
void RplidarInterface::disconnectLaserScanner()
{
	m_doThreadFunc = false;
	KuTimer Ktimer;

	if(m_bConnected==true){
		while(1){
			if(m_bCaptureEndFlag==false){
				Ktimer.sleepMS(10);
				continue;
			}
			else{
				m_bConnected = false;
				terminate();
				RPLidar.close();
				return;
			}
		}
	}
}