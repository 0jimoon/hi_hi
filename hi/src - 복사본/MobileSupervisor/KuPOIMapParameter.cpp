#include "stdafx.h"
#include "KuPOIMapParameter.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#endif

KuPOIMapParameter::KuPOIMapParameter()
{

}

KuPOIMapParameter::~KuPOIMapParameter()
{

}

bool  KuPOIMapParameter::initialize()
{	
	string strInIFileName = "./data/map/POI_Map.txt";  	//�⺻���� �̸��� kuns.ini���Ͽ� ��õǾ� �ִ�. 
	m_pINIReaderWriter = new KuINIReadWriter(strInIFileName); //�������� �б�

	if (m_pINIReaderWriter->ParseError() < 0) { //������ ���� ���� ��� ���α׷��� �����Ų��.
		cout << "Can't load "<<strInIFileName<<endl;;
		_getch();
		return false;
	}

	const char* constch;	
	m_nTotalfloorNum = m_pINIReaderWriter->getIntValue("BUILDING", "FLOOR", 1);

	char cMapPathName[200];
	memset(cMapPathName,0,sizeof(cMapPathName));


	for(int i=0; i<m_nTotalfloorNum;i++)
	{
		POIMapPara PIOMapData;
		memset(cMapPathName,0,sizeof(cMapPathName));
		sprintf_s(cMapPathName,"FLOOR&NUM%d", i);	
		int nFloor= m_pINIReaderWriter->getIntValue("BUILDING", cMapPathName, 1);
		PIOMapData.Floor=nFloor;

		memset(cMapPathName,0,sizeof(cMapPathName));
		sprintf_s(cMapPathName,"FLOOR_%d", nFloor);	
		int nTotalIDNum= m_pINIReaderWriter->getIntValue(cMapPathName, "POIID", 1);
		
		PIOMapData.POIIDNum=nTotalIDNum;

		for(int j=0; j<nTotalIDNum;j++)
		{
			memset(cMapPathName,0,sizeof(cMapPathName));
			sprintf_s(cMapPathName,"FLOOR_%d_%d", nFloor,j);	
			string strPOIName= m_pINIReaderWriter->getStringValue(cMapPathName, "POI&NAME", "no");
			float fRobX= (float)m_pINIReaderWriter->getDoubleValue(cMapPathName, "ROBX", 0.0);
			float fRobY= (float)m_pINIReaderWriter->getDoubleValue(cMapPathName, "ROBY", 0.0);
			float fRobTh= (float)m_pINIReaderWriter->getDoubleValue(cMapPathName, "ROBTH", 0.0);
			PIOMapData.POIID.push_back(j);
			PIOMapData.POIName.push_back(strPOIName);
			PIOMapData.ROBX.push_back(fRobX);
			PIOMapData.ROBY.push_back(fRobY);
			PIOMapData.ROBTh.push_back(fRobTh);
		}
		m_vecPOIMapPara.push_back(PIOMapData);
	}

	return true;

}

void KuPOIMapParameter::saveParameter()
{

	string strInIFileName = "./ini/KUNS.ini";  	//�⺻���� �̸��� kuns.ini���Ͽ� ��õǾ� �ִ�. 

	CString strTemp;

		
	strTemp.Format(L"%d",m_nTotalfloorNum);
	WritePrivateProfileString(_T("MAP"), _T("FLOOR"), strTemp,_T("./ini/KUNS.ini"));


	strTemp.Format(L"%d",m_nMapSizeXm);
	WritePrivateProfileString(_T("MAP"), _T("MAP_SIZE_X"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%d",m_nMapSizeYm);
	WritePrivateProfileString(_T("MAP"), _T("MAP_SIZE_Y"), strTemp,_T("./ini/KUNS.ini"));


	strTemp.Format(L"%f",m_initRobotPos.getX());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_XPOSE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_initRobotPos.getY());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_YPOSE"), strTemp,_T("./ini/KUNS.ini"));
	strTemp.Format(L"%0.2f",m_initRobotPos.getThetaDeg());
	WritePrivateProfileString(_T("ROBOT"), _T("INIT_THETADEG"), strTemp,_T("./ini/KUNS.ini"));
	
}

void KuPOIMapParameter::setInitRobotPose(KuPose initRobotPos)
{
	m_initRobotPos=initRobotPos;
}

void KuPOIMapParameter::setTotalfloorNum(int nTotalfloorNum)
{
	m_nTotalfloorNum = nTotalfloorNum; 
}

void KuPOIMapParameter::setCurfloor(int nCurfloor)
{
	m_nCurfloor = nCurfloor; 
}

void KuPOIMapParameter::setMapSize(int nSizeXm, int nSizeYm)
{
	//�ۼ��� ������ ũ�⸦ �����Ѵ�. 
	m_nMapSizeXm = nSizeXm; //���� x,y ũ�� ����: m�� ����
	m_nMapSizeYm = nSizeYm; //���� x,y ũ�� ����: m�� ����

}

void KuPOIMapParameter::setPIOMapData(POIMapPara PIOMapData)  
{
	m_vecPOIMapPara.push_back(PIOMapData);
}

//=============================================================================================================================


//get �Լ����--------------------------------------------------------------------------------------------------------------------------

KuPose KuPOIMapParameter::getInitRobotPose()
{
	return m_initRobotPos;
}


int KuPOIMapParameter::getTotalfloorNum()
{
	return m_nTotalfloorNum;
}
int KuPOIMapParameter::getCurfloor()
{
	return m_nCurfloor;
}


int KuPOIMapParameter::getMapSizeXm()
{
	//���� x,y ũ�⸦ �Ѱ��ش�.
	return m_nMapSizeXm;
}
int KuPOIMapParameter::getMapSizeYm()
{
	//���� x,y ũ�⸦ �Ѱ��ش�.
	return m_nMapSizeYm;
}

POIMapPara KuPOIMapParameter::getPIOMapData(int nIndex)  
{
	return m_vecPOIMapPara[nIndex];
}


vector<POIMapPara> KuPOIMapParameter::getvecPIOMapData()
{
	return m_vecPOIMapPara;
}
//======================================================================================================================================

