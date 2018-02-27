#include "stdafx.h"
#include "KuGlobalParhPlannerPr.h"
/**
@brief Korean: ������ �Լ�
@brief English:
*/
KuGlobalParhPlannerPr::KuGlobalParhPlannerPr()
{
	m_listPath.clear(); //��θ� �����ϴ� STL ����Ʈ �ʱ�ȭ.
	m_nMap = NULL;
	m_dMap = NULL;

	m_dRadiusofRobot=300;
	m_dProMapWeight=10.0;
    m_bPOIMapinitflag=false;
    m_bPOMinitflag=false;
}
/**
@brief Korean: �Ҹ��� �Լ�
@brief English:
*/
KuGlobalParhPlannerPr::~KuGlobalParhPlannerPr()
{

	if(m_nMap!=NULL)
	{
		for(int i = 0 ; i < m_nMapSizeX ; i++){
			delete[] m_nMap[i];
			delete[] m_dMap[i];

			m_nMap[i] = NULL;
			m_dMap[i] = NULL;

		}
		delete[] m_nMap;
		delete[] m_dMap;
	}

	cout << "[KuGlobalParhPlannerPr] : Instance is destroyed." <<endl;
}
/**
@brief Korean: �κ� ������ ������ �޾ƿ��� �Լ�
@brief English:
*/
void KuGlobalParhPlannerPr::setRadiusofRobot(int nRadiusofRobot,double dCellSizeM)
{
	m_dRadiusofRobot = (double) nRadiusofRobot;
    m_math.setCellSizeMM(dCellSizeM);
}

/**
@brief Korean: ���� ������ �޾ƿ��� �Լ�
@brief English:
*/
void KuGlobalParhPlannerPr::setGridMap(int** nMap)
{
	m_vecnMap.push_back(nMap);

}

/**
@brief Korean: ���� ������ �޾ƿ��� �Լ�
@brief English:
*/
void KuGlobalParhPlannerPr::setPOMap(double** dMap)
{	
    m_bPOMinitflag=true;
	m_vecdMap.push_back(dMap);
}

/**
@brief Korean: ���� ������ �޾ƿ��� �Լ�
@brief English:
*/
void KuGlobalParhPlannerPr::setPOIMap(vector<POIMapPara> vecPOIMapData)
{
    m_bPOIMapinitflag=true;
	m_vecPOIMap=vecPOIMapData;
}


/**
@brief Korean: ������ ũ�⸦ �޾ƿͼ� ������ �ʱ�ȭ �Լ�
@brief English:
*/
void KuGlobalParhPlannerPr::initializeMapSize(int nMapSizeX, int nMapSizeY)
{
	m_nMapSizeX=nMapSizeX;
	m_nMapSizeY=nMapSizeY;
	initialize();
	m_vecnMap.clear();
	m_vecdMap.clear();
}
/**
@brief Korean: ������ �ʱ�ȭ�ϴ� �Լ� 
@brief English:
*/
void KuGlobalParhPlannerPr::initialize()
{
    m_KuGPPPMPr.setRadiusofRobot(m_dRadiusofRobot);
    m_KuGPPPMPr.initIntCost(m_dRadiusofRobot/m_math.getCellSizeMM()+1);
	m_KuGPPPMPr.initializeMapSize(m_nMapSizeX,m_nMapSizeY);

	m_KUGPPSLPr.setRadiusofRobotp(m_dRadiusofRobot);
	m_KUGPPSLPr.initIntCost(m_dRadiusofRobot/m_math.getCellSizeMM()+1);	
	m_KUGPPSLPr.initializeMapSize(m_nMapSizeX,m_nMapSizeY);
}
/**
@brief Korean: ���� �ʱ�ȭ �Լ�
@brief English:
*/
void KuGlobalParhPlannerPr::initMap()
{
    if(m_nMap==NULL)
    {
    m_nMap = new int*[m_nMapSizeX];

    if(m_nMap){
        for(int i = 0 ; i < m_nMapSizeX ; i++){
            m_nMap[i] = new int[m_nMapSizeY];

        }
    }
    }
}
/**
@brief Korean: ������ �������� �������� �̿��� ��θ� �����ϴ� �Լ�
@brief English:
*/
int KuGlobalParhPlannerPr::generatePath(float fGoalPosX,float fGoalPosY,int nGoalFloor, float fRobotPosX, float fRobotPosY,int nRobotFloor)
{
    if(!m_bPOIMapinitflag){nRobotFloor=nGoalFloor=0;}
	m_listPath.clear();

    if(nRobotFloor==nGoalFloor)
	{
		int nSelIDX=0;
		for(int i=0;i<m_vecPOIMap.size();i++)
		{
			if(m_vecPOIMap[i].Floor==nRobotFloor)
			{
				nSelIDX=i;
			}
		}

		KuPose RobotPos; KuPose GoalPos;

		RobotPos.setXm(fRobotPosX);		RobotPos.setYm(fRobotPosY);
		GoalPos.setXm(fGoalPosX);		GoalPos.setYm(fGoalPosY);

//         m_KuGPPPMPr.setMap(m_vecnMap[nSelIDX],m_math.getCellSizeMM());
//         if(m_bPOMinitflag){ m_KuGPPPMPr.setProMap(m_vecdMap[nSelIDX]);}
//        m_KuGPPPMPr.generatePath(GoalPos, RobotPos); //��� ����
// 
// 		list<KuPose> listPath = m_KuGPPPMPr.getPath();

		m_KUGPPSLPr.setMap(m_vecnMap[nSelIDX],m_math.getCellSizeMM()); 
		if(m_bPOMinitflag)m_KUGPPSLPr.setProMap(m_vecdMap[nSelIDX]); 
		m_KUGPPSLPr.generatePath(GoalPos, RobotPos); //��� ����

		list<KuPose> listPath = m_KUGPPSLPr.getPath();

		if(listPath.size()==0){
         cout<<"ALL_PATH_BLOCKED"<<endl;
			return false; 
		}//��λ��� ������ ���

		list<KuPose> listSPath1=m_KuPathSmoothing.smoothingPath(listPath);

		list<KuPose>::iterator it;
		for (it = listSPath1.begin(); it != listSPath1.end(); it++) {
			it->setID(nRobotFloor);
			m_listPath.push_back(*it);
		}

	}
	else
	{
		int nSelRobotIDX=0;
		int nSelGoalIDX=0;

		int nSelRElveIDX=-1;
		int nSelGElveIDX=-1;
		//�� �ε��� ã��
		for(int i=0;i<m_vecPOIMap.size();i++)
		{
			if(m_vecPOIMap[i].Floor==nRobotFloor)
			{
				nSelRobotIDX=i;
			}
			if(m_vecPOIMap[i].Floor==nGoalFloor)
			{
				nSelGoalIDX=i;
			}
		}
		//�κ���ġ�� ���������� ã��
		for(int i=0;i<m_vecPOIMap[nSelRobotIDX].POIName.size();i++)
		{
			if(m_vecPOIMap[nSelRobotIDX].POIName[i]=="Elevator1")
			{
				nSelRElveIDX=i;
			}
		}
		//����ġ�� ���������� ã��
		for(int i=0;i<m_vecPOIMap[nSelGoalIDX].POIName.size();i++)
		{
			if(m_vecPOIMap[nSelGoalIDX].POIName[i]=="Elevator1")
			{
				nSelGElveIDX=i;
			}
		}

		if(nSelRElveIDX==-1||nSelGElveIDX==-1)
		{
			return false;
		}

		KuPose RobotPos; KuPose GoalPos; KuPose WayPoint ;

		RobotPos.setXm(fRobotPosX);		RobotPos.setYm(fRobotPosY);
		GoalPos.setXm(fGoalPosX);		GoalPos.setYm(fGoalPosY);

		WayPoint.setXm(m_vecPOIMap[nSelRobotIDX].ROBX[nSelRElveIDX]);
		WayPoint.setYm(m_vecPOIMap[nSelRobotIDX].ROBY[nSelRElveIDX]); 

        m_KuGPPPMPr.setMap(m_vecnMap[nSelRobotIDX],m_math.getCellSizeMM());
           if(m_bPOMinitflag) m_KuGPPPMPr.setProMap(m_vecdMap[nSelRobotIDX]);
		m_KuGPPPMPr.generatePath(WayPoint, RobotPos); //��� ����

		list<KuPose> listPath = m_KuGPPPMPr.getPath();
		if(listPath.size()==0){
			return false; 
		}//��λ��� ������ ���

		list<KuPose> listSPath1 =m_KuPathSmoothing.smoothingPath(listPath);
		listPath.clear();

		WayPoint.setXm(m_vecPOIMap[nSelGoalIDX].ROBX[nSelGElveIDX]);
		WayPoint.setYm(m_vecPOIMap[nSelGoalIDX].ROBY[nSelGElveIDX]); 

        m_KuGPPPMPr.setMap(m_vecnMap[nSelGoalIDX],m_math.getCellSizeMM());
		m_KuGPPPMPr.setProMap(m_vecdMap[nSelGoalIDX]); 
		m_KuGPPPMPr.generatePath(GoalPos, WayPoint); //��� ����

		listPath = m_KuGPPPMPr.getPath();
		if(listPath.size()==0){
			return false; 
		}//��λ��� ������ ���

		list<KuPose> listSPath2 =m_KuPathSmoothing.smoothingPath(listPath);

		list<KuPose>::iterator it;

		for (it = listSPath1.begin(); it != listSPath1.end(); it++) {
			it->setID(nRobotFloor);
			m_listPath.push_back(*it);
		}

		for (it = listSPath2.begin(); it != listSPath2.end(); it++) {
			it->setID(nGoalFloor);
			m_listPath.push_back(*it);
		}
	}

	return true;
}
float KuGlobalParhPlannerPr::calPathDist()
{

	list<KuPose>::iterator it;
	float fTotalDist=0;
	for (it = m_listPath.begin(); it != m_listPath.end(); it++) {

		double dDist =hypot(it->getXm(),it->getYm());
		fTotalDist+=dDist;
	}
	return fTotalDist; 
}

list<KuPose> KuGlobalParhPlannerPr::getPath() 
{
	return m_listPath;
}

void KuGlobalParhPlannerPr::setProMapWeight(double dProMapWeight)
{
	m_dProMapWeight=dProMapWeight;
}
