#include "stdafx.h"
#include "KuGradientPathPlannerbasedPIPr.h"
/**
@brief Korean: ������ �Լ�
@brief English:
*/
KuGradientPathPlannerbasedPIPr::KuGradientPathPlannerbasedPIPr()
{
	m_listPath.clear(); //��θ� �����ϴ� STL ����Ʈ �ʱ�ȭ.
	m_nMap = NULL;
	m_fIntCost = NULL;
	m_fAdjCost = NULL;
	m_fNavCost = NULL;
	m_finitIntCost=NULL;
	m_dMap = NULL;
 	m_fPreictCost=NULL;
	m_fRestCost=NULL;
	m_fRemovalCost=NULL;

	m_nCSpaceObstacle=CSPACE_OBSTACLE;
	m_nCSpaceInfinity=CSPACE_OBSTACLE_INFINITY;
	m_dRadiusofRobot=300;
    m_dCellSize=100;
	m_dProMapWeight=10.0;
	m_binitProMapflag=false;

	initIntCost(CSPACE_OBSTACLE);
}
/**
@brief Korean: �Ҹ��� �Լ�
@brief English:
*/
KuGradientPathPlannerbasedPIPr::~KuGradientPathPlannerbasedPIPr()
{
	
	if(m_nMap!=NULL)
	{
		for(int i = 0 ; i < m_nMapSizeX ; i++){
			delete[] m_nMap[i];
			delete[] m_fIntCost[i];
			delete[] m_fAdjCost[i];
			delete[] m_fNavCost[i];
			delete[] m_dMap[i];
			delete[] m_fPreictCost[i];
			delete[] m_fRestCost[i];
			delete[] m_fRemovalCost[i];

			m_nMap[i] = NULL;
			m_fIntCost[i] = NULL;
			m_fAdjCost[i] = NULL;
			m_fNavCost[i] = NULL;
			m_dMap[i] = NULL;
			m_fPreictCost[i] = NULL;
			m_fRestCost[i] = NULL;
			m_fRemovalCost[i] = NULL;
		}
		delete[] m_nMap;
		delete[] m_fIntCost;
		delete[] m_fAdjCost;
		delete[] m_fNavCost;
		delete[] m_dMap;
		delete[] m_fPreictCost;
		delete[] m_fRestCost;
		delete[] m_fRemovalCost;
	}


	for(int i = 0 ; i < (m_nCSpaceObstacle*2)+1 ; i++){
		delete[] m_finitIntCost[i];
	}
	delete[] m_finitIntCost;

	cout << "[KuGradientPathPlannerbasedProMapPr] : Instance is destroyed." <<endl;
}
/**
@brief Korean: �κ� ������ ������ �޾ƿ��� �Լ�
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::setRadiusofRobot(int nRadiusofRobot)
{
        m_dRadiusofRobot = (double) nRadiusofRobot;
}
/**
@brief Korean: ���� ������ �޾ƿ��� �Լ�
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::setMap(int** nMap,double dCellSize)
{
		m_smtnMap = nMap;
        m_math.setCellSizeMM(dCellSize);
        m_dCellSize=dCellSize;
}

/**
@brief Korean: ���� ������ �޾ƿ��� �Լ�
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::setProMap(double** dMap)
{	
    m_binitProMapflag=true;

    if(m_dMap==NULL)
	{
		m_dMap = new double*[m_nMapSizeX];

		for(int i=0; i <m_nMapSizeX; i++){
			m_dMap[i] = new double[m_nMapSizeY];
		}

		for(int i=0;i<m_nMapSizeX; i++){
			for(int j=0; j<m_nMapSizeY; j++){
				m_dMap[i][j] = 1;
			}
		}
	}

	for(int i=0; i<m_nMapSizeX; i++){
		for(int j=0; j<m_nMapSizeY; j++){
			if(dMap[i][j]>0.5)
			{
				m_dMap[i][j] =(int)(dMap[i][j]*m_dProMapWeight)+1;//KuMap::OCCUPIED_AREA;
			}
			else if(dMap[i][j]<0.5)
			{
				m_dMap[i][j] =1.0;
			}
		}
	}

}

/**
@brief Korean: ������ ũ�⸦ �޾ƿͼ� ������ �ʱ�ȭ �Լ�
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::initializeMapSize(int nMapSizeX, int nMapSizeY)
{
	m_nMapSizeX=nMapSizeX;
	m_nMapSizeY=nMapSizeY;
	initialize();

}
/**
@brief Korean: ������ �ʱ�ȭ�ϴ� �Լ� 
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::initialize()
{
    m_binitProMapflag=false;

	if(m_nMap==NULL){
		m_nMap = new int*[m_nMapSizeX];
		m_fIntCost = new float*[m_nMapSizeX];
		m_fAdjCost = new float*[m_nMapSizeX];
		m_fNavCost = new float*[m_nMapSizeX];
		m_fPreictCost= new float*[m_nMapSizeX];
		m_fRemovalCost= new float*[m_nMapSizeX];
		m_fRestCost= new float*[m_nMapSizeX];
        m_dMap= new double*[m_nMapSizeX];

		if(m_nMap){
			for(int i = 0 ; i < m_nMapSizeX ; i++){
				m_nMap[i] = new int[m_nMapSizeY];
				m_fIntCost[i]  = new float[m_nMapSizeY];
				m_fAdjCost [i] = new float[m_nMapSizeY];
				m_fNavCost [i] = new float[m_nMapSizeY];
				m_fPreictCost [i] = new float[m_nMapSizeY];
				m_fRemovalCost [i] = new float[m_nMapSizeY];
				m_fRestCost [i] = new float[m_nMapSizeY];
                m_dMap [i] = new double[m_nMapSizeY];

			}
		}
	}

}
/**
@brief Korean: ���� �ʱ�ȭ �Լ�
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::initMap()
{
	int** nMap = m_smtnMap;

	for(int i=0; i<m_nMapSizeX; i++){
		for(int j=0; j<m_nMapSizeY; j++){

			m_nMap[i][j] = nMap[i][j];
			m_fPreictCost[i][j]=0;
			m_fIntCost[i][j] = 0.;
			m_fAdjCost[i][j] = 0.;
			m_fNavCost[i][j] = INFINITY_VALUE;  

			m_fRemovalCost[i][j] = 0.;
			m_fRestCost[i][j] = 0.;
            //m_dMap[i][j] =0.0;
		}
	}

}
/**
@brief Korean: ������ �������� �������� �̿��� ��θ� �����ϴ� �Լ�
@brief English:
*/
int KuGradientPathPlannerbasedPIPr::generatePath(KuPose GoalPos, KuPose RobotPos)
{
	
	m_RobotPos=RobotPos;
	m_GoalPos=GoalPos;

	initMap();
	m_listPath.clear();

	int nStartPosition[2];
    int nGoalPosition[2];

	nStartPosition[0] = (int)( m_math.MM2AI(RobotPos.getX())); //mm���¸� �迭 ���·� ��ȭ���ش�
	nStartPosition[1] = (int)( m_math.MM2AI(RobotPos.getY()));  //mm���¸� �迭 ���·� ��ȭ���ش�
	nGoalPosition[0] = (int)( m_math.MM2AI(GoalPos.getX())); //mm���¸� �迭 ���·� ��ȭ���ش�.;
	nGoalPosition[1] = (int)( m_math.MM2AI(GoalPos.getY())); //mm���¸� �迭 ���·� ��ȭ���ش�.; 

	if (nStartPosition[0]<1 || nStartPosition[0]> m_nMapSizeX || nStartPosition[1]< 1 || nStartPosition[1]> m_nMapSizeY 
		||nGoalPosition[0]<1 || nGoalPosition[0]> m_nMapSizeX || nGoalPosition[1]< 1 || nGoalPosition[1]> m_nMapSizeY )
	{
		return ALL_PATH_BLOCKED;
	}

	for (int i=-CSPACE_OBSTACLE_INFINITY; i<=CSPACE_OBSTACLE_INFINITY; i++) {
		for (int j=-CSPACE_OBSTACLE_INFINITY; j<=CSPACE_OBSTACLE_INFINITY; j++) {
			if (nStartPosition[0]+i<0 || nStartPosition[0]+i>= m_nMapSizeX ||nStartPosition[1]+ j< 0 || nStartPosition[1]+j>= m_nMapSizeY) continue;
			if (nGoalPosition[0]+i<0 || nGoalPosition[0]+i>= m_nMapSizeX ||nGoalPosition[1]+ j< 0 || nGoalPosition[1]+j>= m_nMapSizeY) continue;
			m_nMap[nStartPosition[0]+i][nStartPosition[1]+j] = 0;
			m_nMap[nGoalPosition[0]+i][nGoalPosition[1]+j] = 0;
		}
	}

	if (nStartPosition[0]<0 || nStartPosition[0]>= m_nMapSizeX ||nStartPosition[1]< 0 || nStartPosition[1]>= m_nMapSizeY) return false;
	if (nGoalPosition[0]<0 || nGoalPosition[0]>= m_nMapSizeX ||nGoalPosition[1]< 0 || nGoalPosition[1]>= m_nMapSizeY) return false;
	if (m_nMap[nStartPosition[0]][nStartPosition[1]]>0)
		return ROBOT_NEAR_OBSTACLE;
	if (m_nMap[nGoalPosition[0]][nGoalPosition[1]]>0)
		return GOAL_NEAR_OBSTACLE;

	// calculate cost--------------------------------------------------------------------------------------------------------------
 	calCastIntrinCost(m_nCSpaceObstacle);// set intrinsic cost
    calAdjCostR(nGoalPosition);// set adjacent cost
	calNavCost();// set navigation cost
	// calculate cost===============================================================================

// 	double dProMapWeight =10;
// 	for (int i=0; i<m_nMapSizeX; i++) {
// 		for (int j=0; j<m_nMapSizeY; j++) {
// 
// 			// 			if(m_dMap[i][j]>1)//==KuMap::OCCUPIED_AREA)
// 			// 			{	
// 			// 				m_fRestCost[i][j] = 1.0;//m_fAdjCost[i][j] - m_fRemovalCost[i][j];
// 			// 			}
// 
// 			m_fAdjCost[i][j]=m_fRemovalCost[i][j];//+m_fRestCost[i][j]*dProMapWeight;
// 		}
// 	}
// 
// 	calNavCost();// set navigation cost
// 	// calculate cost===============================================================================


// 	double **m_dOutMap;
// 	m_dOutMap = new double*[m_nMapSizeX];
// 	for(int i=0; i <m_nMapSizeX; i++){
// 		m_dOutMap[i] = new double[m_nMapSizeY];
// 	}
// 	for(int i=0; i<m_nMapSizeX; i++){
// 		for(int j=0; j<m_nMapSizeY; j++){
// 			m_dOutMap[i][j] = m_fNavCost[i][j];///100000.0;
// 		}
// 	}
// 
// 	KuDrawingInfo::getInstance()->setPathMap(m_dOutMap,m_nMapSizeX,m_nMapSizeY);
	//copyLocalMapToGlobalMap(RobotPos);

    if( extractGradientPath(nStartPosition,nGoalPosition ) ){ //path ����
		return CORRECT_PATH_PLANNED;
    }
    else{
		m_listPath.clear();
		return ALL_PATH_BLOCKED;
    }
}
void KuGradientPathPlannerbasedPIPr::copyLocalMapToGlobalMap(KuPose RobotPos)
{

	int nLocalMapSize = (4000*2)/(double)100; //*2�� LOCALGOAL_DISTANCE�� ������ ���̶�, /100�� mm������ ���ڴ������ϱ� ����
	int m_nLocalMapX=nLocalMapSize+50;
	int m_nLocalMapY=nLocalMapSize+50;
	int m_nGlobalMapX=1000;
	int m_nGlobalMapY=1000;

	double **m_dOutMap;
	m_dOutMap = new double*[m_nGlobalMapX];
	for(int i=0; i <m_nGlobalMapX; i++){
		m_dOutMap[i] = new double[m_nGlobalMapY];
	}
	RobotPos=KuDrawingInfo::getInstance()->getRobotPos();

	int nRobotX = (int)( m_math.MM2AI(RobotPos.getX()));
	int nRobotY = (int)( m_math.MM2AI(RobotPos.getY()));
	int nX=0., nY=0;
	int nHalfSizeOfLocalMapX =m_nLocalMapX/2;
	int nHalfSizeOfLocalMapY =m_nLocalMapY/2;


	for(int i=0;i<m_nLocalMapX; i++){
		for(int j=0; j<m_nLocalMapY; j++){
			nX = nRobotX - nHalfSizeOfLocalMapX + i;
			nY = nRobotY - nHalfSizeOfLocalMapY + j;
			if(m_nLocalMapX-1<i||i<1||m_nLocalMapY-1<j||j<1) continue;
			if(m_nGlobalMapX-1<nX||nX<1||m_nGlobalMapY-1<nY||nY<1) continue;

			m_dOutMap[nX][nY] = m_fNavCost[i][j];///100000.0;

		}
	}


	KuDrawingInfo::getInstance()->setPathMap(m_dOutMap,m_nGlobalMapX,m_nGlobalMapY);
}
/**
@brief Korean:  ���� ����Լ���  ����ϴ�  �Լ�
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::calNavCost()
{
    for (int i=0; i<m_nMapSizeX; i++) {
        for (int j=0; j<m_nMapSizeY; j++) {
			m_fNavCost[i][j] = m_fIntCost[i][j] + m_fRemovalCost[i][j];
        }
    }
}

/**
@brief Korean: ��������� ����ϴ� �Լ�
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::calAdjCost(int nGoalPosition[2])
{
           
        int **nActiveList;
        int **nNewActiveList;
        int nActiveListNo=0;
        int nNewActiveListNo = 0;

        // create sufficient number of variable for active list
		nActiveList = new int*[2];
		nNewActiveList= new int*[2];
		if(nActiveList){
			for(int i = 0 ; i < 2 ; i++){
				nActiveList[i] = new int[m_nMapSizeY*m_nMapSizeX];
				nNewActiveList[i]= new int[m_nMapSizeY*m_nMapSizeX];
			}
		}

        // first active list
        nActiveList[0][0] = nGoalPosition[0];
        nActiveList[1][0] = nGoalPosition[1];
        nActiveListNo = 1;

        // ------------------------------------------------------------------------------ //
        // repeat calculation until adjacent cost of all cell is calculated.
        bool bResult, bComplete;
        int nCurrent_i, nCurrent_j;
        float fCurrentValue;
        bComplete = false;

        while ( (nActiveListNo > 0) && (bComplete==false) ) {
                nNewActiveListNo = 0;
                for (int i=0; i<nActiveListNo; i++) {
                        nCurrent_i = nActiveList[0][i];
                        nCurrent_j = nActiveList[1][i];
                        fCurrentValue = m_fAdjCost[nCurrent_i][nCurrent_j];

                        // finish the operation when the current cell meets start point
                       //if ( abs(nCurrent_i-nStartPosition[0])<=1 && abs(nCurrent_j-nStartPosition[1])<=1 ) bComplete = true;
                        // need to test..... ex. Is this the optimal path when moving obstacle exists....

                        bResult = calAdjCostUp(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if upper cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i-1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostDown(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if lower cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i+1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostLeft(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if left cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j-1;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostRight(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if right cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j+1;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostUpRight(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if upper cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i-1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j+1;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostDownLeft(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if lower cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i+1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j-1;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostLeftUp(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if left cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i-1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j-1;
                                nNewActiveListNo++;
                        }

                        bResult = calAdjCostRightDown(nCurrent_i,nCurrent_j,fCurrentValue);
                        if (bResult) {
                                // register new active list if right cell is updated.
                                nNewActiveList[0][nNewActiveListNo] = nCurrent_i+1;
                                nNewActiveList[1][nNewActiveListNo] = nCurrent_j+1;
                                nNewActiveListNo++;
                        }
                        
                }

                // copy new active list to current active list for next operation
                nActiveListNo = nNewActiveListNo;
                for(int j=0; j<nActiveListNo; j++) {
                        nActiveList[0][j] = nNewActiveList[0][j];
                        nActiveList[1][j] = nNewActiveList[1][j];
                }

                if (nActiveListNo > hypot(m_nMapSizeX,m_nMapSizeY) * 100 ) {
                        printf("Error!! Error!! Active list no. overflow error!!\n");
                        printf("Active list no = %d\n", nActiveListNo);

						for(int i = 0 ; i < 2 ; i++){
							delete[] nActiveList[i];
							delete[] nNewActiveList[i];
						}
						delete[] nActiveList;
						delete[] nNewActiveList;

						return;
                }
		
        }
        // ------------------------------------------------------------------------------ //
	
        // delete dynamic variable
      
		for(int i = 0 ; i < 2 ; i++){
			delete[] nActiveList[i];
			delete[] nNewActiveList[i];
		}
		delete[] nActiveList;
		delete[] nNewActiveList;


}
/**
@brief Korean: �� ��ġ�κ��� ���ʿ����� ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostUp(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
	 	if (m_fIntCost[nI-1][nJ] == INFINITY_VALUE){m_fAdjCost[nI-1][nJ] = INFINITY_VALUE; return false;}	
		if (m_smtnMap[nI-1][nJ] == KuMap::UNKNOWN_AREA) return false;		
		if (m_smtnMap[nI-1][nJ] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of upper cell
        if ( (m_fAdjCost[nI-1][nJ] > (fValue+1.0)) || (m_fAdjCost[nI-1][nJ]==0) ) {
                m_fAdjCost[nI-1][nJ] = (fValue+1.0);
                return true;
        } else return false;
}
/**
@brief Korean: �� ��ġ�κ��� �Ʒ��� ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostDown(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
        if (m_fIntCost[nI+1][nJ] == INFINITY_VALUE){m_fAdjCost[nI+1][nJ] = INFINITY_VALUE; return false;}	
		if (m_smtnMap[nI+1][nJ] == KuMap::UNKNOWN_AREA) return false;		
		if (m_smtnMap[nI+1][nJ] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of lower cell
        if ( (m_fAdjCost[nI+1][nJ] > (fValue+1.0)) || (m_fAdjCost[nI+1][nJ]==0) ) {
                m_fAdjCost[nI+1][nJ] = (fValue+1.0);
                return true;
        } else return false;
}
/**
@brief Korean: �� ��ġ�κ��� ���� ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostLeft(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
        if (m_fIntCost[nI][nJ-1] == INFINITY_VALUE){m_fAdjCost[nI][nJ-1] = INFINITY_VALUE; return false;}	
		if (m_smtnMap[nI][nJ-1] == KuMap::UNKNOWN_AREA) return false;	
		if (m_smtnMap[nI][nJ-1] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of left cell
        if ( (m_fAdjCost[nI][nJ-1] > (fValue+1.0)) || (m_fAdjCost[nI][nJ-1]==0) ) {
                m_fAdjCost[nI][nJ-1] = (fValue+1.0);
                return true;
        } else return false;
}
/**
@brief Korean: �� ��ġ�κ��� ������ ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostRight(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
        if (m_fIntCost[nI][nJ+1] == INFINITY_VALUE) {m_fAdjCost[nI][nJ+1] = INFINITY_VALUE; return false;}	
		if (m_smtnMap[nI][nJ+1] == KuMap::UNKNOWN_AREA) return false;	
		if (m_smtnMap[nI][nJ+1] == KuMap::OCCUPIED_AREA) return false;		

        // calculate adjcent cost of right cell
        if ( (m_fAdjCost[nI][nJ+1] > (fValue+1.0)) || (m_fAdjCost[nI][nJ+1]==0) ) {
                m_fAdjCost[nI][nJ+1] = (fValue+1.0);
                return true;
        } else return false;
}
/**
@brief Korean: �� ��ġ�κ��� ��,������ ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostUpRight(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
        if (m_fIntCost[nI-1][nJ+1] == INFINITY_VALUE){m_fAdjCost[nI-1][nJ+1] = INFINITY_VALUE; return false;}	
		if (m_smtnMap[nI-1][nJ+1] == KuMap::UNKNOWN_AREA) return false;	
		if (m_smtnMap[nI-1][nJ+1] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of upper cell
        if ( (m_fAdjCost[nI-1][nJ+1] > (fValue+1.4)) || (m_fAdjCost[nI-1][nJ+1]==0) ) {
                m_fAdjCost[nI-1][nJ+1] = (fValue+1.4);
                return true;
        } else return false;
}
/**
@brief Korean: �� ��ġ�κ��� �Ʒ�,���� ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostDownLeft(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
        if (m_fIntCost[nI+1][nJ-1] == INFINITY_VALUE){m_fAdjCost[nI+1][nJ-1] = INFINITY_VALUE; return false;}	
		if (m_smtnMap[nI+1][nJ-1] == KuMap::UNKNOWN_AREA) return false;		
		if (m_smtnMap[nI+1][nJ-1] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of lower cell
        if ( (m_fAdjCost[nI+1][nJ-1] > (fValue+1.4)) || (m_fAdjCost[nI+1][nJ-1]==0) ) {
                m_fAdjCost[nI+1][nJ-1] = (fValue+1.4);
                return true;
        } else return false;
}
/**
@brief Korean: �� ��ġ�κ��� ��,���� ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostLeftUp(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
        if (m_fIntCost[nI-1][nJ-1] == INFINITY_VALUE) {m_fAdjCost[nI-1][nJ-1] = INFINITY_VALUE; return false;}
		if (m_smtnMap[nI-1][nJ-1] == KuMap::UNKNOWN_AREA) return false;		
		if (m_smtnMap[nI-1][nJ-1] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of left cell
        if ( (m_fAdjCost[nI-1][nJ-1] > (fValue+1.4)) || (m_fAdjCost[nI-1][nJ-1]==0) ) {
                m_fAdjCost[nI-1][nJ-1] = (fValue+1.4);
                return true;
        } else return false;
}
/**
@brief Korean: �� ��ġ�κ��� �Ʒ�,������ ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostRightDown(int nI, int nJ, float fValue)
{
        // boundary condition
        if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
        if (m_fIntCost[nI+1][nJ+1] == INFINITY_VALUE){m_fAdjCost[nI+1][nJ+1]  = INFINITY_VALUE; return false;}	
		if (m_smtnMap[nI+1][nJ+1] == KuMap::UNKNOWN_AREA) return false;		
		if (m_smtnMap[nI+1][nJ+1] == KuMap::OCCUPIED_AREA) return false;		
        // calculate adjcent cost of right cell
        if ( (m_fAdjCost[nI+1][nJ+1] > (fValue+1.4)) || (m_fAdjCost[nI+1][nJ+1]==0) ) {
                m_fAdjCost[nI+1][nJ+1] = (fValue+1.4);
                return true;
        } else return false;
}

/**
@brief Korean: ��θ� �Ѱ��ִ� �Լ�
@brief English:
*/
list<KuPose> KuGradientPathPlannerbasedPIPr::getPath()
{
	return m_listPath;
}
/**
@brief Korean: ��������� ���ӵ��� ����� ���Ͽ� look-up table�� ������ִ� �Լ�
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::initIntCost(int nRange )
{
	//��ֹ� ����� ���Խ�Ų �ʱ��갪�� ������ ���� ���̴�.
	if(m_finitIntCost!=NULL)
	{
        for(int i = 0 ; i < (m_nCSpaceObstacle*2)+1 ; i++){
			delete[] m_finitIntCost[i];
		}

		delete[] m_finitIntCost;
	}
    m_nCSpaceInfinity=(int)(m_dRadiusofRobot/m_dCellSize+0.5);


	//�ʱ� ���� ����� �޸� �Ҵ�----------------------------------------------------
	m_finitIntCost = new float*[(nRange*2)+1];
	if(m_finitIntCost){
		for(int i = 0 ; i < (nRange*2)+1 ; i++){
			m_finitIntCost[i] = new float[(nRange*2)+1];
		}
	}

		m_finitPreCost = new float*[(nRange*2)+1];
		if(m_finitPreCost){
			for(int i = 0 ; i < (nRange*2)+1 ; i++){
				m_finitPreCost[i] = new float[(nRange*2)+1];
			}
		}
	//==========================================================================

	//����ũ ���� ���--------------------------------------------------------------------------
	for(int i=0 ;i<2*nRange+1;i++) {	
		for(int j=0 ;j<2*nRange+1;j++) {	
			double ddistance=sqrt((double)(i-nRange)*(i-nRange)+(j-nRange)*(j-nRange));

			if(ddistance<=m_nCSpaceInfinity){
			m_finitIntCost[i][j] =INFINITY_VALUE;
			}
			else if(ddistance>nRange-1)
			{
				m_finitIntCost[i][j] =0;
			}
			else{
				m_finitIntCost[i][j] =CSPACE_OBSTACLE_ASSISTANCE*sqrt(1-pow(((ddistance-(double)m_nCSpaceInfinity)/(double)nRange),2));//Ÿ�� �𵨸�	
			}
		}
	}

	//============================================================================
	m_nCSpaceObstacle=nRange;

}
/**
@brief Korean: ��������� ����ϴ� �Լ�
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::calCastIntrinCost(int nRange)
{  // intrinsic cost�� ����ϴ� �Լ��̴�. 
	for (int i=nRange ; i<m_nMapSizeX-nRange ; i++) {
	for (int j=nRange ; j<m_nMapSizeY-nRange ; j++){
		if ( m_nMap[i][j]== KuMap::OCCUPIED_AREA ){ 
				m_fIntCost[i][j] = INFINITY_VALUE;	
				for (int n=-nRange ; n<nRange+1 ; n++){
					for (int m=-nRange ; m<nRange+1 ; m++) 	{
						if(n+nRange>m_nMapSizeX||m+nRange>m_nMapSizeY||i+n>m_nMapSizeX||j+m>m_nMapSizeY||
							n+nRange<0||m+nRange<0||i+n<0||j+m<0){
								continue;
						}
						
					if(m_fIntCost[i+n][j+m]!= INFINITY_VALUE&& m_fIntCost[i+n][j+m] <m_finitIntCost[n+nRange][m+nRange])
					{m_fIntCost[i+n][j+m] =	m_finitIntCost[n+nRange][m+nRange] ;}
						
				}}} 
	}}
}

/**
@brief Korean: ��������� ����ϴ� �Լ�
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::calCastPredictCost(int nRange)
{  
	for (int i=nRange ; i<m_nMapSizeX-nRange ; i++) {
		for (int j=nRange ; j<m_nMapSizeY-nRange ; j++){
			if ( m_dMap[i][j]>1){ //== KuMap::OCCUPIED_AREA ){ 
				m_fPreictCost[i][j] = m_dMap[i][j];//CSPACE_PredictVALUE;	
				for (int n=-nRange ; n<nRange+1 ; n++){
					for (int m=-nRange ; m<nRange+1 ; m++) 	{
						if(n+nRange>m_nMapSizeX||m+nRange>m_nMapSizeY||i+n>m_nMapSizeX||j+m>m_nMapSizeY||
							n+nRange<0||m+nRange<0||i+n<0||j+m<0){
								continue;
						}

						if(//m_fPreictCost[i+n][j+m]!= CSPACE_PredictVALUE&&
							m_fPreictCost[i+n][j+m] <m_finitPreCost[n+nRange][m+nRange])
						{
							m_fPreictCost[i+n][j+m] =m_finitPreCost[n+nRange][m+nRange] ;
						}

					}}} 
		}}
}
/**
@brief Korean: ���������κ��� ��θ� �����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::extractGradientPath(int nStartPosition[2],int nGoalPosition[2])
{
	KuPose Path;
	list<KuPose>::iterator iteratorPath;
	int nGradientnum=-1;
	int nCount=0; 
	bool nFinish = false;
	bool bExtract;

	//�κ��� ��ġ�� ��ο� �־��ش�.,--------------------------------------------------------------
	Path.setX( m_math.AI2MM(nStartPosition[0]) ); 
	Path.setY( m_math.AI2MM(nStartPosition[1]) );
	m_listPath.push_back(Path); 
	iteratorPath = m_listPath.begin(); 
	int nX = (int)m_math.MM2AI(iteratorPath->getX());
	int nY = (int)m_math.MM2AI(iteratorPath->getY());
	//==========================================================================================

	//��������  �������� �����.---------------------------------------------------------------
	for(int i=-CSPACE_OBSTACLE_INFINITY;i<CSPACE_OBSTACLE_INFINITY+1;i++){
		for(int j=-CSPACE_OBSTACLE_INFINITY;j<CSPACE_OBSTACLE_INFINITY+1;j++){
			if(nGoalPosition[0]+i>m_nMapSizeX-3||nGoalPosition[0]+i<3||nGoalPosition[1]+j>m_nMapSizeY-3||nGoalPosition[1]+j<3){return false;}
			m_fNavCost[nGoalPosition[0]+i][nGoalPosition[1]+j] = -1;
		} 
	}
	//=========================================================================================

	while ( (!nFinish) ) {
		//path�� ������ ������ �޾� �´�---------------------------------------------------------
		nX = (int)m_math.MM2AI(iteratorPath->getX());
		nY = (int)m_math.MM2AI(iteratorPath->getY());
		//======================================================================================

		bExtract = false;

		if(nX>m_nMapSizeX-3||nX<3||nY>m_nMapSizeY-3||nY<3){return false;	}
		else{	nGradientnum=calGradient(nX,nY,nGradientnum);	}

		//select Gradient path ---------------------------------------------------------------------------------------------------------------------------------------------
		if(nGradientnum==0){
			Path.setX( m_math.AI2MM((nX- 1)));
			Path.setY( m_math.AI2MM(nY-1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==1){
			Path.setX( m_math.AI2MM((nX- 1)));
			Path.setY( m_math.AI2MM(nY-2));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==2){
			Path.setX( m_math.AI2MM((nX)));
			Path.setY( m_math.AI2MM(nY-1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==3){
			Path.setX( m_math.AI2MM((nX+1)));
			Path.setY( m_math.AI2MM(nY-2));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==4){
			Path.setX( m_math.AI2MM((nX+1)));
			Path.setY( m_math.AI2MM(nY-1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==15){
			Path.setX( m_math.AI2MM((nX-2 )));
			Path.setY( m_math.AI2MM(nY-1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==5){
			Path.setX( m_math.AI2MM((nX+2)));
			Path.setY( m_math.AI2MM(nY-1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==14){
			Path.setX( m_math.AI2MM((nX-1)));
			Path.setY( m_math.AI2MM(nY));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==6){
			Path.setX( m_math.AI2MM((nX+1)));
			Path.setY( m_math.AI2MM(nY));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==13){
			Path.setX( m_math.AI2MM((nX-2)));
			Path.setY( m_math.AI2MM(nY+1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==7){
			Path.setX( m_math.AI2MM((nX+2)));
			Path.setY( m_math.AI2MM(nY+1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==12){
			Path.setX( m_math.AI2MM((nX- 1)));
			Path.setY( m_math.AI2MM(nY+1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}else if(nGradientnum==11){
			Path.setX( m_math.AI2MM((nX- 1)));
			Path.setY( m_math.AI2MM(nY+2));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==10){
			Path.setX( m_math.AI2MM((nX)));
			Path.setY( m_math.AI2MM(nY+1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==9){
			Path.setX( m_math.AI2MM((nX+1)));
			Path.setY( m_math.AI2MM(nY+2));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		else if(nGradientnum==8){
			Path.setX( m_math.AI2MM((nX+1)));
			Path.setY( m_math.AI2MM(nY+1));
			m_listPath.push_back(Path);
			iteratorPath++;
			bExtract = true;
		}
		//select Gradient path =======================================================================================

		if (bExtract==false) {	return false; } //fail extract path

		if ( abs(nX- nGoalPosition[0] )+abs(nY - nGoalPosition[1] )<=10 ) {
			Path.setX( m_math.AI2MM((nGoalPosition[0] )) );
			Path.setY( m_math.AI2MM(nGoalPosition[1]));
			m_listPath.push_back(Path);
			return true; //success extract path
		}
		if(m_listPath.size()>PATH_MAXSIZE) {return false; }
	}

	return false; //fail extract path
}
/**
@brief Korean: ���� ���� �̿��Ͽ� ���� ��ǥ�� ������ �����ϴ� �Լ�
@brief English:
*/
int KuGradientPathPlannerbasedPIPr::calGradient(int nX,int nY,int nGradientnum)
{
	double dCost[16]={0};
	double dMagf=INFINITY_VALUE;
	double dCMagf=INFINITY_VALUE;

	int nCriGradientnum=nGradientnum;
	int nCnum=-1;

	//�Ϲ����� Gradient path----------------------------------------------------------------------------------

	dCost[0]=	(m_fNavCost[nX-1][nY-1]-m_fNavCost[nX][nY])/sqrt(2.);
	dCost[1]=	(m_fNavCost[nX-1][nY-2]-m_fNavCost[nX][nY])/sqrt(5.);
	dCost[2]=	(m_fNavCost[nX][nY-1]-m_fNavCost[nX][nY])/sqrt(1.);
	dCost[3]=	(m_fNavCost[nX+1][nY-2]-m_fNavCost[nX][nY])/sqrt(5.);
	dCost[4]=	(m_fNavCost[nX+1][nY-1]-m_fNavCost[nX][nY])/sqrt(2.);
	dCost[15]=	(m_fNavCost[nX-2][nY-1]-m_fNavCost[nX][nY])/sqrt(5.);
	dCost[5]=	(m_fNavCost[nX+2][nY-1]-m_fNavCost[nX][nY])/sqrt(5.);
	dCost[14]=	(m_fNavCost[nX-1][nY]-m_fNavCost[nX][nY])/sqrt(1.);
	dCost[6]=	(m_fNavCost[nX+1][nY]-m_fNavCost[nX][nY])/sqrt(1.);
	dCost[13]=	(m_fNavCost[nX-2][nY+1]-m_fNavCost[nX][nY])/sqrt(5.);
	dCost[7]=	(m_fNavCost[nX+2][nY+1]-m_fNavCost[nX][nY])/sqrt(5.);
	dCost[12]=	(m_fNavCost[nX-1][nY+1]-m_fNavCost[nX][nY])/sqrt(2.);
	dCost[11]=	(m_fNavCost[nX-1][nY+2]-m_fNavCost[nX][nY])/sqrt(5.);
	dCost[10]=	(m_fNavCost[nX][nY+1]-m_fNavCost[nX][nY])/sqrt(1.);
	dCost[9]=	(m_fNavCost[nX+1][nY+2]-m_fNavCost[nX][nY])/sqrt(5.);
	dCost[8]=	(m_fNavCost[nX+1][nY+1]-m_fNavCost[nX][nY])/sqrt(2.);

	if(nGradientnum!=-1){
		int nCheckBack=(nCriGradientnum+nCriGradientnum+8)/2;
		if(nCheckBack>15){nCheckBack=nCheckBack-16+1;}
		else{nCheckBack=nCheckBack+1;}
		for(int j=nCheckBack;j<nCheckBack+8;j++){
			if(j>15){ dCost[j-16]=INFINITY_VALUE;	}
			else{dCost[j]=INFINITY_VALUE;	}			
		}
	}

	//���Ⱑ �ּҰ� �Ǵ� ���� ����--------------------------------------------------------------
	for(int i=0;i<16;i++){
		if(dMagf>dCost[i]){
			dMagf=dCost[i];
			nGradientnum=i;
		}
	}
	//=========================================================================================

	return  nGradientnum;
}




/**
@brief Korean: ��������� ����ϴ� �Լ�
@brief English:
*/
void KuGradientPathPlannerbasedPIPr::calAdjCostR(int nGoalPosition[2])
{

	int **nActiveList;
	int **nNewActiveList;
	int nActiveListNo=0;
	int nNewActiveListNo = 0;

	// create sufficient number of variable for active list
	nActiveList = new int*[2];
	nNewActiveList= new int*[2];
	if(nActiveList){
		for(int i = 0 ; i < 2 ; i++){
			nActiveList[i] = new int[m_nMapSizeY*m_nMapSizeX];
			nNewActiveList[i]= new int[m_nMapSizeY*m_nMapSizeX];
		}
	}

	// first active list
	nActiveList[0][0] = nGoalPosition[0];
	nActiveList[1][0] = nGoalPosition[1];
	nActiveListNo = 1;

	// ------------------------------------------------------------------------------ //
	// repeat calculation until adjacent cost of all cell is calculated.
	bool bResult, bComplete;
	int nCurrent_i, nCurrent_j;
	float fCurrentValue;
	bComplete = false;

	while ( (nActiveListNo > 0) && (bComplete==false) ) {
		nNewActiveListNo = 0;
		for (int i=0; i<nActiveListNo; i++) {
			nCurrent_i = nActiveList[0][i];
			nCurrent_j = nActiveList[1][i];
			fCurrentValue = m_fRemovalCost[nCurrent_i][nCurrent_j];

			// finish the operation when the current cell meets start point
			//if ( abs(nCurrent_i-nStartPosition[0])<=1 && abs(nCurrent_j-nStartPosition[1])<=1 ) bComplete = true;
			// need to test..... ex. Is this the optimal path when moving obstacle exists....

			bResult = calAdjCostUpR(nCurrent_i,nCurrent_j,fCurrentValue);
			if (bResult) {
				// register new active list if upper cell is updated.
				nNewActiveList[0][nNewActiveListNo] = nCurrent_i-1;
				nNewActiveList[1][nNewActiveListNo] = nCurrent_j;
				nNewActiveListNo++;
			}

			bResult = calAdjCostDownR(nCurrent_i,nCurrent_j,fCurrentValue);
			if (bResult) {
				// register new active list if lower cell is updated.
				nNewActiveList[0][nNewActiveListNo] = nCurrent_i+1;
				nNewActiveList[1][nNewActiveListNo] = nCurrent_j;
				nNewActiveListNo++;
			}

			bResult = calAdjCostLeftR(nCurrent_i,nCurrent_j,fCurrentValue);
			if (bResult) {
				// register new active list if left cell is updated.
				nNewActiveList[0][nNewActiveListNo] = nCurrent_i;
				nNewActiveList[1][nNewActiveListNo] = nCurrent_j-1;
				nNewActiveListNo++;
			}

			bResult = calAdjCostRightR(nCurrent_i,nCurrent_j,fCurrentValue);
			if (bResult) {
				// register new active list if right cell is updated.
				nNewActiveList[0][nNewActiveListNo] = nCurrent_i;
				nNewActiveList[1][nNewActiveListNo] = nCurrent_j+1;
				nNewActiveListNo++;
			}

			bResult = calAdjCostUpRightR(nCurrent_i,nCurrent_j,fCurrentValue);
			if (bResult) {
				// register new active list if upper cell is updated.
				nNewActiveList[0][nNewActiveListNo] = nCurrent_i-1;
				nNewActiveList[1][nNewActiveListNo] = nCurrent_j+1;
				nNewActiveListNo++;
			}

			bResult = calAdjCostDownLeftR(nCurrent_i,nCurrent_j,fCurrentValue);
			if (bResult) {
				// register new active list if lower cell is updated.
				nNewActiveList[0][nNewActiveListNo] = nCurrent_i+1;
				nNewActiveList[1][nNewActiveListNo] = nCurrent_j-1;
				nNewActiveListNo++;
			}

			bResult = calAdjCostLeftUpR(nCurrent_i,nCurrent_j,fCurrentValue);
			if (bResult) {
				// register new active list if left cell is updated.
				nNewActiveList[0][nNewActiveListNo] = nCurrent_i-1;
				nNewActiveList[1][nNewActiveListNo] = nCurrent_j-1;
				nNewActiveListNo++;
			}

			bResult = calAdjCostRightDownR(nCurrent_i,nCurrent_j,fCurrentValue);
			if (bResult) {
				// register new active list if right cell is updated.
				nNewActiveList[0][nNewActiveListNo] = nCurrent_i+1;
				nNewActiveList[1][nNewActiveListNo] = nCurrent_j+1;
				nNewActiveListNo++;
			}

		}

		// copy new active list to current active list for next operation
		nActiveListNo = nNewActiveListNo;
		for(int j=0; j<nActiveListNo; j++) {
			nActiveList[0][j] = nNewActiveList[0][j];
			nActiveList[1][j] = nNewActiveList[1][j];
		}

		if (nActiveListNo > hypot(m_nMapSizeX,m_nMapSizeY) * 100 ) {
			printf("Error!! Error!! Active list no. overflow error!!\n");
			printf("Active list no = %d\n", nActiveListNo);

			for(int i = 0 ; i < 2 ; i++){
				delete[] nActiveList[i];
				delete[] nNewActiveList[i];
			}
			delete[] nActiveList;
			delete[] nNewActiveList;

			return;
		}

	}
	// ------------------------------------------------------------------------------ //

	// delete dynamic variable

	for(int i = 0 ; i < 2 ; i++){
		delete[] nActiveList[i];
		delete[] nNewActiveList[i];
	}
	delete[] nActiveList;
	delete[] nNewActiveList;


}
/**
@brief Korean: �� ��ġ�κ��� ���ʿ����� ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostUpR(int nI, int nJ, float fValue)
{
	// boundary condition
	if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
	if (m_fIntCost[nI-1][nJ] == INFINITY_VALUE){m_fRemovalCost[nI-1][nJ] = INFINITY_VALUE; return false;}	
	if (m_smtnMap[nI-1][nJ] == KuMap::UNKNOWN_AREA) return false;		
	if (m_smtnMap[nI-1][nJ] == KuMap::OCCUPIED_AREA) return false;		
	//if (m_dMap[nI-1][nJ]>0)return false;//		 == KuMap::OCCUPIED_AREA) return false;		
	// calculate adjcent cost of upper cell


	if (m_dMap[nI-1][nJ]>1)
	{
        if ( (m_fRemovalCost[nI-1][nJ] > (fValue+1.0*m_dMap[nI-1][nJ])) || (m_fRemovalCost[nI-1][nJ]==0) ) {
			m_fRemovalCost[nI-1][nJ] = (fValue+1.0*m_dMap[nI-1][nJ]);
			return true;
		}else return false;
	}
	else
	{
		if ( (m_fRemovalCost[nI-1][nJ] > (fValue+1.0)) || (m_fRemovalCost[nI-1][nJ]==0) ) {
			m_fRemovalCost[nI-1][nJ] = (fValue+1.0);
			return true;
		}else return false;
	}


}
/**
@brief Korean: �� ��ġ�κ��� �Ʒ��� ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostDownR(int nI, int nJ, float fValue)
{
	// boundary condition
	if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
	if (m_fIntCost[nI+1][nJ] == INFINITY_VALUE){m_fRemovalCost[nI+1][nJ] = INFINITY_VALUE; return false;}	
	if (m_smtnMap[nI+1][nJ] == KuMap::UNKNOWN_AREA) return false;		
	if (m_smtnMap[nI+1][nJ] == KuMap::OCCUPIED_AREA) return false;		
	//if (m_dMap[nI+1][nJ]>0)return false;// == KuMap::OCCUPIED_AREA) return false;		
	// calculate adjcent cost of lower cell

	if (m_dMap[nI+1][nJ]>1)
	{
		if ( (m_fRemovalCost[nI+1][nJ] > (fValue+1.0*m_dMap[nI+1][nJ])) || (m_fRemovalCost[nI+1][nJ]==0) ) {
			m_fRemovalCost[nI+1][nJ] = (fValue+1.0*m_dMap[nI+1][nJ]);
			return true;
		}else return false;
	}
	else
	{
		if ( (m_fRemovalCost[nI+1][nJ] > (fValue+1.0)) || (m_fRemovalCost[nI+1][nJ]==0) ) {
			m_fRemovalCost[nI+1][nJ] = (fValue+1.0);
			return true;
		}else return false;
	}

}
/**
@brief Korean: �� ��ġ�κ��� ���� ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostLeftR(int nI, int nJ, float fValue)
{
	// boundary condition
	if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
	if (m_fIntCost[nI][nJ-1] == INFINITY_VALUE){m_fRemovalCost[nI][nJ-1] = INFINITY_VALUE; return false;}	
	if (m_smtnMap[nI][nJ-1] == KuMap::UNKNOWN_AREA) return false;	
	if (m_smtnMap[nI][nJ-1] == KuMap::OCCUPIED_AREA) return false;		
	//if (m_dMap[nI][nJ-1]>0)return false;// == KuMap::OCCUPIED_AREA) return false;		
	// calculate adjcent cost of left cell

	if (m_dMap[nI][nJ-1]>1)	
	{
		if ( (m_fRemovalCost[nI][nJ-1] > (fValue+1.0*m_dMap[nI][nJ-1])) || (m_fRemovalCost[nI][nJ-1]==0) ) {

			m_fRemovalCost[nI][nJ-1] = (fValue+1.0*m_dMap[nI][nJ-1]);
			return true;
		}else return false;
	}
	else
	{
		if ( (m_fRemovalCost[nI][nJ-1] > (fValue+1.0)) || (m_fRemovalCost[nI][nJ-1]==0) ) {

			m_fRemovalCost[nI][nJ-1] = (fValue+1.0);return true;
		}else return false;
	}

}
/**
@brief Korean: �� ��ġ�κ��� ������ ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostRightR(int nI, int nJ, float fValue)
{
	// boundary condition
	if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
	if (m_fIntCost[nI][nJ+1] == INFINITY_VALUE) {m_fRemovalCost[nI][nJ+1] = INFINITY_VALUE; return false;}	
	if (m_smtnMap[nI][nJ+1] == KuMap::UNKNOWN_AREA) return false;	
	if (m_smtnMap[nI][nJ+1] == KuMap::OCCUPIED_AREA) return false;		
	//if (m_dMap[nI][nJ+1]>0)return false;// == KuMap::OCCUPIED_AREA) return false;		
	// calculate adjcent cost of right cell

	if (m_dMap[nI][nJ+1]>1)
	{
		if ( (m_fRemovalCost[nI][nJ+1] > (fValue+1.0*m_dMap[nI][nJ+1])) || (m_fRemovalCost[nI][nJ+1]==0) ) {

			m_fRemovalCost[nI][nJ+1] = (fValue+1.0*m_dMap[nI][nJ+1]);
			return true;
		}else return false;
	}
	else
	{
		if ( (m_fRemovalCost[nI][nJ+1] > (fValue+1.0)) || (m_fRemovalCost[nI][nJ+1]==0) ) {

			m_fRemovalCost[nI][nJ+1] = (fValue+1.0);
			return true;
		}else return false;

	}


}
/**
@brief Korean: �� ��ġ�κ��� ��,������ ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostUpRightR(int nI, int nJ, float fValue)
{
	// boundary condition
	if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
	if (m_fIntCost[nI-1][nJ+1] == INFINITY_VALUE){m_fRemovalCost[nI-1][nJ+1] = INFINITY_VALUE; return false;}	
	if (m_smtnMap[nI-1][nJ+1] == KuMap::UNKNOWN_AREA) return false;	
	if (m_smtnMap[nI-1][nJ+1] == KuMap::OCCUPIED_AREA) return false;	
	//if (m_dMap[nI-1][nJ+1] >0)return false;//== KuMap::OCCUPIED_AREA) return false;		

	// calculate adjcent cost of upper cell


	if (m_dMap[nI-1][nJ+1]>1)
	{
		if ( (m_fRemovalCost[nI-1][nJ+1] > (fValue+1.4*m_dMap[nI-1][nJ+1])) || (m_fRemovalCost[nI-1][nJ+1]==0) ) {
			m_fRemovalCost[nI-1][nJ+1] = (fValue+1.4*m_dMap[nI-1][nJ+1]);
			return true;
		}else return false;
	}
	else
	{
		if ( (m_fRemovalCost[nI-1][nJ+1] > (fValue+1.4)) || (m_fRemovalCost[nI-1][nJ+1]==0) ) {
			m_fRemovalCost[nI-1][nJ+1] = (fValue+1.4);
			return true;
		}else return false;
	}

}
/**
@brief Korean: �� ��ġ�κ��� �Ʒ�,���� ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostDownLeftR(int nI, int nJ, float fValue)
{
	// boundary condition
	if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
	if (m_fIntCost[nI+1][nJ-1] == INFINITY_VALUE){m_fRemovalCost[nI+1][nJ-1] = INFINITY_VALUE; return false;}	
	if (m_smtnMap[nI+1][nJ-1] == KuMap::UNKNOWN_AREA) return false;		
	if (m_smtnMap[nI+1][nJ-1] == KuMap::OCCUPIED_AREA) return false;		
	//if (m_dMap[nI+1][nJ-1] >0)return false;//== KuMap::OCCUPIED_AREA) return false;		

	// calculate adjcent cost of lower cell

	if (m_dMap[nI+1][nJ-1] >1)	
	{
		if ( (m_fRemovalCost[nI+1][nJ-1] > (fValue+1.4*m_dMap[nI+1][nJ-1])) || (m_fRemovalCost[nI+1][nJ-1]==0) ) {
			m_fRemovalCost[nI+1][nJ-1] = (fValue+1.4*m_dMap[nI+1][nJ-1] );
			return true;
		}else return false;
	}		
	else
	{
		if ( (m_fRemovalCost[nI+1][nJ-1] > (fValue+1.4)) || (m_fRemovalCost[nI+1][nJ-1]==0) ) {
			m_fRemovalCost[nI+1][nJ-1] = (fValue+1.4);
			return true;
		}else return false;
	}

}
/**
@brief Korean: �� ��ġ�κ��� ��,���� ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostLeftUpR(int nI, int nJ, float fValue)
{
	// boundary condition
	if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
	if (m_fIntCost[nI-1][nJ-1] == INFINITY_VALUE) {m_fRemovalCost[nI-1][nJ-1] = INFINITY_VALUE; return false;}
	if (m_smtnMap[nI-1][nJ-1] == KuMap::UNKNOWN_AREA) return false;		
	if (m_smtnMap[nI-1][nJ-1] == KuMap::OCCUPIED_AREA) return false;	
	//if (m_dMap[nI-1][nJ-1]>0)return false;// == KuMap::OCCUPIED_AREA) return false;	


	// calculate adjcent cost of left cell
	if (m_dMap[nI-1][nJ-1]>1)
	{
		if ( (m_fRemovalCost[nI-1][nJ-1] > (fValue+1.4*m_dMap[nI-1][nJ-1])) || (m_fRemovalCost[nI-1][nJ-1]==0) ) {

			m_fRemovalCost[nI-1][nJ-1] = (fValue+1.4*m_dMap[nI-1][nJ-1]);
			return true;
		}else return false;
	}
	else
	{
		if ( (m_fRemovalCost[nI-1][nJ-1] > (fValue+1.4)) || (m_fRemovalCost[nI-1][nJ-1]==0) ) {

			m_fRemovalCost[nI-1][nJ-1] = (fValue+1.4);
			return true;
		}else return false;
	}


}
/**
@brief Korean: �� ��ġ�κ��� �Ʒ�,������ ������ ��������� ����ϴ� �Լ�
@brief English:
*/
bool KuGradientPathPlannerbasedPIPr::calAdjCostRightDownR(int nI, int nJ, float fValue)
{
	// boundary condition
	if (nI<1 || nI>m_nMapSizeX-2 || nJ<1 || nJ>m_nMapSizeY-2) return false;
	if (m_fIntCost[nI+1][nJ+1] == INFINITY_VALUE){m_fRemovalCost[nI+1][nJ+1]  = INFINITY_VALUE; return false;}	
	if (m_smtnMap[nI+1][nJ+1] == KuMap::UNKNOWN_AREA) return false;		
	if (m_smtnMap[nI+1][nJ+1] == KuMap::OCCUPIED_AREA) return false;	
	//if (m_dMap[nI+1][nJ+1]>0)return false;// == KuMap::OCCUPIED_AREA) return false;		

	// calculate adjcent cost of right cell

	if (m_dMap[nI+1][nJ+1]>1)
	{
		if ( (m_fRemovalCost[nI+1][nJ+1] > (fValue+1.4*m_dMap[nI+1][nJ+1])) || (m_fRemovalCost[nI+1][nJ+1]==0) ) {
			m_fRemovalCost[nI+1][nJ+1] = (fValue+1.4*m_dMap[nI+1][nJ+1]);
			return true;
		}
		else return false;
	}
	else
	{
		if ( (m_fRemovalCost[nI+1][nJ+1] > (fValue+1.4)) || (m_fRemovalCost[nI+1][nJ+1]==0) ) {
			m_fRemovalCost[nI+1][nJ+1] = (fValue+1.4);
			return true;
		}
		else return false;
	}

}
void KuGradientPathPlannerbasedPIPr::setProMapWeight(double dProMapWeight)
{
	m_dProMapWeight=dProMapWeight;
}
