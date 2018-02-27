#include "stdafx.h"
#include "KuVrRangeInterface.h"

KuVrRangeInterface::KuVrRangeInterface()
{
    m_nLaserData360 = m_KuUtil.generateIntType1DArray(MAX_LASER_IDX);
	m_nLaserData181= m_KuUtil.generateIntType1DArray(MAX_LASER_IDX_181);
    m_nMap=NULL;

}
KuVrRangeInterface::~KuVrRangeInterface()
{
    if(m_nMap!=NULL)
{
        for(int i = 0 ; i < m_nMapX ; i++){
            delete[] m_nMap[i];
            m_nMap[i] = NULL;

        }
        delete[] m_nMap;

    }
}

/**
 @brief Korean: 가상의 센서와 연결하는 함수
 @brief English:
*/
bool KuVrRangeInterface::connect(int** nMap,int nMapX,int nMapY)
{
    m_nMapX= nMapX;
    m_nMapY= nMapY;

        if(m_nMap==NULL){
        //처음 m_nMap을 생성할때
        m_nMap = new int*[m_nMapX];
        if(m_nMap){
            for(int i = 0 ; i < m_nMapX ; i++){
                m_nMap[i] = new int[m_nMapY];
            }
        }
    }


    for(int i=0; i<m_nMapX; i++){
        for(int j=0; j<m_nMapY; j++){
//             if(nMap[i][j]==KuMap::OCCUPIED_AREA)
//                 m_nMap[i][j] = KuMap::OCCUPIED_AREA;       
			m_nMap[i][j] = nMap[i][j];
        }
    }

    m_math.setCellSizeMM(Sensor::CELLSIZE);
    //cout<<"Cell size  "<<m_math.getCellSizeMM()<<endl;
    return true;
}

/**
 @brief Korean: 가상의 181개 레이저 데이터를 넘겨주는 함수
 @brief English:
*/
int_1DArray KuVrRangeInterface::getData(KuPose RobotPos)
{
	if(m_nMap==NULL){
		//가상 레이저와 연결이 되어 있지 않은 상태
		return m_nLaserData360;
	}

	double dLaserOffSet =0.0;

	double dGradientX,dGradientY;
	double dComDist,f;
	double RayOfX,RayOfY;
	int nPointX=0, nPointY=0;
	int nX=0,nY=0;
	int nAngle=0;
	double dAngle=0;
	double dCellSize=Sensor::CELLSIZE;
	double  dIntervalDistance=(double)m_math.getCellSizeMM()/500.0;

	nPointX = m_math.MM2AI((RobotPos.getX()+dLaserOffSet*cos(RobotPos.getThetaRad())));
	nPointY = m_math.MM2AI((RobotPos.getY()+dLaserOffSet*sin(RobotPos.getThetaRad())));
	vector<int> vecnX;
	vector<int> vecnY;

	int nSize=2;
	for(int i=nPointX-nSize; i<nPointX+nSize+1; i++ ){
		for(int j=nPointY-nSize; j<nPointY+nSize+1; j++){
			if(m_nMap[i][j] ==KuMap::WARNING_AREA)
			{
				m_nMap[i][j] = KuMap::EMPTY_AREA;	
				vecnX.push_back(i);
				vecnY.push_back(j);
			}
		}
	}


	for (int i=0; i<MAX_LASER_IDX; i++){

		dAngle = (i-90+RobotPos.getThetaDeg());

		if(dAngle > 180) dAngle -=360;
		else if(dAngle < -180) dAngle += 360;

		nX=nY=0;
		dGradientX = cos( dAngle*D2R );	// 현재 처리하는 레이저 점의 기울기를 구한다.
		dGradientY = sin( dAngle*D2R );
		dComDist=0;
		f=0.;

		m_nLaserData360[i] = MAX_LASER_DISTANCE;

		while(dComDist  <= m_math.MM2AI(MAX_LASER_DISTANCE)) {

			RayOfX = f * dGradientX;
			RayOfY = f * dGradientY;
			dComDist = sqrt(RayOfX*RayOfX + RayOfY*RayOfY);			
			
			f=f+dIntervalDistance;

	
// 			if (dComDist<4.5){
// 				m_nLaserData181[i] = 0.0;
// 				continue;	}

			nX = (int)(nPointX + RayOfX);
			nY = (int)(nPointY + RayOfY);

			if(nX<1||nY<1||nX>m_nMapX||nY>m_nMapY) continue;

			if(m_nMap[nX][nY] ==  KuMap::UNKNOWN_AREA||m_nMap[nX][nY] == KuMap::OCCUPIED_AREA||m_nMap[nX][nY] == KuMap::WARNING_AREA ){
				m_nLaserData360[i] = (int)( m_math.AI2MM(dComDist));
				break;
			}
		}

	}

	for(int i=0;i<vecnX.size();i++)
	{
		m_nMap[vecnX[i]][vecnY[i]] = KuMap::OCCUPIED_AREA;		
	}

	return m_nLaserData360;
}


int_1DArray KuVrRangeInterface::getData(KuPose RobotPos,vector<KuPose> vecRobotPos )
{
	if(m_nMap==NULL){
		//가상 레이저와 연결이 되어 있지 않은 상태
		return m_nLaserData360;
	}


	double dLaserOffSet =0.0;

	double dGradientX,dGradientY;
	double dComDist,f;
	double RayOfX,RayOfY;
	int nPointX=0, nPointY=0;
	int nX=0,nY=0;
	int nAngle=0;
	double dAngle=0;
	double dCellSize=Sensor::CELLSIZE;
	double  dIntervalDistance=(double)m_math.getCellSizeMM()/500.0;


	vector<int> vecnX;
	vector<int> vecnY;

	for (int i=0;i<vecRobotPos.size();i++)
	{
		nPointX = m_math.MM2AI((vecRobotPos[i].getX()+dLaserOffSet*cos(vecRobotPos[i].getThetaRad())));
		nPointY = m_math.MM2AI((vecRobotPos[i].getY()+dLaserOffSet*sin(vecRobotPos[i].getThetaRad())));

		int nSize=2;
		for(int i=nPointX-nSize; i<nPointX+nSize+1; i++ ){
			for(int j=nPointY-nSize; j<nPointY+nSize+1; j++){
				if(m_nMap[i][j] ==KuMap::WARNING_AREA)
				{
					m_nMap[i][j] = KuMap::EMPTY_AREA;	
					vecnX.push_back(i);
					vecnY.push_back(j);
				}
			}
		}


	}
	
	nPointX = m_math.MM2AI((RobotPos.getX()+dLaserOffSet*cos(RobotPos.getThetaRad())));
	nPointY = m_math.MM2AI((RobotPos.getY()+dLaserOffSet*sin(RobotPos.getThetaRad())));


	for (int i=0; i<MAX_LASER_IDX; i++){

		dAngle = (i-90+RobotPos.getThetaDeg());

		if(dAngle > 180) dAngle -=360;
		else if(dAngle < -180) dAngle += 360;

		nX=nY=0;
		dGradientX = cos( dAngle*D2R );	// 현재 처리하는 레이저 점의 기울기를 구한다.
		dGradientY = sin( dAngle*D2R );
		dComDist=0;
		f=0.;

		m_nLaserData360[i] = MAX_LASER_DISTANCE;

		while(dComDist  <= m_math.MM2AI(MAX_LASER_DISTANCE)) {

			RayOfX = f * dGradientX;
			RayOfY = f * dGradientY;
			dComDist = sqrt(RayOfX*RayOfX + RayOfY*RayOfY);			

			f=f+dIntervalDistance;


			// 			if (dComDist<4.5){
			// 				m_nLaserData181[i] = 0.0;
			// 				continue;	}

			nX = (int)(nPointX + RayOfX);
			nY = (int)(nPointY + RayOfY);

			if(nX<1||nY<1||nX>m_nMapX||nY>m_nMapY) continue;

			if(m_nMap[nX][nY] ==  KuMap::UNKNOWN_AREA||m_nMap[nX][nY] == KuMap::OCCUPIED_AREA||m_nMap[nX][nY] == KuMap::WARNING_AREA ){
				m_nLaserData360[i] = (int)( m_math.AI2MM(dComDist));
				break;
			}
		}

	}

	for(int i=0;i<vecnX.size();i++)
	{
		m_nMap[vecnX[i]][vecnY[i]] = KuMap::OCCUPIED_AREA;		
	}

	return m_nLaserData360;
}
