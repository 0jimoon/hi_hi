#include "stdafx.h"
#include "KuElevatorPr.h"

KuElevatorPr::KuElevatorPr()
{
	m_dElvLength = 1500.0;
	m_dElvWidth = 800.0;//원래 대략 1.1m인데.....정확히 측정안되네 한 950정도로
	m_nState = 1;
	m_nCountNum = 0;
	m_dDelEncoderThetaDeg = 0.0;
}

KuElevatorPr::~KuElevatorPr()
{

}

void KuElevatorPr::init()
{

}

int KuElevatorPr::EnterElevatorState(int_1DArray KinectRangeData, KuPose POIElevatorPos, KuPose RobotPos, KuPose DelEncoder,double *dTransVel, double *dRotVel)
{
	//double dTemp_TransVel, dTemp_RotVel;

	switch(m_nState)
	{
	case 1://poi포즈 향해서 회전
		{
			printf("case1 \n");
			double dDelPIOAngleDeg = 0.0;
			if(RobotPos.getThetaDeg()>=0)
				dDelPIOAngleDeg = POIElevatorPos.getThetaDeg() - RobotPos.getThetaDeg();
			else
				dDelPIOAngleDeg = -POIElevatorPos.getThetaDeg() - RobotPos.getThetaDeg();
				
			if(fabs(dDelPIOAngleDeg) > 1.0)
			{
				*dTransVel = 0.0;
				*dRotVel = dDelPIOAngleDeg/10.0+5*dDelPIOAngleDeg/fabs(dDelPIOAngleDeg);
			}
			else
			{
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				m_nState +=1;
			}
			printf("로봇각도는 %.2f",RobotPos.getThetaDeg());
			break;
		}
	case 2://초기 엘리베이터 포즈 측정
		{
			printf("case2 \n");
			m_ElvPos = CalElvPose(KinectRangeData,RobotPos);

			if(m_ElvPos.getX()!=0 || m_ElvPos.getY()!=0)
			{
				m_FirstGoalPos = FirstGoal(m_ElvPos,RobotPos);
				m_FirstGoalPos.setID(RobotPos.getID());
				KuDrawingInfo::getInstance()->setGoalPos(m_FirstGoalPos);
//				KuDrawingInfo::getInstance()->setElevatorPose(m_ElvPos);
//				KuDrawingInfo::getInstance()->setRenderElevatorflag(true);
				m_nState +=1;
			}
			else
			{
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				
			}
			break;
		}

	case 3://첫번재 골지점으로 회전
		{
			printf("case3 \n");

			double dFirstGoalAngleDeg = m_FirstGoalPos.getThetaDeg() - RobotPos.getThetaDeg();
			if (dFirstGoalAngleDeg >= 180.0){ dFirstGoalAngleDeg -= 360.0; }			
			else if (dFirstGoalAngleDeg <= -180.0){ dFirstGoalAngleDeg += 360.0; }
			if (fabs(dFirstGoalAngleDeg) > 1.0) { 
				*dTransVel = 0.0;
				*dRotVel = dFirstGoalAngleDeg/5.0+5*dFirstGoalAngleDeg/fabs(dFirstGoalAngleDeg);
			}
			else
			{
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				m_nState +=1;
			}
			break;
		}

	case 4://첫번째 골지점으로 직진
		{
			printf("case4 \n");
			double dFirstGoalDist = sqrt(pow(m_FirstGoalPos.getX()-RobotPos.getX(),2.0) + pow(m_FirstGoalPos.getY()-RobotPos.getY(),2.0));
			printf("거리값은 %.2f",dFirstGoalDist);
			if(dFirstGoalDist > 50.0)
			{
				*dTransVel = dFirstGoalDist/4;
				*dRotVel = 0.0;
			}
			else
			{
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				m_nState +=1;
			}
			break;
		}
		
	case 5://엘리베이터 향해 회전
		{
			printf("case5 \n");
			KuDrawingInfo::getInstance()->setGoalPos(m_ElvPos);
			
			double dDelX = m_ElvPos.getX() - RobotPos.getX();
			double dDelY = m_ElvPos.getY() - RobotPos.getY();
			double dPathAngleDeg = atan2(dDelY,dDelX)*R2D;
			double dDelAngleDeg = dPathAngleDeg - RobotPos.getThetaDeg();
			
			if (dDelAngleDeg >= 180.){ dDelAngleDeg -= 360.; }			
			else if (dDelAngleDeg <= -180.){ dDelAngleDeg += 360.; }
			if (fabs(dDelAngleDeg) > 1.) { 
				*dTransVel = 0.0;
				*dRotVel = dDelAngleDeg/5.0+5*dDelAngleDeg/fabs(dDelAngleDeg);
			}
			else
			{
				KuPose Temp_ElvPose = CalElvPose(KinectRangeData,RobotPos);
				if(Temp_ElvPose.getX()!=0 || Temp_ElvPose.getY()!=0)//인식 가능할 때만 엘리베이터 포즈 데이터 갱신
				{
					double dDelX = Temp_ElvPose.getX() - RobotPos.getX();
					double dDelY = Temp_ElvPose.getY() - RobotPos.getY();
					double dTemp_SecondGoalDist = sqrt(pow(dDelX,2.0) + pow(dDelY,2.0));
					if(dTemp_SecondGoalDist > 1000.0)
						m_ElvPos = Temp_ElvPose;
				}
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				m_nState +=1;
			}
			break;
		}
		
	case 6://엘리베이터 향해 전진
		{
			printf("case6 \n");
// 			KuPose Temp_ElvPose = CalElvPose(KinectRangeData,RobotPos);
// 
// 			if(Temp_ElvPose.getX()!=0 || Temp_ElvPose.getY()!=0)//인식 가능할 때만 엘리베이터 포즈 데이터 갱신
// 			{
// 				double dDelX = Temp_ElvPose.getX() - RobotPos.getX();
// 				double dDelY = Temp_ElvPose.getY() - RobotPos.getY();
// 				double dTemp_SecondGoalDist = sqrt(pow(dDelX,2.0) + pow(dDelY,2.0));
// 				if(dTemp_SecondGoalDist > 1000.0)
// 					m_ElvPos = Temp_ElvPose;
// 			}

			double dDelX = m_ElvPos.getX() - RobotPos.getX();
			double dDelY = m_ElvPos.getY() - RobotPos.getY();
			double dSecondGoalDist = sqrt(pow(dDelX,2.0) + pow(dDelY,2.0));
			double dDelPositionThetaDeg = atan2(dDelY,dDelX)*R2D;
			double dDelThetaDeg = dDelPositionThetaDeg - RobotPos.getThetaDeg();
			if (dDelThetaDeg >= 180.){ dDelThetaDeg -= 360.; }			
			else if (dDelThetaDeg <= -180.){ dDelThetaDeg += 360.; }
// 			if (fabs(dDelThetaDeg) > 1.) { 
// 				*dTransVel = 0.0;
// 				*dRotVel = dDelThetaDeg/5.0+5*dDelThetaDeg/fabs(dDelThetaDeg);
// 			}

			if(dSecondGoalDist > 500.0)
			{
				*dTransVel = dSecondGoalDist/4;
				*dRotVel = dDelThetaDeg;
			}
			else
			{
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				m_nState +=1;
			}
			break;
		}
		
	case 7://문까지 거리 등록
		{
			printf("case7 \n");
			m_dXmin = CriterionDistanceX(KinectRangeData);

			if(m_dXmin != 0)
			{
				m_nState +=1;
			}
			else
			{
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				break;
			}
		}
		
	case 8://문열림 체크
		{
			printf("case8 \n");
			double dTemp_Dist = CriterionDistanceX(KinectRangeData);

			if(dTemp_Dist - m_dXmin > m_dElvLength)
			{
				m_nState +=1;
			}
			else
			{
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				break;
			}
		}
		
	case 9://문열렸으니 전진
		{
			printf("case9 \n");
			if(CriterionDistanceX(KinectRangeData) > 750.0)
			{
				*dTransVel = 200.0;
				*dRotVel = 0.0;
			}
			else
			{
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				m_nState +=1;
			}
			break;
		}
	case 10://들어가서 회전(이거 파이오니어 하드웨어적 문제임 실제로는 그대로 빠짐)
		{
			printf("case10 \n");
			m_dDelEncoderThetaDeg += fabs(DelEncoder.getThetaDeg());
			if(m_dDelEncoderThetaDeg <180)
			{
				*dTransVel = 0.0;
				*dRotVel = int(180-m_dDelEncoderThetaDeg+1.0);
			}
			else
			{
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				m_nState +=1;
			}
			break;
		}
	case 11://나올 때 후진
		{
			printf("case11 \n");
			if(m_nCountNum < 250)
			{
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				m_nCountNum += 1;
			}
			else if(m_nCountNum >= 250 && m_nCountNum < 350)
			{
				*dTransVel = 200.0;
				*dRotVel = 0.0;
				m_nCountNum += 1;
			}
			else
			{
				*dTransVel = 0.0;
				*dRotVel = 0.0;
				m_nState = 0;////state 0이면 완전히 종료!
			}
			printf("countnumber = %d \n", m_nCountNum);
			break;
		}
		
	}

	return m_nState;
}

KuPose KuElevatorPr::CalElvPose(int_1DArray KinectRangeData, KuPose RobotPos)
{
	KuPose ElvPose;//Output
	vector<KuPose> vecXYData;
	KuPose Temp_xydata;

	for(int i=0;i<Sensor::KINECT_SENSOR_FOV;i++)//kinect range data를 x,y좌표로 바꿈

	{
		if(KinectRangeData[i] != 1000000)
		{
			double dAngleRad=(double)(i-Sensor::KINECT_SENSOR_FOV/2)*D2R;
			double dX=RobotPos.getX() + KinectRangeData[i]*cos(dAngleRad + RobotPos.getThetaRad());
			double dY=RobotPos.getY() + KinectRangeData[i]*sin(dAngleRad + RobotPos.getThetaRad());
			Temp_xydata.setX(dX);
			Temp_xydata.setY(dY);
			vecXYData.push_back(Temp_xydata);
		}
	}

	if(vecXYData.size()==0)
		return ElvPose;//예외처리

	double dFirstDist=0.0;
	double dSecondDist=0.0;
	double dTemp_dist=0.0;
	int nFirstID=0,nSecondID=0,nTemp_index=0;

	for(int j=0;j<vecXYData.size()-1;j++)//vecXYDate에서 서로인접한 거리가 가장큰 두개 index찾는다
	{
		dTemp_dist=sqrt(pow(vecXYData[j].getX()-vecXYData[j+1].getX(),2.0)+pow(vecXYData[j].getY()-vecXYData[j+1].getY(),2.0));

		if(dTemp_dist>dFirstDist)
		{
			dSecondDist=dFirstDist;
			dFirstDist=dTemp_dist;
			nSecondID=nFirstID;
			nFirstID=j;	
		}
		else if(dTemp_dist>dSecondDist)
		{
			dSecondDist=dTemp_dist;
			nSecondID=j;
		}
	}

	if(nFirstID>nSecondID)//index재정비, 0~nSecondID-1, nFirstID+1~end로 후에 쓸거다, nFistID가 좌측기둥부분, nSecondID가 우측기둥부분
		nFirstID+=1;
	else
	{
		nTemp_index=nSecondID+1;
		nSecondID=nFirstID;
		nFirstID=nTemp_index;
	}

	if(nSecondID<3 || nFirstID>vecXYData.size()-3)//예외처리
		return ElvPose;

	double dCentralPointX = (vecXYData[nFirstID+1].getX()+vecXYData[nSecondID-1].getX())/2;//양쪽기둥 사이 중점 좌표구함
	double dCentralPointY = (vecXYData[nFirstID+1].getY()+vecXYData[nSecondID-1].getY())/2;

	//구한 좌표 사이 거리 엘베보다 짧은 경우 예외처리
	double dElvWidth_X = vecXYData[nFirstID+1].getX() - vecXYData[nSecondID-1].getX();
	double dElvWidth_Y = vecXYData[nFirstID+1].getY() - vecXYData[nSecondID-1].getY();
	double dElvWidth = sqrt(pow(dElvWidth_X,2.0) + pow(dElvWidth_Y,2.0));
	if(dElvWidth < m_dElvWidth)
		return ElvPose;
	/////////////////////////////////////////////////

	int nNum=vecXYData.size()-(nFirstID-nSecondID+1);
	double dX_sum=0.0,dY_sum=0.0,dXX_sum=0.0,dXY_sum=0.0;//Least square에 필요

	for(int k=0;k<nSecondID;k++)
	{
		dX_sum += vecXYData[k].getX();
		dY_sum += vecXYData[k].getY();
		dXX_sum += vecXYData[k].getX()*vecXYData[k].getX();
		dXY_sum += vecXYData[k].getX()*vecXYData[k].getY();
	}

	for(int p=nFirstID+1;p<vecXYData.size();p++)
	{
		dX_sum += vecXYData[p].getX();
		dY_sum += vecXYData[p].getY();
		dXX_sum += vecXYData[p].getX()*vecXYData[p].getX();
		dXY_sum += vecXYData[p].getX()*vecXYData[p].getY();
	}

	if(nNum*dXX_sum-dX_sum*dX_sum==0)
		return ElvPose;

	double dA = (nNum*dXY_sum-dX_sum*dY_sum)/(nNum*dXX_sum-dX_sum*dX_sum);//ax+b=y서의 계수들 구함
	double dB = (-dX_sum*dXY_sum+dXX_sum*dY_sum)/(nNum*dXX_sum-dX_sum*dX_sum);

	double dElvPoseX = (dCentralPointX + dA*dCentralPointY - dA*dB)/(pow(dA,2.0) + 1);//추출한 선 위에 dCentralPoint 사영시킴
	double dElvPoseY = dA*dElvPoseX + dB;
	double dAngleDeg = atan2(dA,1.0)*R2D;//엘리베이터(벽) 각도

	if(dAngleDeg>90.0)
		dAngleDeg-=180.0;
	else if(dAngleDeg<-90.0)
		dAngleDeg+=180.0;

	ElvPose.setX(dElvPoseX);
	ElvPose.setY(dElvPoseY);
	ElvPose.setThetaDeg(dAngleDeg);//-90~90도

	return ElvPose;
}

KuPose KuElevatorPr::FirstGoal(KuPose ElvPose,KuPose RobotPos)
{
	KuPose FirstGoalPose;//Output
	double dA = tan(ElvPose.getThetaDeg()*D2R);
	double dGoalX = (pow(dA,2.0)*RobotPos.getX() - dA*RobotPos.getY() + ElvPose.getX() + dA*ElvPose.getY())/(pow(dA,2.0) + 1);
	double dGoalY = dA*(dGoalX-RobotPos.getX()) + RobotPos.getY();
	FirstGoalPose.setX(dGoalX);
	FirstGoalPose.setY(dGoalY);
	double dDelX = dGoalX - RobotPos.getX();
	double dDelY = dGoalY - RobotPos.getY();
	double dHeadingAngleDeg = atan2(dDelY,dDelX)*R2D;
	FirstGoalPose.setThetaDeg(dHeadingAngleDeg);//로봇이 첫번째 골 향하는 각

	return FirstGoalPose;
}

double KuElevatorPr::CriterionDistanceX(int_1DArray KinectRangeData)
{
	double CriterionDistance = 0.0;
	double totalDistance = 0.0;
	int pointNum = 0;
	for(int i=26;i<31;i++)
	{
		if(KinectRangeData[i] != 1000000)
		{
			double dAngleRad=(double)(i-Sensor::KINECT_SENSOR_FOV/2)*D2R;
			double dX = KinectRangeData[i]*cos(dAngleRad);
			totalDistance += dX;
			pointNum += 1;
		}
	}

	if(pointNum != 0)
		CriterionDistance = totalDistance/pointNum;

	return CriterionDistance;
}