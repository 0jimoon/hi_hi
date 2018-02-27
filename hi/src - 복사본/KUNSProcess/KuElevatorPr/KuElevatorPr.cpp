#include "stdafx.h"
#include "KuElevatorPr.h"

KuElevatorPr::KuElevatorPr()
{
	m_dElvLength = 1500.0;
	m_dElvWidth = 800.0;//���� �뷫 1.1m�ε�.....��Ȯ�� �����ȵǳ� �� 950������
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
	case 1://poi���� ���ؼ� ȸ��
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
			printf("�κ������� %.2f",RobotPos.getThetaDeg());
			break;
		}
	case 2://�ʱ� ���������� ���� ����
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

	case 3://ù���� ���������� ȸ��
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

	case 4://ù��° ���������� ����
		{
			printf("case4 \n");
			double dFirstGoalDist = sqrt(pow(m_FirstGoalPos.getX()-RobotPos.getX(),2.0) + pow(m_FirstGoalPos.getY()-RobotPos.getY(),2.0));
			printf("�Ÿ����� %.2f",dFirstGoalDist);
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
		
	case 5://���������� ���� ȸ��
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
				if(Temp_ElvPose.getX()!=0 || Temp_ElvPose.getY()!=0)//�ν� ������ ���� ���������� ���� ������ ����
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
		
	case 6://���������� ���� ����
		{
			printf("case6 \n");
// 			KuPose Temp_ElvPose = CalElvPose(KinectRangeData,RobotPos);
// 
// 			if(Temp_ElvPose.getX()!=0 || Temp_ElvPose.getY()!=0)//�ν� ������ ���� ���������� ���� ������ ����
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
		
	case 7://������ �Ÿ� ���
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
		
	case 8://������ üũ
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
		
	case 9://���������� ����
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
	case 10://���� ȸ��(�̰� ���̿��Ͼ� �ϵ������ ������ �����δ� �״�� ����)
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
	case 11://���� �� ����
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
				m_nState = 0;////state 0�̸� ������ ����!
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

	for(int i=0;i<Sensor::KINECT_SENSOR_FOV;i++)//kinect range data�� x,y��ǥ�� �ٲ�

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
		return ElvPose;//����ó��

	double dFirstDist=0.0;
	double dSecondDist=0.0;
	double dTemp_dist=0.0;
	int nFirstID=0,nSecondID=0,nTemp_index=0;

	for(int j=0;j<vecXYData.size()-1;j++)//vecXYDate���� ���������� �Ÿ��� ����ū �ΰ� indexã�´�
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

	if(nFirstID>nSecondID)//index������, 0~nSecondID-1, nFirstID+1~end�� �Ŀ� ���Ŵ�, nFistID�� ������պκ�, nSecondID�� ������պκ�
		nFirstID+=1;
	else
	{
		nTemp_index=nSecondID+1;
		nSecondID=nFirstID;
		nFirstID=nTemp_index;
	}

	if(nSecondID<3 || nFirstID>vecXYData.size()-3)//����ó��
		return ElvPose;

	double dCentralPointX = (vecXYData[nFirstID+1].getX()+vecXYData[nSecondID-1].getX())/2;//���ʱ�� ���� ���� ��ǥ����
	double dCentralPointY = (vecXYData[nFirstID+1].getY()+vecXYData[nSecondID-1].getY())/2;

	//���� ��ǥ ���� �Ÿ� �������� ª�� ��� ����ó��
	double dElvWidth_X = vecXYData[nFirstID+1].getX() - vecXYData[nSecondID-1].getX();
	double dElvWidth_Y = vecXYData[nFirstID+1].getY() - vecXYData[nSecondID-1].getY();
	double dElvWidth = sqrt(pow(dElvWidth_X,2.0) + pow(dElvWidth_Y,2.0));
	if(dElvWidth < m_dElvWidth)
		return ElvPose;
	/////////////////////////////////////////////////

	int nNum=vecXYData.size()-(nFirstID-nSecondID+1);
	double dX_sum=0.0,dY_sum=0.0,dXX_sum=0.0,dXY_sum=0.0;//Least square�� �ʿ�

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

	double dA = (nNum*dXY_sum-dX_sum*dY_sum)/(nNum*dXX_sum-dX_sum*dX_sum);//ax+b=y���� ����� ����
	double dB = (-dX_sum*dXY_sum+dXX_sum*dY_sum)/(nNum*dXX_sum-dX_sum*dX_sum);

	double dElvPoseX = (dCentralPointX + dA*dCentralPointY - dA*dB)/(pow(dA,2.0) + 1);//������ �� ���� dCentralPoint �翵��Ŵ
	double dElvPoseY = dA*dElvPoseX + dB;
	double dAngleDeg = atan2(dA,1.0)*R2D;//����������(��) ����

	if(dAngleDeg>90.0)
		dAngleDeg-=180.0;
	else if(dAngleDeg<-90.0)
		dAngleDeg+=180.0;

	ElvPose.setX(dElvPoseX);
	ElvPose.setY(dElvPoseY);
	ElvPose.setThetaDeg(dAngleDeg);//-90~90��

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
	FirstGoalPose.setThetaDeg(dHeadingAngleDeg);//�κ��� ù��° �� ���ϴ� ��

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