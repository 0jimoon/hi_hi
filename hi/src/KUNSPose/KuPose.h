/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mecanical Engineering Korea University                                    
(c) Copyright 2010 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description : ���� ��⿡�� ���Ǵ� ���� ��ġ�������� ����ϴ� �ִ� data Ŭ����
$Data: 2010/03                                                                           
$Author: Joong-Tae Park     
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/
/**
@author: �ۼ��� ǥ�ø� �մϴ�.
@date: �ۼ��ϵ��� ��¥ ǥ�ø� �մϴ�. ���� ��� �ؾ� �մϴ�.
*/


#ifndef KUNS_POSE_H
#define KUNS_POSE_H

#include <cmath>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv/cv.h>

using namespace cv;
using namespace std;
class KuPose
{
private:
	//������ǥ �󿡼��� x,y,theta���� �����ϴ� ����. ex)�κ�pose, goal pose, etc.
	double m_dX; //unit mm
	double m_dY; //unit mm
	double m_dZ; //unit mm
	double m_dThetaDeg; //unit degree
	double m_dThetaRad; //unit radian
	double m_dPixX;//�ȼ� X
	double m_dPixY;//�ȼ� Y

	int m_nID;
	int m_nObsID;
	double m_dDist;
	double m_dPro;
	int m_nGroup;

	//float m_fResultX;
	//float m_fResultY;
	Point2f m_fResult;

public:
	void init();

	double getPixX();
	double getPixY();
	double getX();
	double getY();
	double getZ();
	double getXm();
	double getYm();
	double getZm();
	double getThetaDeg();
	double getThetaRad();
	double getDist();
	int getID();
	double getPro();
	int getObsID();
	int getGroup();
	Point2f getLastResult();
	


	void setPixX(double dPixX);
	void setPixY(double dPixY);
	void setX(double dX);
	void setY(double dY);
	void setZ(double dZ);
	void setXm(double dXm);
	void setYm(double dYm);
	void setZm(double dZm);	
	void setThetaDeg(double dThetaDeg);
	void setThetaRad(double dThetaRad);
	void setDist(double dDist);
	void setID(int nID);
	void setPro(double dPro);
	void setObsID(int nObsID);
	void setGroup(int nGroup);
	void setLastResult(Point2f Resultpoint);

	KuPose( );
	~KuPose();
};
#endif /*KUNS_POSE_H*/