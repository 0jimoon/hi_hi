/*______________________________________________________________________________________________
PROJECT : Intelligent Robotics Lab. Department of Mechanical Engineering Korea University                                    
(c) Copyright 2012 KUME Intelligent Robotics Lab.                                             
All rights reserved                                                                         
$Directed by Jae-Bok Song                                                                     
$Designed by Joong-Tae Park                                                                  
$Description :bmp, txt등 여러종류의 지도정보를 저장하고 있는 singletone type의 클래스.
$Created on: 2012. 6. 2.                                                                        
$Author: Joong-Tae Park      
$e-mail: jtpark1114@gmail.com
______________________________________________________________________________________________*/



#ifndef CMAP_REPOSITORY_H_
#define CMAP_REPOSITORY_H_

#include <iostream>
#include <fstream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "../KUNSUtil/KUNSSingletone/KuSingletone.h"
#include "../KUNSUtil/KUNSSmartPointer/KuSmartPointer.h"
#include "../KUNSGUI/KuDrawingInfo.h"
#include "../KUNSUtil/KuUtil.h"
#include "KuMap.h"
#include "../MobileSupervisor/KuRobotParameter.h"

using namespace std;
using namespace cv;

typedef struct _tagCADMapData
{
	double x, y; // center points
	int idx;
	int imageSizeX;
	int imageSizeY;
	Mat sourceImage;

} CCADMapData;

class kuMapRepository : public KuSingletone <kuMapRepository>
{
private:
	KuMap* m_smtpMap; //지도정보를 저장할 공간.
	KuMap* m_pVelocityMap; //지도정보를 저장할 공간.
	int** m_nRefMap; //지도정보를 저장할 공간.
	IplImage*	m_IplMapImage; //OpenCV라이브러 타입의 변수로써, BMP 파일을 읽어드려 임시로 저장할 변수
	KuUtil m_KuUtil;
	string m_strMapFilePath;
	bool m_bUsingRefMapflag;
	vector<CCADMapData> m_vecCADMapData;
	bool m_bCompensationflag;
public:
	bool loadMap(string strMapFilePath);
	KuMap* getMap();
	void generateMap(string strMapFilePath, KuMap* pMap);
	void compensateMap(int nSizeX, int nSizeY, int** nMap);
	void filterMap(KuMap* pMap);
	void filterMapForOccupiedGrid(KuMap* pMap);
	void saveMap(KuMap*pMap,bool bLocalMppingflag = false);
	bool loadVelocityMap(string strMapFilePath);
	int **getVelocityMap();
	//bool loadRefMap(string strMapFilePath);
	int** getRefMap();
	bool loadRefMap(string strMapFilePath,int nMapSizeX,int nMapSizeY);
	void doMorphologyForOccupiedGrid(KuMap* pMap);

private:
	void doMorphologyClose(KuMap* pMap);
	void doMorphologyOpen(KuMap* pMap);
	void doMorphologyCloseForOccupiedGrid(KuMap* pMap);
	void doMorphologyOpenForOccupiedGrid(KuMap* pMap);


public:
	kuMapRepository();
	~kuMapRepository();
};


#endif