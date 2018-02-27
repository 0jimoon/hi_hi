#pragma once
#include "atltypes.h"
#include "glut.h"
#include "glaux.h"	
#include <vector>
#include <list>
#include "../KUNSUtil/KuUtil.h"
#include "../KUNSMath/KuMath.h"
#include "../KUNSPose/KuPose.h"
#include "../KUNSGUI/KuDrawingInfo.h"
#include "../KUNSMap/KuMap.h"
#include "../Sensor/Sensor.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/legacy/compat.hpp"
#include "../MobileSupervisor/KuPOIMapParameter.h"

// CKUNSUI3DDlg dialog

typedef struct _tagGLFONT
{
	GLuint base;
	int widths[256];
	int height;
} GLFONT;

using namespace cv;

static const Vec3b bcolor[] =
{
	Vec3b(0,0,50),		Vec3b(0,0,150),		Vec3b(0,0,250),
	Vec3b(0,100,50),	Vec3b(0,100,150),	Vec3b(0,100,250),
	Vec3b(0,200,50),	Vec3b(0,200,150),	Vec3b(0,200,250),

	Vec3b(50,0,50),		Vec3b(50,0,150),	Vec3b(50,0,250),
	Vec3b(50,100,50),	Vec3b(50,100,150),	Vec3b(50,100,250),
	Vec3b(50,200,50),	Vec3b(50,200,150),	Vec3b(50,200,250),

	Vec3b(100,0,50),	Vec3b(100,0,150),	Vec3b(100,0,250),
	Vec3b(100,100,50),	Vec3b(100,100,150),	Vec3b(100,100,250),
	Vec3b(100,250,50),	Vec3b(100,250,150),	Vec3b(100,250,250),

	Vec3b(150,0,50),	Vec3b(150,0,150),	Vec3b(150,0,250),
	Vec3b(150,100,50),	Vec3b(150,100,150),	Vec3b(150,100,250),
	Vec3b(150,200,50),	Vec3b(150,200,150),	Vec3b(150,200,250),

	Vec3b(200,0,50),	Vec3b(200,0,150),	Vec3b(200,0,250),
	Vec3b(200,100,50),	Vec3b(200,100,150), Vec3b(200,100,250),
	Vec3b(200,200,50),	Vec3b(200,200,150),	Vec3b(200,200,250),

	Vec3b(250,0,50),	Vec3b(250,0,150),	Vec3b(250,0,250),
	Vec3b(250,100,50),	Vec3b(250,100,150),	Vec3b(250,100,250),
	Vec3b(250,200,50),	Vec3b(250,200,150),	Vec3b(250,200,250),

};//54

/*
static const Vec3b bcolor[] =
{
	Vec3b(0,0,50),		Vec3b(0,0,100),		Vec3b(0,0,150),		Vec3b(0,0,200),		Vec3b(0,0,250),
	Vec3b(0,50,50),		Vec3b(0,50,100),	Vec3b(0,50,150),	Vec3b(0,50,200),	Vec3b(0,50,250),
	Vec3b(0,100,50),	Vec3b(0,100,100),	Vec3b(0,100,150),	Vec3b(0,100,200),	Vec3b(0,100,250),
	Vec3b(0,150,50),	Vec3b(0,150,100),	Vec3b(0,150,150),	Vec3b(0,150,200),	Vec3b(0,150,250),
	Vec3b(0,200,50),	Vec3b(0,200,100),	Vec3b(0,200,150),	Vec3b(0,200,200),	Vec3b(0,200,250),
	Vec3b(0,250,50),	Vec3b(0,250,100),	Vec3b(0,250,150),	Vec3b(0,250,200),	Vec3b(0,250,250),

	Vec3b(50,0,50),		Vec3b(50,0,100),	Vec3b(50,0,150),	Vec3b(50,0,200),	Vec3b(50,0,250),
	Vec3b(50,50,50),	Vec3b(50,50,100),	Vec3b(50,50,150),	Vec3b(50,50,200),	Vec3b(50,50,250),
	Vec3b(50,100,50),	Vec3b(50,100,100),	Vec3b(50,100,150),	Vec3b(50,100,200),	Vec3b(50,100,250),
	Vec3b(50,150,50),	Vec3b(50,150,100),	Vec3b(50,150,150),	Vec3b(50,150,200),	Vec3b(50,150,250),
	Vec3b(50,200,50),	Vec3b(50,200,100),	Vec3b(50,200,150),	Vec3b(50,200,200),	Vec3b(50,200,250),
	Vec3b(50,250,50),	Vec3b(50,250,100),	Vec3b(50,250,150),	Vec3b(50,250,200),	Vec3b(50,250,250),

	Vec3b(100,0,50),	Vec3b(100,0,100),	Vec3b(100,0,150),	Vec3b(100,0,200),	Vec3b(100,0,250),
	Vec3b(100,50,50),	Vec3b(100,50,100),	Vec3b(100,50,150),	Vec3b(100,50,200),	Vec3b(100,50,250),
	Vec3b(100,100,50),	Vec3b(100,100,100),	Vec3b(100,100,150),	Vec3b(100,100,200),	Vec3b(100,100,250),
	Vec3b(100,150,50),	Vec3b(100,150,100),	Vec3b(100,150,150),	Vec3b(100,150,200),	Vec3b(100,150,250),
	Vec3b(100,200,50),	Vec3b(100,200,100),	Vec3b(100,200,150),	Vec3b(100,200,200),	Vec3b(100,200,250),
	Vec3b(100,250,50),	Vec3b(100,250,100),	Vec3b(100,250,150),	Vec3b(100,250,200),	Vec3b(100,250,250),

	Vec3b(150,0,50),	Vec3b(150,0,100),	Vec3b(150,0,150),	Vec3b(150,0,200),	Vec3b(150,0,250),
	Vec3b(150,50,50),	Vec3b(150,50,100),	Vec3b(150,50,150),	Vec3b(150,50,200),	Vec3b(150,50,250),
	Vec3b(150,100,50),	Vec3b(150,100,100),	Vec3b(150,100,150),	Vec3b(150,100,200),	Vec3b(150,100,250),
	Vec3b(150,150,50),	Vec3b(150,150,100),	Vec3b(150,150,150),	Vec3b(150,150,200),	Vec3b(150,150,250),
	Vec3b(150,200,50),	Vec3b(150,200,100),	Vec3b(150,200,150),	Vec3b(150,200,200),	Vec3b(150,200,250),
	Vec3b(150,250,50),	Vec3b(150,250,100),	Vec3b(150,250,150),	Vec3b(150,250,200),	Vec3b(150,250,250),

	Vec3b(200,0,50),	Vec3b(200,0,100),	Vec3b(200,0,150),	Vec3b(200,0,200),	Vec3b(200,0,250),
	Vec3b(200,50,50),	Vec3b(200,50,100),	Vec3b(200,50,150),	Vec3b(200,50,200),	Vec3b(200,50,250),
	Vec3b(200,100,50),	Vec3b(200,100,100),	Vec3b(200,100,150),	Vec3b(200,100,200),	Vec3b(200,100,250),
	Vec3b(200,150,50),	Vec3b(200,150,100),	Vec3b(200,150,150),	Vec3b(200,150,200),	Vec3b(200,150,250),
	Vec3b(200,200,50),	Vec3b(200,200,100),	Vec3b(200,200,150),	Vec3b(200,200,200),	Vec3b(200,200,250),
	Vec3b(200,250,50),	Vec3b(200,250,100),	Vec3b(200,250,150),	Vec3b(200,250,200),	Vec3b(200,250,250),

	Vec3b(250,0,50),	Vec3b(250,0,100),	Vec3b(250,0,150),	Vec3b(250,0,200),	Vec3b(250,0,250),
	Vec3b(250,50,50),	Vec3b(250,50,100),	Vec3b(250,50,150),	Vec3b(250,50,200),	Vec3b(250,50,250),
	Vec3b(250,100,50),	Vec3b(250,100,100),	Vec3b(250,100,150),	Vec3b(250,100,200),	Vec3b(250,100,250),
	Vec3b(250,150,50),	Vec3b(250,150,100),	Vec3b(250,150,150),	Vec3b(250,150,200),	Vec3b(250,150,250),
	Vec3b(250,200,50),	Vec3b(250,200,100),	Vec3b(250,200,150),	Vec3b(250,200,200),	Vec3b(250,200,250),
	Vec3b(250,250,50),	Vec3b(250,250,100),	Vec3b(250,250,150),	Vec3b(250,250,200),	Vec3b(250,250,250),

};*/

class CKUNSUI3DDlg : public CDialog
{
	DECLARE_DYNAMIC(CKUNSUI3DDlg)

public:
	CKUNSUI3DDlg(CWnd* pParent = NULL);   // standard constructor
	virtual ~CKUNSUI3DDlg();

	// Dialog Data
	enum { IDD = IDD_KUNSUI3D_DIALOG };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	DECLARE_MESSAGE_MAP()
private:
	CCriticalSection m_CriticalSection;
private:
	double m_dMapHeight, m_dHeightClassifedNode;
	double m_dTopologicalMapHeight;
	bool m_bLButtonDown; 	// Left mouse button down
	bool m_bRButtonDown;
	CPoint m_ptLButtonDownPos; // Position of mouse cursor when it's clicked
	CPoint m_RButtonDownPos;
	GLdouble m_yRotate; // Rotation in y direction
	GLdouble m_xRotate; // Rotation in x direction
	GLFONT* m_pFont; // Pointer variable of GLFONT
	int m_GLPixelIndex;
	HGLRC m_hGLContext;
	GLdouble m_dZoom; // Zoom
	CDC *m_pCamDC, *m_pCamDC2;

	bool m_bSetRobotPos;
	bool m_bSetGoalPos;
	bool m_bSetVirtualObstacleFlag;
	bool m_bSetObsAreas;
	bool m_bSetObsGoalPos;

	KuMath m_math;

	int m_nMapSizeX;
	int m_nMapSizeY;
	double m_dCellSize;
	int m_nDrawingSizeHalf;
	int m_nLocalMapSizeX;
	int m_nLocalMapSizeY;

	int m_nLastMousePointX;
	int m_nLastMousePointY;
	float m_fXOffset;
	float m_fYOffset;

	bool m_bChangePose;

	IplImage * m_IplCeilingImage;
	IplImage * m_IplKinectImg;	
	CvvImage m_cvCeilingImage;
	CvvImage m_cvKinectImage;
	KuPose* m_pKinect3DPos;


private:
	void Initialize();
	GLFONT* OpenGL_Font_Create(HDC hdc, const char* typeface, int height, int weight, DWORD italic); // Create font
	void OpenGL_Font_Destroy(GLFONT* font); // Destroy the font
	void OpenGL_Font_Printf(GLFONT* font, int align, const char* format); // Print the font
	void OpenGL_Font_Puts(GLFONT* font, const char* s); // Put "s" to the font
	void Render_Axis(void); // Render x, y, z axes
	void Render_Grid(const int& nMapWidth, const int& nMapHeight); 	// Render grid
	bool OpenGL_CreateViewGLContext(HDC hDC); 	// Create OpenGL Context
	void OpenGL_RenderScene(void); 	// OpenGL rendering source codes
	bool OpenGL_SetupPixelFormat(HDC hDC); // Setup pixel formats
	void Render_3DRect(double dSizeX, double dSizeY, double dSizeZ);
	void Render_3DRectWire(double dSizeX, double dSizeY, double dSizeZ);
	//void DrawSphere(GLfloat red, GLfloat green, GLfloat blue);
	void createCylinder(GLfloat centerx, GLfloat centery, GLfloat centerz, GLfloat radius, GLfloat h);
	void DrawSphere(GLfloat radius, GLfloat h);

	void renderRobot();
	void renderVirtualRobot();

	void renderPath();
	void renderWayPointList();

	void renderLaserData();
	void renderKinectRangeData();

	void renderBuiltMap();
	void renderBuildingMap();
	void renderGoalPosition();
	void renderObsGoalPosition();

	void renderGoalList();	

	void renderLeastCubicSplinePath();
	void renderTargetPos();

	void renderCADMap();

	void renderSample();
	void reder3DPointCloud();

	void renderLocalPath();
	void renderObsLocalPath();
	void rendervecObsLocalPath();
	void renderVirtualObstacle();
	void renderVirtualGoal();
	void renderProOBBuildingMap();
	void renderObstacleAreas();
	void renderOBBuildingMap();
	void renderTrackedObstID();
	
	void renderPOIMap();
	void renderRplidarData();

	void renderLocalBuildingMap();

	void renderRangeData();
	
	void renderpredictedProMap();

	void renderpredictedTrajectory();
	void renderCubicPath();

	void renderpredictedRobotTrajectory();


public:
	void setCamInfoDC(CDC* pDC);
	void setCamInfo2DC(CDC* pDC);

	void Zoom_In(double dVal);
	void Zoom_Out(double dVal);


	void setRobotPosFlag(bool bFlag);
	void setGoalPosFlag(bool bFlag);
	void setObsAreaPosFlag(bool bFlag);
	void setObsAreaGoalPosFlag(bool bFlag);

	AUX_RGBImageRec *LoadBMPFile(char *filename);


	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnPaint();
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnDestroy();
	virtual BOOL OnInitDialog();
	afx_msg void OnSize(UINT nType, int cx, int cy);	
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);


	virtual BOOL PreTranslateMessage(MSG* pMsg);
};
