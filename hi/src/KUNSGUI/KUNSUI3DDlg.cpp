// 3DMapDlg\3DMapDlg.cpp : implementation file
//
#include "stdafx.h"
#include "../../KUNS.h"
#include "KUNSUI3DDlg.h"
#include "../../KUNSDlg.h"


#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#endif

IMPLEMENT_DYNAMIC(CKUNSUI3DDlg, CDialog)
	CKUNSUI3DDlg::CKUNSUI3DDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CKUNSUI3DDlg::IDD, pParent)
	, m_bLButtonDown(false)
	, m_ptLButtonDownPos(0)
	, m_yRotate(90)
	, m_xRotate(90)
	, m_pFont(NULL)
	, m_GLPixelIndex(0)
	, m_dZoom(1)
{

	m_bSetRobotPos = false;
	m_bSetGoalPos = false;
	m_bSetObsAreas =false;
	m_bSetObsGoalPos=false;

	m_fXOffset = 0.0;
	m_fYOffset = 0.0;


	m_dMapHeight = 0.02;
	m_dHeightClassifedNode = m_dMapHeight+0.15;
	m_dTopologicalMapHeight = m_dHeightClassifedNode;

	m_IplCeilingImage = cvCreateImage(cvSize( Sensor::CEILING_IMAGE_WIDTH, Sensor::CEILING_IMAGE_HEIGHT),8,1);

	m_IplKinectImg = cvCreateImage(cvSize( Sensor::IMAGE_WIDTH, Sensor::IMAGE_HEIGHT),8,3);

	m_pKinect3DPos = new KuPose[Sensor::KINECT_IMAGE_WIDTH * Sensor::KINECT_IMAGE_HEIGHT];

}

CKUNSUI3DDlg::~CKUNSUI3DDlg()
{

	if(m_IplKinectImg!=NULL)
	{
		cvReleaseImage(&m_IplKinectImg);
		m_IplKinectImg=NULL;
	}
	if(m_pKinect3DPos!=NULL)
	{
		delete [] m_pKinect3DPos;
		m_pKinect3DPos=NULL;
	}
}

void CKUNSUI3DDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
}


BEGIN_MESSAGE_MAP(CKUNSUI3DDlg, CDialog)
	ON_WM_CREATE()
	ON_WM_PAINT()
	ON_WM_MOUSEMOVE()
	ON_WM_LBUTTONDOWN()
	ON_WM_LBUTTONUP()
	ON_WM_DESTROY()
	ON_WM_SIZE()
	ON_WM_MOUSEWHEEL()
	ON_WM_RBUTTONDOWN()
	ON_WM_RBUTTONUP()
END_MESSAGE_MAP()


// Setup pixel formats
bool CKUNSUI3DDlg::OpenGL_SetupPixelFormat(HDC hDC)
{
	PIXELFORMATDESCRIPTOR pixelDesc;

	pixelDesc.nSize = sizeof(PIXELFORMATDESCRIPTOR);
	pixelDesc.nVersion = 1;

	pixelDesc.dwFlags = PFD_DRAW_TO_WINDOW | 
		PFD_SUPPORT_OPENGL |
		PFD_DOUBLEBUFFER |
		PFD_STEREO_DONTCARE;

	pixelDesc.iPixelType = PFD_TYPE_RGBA;
	pixelDesc.cColorBits = 32;
	pixelDesc.cRedBits = 8;
	pixelDesc.cRedShift = 16;
	pixelDesc.cGreenBits = 8;
	pixelDesc.cGreenShift = 8;
	pixelDesc.cBlueBits = 8;
	pixelDesc.cBlueShift = 0;
	pixelDesc.cAlphaBits = 0;
	pixelDesc.cAlphaShift = 0;
	pixelDesc.cAccumBits = 64;
	pixelDesc.cAccumRedBits = 16;
	pixelDesc.cAccumGreenBits = 16;
	pixelDesc.cAccumBlueBits = 16;
	pixelDesc.cAccumAlphaBits = 0;
	pixelDesc.cDepthBits = 32;
	pixelDesc.cStencilBits = 8;
	pixelDesc.cAuxBuffers = 0;
	pixelDesc.iLayerType = PFD_MAIN_PLANE;
	pixelDesc.bReserved = 0;
	pixelDesc.dwLayerMask = 0;
	pixelDesc.dwVisibleMask = 0;
	pixelDesc.dwDamageMask = 0;

	m_GLPixelIndex = ChoosePixelFormat(hDC,&pixelDesc);
	if(m_GLPixelIndex==0) // Choose default
	{
		m_GLPixelIndex = 1;
		if(DescribePixelFormat(hDC,m_GLPixelIndex,
			sizeof(PIXELFORMATDESCRIPTOR),&pixelDesc)==0)
		{
			return FALSE;
		}
	}

	if(SetPixelFormat(hDC,m_GLPixelIndex,&pixelDesc)==FALSE)
	{
		return FALSE;
	}

	return TRUE;
}

int CKUNSUI3DDlg::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CDialog::OnCreate(lpCreateStruct) == -1)
		return -1;

	// TODO:  Add your specialized creation code here
	//OpenGL_SetupPixelFormat(::GetDC(m_hWnd));

	return 0;
}

void CKUNSUI3DDlg::OnPaint()
{
	CPaintDC dc(this); // device context for painting
	// TODO: Add your message handler code here
	if (IsIconic())
	{
		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;
	}
	else
	{

		OpenGL_RenderScene();
		SwapBuffers(dc.m_ps.hdc);				
		CDialog::OnPaint();

	}

	// Do not call CDialog::OnPaint() for painting messages
}

// OpenGL rendering source codes
void CKUNSUI3DDlg::OpenGL_RenderScene(void)
{
	//glClearColor(1.0, 1.0, 1.0, 1.0);
	glClearColor(0.65,0.65,0.65,0.3);
	GLfloat lightPos[] = {-50, 50, 40, 0};
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();

	// -------------- Draw here --------------
	glPushMatrix();

	glTranslated(0.0, 0.0, -13.0 + m_dZoom);
	glTranslated(m_fXOffset, m_fYOffset, 0.0);
	glRotatef(m_xRotate, 1.0, 0.0, 0.0);
	glRotatef(m_yRotate, 0.0, 1.0, 0.0);
	//glRotatef(m_yRotate-RobotPos.getThetaDeg()+90, 0.0, 1.0, 0.0);
	glRotatef(-90.0, 1.0, 0.0, 0.0);

	glPushMatrix();
	RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	glTranslated(-RobotPos.getXm(), -RobotPos.getYm(),0);

	glEnable(GL_LINE_SMOOTH);
	Render_Axis();
	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_LIGHT0);
	Render_Grid(2000, 2000);
	glEnable(GL_LIGHT0);
	renderRobot(); //로봇을 그려준다.


	if(KuDrawingInfo::getInstance()->getRenderLaserflag()==true){ 
		renderLaserData(); //레이저 데이터를 그리도록 플래그 셋팅한 후에 화면에 그려준다.
		renderRplidarData();
	}	
	if(KuDrawingInfo::getInstance()->getRenderMapflag()==true){ //이미 만들어진 격자지도를 화면에 그려준다.
		//영역 구분 지도가 그려질때는 보통 그리드 지도는 그리지 않기위해서
		if(KuDrawingInfo::getInstance()->getRenderBuildingMapflag()==true){
			renderBuildingMap(); //레이저로 작성되는 지도를 그린다.
		}
		else
		{
			renderBuiltMap();			
		}
	}

	if(KuDrawingInfo::getInstance()->getRenderCADMapflag()==true)
	{
		renderCADMap();
	}


	if(KuDrawingInfo::getInstance()->getRenderPathflag()==true){
		renderPath();		
		renderWayPointList();
	}

	renderTargetPos();

	renderGoalPosition();
	renderObsGoalPosition();

	renderSample();

	if(KuDrawingInfo::getInstance()->getRenderKinect3DCloudFlag() == true){
		reder3DPointCloud();
	}
	renderKinectRangeData();
	renderLocalPath();
	renderObsLocalPath();

	rendervecObsLocalPath();

	renderProOBBuildingMap();
	renderLocalBuildingMap();
	renderOBBuildingMap();
	renderRangeData();

	renderVirtualObstacle();
	renderTrackedObstID();
	renderVirtualGoal();
	renderpredictedTrajectory();
	renderCubicPath();


	renderpredictedProMap();
	//renderObstacleAreas();
	renderpredictedRobotTrajectory();

	renderPOIMap();

	glPopMatrix();

	glPopMatrix();

	// -------------- Draw here --------------

}

void CKUNSUI3DDlg::Render_3DRect(double dSizeX, double dSizeY, double dSizeZ)
{
	double dHalfSizeX = dSizeX / 2;
	double dHalfSizeY = dSizeY / 2;
	double dHalfSizeZ = dSizeZ / 2;

	GLUquadricObj *pObj;
	pObj = gluNewQuadric();
	gluQuadricNormals(pObj, GLU_SMOOTH);

	glDisable(GL_CULL_FACE );
	glEnable(GL_LINE_SMOOTH);

	glPushMatrix();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glEnd();

	glPopMatrix();

	glEnable(GL_CULL_FACE);

	gluDeleteQuadric(pObj);
}

void CKUNSUI3DDlg::Render_3DRectWire(double dSizeX, double dSizeY, double dSizeZ)
{
	double dHalfSizeX = dSizeX / 2;
	double dHalfSizeY = dSizeY / 2;
	double dHalfSizeZ = dSizeZ / 2;

	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	//glDisable(GL_CULL_FACE );
	//	glDisable(GL_LINE_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glEnable( GL_POLYGON_SMOOTH );

	glPushMatrix();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(-dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, dHalfSizeY, -dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, -dHalfSizeY, -dHalfSizeZ);
	glEnd();

	glBegin(GL_POLYGON);
	glVertex3d(dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glVertex3d(dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, dHalfSizeY, dHalfSizeZ);
	glVertex3d(-dHalfSizeX, -dHalfSizeY, dHalfSizeZ);
	glEnd();

	glPopMatrix();
	glPolygonMode(GL_FRONT, GL_FILL);
	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_POLYGON_SMOOTH);
}

void CKUNSUI3DDlg::setRobotPosFlag(bool bFlag)
{
	m_bSetRobotPos = bFlag;
	m_yRotate =0;
	m_xRotate = 90;
	InvalidateRect(NULL,FALSE);
}

void CKUNSUI3DDlg::setGoalPosFlag(bool bFlag)
{
	m_bSetGoalPos = bFlag;
	m_yRotate =0;
	m_xRotate = 90;
	InvalidateRect(NULL,FALSE);
}

void CKUNSUI3DDlg::setObsAreaPosFlag(bool bFlag)
{
	m_bSetObsAreas = bFlag;
	m_yRotate =0;
	m_xRotate = 90;
	InvalidateRect(NULL,FALSE);
}

void CKUNSUI3DDlg::setObsAreaGoalPosFlag(bool bFlag)
{
	m_bSetObsGoalPos = bFlag;
	m_yRotate =0;
	m_xRotate = 90;
	InvalidateRect(NULL,FALSE);
}

void CKUNSUI3DDlg::OnMouseMove(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	if(m_bSetRobotPos == false &&  m_bSetGoalPos == false && m_bLButtonDown){
		CSize rotate = m_ptLButtonDownPos - point;
		m_ptLButtonDownPos = point;

		m_yRotate -= rotate.cx;
		m_xRotate -= rotate.cy;

		InvalidateRect(NULL,FALSE);
	}
	else if(m_bSetRobotPos && m_bLButtonDown){

		m_fXOffset -= (float)(m_nLastMousePointX - (int)point.x)/50.0;
		m_fYOffset += (float)(m_nLastMousePointY - (int)point.y)/50.0;
		m_nLastMousePointX = point.x;
		m_nLastMousePointY = point.y;
		InvalidateRect(NULL,FALSE);
	}

	CDialog::OnMouseMove(nFlags, point);

}

void CKUNSUI3DDlg::OnLButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bLButtonDown = TRUE;
	m_ptLButtonDownPos = point;

	m_nLastMousePointX = point.x;
	m_nLastMousePointY = point.y;

	CDialog::OnLButtonDown(nFlags, point);
}

void CKUNSUI3DDlg::OnLButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bLButtonDown = FALSE;
	if(m_bLButtonDown){
		CSize rotate = m_ptLButtonDownPos - point;
		m_ptLButtonDownPos = point;
		m_yRotate -= rotate.cx;
		m_xRotate -= rotate.cy;
		InvalidateRect(NULL,FALSE);

	}	
	CDialog::OnLButtonUp(nFlags, point);
}

void CKUNSUI3DDlg::OnRButtonDown(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bRButtonDown = TRUE;
	m_RButtonDownPos = point;

	m_nLastMousePointX = point.x;
	m_nLastMousePointY = point.y;

	CDialog::OnRButtonDown(nFlags, point);
}

void CKUNSUI3DDlg::OnRButtonUp(UINT nFlags, CPoint point)
{
	// TODO: Add your message handler code here and/or call default
	m_bRButtonDown = FALSE;
	int nUISize = 250;		//DrawUI 사이즈 640*640;
	double dUI2Grid = 30;	//한격자간격은 4 point간격

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	KuPose GoalPos;
	KuPose ObsAreas;


	if(m_bSetRobotPos)
	{	
		glPushMatrix();
		glRotatef(m_xRotate, 1.0, 0.0, 0.0);
		glRotatef(m_yRotate, 0.0, 1.0, 0.0);
		glRotatef(-90.0, 1.0, 0.0, 0.0);	

		GLdouble model[16];
		GLdouble proj[16];
		GLint viewport[4];
		glGetDoublev(GL_MODELVIEW_MATRIX,model);
		glGetDoublev(GL_PROJECTION_MATRIX,proj);
		glGetIntegerv(GL_VIEWPORT,viewport);	

		double pos[3];
		memset(pos,0,sizeof(pos));
		double win[3]={ (double)point.x , viewport[3] - (double)point.y,0};

		gluUnProject(win[0], win[1],win[2],model,proj,viewport, &pos[0],&pos[1],&pos[2]);
		glPopMatrix();		

		RobotPos.setXm(pos[0] *(13.0 - m_dZoom)+RobotPos.getXm());
		RobotPos.setYm(pos[1] *(13.0 - m_dZoom)+RobotPos.getYm());
		RobotPos.setID(KuDrawingInfo::getInstance()->getCurFloor());
		KuDrawingInfo::getInstance()->setRobotPos(RobotPos);

	}
	else if(m_bSetGoalPos){	

		glPushMatrix();
		glRotatef(m_xRotate, 1.0, 0.0, 0.0);
		glRotatef(m_yRotate, 0.0, 1.0, 0.0);
		glRotatef(-90.0, 1.0, 0.0, 0.0);	

		GLdouble model[16];
		GLdouble proj[16];
		GLint viewport[4];
		glGetDoublev(GL_MODELVIEW_MATRIX,model);
		glGetDoublev(GL_PROJECTION_MATRIX,proj);
		glGetIntegerv(GL_VIEWPORT,viewport);	

		double pos[3];
		memset(pos,0,sizeof(pos));
		double win[3]={ (double)point.x , viewport[3] - (double)point.y,0};

		gluUnProject(win[0], win[1],win[2],model,proj,viewport, &pos[0],&pos[1],&pos[2]);
		glPopMatrix();		

		GoalPos.setXm(pos[0] *(13.0 - m_dZoom)+RobotPos.getXm());
		GoalPos.setYm(pos[1] *(13.0 - m_dZoom)+RobotPos.getYm());
		GoalPos.setID(KuDrawingInfo::getInstance()->getCurFloor());
		KuDrawingInfo::getInstance()->setGoalPos(GoalPos);

	}
	else if(m_bSetObsAreas)
	{
		glPushMatrix();
		glRotatef(m_xRotate, 1.0, 0.0, 0.0);
		glRotatef(m_yRotate, 0.0, 1.0, 0.0);
		glRotatef(-90.0, 1.0, 0.0, 0.0);	

		GLdouble model[16];
		GLdouble proj[16];
		GLint viewport[4];
		glGetDoublev(GL_MODELVIEW_MATRIX,model);
		glGetDoublev(GL_PROJECTION_MATRIX,proj);
		glGetIntegerv(GL_VIEWPORT,viewport);	

		double pos[3];
		memset(pos,0,sizeof(pos));
		double win[3]={ (double)point.x , viewport[3] - (double)point.y,0};

		gluUnProject(win[0], win[1],win[2],model,proj,viewport, &pos[0],&pos[1],&pos[2]);
		glPopMatrix();		

		ObsAreas.setXm(pos[0] *(13.0 - m_dZoom)+RobotPos.getXm());
		ObsAreas.setYm(pos[1] *(13.0 - m_dZoom)+RobotPos.getYm());
		ObsAreas.setID(KuDrawingInfo::getInstance()->getPatternID());
		KuDrawingInfo::getInstance()->setObstacleAreas(ObsAreas);
		m_bSetObsAreas=false;
		m_bSetObsGoalPos=true;
	}
	else if(m_bSetObsGoalPos){	

		glPushMatrix();
		glRotatef(m_xRotate, 1.0, 0.0, 0.0);
		glRotatef(m_yRotate, 0.0, 1.0, 0.0);
		glRotatef(-90.0, 1.0, 0.0, 0.0);	

		GLdouble model[16];
		GLdouble proj[16];
		GLint viewport[4];
		glGetDoublev(GL_MODELVIEW_MATRIX,model);
		glGetDoublev(GL_PROJECTION_MATRIX,proj);
		glGetIntegerv(GL_VIEWPORT,viewport);	

		double pos[3];
		memset(pos,0,sizeof(pos));
		double win[3]={ (double)point.x , viewport[3] - (double)point.y,0};

		gluUnProject(win[0], win[1],win[2],model,proj,viewport, &pos[0],&pos[1],&pos[2]);
		glPopMatrix();		

		GoalPos.setXm(pos[0] *(13.0 - m_dZoom)+RobotPos.getXm());
		GoalPos.setYm(pos[1] *(13.0 - m_dZoom)+RobotPos.getYm());
		GoalPos.setID(KuDrawingInfo::getInstance()->getCurFloor());
		KuDrawingInfo::getInstance()->setObsGoalPos(GoalPos);
		m_bSetObsAreas=true;
		m_bSetObsGoalPos=false;

	}
	

	m_fXOffset = 0;			//초기화
	m_fYOffset = 0;

	InvalidateRect(NULL,FALSE);
	CDialog::OnRButtonUp(nFlags, point);
}


// Create font
GLFONT* CKUNSUI3DDlg::OpenGL_Font_Create(HDC hdc, const char* typeface, int height, int weight, DWORD italic)
{
	GLFONT *font;
	HFONT fontid;
	int charset;

	if((font = (GLFONT *)calloc(1, sizeof(GLFONT))) == (GLFONT *)0)
		return((GLFONT *)0);

	if((font->base = glGenLists(256)) == 0)
	{
		free(font);
		return (0);
	}

	if(_stricmp(typeface, "symbol") == 0)
		charset = SYMBOL_CHARSET;
	else
		charset = ANSI_CHARSET;

	fontid = CreateFont(height, 0, 0, 0, weight, italic, FALSE, FALSE,
		charset, OUT_TT_PRECIS, CLIP_DEFAULT_PRECIS,
		DRAFT_QUALITY, DEFAULT_PITCH, (LPCWSTR)typeface);

	SelectObject(hdc, fontid);

	wglUseFontBitmaps(hdc, 0, 256, font->base);
	GetCharWidth(hdc, 0, 255, font->widths);
	font->height = height;

	return (font);
}

// Destroy the font
void CKUNSUI3DDlg::OpenGL_Font_Destroy(GLFONT* font)
{
	if(font == (GLFONT *)0)
		return;

	glDeleteLists(font->base, 256);
	free(font);
}

// Print the font
void CKUNSUI3DDlg::OpenGL_Font_Printf(GLFONT* font, int align, const char* format)
{
	glDisable(GL_LIGHTING);

	va_list ap;
	unsigned char s[1024], *ptr;
	int width;

	if(font == (GLFONT *)0 || format == (char *)0)
		return;

	va_start(ap, format);
	vsprintf((char *)s, format, ap);
	va_end(ap);

	for(ptr = s, width = 0; *ptr; ptr ++)
		width += font->widths[*ptr];

	if(align < 0)
		glBitmap(0, 0, 0.0f, 0.0f, (float)-width, 0.0f, NULL);
	else if(align == 0)
		glBitmap(0, 0, 0.0f, 0.0f, (float)-width / (float)2, 0.0f, NULL);

	OpenGL_Font_Puts(font, (const char *)s);

	glEnable(GL_LIGHTING);
}

// Put "s" to the font
void CKUNSUI3DDlg::OpenGL_Font_Puts(GLFONT* font, const char* s)
{
	if(font == (GLFONT *)0 || s == NULL)
		return;

	glPushAttrib(GL_LIST_BIT);
	glListBase(font->base);
	glCallLists(strlen(s), GL_UNSIGNED_BYTE, s);
	glPopAttrib();
}

// Render x, y, z axes
void CKUNSUI3DDlg::Render_Axis(void)
{
	char c[4];

	glPushMatrix();

	glColor3ub(100, 100, 100);

	glRasterPos3f(1.2, 0, -0.1);
	sprintf_s(c, "X");
	OpenGL_Font_Printf(m_pFont, 0, c);

	glRasterPos3f(0, 1.2, -0.1);
	sprintf_s(c, "Y");
	OpenGL_Font_Printf(m_pFont, 0, c);

	glRasterPos3f(0, 0, 1.2);
	sprintf_s(c, "Z");
	OpenGL_Font_Printf(m_pFont, 0, c);

	glColor3ub(255, 0, 0);

	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(1, 0, 0);
	glEnd();

	glColor3ub(0, 255, 0);

	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 1, 0);
	glEnd();

	glColor3ub(0, 0, 255);

	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 1);
	glEnd();

	glPopMatrix();
}

void CKUNSUI3DDlg::OnDestroy()
{
	CDialog::OnDestroy();

	// TODO: Add your message handler code here
	if(wglGetCurrentContext() != NULL)
		wglMakeCurrent(NULL,NULL);

	if(m_hGLContext != NULL)
	{
		wglDeleteContext(m_hGLContext);
		m_hGLContext = NULL;
	}

	OpenGL_Font_Destroy(m_pFont);
}

BOOL CKUNSUI3DDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  Add extra initialization here
	HWND hWnd = GetSafeHwnd();
	HDC hDC = ::GetDC(hWnd);

	if(OpenGL_SetupPixelFormat(hDC) == false)
		return 0;

	if(OpenGL_CreateViewGLContext(hDC)==false)
		return 0;

	glShadeModel(GL_SMOOTH);
	glClearColor((float)1.0, (float)1.0, (float)1.0, 1.0);
	//	glClearDepth(1.0f);

	//	glPolygonMode(GL_FRONT,GL_LINE);
	//	glPolygonMode(GL_BACK,GL_LINE);

	//	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glEnable(GL_CULL_FACE);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);

	// light
	GLfloat ambientLight[] = {0.5, 0.5, 0.5, 1.0};
	GLfloat diffuseLight[] = {0.5, 1.0, 1.0, 1.0};
	GLfloat specular[] = {1.0, 1.0, 1.0, 1.0};

	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);

	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);
	//	glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
	glEnable(GL_LIGHT0);

	glEnable(GL_NORMALIZE);
	//	glClearColor(0, 0, 0, 1.0);

	m_pFont = OpenGL_Font_Create(hDC, "Verdana", 13, 1, 0);

	Initialize();

	return TRUE;  // return TRUE unless you set the focus to a control
	// EXCEPTION: OCX Property Pages should return FALSE
}

void CKUNSUI3DDlg::OnSize(UINT nType, int cx, int cy)
{
	CDialog::OnSize(nType, cx, cy);

	// TODO: Add your message handler code here
	GLsizei width,height;
	GLdouble aspect;

	width = cx;
	height = cy;

	if(cy==0)
		aspect = (GLdouble)width;
	else
		aspect = (GLdouble)width/(GLdouble)height;

	glViewport(0,0,width,height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45,aspect,1,200.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glDrawBuffer(GL_BACK);

	glEnable(GL_DEPTH_TEST);
}


void CKUNSUI3DDlg::Initialize()
{
	m_bChangePose = false;

	m_bLButtonDown = FALSE;
	m_bRButtonDown = FALSE;
	m_fXOffset = 0.0;
	m_fYOffset = 0.0;

	m_nMapSizeX = 0;
	m_nMapSizeY = 0;


	m_dCellSize = (double)(Sensor::CELLSIZE)/1000.0;
	
	// 전체 지도 중 그림을 그릴 크기
	float fTemp = (float)0.1;
	m_nDrawingSizeHalf = (int)(50.0 / fTemp);	// 상하좌우 20m 범위를 그리는 것이 초기값.

}

// Render grid
void CKUNSUI3DDlg::Render_Grid(const int& nMapWidth, const int& nMapHeight)
{

	double i, j;
	double dHalfSizeX, dHalfSizeY;

	dHalfSizeX = nMapWidth * 0.1 / 2.;
	dHalfSizeY = nMapHeight * 0.1 / 2.;

	glColor4ub(200,200,200,60);

	glPushMatrix();

	glTranslated(100,100,0);


	for(i = -dHalfSizeX; i <= dHalfSizeX; i++)
	{
		glBegin(GL_LINES);
		glVertex3f((float)i, dHalfSizeY, -0.0f);
		glVertex3f((float)i, -dHalfSizeY, -0.0f);
		glEnd();
	}

	for(j = -dHalfSizeY; j <= dHalfSizeY; j++)
	{
		glBegin(GL_LINES);
		glVertex3f(-dHalfSizeX, (float)j, -0.0f);
		glVertex3f(dHalfSizeX, (float)j, -0.0f);
		glEnd();
	}

	glColor4ub(200,200,200,10);

	for(i = -dHalfSizeX; i <= dHalfSizeX; i+=0.1)
	{
		glBegin(GL_LINES);
		glVertex3f((float)i, dHalfSizeY, -0.0f);
		glVertex3f((float)i, -dHalfSizeY, -0.0f);
		glEnd();
	}

	for(j = -dHalfSizeY; j <= dHalfSizeY; j+=0.1)
	{
		glBegin(GL_LINES);
		glVertex3f(-dHalfSizeX, (float)j, -0.0f);
		glVertex3f(dHalfSizeX, (float)j, -0.0f);
		glEnd();
	}

	glPopMatrix();
}

// Create OpenGL Context
bool CKUNSUI3DDlg::OpenGL_CreateViewGLContext(HDC hDC)
{
	m_hGLContext = wglCreateContext(hDC);

	if(m_hGLContext==NULL)
		return FALSE;

	if(wglMakeCurrent(hDC,m_hGLContext)==FALSE)
		return FALSE;

	return TRUE;
}

void CKUNSUI3DDlg::Zoom_In(double dVal)
{
	m_dZoom += dVal;
	m_nDrawingSizeHalf -= (int)(10.0/m_dCellSize);
	if (m_nDrawingSizeHalf<=(int)(5.0/m_dCellSize)){
		m_nDrawingSizeHalf = (int)(15.0/m_dCellSize);	// 최소 15m는 그리도록
	}
	InvalidateRect(NULL,FALSE);
}

void CKUNSUI3DDlg::Zoom_Out(double dVal)
{
	m_dZoom -= dVal;
	m_nDrawingSizeHalf += (int)(10.0/m_dCellSize);


	if (m_nMapSizeX>=m_nMapSizeY &&  m_nDrawingSizeHalf>m_nMapSizeX/2)
		m_nDrawingSizeHalf = m_nMapSizeX/2;
	else if (m_nMapSizeX<m_nMapSizeY &&  m_nDrawingSizeHalf>m_nMapSizeY/2)
		m_nDrawingSizeHalf = m_nMapSizeY/2;

	InvalidateRect(NULL,FALSE);
}

void CKUNSUI3DDlg::setCamInfoDC(CDC* pDC)
{
	m_pCamDC = pDC;	
}

void CKUNSUI3DDlg::setCamInfo2DC(CDC* pDC)
{
	m_pCamDC2 = pDC;	
}

void CKUNSUI3DDlg::renderVirtualRobot()
{
	glPushMatrix();	

	KuPose VirtualRobotPos;
	VirtualRobotPos = KuDrawingInfo::getInstance()->getAuxiliaryRobotPos();

	glTranslated(VirtualRobotPos.getXm(), VirtualRobotPos.getYm(), 0);
	glRotatef(VirtualRobotPos.getThetaDeg(), 0.0, 0.0, 1.0);

	//------------------------ pioneer 3AT 비슷하게.... --------------------------------------//
	// 변수 지정해서 하려 했는데 이상하게 꼬였음.... ㅠㅠ
	double h1 = 0.07;
	double h2 = 0.25;
	double h3 = 0.27;
	double dx1 = 0.2;
	double dy1 = 0.12;
	double dx11 = 0.24;
	double dx2 = 0.23;
	double dy2 = 0.15;
	double dx22 = 0.27;
	double r1 = 0.1;
	double r2 = 0.05;
	double wx = 0.13;
	double wy = 0.17;

	// ----------------------- 옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h2);
	glVertex3f(dx1, -dy1, h1);

	glVertex3f(dx11, 0, h2);
	glVertex3f(dx11, 0, h1);

	glVertex3f(dx1, dy1, h2);
	glVertex3f(dx1, dy1, h1);

	glVertex3f(-dx1, dy1, h2);
	glVertex3f(-dx1, dy1, h1);

	glVertex3f(-dx11, 0, h2);
	glVertex3f(-dx11, 0, h1);

	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();

	// 모서리에 라인을 그려서 강조
	glColor3ub(0, 0, 255);
	glBegin(GL_LINES);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(dx1, -dy1, h2);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glBegin(GL_LINES);	
	glVertex3f(dx11, 0, h2);
	glVertex3f(dx11, 0, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(dx1, dy1, h2);
	glVertex3f(dx1, dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx1, dy1, h2);
	glVertex3f(-dx1, dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx11, 0, h2);
	glVertex3f(-dx11, 0, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();

	//------------------------------------- 윗면
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glVertex3f(dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h2);

	glVertex3f(dx22, 0, h3);
	glVertex3f(dx22, 0, h2);

	glVertex3f(dx2, dy2, h3);
	glVertex3f(dx2, dy2, h2);

	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h2);

	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx22, 0, h2);

	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex3f(dx22, 0, h3);
	glVertex3f(dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h3);
	glEnd();
	// 기둥 4개
	glPushMatrix();
	glTranslated(0.15, 0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(-0.15, 0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(-0.15, -0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(0.15, -0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();

	//------------------------------------- 윗면2
	glPushMatrix();
	glTranslated(0,0,0.1);
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glVertex3f(dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h2);

	glVertex3f(dx22, 0, h3);
	glVertex3f(dx22, 0, h2);

	glVertex3f(dx2, dy2, h3);
	glVertex3f(dx2, dy2, h2);

	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h2);

	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx22, 0, h2);

	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex3f(dx22, 0, h3);
	glVertex3f(dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h3);
	glEnd();
	glPopMatrix();

	//---------------------------------------------------- 바퀴
	glPushMatrix();
	glColor3ub(0, 0, 255);
	glTranslated(wx,wy,r1);
	glRotatef(90, 1.0, 0.0, 0.0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(-2.0*wx,0,0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(0,0,2.0*wy);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(2.0*wx,0,0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);	
	glPopMatrix();

	// -------------------------------------------- 뒷 박스
	glPushMatrix();
	h3+=0.1;
	glTranslated(-0.15,0, h3);
	dx1 = 0.1;dx2 = dx1;
	dy1 = 0.12; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glColor3ub(0, 0, 255);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glEnd();
	glPopMatrix();

	// -------------------------------------------- 레이저스캐너 기둥
	glPushMatrix();
	glTranslated(0.1,0.1,h3);
	dx1 = 0.03;
	dy1 = 0.01;
	h1 = 0.2;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glBegin(GL_POLYGON);
	glVertex3f(dx1, dy1, h1);
	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glTranslated(0,-0.2,0);
	//옆면 4개
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glBegin(GL_POLYGON);
	glVertex3f(dx1, dy1, h1);
	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glPopMatrix();

	// -------------------------------------------- 레이저스캐너
	glPushMatrix();
	glTranslated(0.1,0,0.55);
	glRotatef(0, 0.0, 1.0, 0.0);
	glTranslated(-0.1,0,-0.55);
	// 맨 아래
	h3+=0.1;
	glTranslated(0.15,0,h3);
	dx1 = 0.075;dx2 = dx1;
	dx11 = 0.095; dx22 = dx11;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.02;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	//윗면
	glColor3ub(0, 0, 255);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx22, 0, h1);
	glEnd();

	// 중간
	glTranslated(0,0,h1);
	dx1 = 0.075;dx2 = dx1;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);

	glVertex3f(0, dy2, h1);
	glVertex3f(0, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	dx1 = 0.045;dx2 = 0.055;
	dx11 = 0.060; dx22 = 0.075;
	dy1 = 0.055; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(0, dy2, h1);
	glVertex3f(0, dy2, 0);

	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);
	glEnd();


	// 윗 부분
	glTranslated(0,0,h1);
	dx1 = 0.075;dx2 = dx1;
	dx11 = 0.095; dx22 = dx11;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.06;
	//옆면 4개
	glColor3ub(0, 0, 255);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	//윗면
	glColor3ub(0, 0, 255);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx22, 0, h1);
	glEnd();

	glPopMatrix();
	//-------------------------------------------------------------------------------------------//
	glPopMatrix();


}


void CKUNSUI3DDlg::renderRobot()
{
	glPushMatrix();	

	KuPose RobotPos;
	RobotPos = KuDrawingInfo::getInstance()->getRobotPos();


	glTranslated(RobotPos.getXm(), RobotPos.getYm(), 0);
	glRotatef(RobotPos.getThetaDeg(), 0.0, 0.0, 1.0);
	
	if(RobotPos.getID()!= KuDrawingInfo::getInstance()->getCurFloor()) {glPopMatrix();return;}

	//------------------------ pioneer 3AT 비슷하게.... --------------------------------------//
	double h1 = 0.07;
	double h2 = 0.25;
	double h3 = 0.27;
	double dx1 = 0.2;
	double dy1 = 0.12;
	double dx11 = 0.24;
	double dx2 = 0.23;
	double dy2 = 0.15;
	double dx22 = 0.27;
	double r1 = 0.1;
	double r2 = 0.05;
	double wx = 0.13;
	double wy = 0.17;

	// ----------------------- 옆면 4개
	glColor3ub(200, 0, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h2);
	glVertex3f(dx1, -dy1, h1);

	glVertex3f(dx11, 0, h2);
	glVertex3f(dx11, 0, h1);

	glVertex3f(dx1, dy1, h2);
	glVertex3f(dx1, dy1, h1);

	glVertex3f(-dx1, dy1, h2);
	glVertex3f(-dx1, dy1, h1);

	glVertex3f(-dx11, 0, h2);
	glVertex3f(-dx11, 0, h1);

	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();

	// 모서리에 라인을 그려서 강조
	glColor3ub(0, 0, 0);
	glBegin(GL_LINES);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(dx1, -dy1, h2);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glBegin(GL_LINES);	
	glVertex3f(dx11, 0, h2);
	glVertex3f(dx11, 0, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(dx1, dy1, h2);
	glVertex3f(dx1, dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx1, dy1, h2);
	glVertex3f(-dx1, dy1, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx11, 0, h2);
	glVertex3f(-dx11, 0, h1);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(-dx1, -dy1, h2);
	glVertex3f(-dx1, -dy1, h1);
	glEnd();

	//------------------------------------- 윗면
	glColor3ub(50, 50, 50);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glVertex3f(dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h2);

	glVertex3f(dx22, 0, h3);
	glVertex3f(dx22, 0, h2);

	glVertex3f(dx2, dy2, h3);
	glVertex3f(dx2, dy2, h2);

	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h2);

	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx22, 0, h2);

	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex3f(dx22, 0, h3);
	glVertex3f(dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h3);
	glEnd();
	// 기둥 4개
	glPushMatrix();
	glTranslated(0.15, 0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(-0.15, 0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(-0.15, -0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();
	glPushMatrix();
	glTranslated(0.15, -0.1, h3+0.1);
	glRotatef(-180, 1.0, 0.0, 0.0);
	glutSolidCone(0.02, 0.2, 4, 4);
	glPopMatrix();

	//------------------------------------- 윗면2
	glPushMatrix();
	glTranslated(0,0,0.1);
	glColor3ub(50, 50, 50);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glVertex3f(dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h2);

	glVertex3f(dx22, 0, h3);
	glVertex3f(dx22, 0, h2);

	glVertex3f(dx2, dy2, h3);
	glVertex3f(dx2, dy2, h2);

	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h2);

	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx22, 0, h2);

	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(-dx2, -dy2, h2);
	glEnd();
	glBegin(GL_POLYGON);
	glVertex3f(dx22, 0, h3);
	glVertex3f(dx2, dy2, h3);
	glVertex3f(-dx2, dy2, h3);
	glVertex3f(-dx22, 0, h3);
	glVertex3f(-dx2, -dy2, h3);
	glVertex3f(dx2, -dy2, h3);
	glEnd();
	glPopMatrix();

	//---------------------------------------------------- 바퀴
	glPushMatrix();
	glColor3ub(50, 50, 50);	
	glTranslated(wx,wy,r1);
	glRotatef(90, 1.0, 0.0, 0.0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(-2.0*wx,0,0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(0,0,2.0*wy);
	glutSolidTorus(0.05, r1-0.05, 8, 10);
	glTranslated(2.0*wx,0,0);
	glutSolidTorus(0.05, r1-0.05, 8, 10);	
	glPopMatrix();

	// -------------------------------------------- 뒷 박스
	glPushMatrix();
	h3+=0.1;
	glTranslated(-0.15,0, h3);
	dx1 = 0.1;dx2 = dx1;
	dy1 = 0.12; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(100, 100, 100);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glColor3ub(100, 100, 100);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glEnd();
	glPopMatrix();

	// -------------------------------------------- 레이저스캐너 기둥
	glPushMatrix();
	glTranslated(0.1,0.1,h3);
	dx1 = 0.03;
	dy1 = 0.01;
	h1 = 0.2;
	//옆면 4개
	glColor3ub(0, 220, 0);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glBegin(GL_POLYGON);
	glVertex3f(dx1, dy1, h1);
	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glTranslated(0,-0.2,0);
	//옆면 4개
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glVertex3f(dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, 0);

	glVertex3f(dx1, dy1, h1);
	glVertex3f(dx1, dy1, 0);

	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, dy1, 0);

	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(-dx1, -dy1, 0);
	glEnd();
	//윗면
	glBegin(GL_POLYGON);
	glVertex3f(dx1, dy1, h1);
	glVertex3f(-dx1, dy1, h1);
	glVertex3f(-dx1, -dy1, h1);
	glVertex3f(dx1, -dy1, h1);
	glEnd();
	glPopMatrix();

	// -------------------------------------------- 레이저스캐너
	glPushMatrix();
	glTranslated(0.1,0,0.55);
	glRotatef(0, 0.0, 1.0, 0.0);
	glTranslated(-0.1,0,-0.55);
	// 맨 아래
	h3+=0.1;
	glTranslated(0.15,0,h3);
	dx1 = 0.075;dx2 = dx1;
	dx11 = 0.095; dx22 = dx11;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.02;
	//옆면 4개
	glColor3ub(0, 0, 200);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	//윗면
	glColor3ub(0, 0, 200);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx22, 0, h1);
	glEnd();

	// 중간
	glTranslated(0,0,h1);
	dx1 = 0.075;dx2 = dx1;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(0, 0, 200);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);

	glVertex3f(0, dy2, h1);
	glVertex3f(0, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	dx1 = 0.045;dx2 = 0.055;
	dx11 = 0.060; dx22 = 0.075;
	dy1 = 0.055; dy2 = dy1;
	h1 = 0.1;
	//옆면 4개
	glColor3ub(50, 50, 50);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(0, dy2, h1);
	glVertex3f(0, dy2, 0);

	glVertex3f(0, -dy2, h1);
	glVertex3f(0, -dy2, 0);
	glEnd();


	// 윗 부분
	glTranslated(0,0,h1);
	dx1 = 0.075;dx2 = dx1;
	dx11 = 0.095; dx22 = dx11;
	dy1 = 0.075; dy2 = dy1;
	h1 = 0.06;
	//옆면 4개
	glColor3ub(0, 0, 200);
	glBegin(GL_QUAD_STRIP);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx1, -dy2, 0);

	glVertex3f(dx22, 0, h1);
	glVertex3f(dx11, 0, 0);

	glVertex3f(dx2, dy2, h1);
	glVertex3f(dx1, dy2, 0);

	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, dy2, 0);

	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(-dx2, -dy2, 0);
	glEnd();
	//윗면
	glColor3ub(0, 0, 200);
	glBegin(GL_POLYGON);
	glVertex3f(dx2, dy2, h1);
	glVertex3f(-dx2, dy2, h1);
	glVertex3f(-dx2, -dy2, h1);
	glVertex3f(dx2, -dy2, h1);
	glVertex3f(dx22, 0, h1);
	glEnd();

	glPopMatrix();
	//-------------------------------------------------------------------------------------------//
	glPopMatrix();

}


void CKUNSUI3DDlg::renderBuildingMap()
{
	double** dMap = KuDrawingInfo::getInstance()->getBuildingMap(&m_nMapSizeX, &m_nMapSizeY);
	if(NULL==dMap ) return;

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	double dHeightofLaserSensor =KuRobotParameter::getInstance()->getURG04LXLaserHeight()*MM2M;
	double dHeightofGroud = 30*MM2M;

	GLfloat x,y,z;
	float fColor,fColor1;	
	float fCellSize = m_dCellSize;//CELLSIZE;


	double dUnknownPro = 0.5;
	double dCriterionOfHighPro = 0.9;
	double dCriterionOfLowPro = 0.1;

	GLfloat step = 0.5 * fCellSize;


	int nCenterX = (int)( RobotPos.getXm()/(double)fCellSize );
	int nCenterY = (int)( RobotPos.getYm()/(double)fCellSize );

	glPushMatrix();

	for (int i=nCenterX-m_nDrawingSizeHalf; i<nCenterX+m_nDrawingSizeHalf; i++) {
		for (int j=nCenterY-m_nDrawingSizeHalf; j<nCenterY+m_nDrawingSizeHalf; j++) {

			if (i<0 || i>=m_nMapSizeX || j<0 || j>=m_nMapSizeY) continue;

			x = (float)(i) * fCellSize;
			y = (float)(j) * fCellSize;


			if (dMap[i][j]<=dCriterionOfLowPro) { 
				z = dHeightofGroud;
				fColor = 255; fColor1 = 255;
			}
			else if (dMap[i][j]<dUnknownPro) {
				//continue;
				z = dHeightofGroud;
				fColor = 255;
				fColor1 = (float)255.0*(1+(dMap[i][j]-dCriterionOfLowPro)/(dCriterionOfLowPro-0.5));
			}
			else if (dMap[i][j]==dUnknownPro) {
				continue;
			}
			else if (dMap[i][j]>=dCriterionOfHighPro) {
				z = dHeightofLaserSensor;
				fColor = 0.0; 
				fColor1 = 255;
			}
			else if (dMap[i][j]>dUnknownPro) {
				z =dHeightofLaserSensor;
				fColor = 0.0;
				fColor1 = (float)255.0*((dMap[i][j]-0.5)/(dCriterionOfHighPro-0.5)); 
			}			
			else{
				continue;				
			}

			glColor4ub(fColor,fColor,fColor,fColor1);

			// 다각형을 그리는 부분
			// 윗면
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,0.0f,1.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y-step,z);
			glEnd();
			// 옆면들
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,-1.0f,0.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y-step,0);
			glVertex3f(x+step,y-step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(1.0f,0.0f,0.0f);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y-step,0);
			glVertex3f(x+step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,1.0f,0.0f);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y+step,0);
			glVertex3f(x-step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(-1.0f,0.0f,0.0f);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y+step,0);
			glVertex3f(x-step,y-step,0);												
			glEnd();
			//}
		}	
	}

	glPopMatrix();
}


void CKUNSUI3DDlg::renderBuiltMap()
{	
	GLfloat x,y,z;
	GLfloat step;
	int nCenterX, nCenterY;
	float fCellSize = m_dCellSize;//.1;//CELLSIZE;
	int nMapInitValue = KuMap::EMPTY_AREA;
	int nMapOccupiedValue = KuMap::OCCUPIED_AREA;
	int nMapUnknownValue = KuMap::UNKNOWN_AREA;
	int nMapCADValue = KuMap::FIXED_CAD_AREA;
	int nMapLuggageValue = KuMap::LUGGAGE_AREA;
	int nMapDynamicEnvValue = KuMap::DYNAMIC_ENVIRONMENT_AREA;
	int nMapSafeAreaValue = KuMap::SAFE_AREA;
	
	int i, j, k, m;

	step = 0.5 * fCellSize;

	KuPose RobotPos;
	RobotPos = KuDrawingInfo::getInstance()->getRobotPos();

	int**  nMap = KuDrawingInfo::getInstance()->getMap();
	if(NULL==nMap) return;

	m_nMapSizeX= KuDrawingInfo::getInstance()->getMapSizeX();
	m_nMapSizeY =KuDrawingInfo::getInstance()->getMapSizeY();

	bool* pbMapRendered = new bool [m_nMapSizeX * m_nMapSizeY]; // for checking rendered cells
	int nRegionWidth(0), nRegionHeight(0);
	int nEndX(0), nEndY(0);
	int nCurrCellType(0);
	int nExp;
	int nRegion(0);
	int nXmin, nXmax, nYmin, nYmax;
	bool bExpandHorizontal, bExpandVertical;
	const float fLowVal(-0.5 * fCellSize);
	const float fHighVal(0.5 * fCellSize);
	float fCellPosX, fCellPosY;

	// init
	for(i = 0; i < m_nMapSizeX * m_nMapSizeY; i++)
	{
		pbMapRendered[i] = false; // not rendered
	}

	nCenterX = (int)( RobotPos.getXm()/(double)fCellSize );
	nCenterY = (int)( RobotPos.getYm()/(double)fCellSize );

	//glDisable(GL_CULL_FACE);
	glPushMatrix();

	nXmin = nCenterX - m_nDrawingSizeHalf;
	nXmax = nCenterX + m_nDrawingSizeHalf;
	nYmin = nCenterY - m_nDrawingSizeHalf;
	nYmax = nCenterY + m_nDrawingSizeHalf;

	if(nXmin < 0) nXmin = 0;
	if(nXmax > m_nMapSizeX) nXmax = m_nMapSizeX;
	if(nYmin < 0) nYmin = 0;
	if(nYmax > m_nMapSizeY) nYmax = m_nMapSizeY;

	// render
	for(i = nXmin; i < nXmax; i++)
	{
		for(j = nYmin; j < nYmax; j++)
		{
			if(!pbMapRendered[j * m_nMapSizeX + i] && // if this cell is not rendered yet
				(nMap[i][j] == nMapInitValue || nMap[i][j] == nMapOccupiedValue
				|| nMap[i][j] ==nMapCADValue|| nMap[i][j] ==nMapLuggageValue|| nMap[i][j] ==nMapDynamicEnvValue)||nMap[i][j]==nMapSafeAreaValue) // for empty or occupied cells
			{
				// init
				nRegionWidth = 1;
				nRegionHeight = 1;
				nCurrCellType = nMap[i][j];
				bExpandHorizontal = true;
				bExpandVertical = true;
				nExp = 1;

				while(bExpandHorizontal || bExpandVertical)
				{
					// expand: horizontal
					if(bExpandHorizontal)
					{
						if(i + nExp < nCenterX + m_nDrawingSizeHalf)
						{
							for(k = 0; k < nRegionHeight; k++)
							{
								if(nMap[i + nExp][j + k] != nCurrCellType || pbMapRendered[(j + k) * m_nMapSizeX + (i + nExp)])
								{
									bExpandHorizontal = false; // different cell type is detected

									break;
								}
							}

							if(bExpandHorizontal)
							{
								for(k = 0; k < nRegionHeight; k++)
								{
									pbMapRendered[(j + k) * m_nMapSizeX + (i + nExp)] = true;
								}

								nRegionWidth++;
							}
						}
						else
						{
							bExpandHorizontal = false;
						}
					}

					// expand: vertical
					if(bExpandVertical)
					{
						if(j + nExp < nCenterY + m_nDrawingSizeHalf)
						{
							for(k = 0; k < nRegionWidth; k++)
							{
								if(nMap[i + k][j + nExp] != nCurrCellType || pbMapRendered[(j + nExp) * m_nMapSizeX + (i + k)])
								{
									bExpandVertical = false; // different cell type is detected

									break;
								}
							}

							if(bExpandVertical)
							{
								for(k = 0; k < nRegionWidth; k++)
								{
									pbMapRendered[(j + nExp) * m_nMapSizeX + (i + k)] = true;
								}

								nRegionHeight++;
							}
						}
						else
						{
							bExpandVertical = false;
						}
					}

					nExp++;
				}

				nRegion++;

				// draw					
				fCellPosX = (float)i * fCellSize;
				fCellPosY = (float)j * fCellSize;

				if(m_xRotate == 90.)
				{
					z = 0.01;
				}
				else
				{
					z = 0.1;
				}

				glPushMatrix();

				if(nCurrCellType == nMapOccupiedValue) // occupied
				{
					glColor4ub(0, 0, 0, 255);

					// 윗면
					glBegin(GL_POLYGON);
					glNormal3f(0.0f,0.0f,1.0f);
					glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glEnd();

					if(m_xRotate != 90.)
					{
						// 옆면들
						glBegin(GL_POLYGON);
						glNormal3f(-1.0f,0.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,-1.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(1.0f,0.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,1.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
					}
				}
				else if(nCurrCellType == nMapCADValue)
				{
					glColor4ub(255, 0, 255, 255);

					// 윗면
					glBegin(GL_POLYGON);
					glNormal3f(0.0f,0.0f,1.0f);
					glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glEnd();

					if(m_xRotate != 90.)
					{
						// 옆면들
						glBegin(GL_POLYGON);
						glNormal3f(-1.0f,0.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,-1.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(1.0f,0.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,1.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
					}
				}				
				else if(nCurrCellType == nMapDynamicEnvValue)
				{
					glColor4ub(0, 100, 0, 255);

					// 윗면
					glBegin(GL_POLYGON);
					glNormal3f(0.0f,0.0f,1.0f);
					glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glEnd();

					if(m_xRotate != 90.)
					{
						// 옆면들
						glBegin(GL_POLYGON);
						glNormal3f(-1.0f,0.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,-1.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(1.0f,0.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,1.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
					}
				}
				else if(nCurrCellType == nMapSafeAreaValue)
				{
					glColor4ub(255, 0, 0, 255);

					// 윗면
					glBegin(GL_POLYGON);
					glNormal3f(0.0f,0.0f,1.0f);
					glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glEnd();

					if(m_xRotate != 90.)
					{
						// 옆면들
						glBegin(GL_POLYGON);
						glNormal3f(-1.0f,0.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,-1.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(1.0f,0.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,1.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
					}
				}				
				else if(nCurrCellType == nMapLuggageValue)
				{
					glColor4ub(0, 255, 0, 255);

					// 윗면
					glBegin(GL_POLYGON);
					glNormal3f(0.0f,0.0f,1.0f);
					glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
					glEnd();

					if(m_xRotate != 90.)
					{
						// 옆면들
						glBegin(GL_POLYGON);
						glNormal3f(-1.0f,0.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,-1.0f,0.0f);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(1.0f,0.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, z);
						glEnd();
						glBegin(GL_POLYGON);
						glNormal3f(0.0f,1.0f,0.0f);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0);
						glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, z);
						glEnd();
					}
				}
				else if(nCurrCellType == nMapInitValue) // empty
				{
					glColor4ub(255, 255, 255, 255);

					// 윗면
					glBegin(GL_POLYGON);
					glNormal3f(0.0f,0.0f,1.0f);
					glVertex3f(fLowVal + fCellPosX, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0.01);
					glVertex3f(fLowVal + fCellPosX, fLowVal + fCellPosY, 0.01);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fLowVal + fCellPosY, 0.01);
					glVertex3f(fHighVal + fCellPosX + (nRegionWidth - 1) * fCellSize, fHighVal + fCellPosY + (nRegionHeight - 1) * fCellSize, 0.01);
					glEnd();
				}

				glPopMatrix();
			}
		}
	}

	glPopMatrix();
	//glEnable(GL_CULL_FACE);

	delete [] pbMapRendered;
}


void CKUNSUI3DDlg::renderSample()
{

	// MCL의 샘플데이터를 받아서 그린다
	GLfloat x,y,z;
	vector<Sample> vecParticle = KuDrawingInfo::getInstance()->getParticle();
	vector<Sample>::iterator vSample;	


	if(m_dZoom<=0){
		glPointSize((GLfloat)1.5*2.0);
	}
	else{
		glPointSize((GLfloat)m_dZoom*2.0);
	}


	glColor3f(1.0f, 0.0f, 0.0f);
	for (vSample = vecParticle.begin(); vSample != vecParticle.end(); vSample++) {
		x = vSample->x/1000.0;
		y = vSample->y/1000.0;
		z = 1.15;
		glBegin(GL_POINTS);
		glVertex3f(x,y,z);
		glEnd();
	}
}



void CKUNSUI3DDlg::renderGoalPosition()
{
	KuPose GoalPos = KuDrawingInfo::getInstance()->getGoalPos();
	if(GoalPos.getID()==KuDrawingInfo::getInstance()->getCurFloor())
	{
		GLUquadricObj *pObj;
		pObj = gluNewQuadric();
		gluQuadricNormals(pObj,GLU_SMOOTH);

		glPushMatrix();
		glColor4ub(100,0,100,200);
		glTranslated(GoalPos.getXm(), GoalPos.getYm(), 0.17);
		gluDisk(pObj, 0., 0.3, 15, 10);
		glPopMatrix();
	}
}


void CKUNSUI3DDlg::renderObsGoalPosition()
{
	KuPose GoalPos = KuDrawingInfo::getInstance()->getObsGoalPos();
	if(GoalPos.getID()==KuDrawingInfo::getInstance()->getCurFloor())
	{
		GLUquadricObj *pObj;
		pObj = gluNewQuadric();
		gluQuadricNormals(pObj,GLU_SMOOTH);

		glPushMatrix();
		glColor4ub(100,0,100,200);
		glTranslated(GoalPos.getXm(), GoalPos.getYm(), 0.17);
		gluDisk(pObj, 0., 0.3, 15, 10);
		glPopMatrix();
	}
}

void CKUNSUI3DDlg::renderTargetPos()
{
	KuPose GoalPos = KuDrawingInfo::getInstance()->getTargetPos();

	GLUquadricObj *pObj;
	pObj = gluNewQuadric();
	gluQuadricNormals(pObj, GLU_SMOOTH);

	glPushMatrix();
	glColor4ub(0,200,200,200);
	glTranslated(GoalPos.getXm(), GoalPos.getYm(), 0.15+0.05);
	gluDisk(pObj, 0., 0.15, 15, 10);
	glPopMatrix();

}

void CKUNSUI3DDlg::renderGoalList()
{
	list<KuPose> GoalPosList = KuDrawingInfo::getInstance()->getGoalPosList();

	int nGoalNum = 0;
	for(list<KuPose>::iterator iter=GoalPosList.begin();iter !=GoalPosList.end();iter++ ){

		double x = iter->getXm();
		double y = iter->getYm();
		double 	z = 0.17;
		GLUquadricObj *pObj;
		pObj = gluNewQuadric();
		gluQuadricNormals(pObj, GLU_SMOOTH);

		glPushMatrix();
		glColor4ub(0,255,0,255);
		glTranslated(x, y, z);
		gluDisk(pObj, 0., 0.3, 15, 10);


		glColor3f (0, 0, 0);
		glRasterPos3f(0, 0, 0.1);
		char cGoalNum[20];
		memset(cGoalNum,0,sizeof(cGoalNum));
		//	sprintf(cGoalNum,"Goal_%d",nGoalNum);
		for (int i = 0; i < 20; i++) {
			glutBitmapCharacter(GLUT_BITMAP_8_BY_13, cGoalNum[i]);
			//GLUT_BITMAP_8_BY_13
			////GLUT_BITMAP_8_BY_15

		}
		nGoalNum++;
		glPopMatrix();
	}
}



void CKUNSUI3DDlg::renderPath()
{

	list<KuPose> PathList = KuDrawingInfo::getInstance()->getPath();
	int nMapSizeX;
	int nMapSizeY;
	bool bexMapflag=true;
	double** dMap = KuDrawingInfo::getInstance()->getPathMap(&nMapSizeX,&nMapSizeY );
	if(NULL==dMap||nMapSizeX==0||nMapSizeY==0 ) {bexMapflag=false;}

	if(PathList.size()==0){
		return ;
	}

	GLfloat x,y,z;
	list<KuPose>::iterator it;
	
	float tempX, tempY, tempZ;

	glPushMatrix();	
	glLineWidth(2.0);
	glColor3ub(0,0,255);
	int nCurfloor=-1;
	int nNextfloor=-1;


	//Path
	for (it = PathList.begin(); it != PathList.end(); it++) {

		if(it == PathList.begin()){
			tempX = it->getXm();
			tempY = it->getYm();
			if(bexMapflag==false){tempZ = 0.15+0.025;}
			else {tempZ = 0.15+0.025;}//dMap[(int)(x*10+0.5)][(int)(y*10+0.5)]/100.0+0.15+0.025;}
			nCurfloor=it->getID();
			continue;
		}	
	
		x = it->getXm();
		y = it->getYm();
		nNextfloor=it->getID();
		if(bexMapflag==false){z = 0.15+0.025;}
		else {z =0.15+0.025;}////dMap[(int)(x*10+0.5)][(int)(y*10+0.5)]/100.0+0.15+0.025;}
	
		if(nCurfloor==nNextfloor)
		{
				//if(KuDrawingInfo::getInstance()->getCurFloor()==nCurfloor)
				{
					glBegin(GL_LINES);
					glVertex3f(tempX,tempY,tempZ);
					glVertex3f(x,y,z);
					glEnd();
				}		
		}

		tempX = it->getXm();
		tempY = it->getYm();
		nCurfloor=it->getID();


	}
	glLineWidth(1.0);
	glPopMatrix();	


}


void CKUNSUI3DDlg::renderLaserData()
{

	int_1DArray nData;
	double dOffset =KuRobotParameter::getInstance()->getURG04LXLaserXOffset();
	double dSensorHeight = (KuRobotParameter::getInstance()->getURG04LXLaserHeight()+20)*MM2M;

	nData=KuDrawingInfo::getInstance()->getLaserData181();
	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();


	glPushMatrix();	
	glPointSize((GLfloat)m_dZoom*2.0);
	glColor3f(0.0f, 1.0f, 0.0f);	
	glBegin(GL_POINTS);
	for(int i=0;i<181;i++){
		if(nData[i]==-1||nData[i]==0) continue;
		double dAngleRad = (double)(i - 90) * D2R;
		double dX = RobotPos.getX() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());

		double dY = RobotPos.getY() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

		dX = dX*MM2M;
		dY = dY*MM2M;
		glVertex3f(dX, dY, dSensorHeight);
	}
	glEnd();
	glPopMatrix();


}


void CKUNSUI3DDlg::renderRplidarData()
{

	int_1DArray nData;
	double dOffset =KuRobotParameter::getInstance()->getRplidarXOffset();
	double dSensorHeight = (KuRobotParameter::getInstance()->getRplidaHeight()+20)*MM2M;

	nData=KuDrawingInfo::getInstance()->getRplidarData181();
	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();


	glPushMatrix();	
	glPointSize((GLfloat)m_dZoom*2.0);
	glColor3f(0.0f, 0.0f, 1.0f);	
	glBegin(GL_POINTS);
	for(int i=0;i<181;i++){
		if(nData[i]==-1||nData[i]==0) continue;
		double dAngleRad = (double)(i - 90) * D2R;
		double dX = RobotPos.getX() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());

		double dY = RobotPos.getY() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

		dX = dX*MM2M;
		dY = dY*MM2M;
		glVertex3f(dX, dY, dSensorHeight);
	}
	glEnd();
	glPopMatrix();


}

void CKUNSUI3DDlg::renderKinectRangeData()
{

	int_1DArray nData= KuDrawingInfo::getInstance()->getKinectRangeData();

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	double dSensorHeight = (double)KuRobotParameter::getInstance()->getKinectHeight()/1000;
	double dOffset= (double)KuRobotParameter::getInstance()->getKinectXOffset();
	glPushMatrix();	
	glPointSize((GLfloat)m_dZoom*2.5);
	glColor3f(1.0f, 0.0f, 0.0f);	
	glBegin(GL_POINTS);
	for(int i=0;i<Sensor::KINECT_SENSOR_FOV;i++){

		if(nData[i] != 1000000){
			double dAngleRad = (double)(i -  Sensor::KINECT_SENSOR_FOV/2) * D2R;
			double dX = RobotPos.getX() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
				((double)nData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());

			double dY = RobotPos.getY() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
				((double)nData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

			glVertex3f(dX/1000., dY/1000.,dSensorHeight);
		}
	}
	glEnd();
	glPopMatrix();

}

AUX_RGBImageRec* CKUNSUI3DDlg::LoadBMPFile(char *filename)
{
	FILE *hFile = NULL;

	if(!filename) return NULL;

	hFile = fopen(filename, "r");
	wchar_t pwstrDest[100];
	if(hFile) {
		fclose(hFile);
		int nLen = ( int )strlen( filename ) + 1;
		mbstowcs( pwstrDest, filename, nLen );
		return auxDIBImageLoad(pwstrDest);
	}

	return NULL;
}


void CKUNSUI3DDlg::renderWayPointList()
{

	list<KuPose> WayPointList = KuDrawingInfo::getInstance()->getWayPointList();

	if(WayPointList.size()==0){
		return ;
	}

	GLfloat x,y,z;
	list<KuPose>::iterator it;
	z = 0.15+0.05;

	//Path
	for (it = WayPointList.begin(); it != WayPointList.end(); it++) {

		GLUquadricObj *pObj;
		pObj = gluNewQuadric();
		gluQuadricNormals(pObj, GLU_SMOOTH);
		glPushMatrix();
		glColor4ub(0,100,200,200);
		glTranslated(it->getXm(), it->getYm(), z);
		gluDisk(pObj, 0., 0.15, 15, 10);
		glPopMatrix();

	}


}


void CKUNSUI3DDlg::renderCADMap()
{

	int nCADMapSizeX=0, nCADMapSizeY=0;

	int** nMap = KuDrawingInfo::getInstance()->getCADMap(&nCADMapSizeX,&nCADMapSizeY);
	if(NULL==nMap ) return;

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();


	GLfloat x,y,z;
	float fColor,fColor1;	
	float fCellSize = 0.1;//CELLSIZE;
	double dHeight=0.01;
	double dGap=0.1;


	GLfloat step = 0.5 * fCellSize;


	int nCenterX = (int)( RobotPos.getXm()/(double)fCellSize );
	int nCenterY = (int)( RobotPos.getYm()/(double)fCellSize );

	glPushMatrix();

	for (int i=nCenterX-m_nDrawingSizeHalf; i<nCenterX+m_nDrawingSizeHalf; i++) {
		for (int j=nCenterY-m_nDrawingSizeHalf; j<nCenterY+m_nDrawingSizeHalf; j++) {

			if (i<0 || i>=nCADMapSizeX || j<0 || j>=nCADMapSizeY) continue;

			x = (float)(i) * fCellSize;
			y = (float)(j) * fCellSize;

			if(nMap[i][j]== KuMap::FIXED_CAD_AREA){
				z=m_dMapHeight+dHeight;
				glColor4ub(255,0,255,255);
			}	
			else if(nMap[i][j]== KuMap::DYNAMIC_ENVIRONMENT_AREA){
				z=m_dMapHeight+dHeight;
				glColor4ub(0,255,0,50);
			}		
			else{
				continue;				
			}


			// 다각형을 그리는 부분
			// 윗면
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,0.0f,1.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y-step,z);
			glEnd();
			// 옆면들
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,-1.0f,0.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y-step,z-dGap);
			glVertex3f(x+step,y-step,z-dGap);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(1.0f,0.0f,0.0f);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y-step,z-dGap);
			glVertex3f(x+step,y+step,z-dGap);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,1.0f,0.0f);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y+step,z-dGap);
			glVertex3f(x-step,y+step,z-dGap);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(-1.0f,0.0f,0.0f);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y+step,z-dGap);
			glVertex3f(x-step,y-step,z-dGap);												
			glEnd();

		}	
	}

	glPopMatrix();
}


BOOL CKUNSUI3DDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.

	return CDialog::PreTranslateMessage(pMsg);
}



/**
@brief Korean:Xtion 3D정보를 화면에 그려준다. 
@brief English: Draws Xtion 3D information
*/
void CKUNSUI3DDlg::reder3DPointCloud()
{

	GLfloat x,y,z;
	KuPose* pGlobal3DPose = KuDrawingInfo::getInstance()->getKinectGlobal3DPos();
	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();

	KuDrawingInfo::getInstance()->getKinectImageData(m_IplKinectImg);

	if(m_dZoom<=0){
		glPointSize((GLfloat)1.5);
	}
	else{
		glPointSize((GLfloat)1.5);
	}
	
	glColor3f(1.0f, 0.0f, 0.0f);

	int nPixX,nPixY;
	glBegin(GL_POINTS);

	for (int i=0; i< Sensor::IMAGE_HEIGHT;i++) {
		for (int j=0; j< Sensor::IMAGE_WIDTH ;j++) {
			int nIdx = j+i*320;
			if(pGlobal3DPose[nIdx].getID()==1){ //유효한 3차원 좌표들이 들어가있다는 의미

				x = RobotPos.getXm() + cos(RobotPos.getThetaRad())*pGlobal3DPose[nIdx].getXm() -sin(RobotPos.getThetaRad())*pGlobal3DPose[nIdx].getYm();
				y = RobotPos.getYm() + sin(RobotPos.getThetaRad())*pGlobal3DPose[nIdx].getXm() + cos(RobotPos.getThetaRad())*pGlobal3DPose[nIdx].getYm();
				z = pGlobal3DPose[nIdx].getZm();
				nPixX = pGlobal3DPose[nIdx].getPixX();
				nPixY = pGlobal3DPose[nIdx].getPixY();

				glColor3ub(m_IplKinectImg->imageData[(nPixX+ nPixY*320)*3+2],
					m_IplKinectImg->imageData[(nPixX+ nPixY*320)*3+1],
					m_IplKinectImg->imageData[(nPixX+ nPixY*320)*3]
				);
				glVertex3f(x,y,z);

			}
		}
	}
	glEnd();

}

void CKUNSUI3DDlg::rendervecObsLocalPath()
{

	vector<list<KuPose>> vecPathList = KuDrawingInfo::getInstance()->getvecObsLocalPath();


	if(vecPathList.size()==0){
		return ;
	}

	GLfloat x,y,z;
	list<KuPose>::iterator it;



	float tempX, tempY;

	glPushMatrix();	
	glLineWidth(2.0);
	glColor3ub(100,255,50);

	//Path
	for (int i=0;i<vecPathList.size();i++)
	{
		for (it = vecPathList[i].begin(); it != vecPathList[i].end(); it++) {

			if(it == vecPathList[i].begin()){
				tempX = it->getXm();
				tempY = it->getYm();
				continue;
			}	
			glBegin(GL_LINES);
			x = tempX; 
			y = tempY;
			z = 0.15+0.025;
			glVertex3f(x,y,z);
			x = it->getXm();
			y = it->getYm();
			z = 0.15+0.025;
			glVertex3f(x,y,z);
			glEnd();

			tempX = it->getXm();
			tempY = it->getYm();


		}
	}
	
	glLineWidth(1.0);
	glPopMatrix();	


}


void CKUNSUI3DDlg::renderObsLocalPath()
{

	list<KuPose> PathList = KuDrawingInfo::getInstance()->getObsLocalPath();


	if(PathList.size()==0){
		return ;
	}

	GLfloat x,y,z;
	list<KuPose>::iterator it;



	float tempX, tempY;

	glPushMatrix();	
	glLineWidth(2.0);
	glColor3ub(100,255,50);

	//Path
	for (it = PathList.begin(); it != PathList.end(); it++) {

		if(it == PathList.begin()){
			tempX = it->getXm();
			tempY = it->getYm();
			continue;
		}	
		glBegin(GL_LINES);
		x = tempX; 
		y = tempY;
		z = 0.15+0.025;
		glVertex3f(x,y,z);
		x = it->getXm();
		y = it->getYm();
		z = 0.15+0.025;
		glVertex3f(x,y,z);
		glEnd();

		tempX = it->getXm();
		tempY = it->getYm();


	}
	glLineWidth(1.0);
	glPopMatrix();	


}
void CKUNSUI3DDlg::renderLocalPath()
{

	list<KuPose> PathList = KuDrawingInfo::getInstance()->getLocalPath();


	if(PathList.size()==0){
		return ;
	}

	GLfloat x,y,z;
	list<KuPose>::iterator it;



	float tempX, tempY;

	glPushMatrix();	
	glLineWidth(2.0);
	glColor3ub(255,0,0);

	//Path
	for (it = PathList.begin(); it != PathList.end(); it++) {

		if(it == PathList.begin()){
			tempX = it->getXm();
			tempY = it->getYm();
			continue;
		}	
		glBegin(GL_LINES);
		x = tempX; 
		y = tempY;
		z = 0.15+0.025;
		glVertex3f(x,y,z);
		x = it->getXm();
		y = it->getYm();
		z = 0.15+0.025;
		glVertex3f(x,y,z);
		glEnd();

		tempX = it->getXm();
		tempY = it->getYm();


	}
	glLineWidth(1.0);
	glPopMatrix();	


}
#define GL_PI 3.1415f

void CKUNSUI3DDlg::createCylinder(GLfloat centerx, GLfloat centery, GLfloat centerz, GLfloat radius, GLfloat h)
{
    /* function createCyliner()
    원기둥의 중심 x,y,z좌표, 반지름, 높이를 받아 원기둥을 생성하는 함수(+z방향으로 원에서 늘어남)
    centerx : 원기둥 원의 중심 x좌표
    centery : 원기둥 원의 중심 y좌표
    centerz : 원기둥 원의 중심 z좌표
    radius : 원기둥의 반지름
    h : 원기둥의 높이
    */
    GLfloat x, y, angle;
 
    glBegin(GL_TRIANGLE_FAN);           //원기둥의 윗면
    glNormal3f(0.0f, 0.0f, -1.0f);
//    glColor3ub(139, 69, 19);
    glVertex3f(centerx, centery, centerz);
 
    for(angle = 0.0f; angle <= (2.0f*GL_PI); angle += (GL_PI/8.0f))
    {
        x = centerx + radius*sin(angle);
        y = centery + radius*cos(angle);
        glNormal3f(0.0f, 0.0f, -1.0f);
        glVertex3f(x, y, centerz);
    }
    glEnd();
 
    glBegin(GL_QUAD_STRIP);            //원기둥의 옆면
    for(angle = 0.0f; angle <= (2.0f*GL_PI); angle += (GL_PI/8.0f))
    {
        x = centerx + radius*sin(angle);
        y = centery + radius*cos(angle);
        glNormal3f(sin(angle), cos(angle), 0.0f);
        glVertex3f(x, y, centerz);
        glVertex3f(x, y, centerz + h);
    }
    glEnd();
 
    glBegin(GL_TRIANGLE_FAN);           //원기둥의 밑면
    glNormal3f(0.0f, 0.0f, 1.0f);
    glVertex3f(centerx, centery, centerz + h);
    for(angle = (2.0f*GL_PI); angle >= 0.0f; angle -= (GL_PI/8.0f))
    {
        x = centerx + radius*sin(angle);
        y = centery + radius*cos(angle);
        glNormal3f(0.0f, 0.0f, 1.0f);
        glVertex3f(x, y, centerz + h);
    }
    glEnd();
}

void CKUNSUI3DDlg::DrawSphere(GLfloat radius, GLfloat h)
{
	GLfloat x, y, z, delta = 0.1;
	GLfloat size=radius;

	//glColor3f(red, green, blue);
	glColor3f(0.f, 0.f, 0.f);
	glBegin(GL_TRIANGLE_STRIP);
	for( GLfloat phi = 0; phi < 1.6; phi += 0.1 ){
// 		if(phi < 1.6)glColor3f(0.f, 0.f, 0.f);
// 		else if(phi >= 1.6)glColor3f(1.f, 0.75f, 0.62f);
		for( GLfloat theta = 0; theta <= 6.3 ; theta += 0.1 ){
			x = size*sin(phi) * cos(theta);
			y = size*sin(phi) * sin(theta);
			z = size*cos(phi);
			glVertex3f( x, y, z+h);
			phi += 0.1;
			x = size*sin(phi) * cos(theta);
			y = size*sin(phi) * sin(theta);
			z = size*cos(phi);
			glVertex3f( x, y, z+h);
			phi -= 0.1;
		}
	}
	glEnd();
	
	glColor3f(1.f, 0.54f, 0.45f);
	glBegin(GL_TRIANGLE_STRIP);
	for( GLfloat phi =1.6; phi < 3.2; phi += 0.1 ){
// 		if(phi < 1.6)glColor3f(0.f, 0.f, 0.f);
// 		else if(phi >= 1.6)glColor3f(1.f, 0.75f, 0.62f);
		for( GLfloat theta = 0; theta <= 6.3 ; theta += 0.1 ){
			x = size*sin(phi) * cos(theta);
			y = size*sin(phi) * sin(theta);
			z = size*cos(phi);
			glVertex3f( x, y, z+h);
			phi += 0.1;
			x = size*sin(phi) * cos(theta);
			y = size*sin(phi) * sin(theta);
			z = size*cos(phi);
			glVertex3f( x, y, z+h);
			phi -= 0.1;
		}
	}
	glEnd();
}
void CKUNSUI3DDlg::renderVirtualObstacle()
{
	vector<KuPose>::iterator it;
	vector<KuPose> vecVirtualRobotPos;
	KuDrawingInfo::getInstance()->getMultiObstaclePosVector(&vecVirtualRobotPos);

	int** nMap = KuDrawingInfo::getInstance()->getMap();
	if(nMap == NULL) return;

	m_nMapSizeX= KuDrawingInfo::getInstance()->getMapSizeX();
	m_nMapSizeY= KuDrawingInfo::getInstance()->getMapSizeY();


	for(it = vecVirtualRobotPos.begin(); it!= vecVirtualRobotPos.end(); it++){

		glPushMatrix();		
		KuPose RobotPos;
		RobotPos = *it; 

		glTranslated(RobotPos.getXm(), RobotPos.getYm(), 0);
		glRotatef(RobotPos.getThetaRad(), 0.0, 0.0, 1.0);


		double h1 = 0.07;
		double h2 = 0.25;
		double h3 = 0.27;
		double dx1 = 0.2;
		double dy1 = 0.12;
		double dx11 = 0.24;
		double dx2 = 0.23;
		double dy2 = 0.15;
		double dx22 = 0.27;
		double r1 = 0.1;
		double r2 = 0.05;
		double wx = 0.13;
		double wy = 0.17;
		double dPillarHeight=0.3;

		if(RobotPos.getID()>=KuMap::OCCUPIED_OBSTACLES_GRID)
		{
			glColor3f (255, 0, 0);
			glRasterPos3f(0, 0, 0.1);

			int nLandmarkID=RobotPos.getID()-KuMap::OCCUPIED_OBSTACLES_GRID;
			char cLandmarkID[20];
			memset(cLandmarkID,0,sizeof(cLandmarkID));
			sprintf(cLandmarkID,"ID:%d",nLandmarkID);

			glPushMatrix();
			int ngroupID=RobotPos.getZm();

			//머리
			DrawSphere(0.2, m_dMapHeight+1.0);

			float fR=bcolor[ngroupID%54](0);
			float fB=bcolor[ngroupID%54](1);
			float fG=bcolor[ngroupID%54](2);

			//몸통
			glColor3ub(fR, fB, fG);
			createCylinder(0.0, 0.0, m_dMapHeight+0.4, 0.2, 0.4);
			//다리			
			glColor3ub(39, 59, 119);
			createCylinder(0.1*cos(RobotPos.getThetaRad()+GL_PI/2.0), 0.1*sin(RobotPos.getThetaRad()+GL_PI/2.0), m_dMapHeight+0.1, 0.05, 0.3);
			createCylinder(-0.1*cos(RobotPos.getThetaRad()+GL_PI/2.0), -0.1*sin(RobotPos.getThetaRad()+GL_PI/2.0), m_dMapHeight+0.1, 0.05, 0.3);
			//팔
			glColor3ub(29, 255, 2);
			createCylinder(0.22*cos(RobotPos.getThetaRad()+GL_PI/2.0), 0.22*sin(RobotPos.getThetaRad()+GL_PI/2.0), m_dMapHeight+0.5, 0.03, 0.3);
			createCylinder(-0.22*cos(RobotPos.getThetaRad()+GL_PI/2.0), -0.22*sin(RobotPos.getThetaRad()+GL_PI/2.0), m_dMapHeight+0.5, 0.03, 0.3);

			glPopMatrix();
		}
				

		glPopMatrix();
	}	
	//-------------------------------------------------------------------------------------------//


}

void CKUNSUI3DDlg::renderVirtualGoal()
{
	vector<KuPose>::iterator it;
	vector<KuPose> vecVirtualRobotPos;
	KuDrawingInfo::getInstance()->getMultiObstacleGoalPosVector(&vecVirtualRobotPos);

	int** nMap = KuDrawingInfo::getInstance()->getMap();
	if(nMap == NULL) return;

	m_nMapSizeX= KuDrawingInfo::getInstance()->getMapSizeX();
	m_nMapSizeY= KuDrawingInfo::getInstance()->getMapSizeY();


	for(it = vecVirtualRobotPos.begin(); it!= vecVirtualRobotPos.end(); it++){

		glPushMatrix();		
		KuPose RobotPos;
		RobotPos = *it; 

		glTranslated(RobotPos.getXm(), RobotPos.getYm(), 1.0);
		glRotatef(RobotPos.getThetaRad(), 0.0, 0.0, 1.0);



		if(RobotPos.getID()>=KuMap::OCCUPIED_OBSTACLES_GRID)
		{
			glColor3f (255, 0, 0);
			glRasterPos3f(0, 0, 0.1);

			int nLandmarkID=RobotPos.getID()-KuMap::OCCUPIED_OBSTACLES_GRID;
			char cLandmarkID[20];
			memset(cLandmarkID,0,sizeof(cLandmarkID));
			sprintf(cLandmarkID,"ID:%d",nLandmarkID);

			glPushMatrix();
			int ngroupID=RobotPos.getZm();

			float fR=bcolor[ngroupID%54](0);
			float fB=bcolor[ngroupID%54](1);
			float fG=bcolor[ngroupID%54](2);

			glColor4ub(fR,fB,fG,200);

			GLUquadricObj *pObj;
			pObj = gluNewQuadric();

			gluSphere(pObj, 0.2, 60, 30);
			glColor4ub(255, 0, 0, 250);  

			glEnd();
			glPopMatrix();

		}


		glPopMatrix();
	}	
	//-------------------------------------------------------------------------------------------//


}

void CKUNSUI3DDlg::renderTrackedObstID()
{
	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	vector<KuPose> vecTrackedObstID;
	KuDrawingInfo::getInstance()->getTrackedObstacle(&vecTrackedObstID);

	int** nMap = KuDrawingInfo::getInstance()->getMap();
	if(nMap == NULL) return;

	m_nMapSizeX= KuDrawingInfo::getInstance()->getMapSizeX();
	m_nMapSizeY= KuDrawingInfo::getInstance()->getMapSizeY();


	glPushMatrix();	
	 
	for(int i = 0; i< vecTrackedObstID.size(); i++)
	{

		char cLandmarkID[20];

		glColor3f (0, 0, 255);
		glRasterPos3f(vecTrackedObstID[i].getXm(),vecTrackedObstID[i].getYm() , 1);	


		memset(cLandmarkID,0,sizeof(cLandmarkID));
		sprintf(cLandmarkID,"%d",vecTrackedObstID[i].getObsID());

		/*
		double m=RobotPos.getThetaRad()-1;
		double sensor=
	
		if (m_math.calcDistBetweenPosesM(RobotPos, vecTrackedObstID[i]) < 10)
		{

		}
		*/

		for (int j = 0; j< vecTrackedObstID.size(); j++)
		{
			glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, cLandmarkID[j]);
		}
	}

	for (int i=0; i < vecTrackedObstID.size(); i++)
	{
		if (vecTrackedObstID[i].getGroup() > -1)
		{
			char cLandmarkID[20];

			glColor3f (255, 0, 0);
			glRasterPos3f(vecTrackedObstID[i].getXm(),vecTrackedObstID[i].getYm() , 2);	


			memset(cLandmarkID,0,sizeof(cLandmarkID));
			sprintf(cLandmarkID,"%d",vecTrackedObstID[i].getGroup());

			for (int j = 0; j< vecTrackedObstID.size(); j++)
			{
				glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, cLandmarkID[j]);
			}
		}
	}


			
		
	glPopMatrix();
	
	
	//-------------------------------------------------------------------------------------------//
}


void CKUNSUI3DDlg::renderProOBBuildingMap()
{
	if(KuDrawingInfo::getInstance()->getCurFloor()!=1) return;
	int nMapSizeX;
	int nMapSizeY;

	double** dMap = KuDrawingInfo::getInstance()->getProObBuildingMap(&nMapSizeX, &nMapSizeY);
	if(NULL==dMap ) return;

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	double dHeightofLaserSensor =KuRobotParameter::getInstance()->getURG04LXLaserHeight()*MM2M;
	double dHeightofGroud = 30*MM2M;

	GLfloat x,y,z;
	float fColor,fColor1;	
	float fCellSize = 0.1;


	double dUnknownPro = 0.5;
	double dCriterionOfHighPro = 0.9;
	double dCriterionOfLowPro = 0.1;

	GLfloat step = 0.5 * fCellSize*10;// * fCellSize;
	double dCell =Sensor::CELLSIZE/10.0;
	nMapSizeX = nMapSizeX * 1/dCell; //m -> mm 변환하고 mm-> 한격자가 100mm인 배열 형태로 변환
	nMapSizeY = nMapSizeY * 1/dCell; //m -> mm 변환하고 mm-> 한격자가 100mm인 배열 형태로 변환


	int nCenterX = (int)( RobotPos.getXm()/(double)(dCell/10));
	int nCenterY = (int)( RobotPos.getYm()/(double)(dCell/10));
	int nDrawingSizeHalf=m_nDrawingSizeHalf/10;
	glPushMatrix();

	for (int i=nCenterX-nDrawingSizeHalf; i<nCenterX+nDrawingSizeHalf; i++) {
		for (int j=nCenterY-nDrawingSizeHalf; j<nCenterY+nDrawingSizeHalf; j++) {

			if (i<0 || i>=nMapSizeX || j<0 || j>=nMapSizeY) continue;

			x = (float)(i) * fCellSize*10;
			y = (float)(j) * fCellSize*10;
			int nx=(int)i;
			int ny=(int)j;

			if(dMap[nx][ny]<0.0) continue;
			if (dMap[nx][ny]<=dCriterionOfLowPro) { 
				continue;
// 				z = dHeightofGroud;
// 				fColor = 0; fColor1 = 0;

			}
			else if (dMap[nx][ny]<dUnknownPro) {
				continue;
// 				z = dHeightofGroud;
// 				//fColor = 100;
// 				fColor1 = (float)255.0*(1+(dMap[nx][ny]-dCriterionOfLowPro)/(dCriterionOfLowPro-0.5));
			}
			else if (dMap[nx][ny]==dUnknownPro) {
				continue;
			}
			else if (dMap[nx][ny]>=dCriterionOfHighPro) {
				z = dHeightofLaserSensor;
				fColor = 255;//0.00001; 
				fColor1 = 255;
			}
			else if (dMap[nx][j]>dUnknownPro) {
				z =dHeightofLaserSensor;
				fColor = (float)255.0*((dMap[nx][ny]-0.5)/(dCriterionOfHighPro-0.5)); 
				fColor1 = (float)255.0*((dMap[nx][ny]-0.5)/(dCriterionOfHighPro-0.5)); 
			}			
			else{
				continue;				
			}

			glColor4ub(fColor,1/fColor,1/fColor,fColor1);

			// 다각형을 그리는 부분
			// 윗면
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,0.0f,1.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y-step,z);
			glEnd();
			// 옆면들
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,-1.0f,0.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y-step,0);
			glVertex3f(x+step,y-step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(1.0f,0.0f,0.0f);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y-step,0);
			glVertex3f(x+step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,1.0f,0.0f);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y+step,0);
			glVertex3f(x-step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(-1.0f,0.0f,0.0f);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y+step,0);
			glVertex3f(x-step,y-step,0);												
			glEnd();
			//}
		}	
	}

	glPopMatrix();
}


void CKUNSUI3DDlg::renderObstacleAreas()
{

	vector<KuPose> vecObstacleAreas = KuDrawingInfo::getInstance()->getObstacleAreas();

	for(int i=0; i<vecObstacleAreas.size();i++)
	{
	GLUquadricObj *pObj;
	pObj = gluNewQuadric();
	gluQuadricNormals(pObj, GLU_SMOOTH);

	glPushMatrix();
	glColor4ub(2000,0,0,20);
	glTranslated(vecObstacleAreas[i].getXm(), vecObstacleAreas[i].getYm(), 0.15+0.05);
	gluDisk(pObj, 0., 2, 15, 10);
	glPopMatrix();

	glPushMatrix();
	glColor4ub(0,200,200,30);
	glTranslated(vecObstacleAreas[i].getXm(), vecObstacleAreas[i].getYm(), 0.15+0.05);
	gluDisk(pObj, 0., 3, 15, 10);
	glPopMatrix();
	}
}


void CKUNSUI3DDlg::renderOBBuildingMap()
{

	int nMapSizeX;
	int nMapSizeY;

	double** dMap = KuDrawingInfo::getInstance()->getPathMap(&nMapSizeX,&nMapSizeY );
	if(NULL==dMap||nMapSizeX==0||nMapSizeY==0 ) return;

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	double dHeightofLaserSensor =KuRobotParameter::getInstance()->getURG04LXLaserHeight()*MM2M;
	double dHeightofGroud = 30*MM2M;

	GLfloat x,y,z;
	float fColor,fColor1;	
	float fCellSize = 0.1;//CELLSIZE;


	double dUnknownPro = 0.5;
	double dCriterionOfHighPro = 0.9;
	double dCriterionOfLowPro = 0.1;

	GLfloat step = 0.5 * fCellSize;


	int nCenterX = (int)( RobotPos.getXm()/(double)fCellSize );
	int nCenterY = (int)( RobotPos.getYm()/(double)fCellSize );

	glPushMatrix();

	for (int i=nCenterX-m_nDrawingSizeHalf; i<nCenterX+m_nDrawingSizeHalf; i++) {
		for (int j=nCenterY-m_nDrawingSizeHalf; j<nCenterY+m_nDrawingSizeHalf; j++) {

			if (i<0 || i>=m_nMapSizeX || j<0 || j>=m_nMapSizeY) continue;

			x = (float)(i) * fCellSize;
			y = (float)(j) * fCellSize;


			if (dMap[i][j]==KuMap::EMPTY_AREA) { 
				continue;
				z =dMap[i][j]/100.0;//dHeightofGroud;
				fColor = 255; fColor1 = 255;
				glColor4ub(fColor,fColor,fColor,fColor1);
			}
			else if (dMap[i][j]==100000||dMap[i][j]>5000){//KuMap::OCCUPIED_AREA) {	
				continue;
				z =dMap[i][j]/100.0;// dHeightofGroud;
				fColor = 0; fColor1 = 255;
				glColor4ub(fColor,fColor,fColor,fColor1);

			}
			else if (dMap[i][j]==KuMap::UNKNOWN_AREA) {	
			continue;
			}
			else if (dMap[i][j]<=dCriterionOfLowPro) { 
				continue;
				z = dMap[i][j]/100.0;//dHeightofGroud;
				fColor = 255; fColor1 = 255;
				glColor4ub(fColor,1/fColor,1/fColor,fColor1);

			}
			else if (dMap[i][j]<=dUnknownPro) {
				continue;
			}
			else if (dMap[i][j]>=80) {
				z = dMap[i][j]/100.0;///500000000.0;//dHeightofLaserSensor;
				fColor = 255; 
				fColor1 = 255;
				glColor4ub(fColor,0,0,fColor1);

			}
			else if (dMap[i][j]>=dCriterionOfHighPro) {
				z = dMap[i][j]/100.0;///500000000.0;//dHeightofLaserSensor;
				fColor = 255; 
				fColor1 = 255;
				glColor4ub(1/fColor,fColor,1/fColor,fColor1);

			}
			else if (dMap[i][j]>dUnknownPro) {
				continue;
				z =dMap[i][j]/100.0;//dHeightofLaserSensor;
				fColor = 255;
				fColor1 = (float)255.0*((dMap[i][j]-0.5)/(dCriterionOfHighPro-0.5)); 
				glColor4ub(fColor,1/fColor,1/fColor,fColor1);
			}			
			else{
				continue;				
			}

			

			// 다각형을 그리는 부분
			// 윗면
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,0.0f,1.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y-step,z);
			glEnd();
			// 옆면들
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,-1.0f,0.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y-step,0);
			glVertex3f(x+step,y-step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(1.0f,0.0f,0.0f);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y-step,0);
			glVertex3f(x+step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,1.0f,0.0f);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y+step,0);
			glVertex3f(x-step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(-1.0f,0.0f,0.0f);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y+step,0);
			glVertex3f(x-step,y-step,0);												
			glEnd();
			//}
		}	
	}

	glPopMatrix();
}

void CKUNSUI3DDlg::renderPOIMap()
{
	POIMapPara PIOMapData=KuDrawingInfo::getInstance()->getPIOMapData();

	GLUquadricObj *pObj;
	pObj = gluNewQuadric();
	gluQuadricNormals(pObj, GLU_SMOOTH);

	int nLandmarkID = -1;
	float fLandmarkX=0.0;
	float fLandmarkY=0.0;
	float fLandmarkTh=0.0;
	if(PIOMapData.Floor==-1) return;

	for(int i=0; i<PIOMapData.POIIDNum; i++){
		glPushMatrix();
		nLandmarkID = i;
		fLandmarkX =  PIOMapData.ROBX[i];
		fLandmarkY = PIOMapData.ROBY[i];
		fLandmarkTh = PIOMapData.ROBTh[i];
		
		// Center point
		glColor4ub(0, 0, 255, 150);
		glTranslatef(fLandmarkX, fLandmarkY,0.1);

		gluSphere(pObj, 0.2, 40, 30);
		glColor4ub(255, 0, 0, 100); //색깔지정 

		glDisable(GL_CULL_FACE); //원판이 아래서도 보이게 하는 명령어
		gluDisk(pObj,0,0.3,40,40); //원판 그리는명령어
		glEnable(GL_CULL_FACE);	

		glColor3f (0.0, 0.0, 0.0);
		glRasterPos3f(0, 0, 0.1);

		char cLandmarkID[50];
		memset(cLandmarkID,0,sizeof(cLandmarkID));
		string strName=PIOMapData.POIName[i];
		int len = (int)strlen(strName.c_str());

		for (int i = 0; i < len; i++) {
			glutBitmapCharacter(GLUT_BITMAP_8_BY_13, strName.c_str()[i]);
		}
		glPopMatrix();
	}

	gluDeleteQuadric(pObj);


}


void CKUNSUI3DDlg::renderLocalBuildingMap()
{

	int nMapSizeX;
	int nMapSizeY;

	int** nMap = KuDrawingInfo::getInstance()->getLocalMap(&nMapSizeX,&nMapSizeY );
	if(NULL==nMap||nMapSizeX==0||nMapSizeY==0 ) return;

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	double dHeightofLaserSensor =KuRobotParameter::getInstance()->getURG04LXLaserHeight()*MM2M;
	double dHeightofGroud = 30*MM2M;

	GLfloat x,y,z;
	float fColor,fColor1;	
	float fCellSize = m_dCellSize;//CELLSIZE;


	double dUnknownPro = 0.5;
	double dCriterionOfHighPro = 0.9;
	double dCriterionOfLowPro = 0.1;

	GLfloat step = 0.5 * fCellSize;


	int nCenterX = (int)( RobotPos.getXm()/(double)fCellSize );
	int nCenterY = (int)( RobotPos.getYm()/(double)fCellSize );

	glPushMatrix();

	for (int i=0; i<nMapSizeX; i++) {
		for (int j=0; j<nMapSizeY; j++) {

			if (i<0 || i>=nMapSizeX || j<0 || j>=nMapSizeY) continue;

			x = (float)(i+nCenterX-nMapSizeX/2.0) * fCellSize;
			y = (float)(j+nCenterY-nMapSizeY/2.0) * fCellSize;


			if (nMap[i][j]==KuMap::EMPTY_AREA) { 
				//continue;
				z =dHeightofGroud;
				fColor = 255; fColor1 = 255;
				glColor4ub(0,fColor,fColor,fColor1);
			}
			else if (nMap[i][j]==KuMap::OCCUPIED_AREA) {	
				//continue;
				z =dHeightofGroud;
				fColor = 0; fColor1 = 255;
				glColor4ub(fColor,0,fColor,fColor1);

			}
			else if (nMap[i][j]==KuMap::UNKNOWN_AREA) {	
				continue;
			}			
			else{
				continue;				
			}



			// 다각형을 그리는 부분
			// 윗면
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,0.0f,1.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y-step,z);
			glEnd();
			// 옆면들
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,-1.0f,0.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y-step,0);
			glVertex3f(x+step,y-step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(1.0f,0.0f,0.0f);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y-step,0);
			glVertex3f(x+step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,1.0f,0.0f);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y+step,0);
			glVertex3f(x-step,y+step,0);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(-1.0f,0.0f,0.0f);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y+step,0);
			glVertex3f(x-step,y-step,0);												
			glEnd();
			//}
		}	
	}

	glPopMatrix();
}



void CKUNSUI3DDlg::renderRangeData()
{

	int_1DArray nData;
	double dOffset =0.0;
	double dSensorHeight = (KuRobotParameter::getInstance()->getURG04LXLaserHeight()+20)*MM2M;

	nData=KuDrawingInfo::getInstance()->getRangeData();
	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	
	glPushMatrix();	
	glPointSize((GLfloat)m_dZoom*2.0);
	glColor3f(0.0f, 1.0f, 0.0f);	
	glBegin(GL_POINTS);
	for(int i=0;i<361;i++){
		if(nData[i]==-1||nData[i]==0) continue;
		double dAngleRad = (double)(i-90) * D2R;
		double dX = RobotPos.getX() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * cos(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * sin(-RobotPos.getThetaRad());

		double dY = RobotPos.getY() + ((double)nData[i] * cos(dAngleRad) + dOffset ) * sin(RobotPos.getThetaRad()) + 
			((double)nData[i] * sin(dAngleRad) ) * cos(RobotPos.getThetaRad());

		dX = dX*MM2M;
		dY = dY*MM2M;
		glVertex3f(dX, dY, dSensorHeight);
	}
	glEnd();
	glPopMatrix();


}

void CKUNSUI3DDlg::renderpredictedProMap()
{

	int nMapSizeX, nMapSizeY;

	double** dMap = KuDrawingInfo::getInstance()->getpredictedProMap(&nMapSizeX, &nMapSizeY);
	if(NULL==dMap ) return;

	KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
	double dHeightofLaserSensor =0.4;
	double dHeightofGroud = 30*MM2M;

	GLfloat x,y,z;
	float fColor,fColor1,fColor2;	
	float fCellSize = m_dCellSize;//CELLSIZE;


	double dUnknownPro = 0.5;
	double dCriterionOfHighPro = 0.9;
	double dCriterionOfLowPro = 0.1;

	GLfloat step = 0.5 * fCellSize;


	int nCenterX = (int)( RobotPos.getXm()/(double)fCellSize );
	int nCenterY = (int)( RobotPos.getYm()/(double)fCellSize );

	glPushMatrix();

	for (int i=nCenterX-m_nDrawingSizeHalf; i<nCenterX+m_nDrawingSizeHalf; i++) {
		for (int j=nCenterY-m_nDrawingSizeHalf; j<nCenterY+m_nDrawingSizeHalf; j++) {

			if (i<0 || i>=m_nMapSizeX || j<0 || j>=m_nMapSizeY) continue;

			x = (float)(i) * fCellSize;
			y = (float)(j) * fCellSize;


			if (dMap[i][j]<=dCriterionOfLowPro) { 
				continue;
			}			
			else if (dMap[i][j]>=dCriterionOfHighPro) {
				z = dHeightofLaserSensor+dMap[i][j]*10;
				fColor = 255; 
				fColor1 = 255;
				fColor2=0;
			}	
			else{
				
				fColor = (float)255.0*((dMap[i][j])/(dCriterionOfHighPro)); 
				fColor2 =(float)255.0*(1.0-(dMap[i][j])/(dCriterionOfHighPro)); 
				fColor1 = (float)255.0*((dMap[i][j])/(dCriterionOfHighPro)); 
				z=dHeightofLaserSensor+dMap[i][j]*10;
			}

			glColor4ub(fColor,fColor2,fColor2,fColor1);

			// 다각형을 그리는 부분
			// 윗면
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,0.0f,1.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y-step,z);
			glEnd();
			// 옆면들
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,-1.0f,0.0f);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y-step,dHeightofLaserSensor);
			glVertex3f(x+step,y-step,dHeightofLaserSensor);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(1.0f,0.0f,0.0f);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y-step,z);
			glVertex3f(x+step,y-step,dHeightofLaserSensor);
			glVertex3f(x+step,y+step,dHeightofLaserSensor);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(0.0f,1.0f,0.0f);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x+step,y+step,z);
			glVertex3f(x+step,y+step,dHeightofLaserSensor);
			glVertex3f(x-step,y+step,dHeightofLaserSensor);												
			glEnd();
			glBegin(GL_POLYGON);
			glNormal3f(-1.0f,0.0f,0.0f);
			glVertex3f(x-step,y-step,z);
			glVertex3f(x-step,y+step,z);
			glVertex3f(x-step,y+step,dHeightofLaserSensor);
			glVertex3f(x-step,y-step,dHeightofLaserSensor);												
			glEnd();
			//}
		}	
	}

	glPopMatrix();
}

void CKUNSUI3DDlg::renderpredictedTrajectory()
{
	vector<KuTrajectory> vecPredictedTraj;
	KuDrawingInfo::getInstance()->getPredictedTraj(&vecPredictedTraj);

	for(int i=0; i<vecPredictedTraj.size();i++)
	{
		for (int j=0; j<vecPredictedTraj[i].PredictedPt.size(); j++)
		//for (int j=0; j<vecPredictedTraj[i].Predicted_x.size(); j++)
		{
			GLUquadricObj *pObj;
			pObj = gluNewQuadric();
			gluQuadricNormals(pObj, GLU_SMOOTH);

			glPushMatrix();
			glColor4ub(0,200,200,200);
			glTranslated(vecPredictedTraj[i].PredictedPt[j].x, vecPredictedTraj[i].PredictedPt[j].y, 0.15+0.05);
			//glTranslated(vecPredictedTraj[i].Predicted_x[j], vecPredictedTraj[i].Predicted_y[j], 0.15+0.05);
			gluDisk(pObj, 0., 0.15, 15, 10);
			glPopMatrix();
		}
	}




	
	vecPredictedTraj.clear();
	
	//-------------------------------------------------------------------------------------------//
}
void CKUNSUI3DDlg::renderpredictedRobotTrajectory()
{
	vector<KuPose> vecPredictedTraj;
	KuDrawingInfo::getInstance()->getPredictedRTraj(&vecPredictedTraj);

	for(int i=0; i<vecPredictedTraj.size();i++)
	{

		GLUquadricObj *pObj;
		pObj = gluNewQuadric();
		gluQuadricNormals(pObj, GLU_SMOOTH);

		glPushMatrix();
		glColor4ub(200,0,200,200);
		glTranslated(vecPredictedTraj[i].getXm(), vecPredictedTraj[i].getYm(), 0.15+0.05);
		//glTranslated(vecPredictedTraj[i].Predicted_x[j], vecPredictedTraj[i].Predicted_y[j], 0.15+0.05);
		gluDisk(pObj, 0., 0.15, 15, 10);
		glPopMatrix();

	}

	vecPredictedTraj.clear();

	//-------------------------------------------------------------------------------------------//
}


void CKUNSUI3DDlg::renderCubicPath()
{
	vector<KuTrajectory> vecCubicPath;
	KuDrawingInfo::getInstance()->getCubicPath(&vecCubicPath);

	for(int i=0; i<vecCubicPath.size();i++)
	{
		for (int j=0; j<vecCubicPath[i].Tra_x.size(); j++)
		//for (int j=0; j<vecPredictedTraj[i].Predicted_x.size(); j++)
		{
			GLUquadricObj *pObj;
			pObj = gluNewQuadric();
			gluQuadricNormals(pObj, GLU_SMOOTH);

			glPushMatrix();
			glColor4ub(200,200,200,200);
			glTranslated(vecCubicPath[i].Tra_x[j], vecCubicPath[i].Tra_y[j], 0.15+0.05);
			gluDisk(pObj, 0., 0.15, 15, 10);
			glPopMatrix();
		}
	}





	vecCubicPath.clear();

	//-------------------------------------------------------------------------------------------//
}
