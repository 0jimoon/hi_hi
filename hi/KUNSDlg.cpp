
// KUNSDlg.cpp : 구현 파일
//

#include "stdafx.h"
#include "KUNS.h"
#include "KUNSDlg.h"
#include "afxdialogex.h"
#include <mmsystem.h>
#ifdef _DEBUG
//#define new DEBUG_NEW
#define  DEBUG_NEW new(THIS_FILE, __LINE__)
#endif


// 응용 프로그램 정보에 사용되는 CAboutDlg 대화 상자입니다.

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 대화 상자 데이터입니다.
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 지원입니다.

// 구현입니다.
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CKUNSDlg 대화 상자




CKUNSDlg::CKUNSDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CKUNSDlg::IDD, pParent)
	,m_bexecuteflag(false)	
	,m_bSetRobotPos(false)
	,m_nBehaviorName(-1)
	, m_bPatternCheck1(FALSE)
	, m_bPatternCheck2(FALSE)
	, m_bPatternCheck3(FALSE)
	, m_bPatternCheck4(FALSE)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CKUNSDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_XPOS_EDIT_CTRL, m_XPosEditCtrl);
	DDX_Control(pDX, IDC_YPOS_EDIT_CTRL, m_YPosEditCtrl);
	DDX_Control(pDX, IDC_TPOS_EDIT_CTRL, m_TPosEditCtrl);
	DDX_Control(pDX, IDC_MAP_LIST, m_MapListBox);

}

BEGIN_MESSAGE_MAP(CKUNSDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_TIMER()
	ON_WM_MOUSEWHEEL()
	ON_COMMAND(ID_BEHAVIOR_MAPBUILDING, &CKUNSDlg::OnBehaviorMapbuilding)
	ON_COMMAND(ID_SENSORCONNECTION_HOKUYOUTM, &CKUNSDlg::OnSensorconnectionHokuyoutm)
	ON_COMMAND(ID_SENSORCONNECTION_WHEELACTUATOR, &CKUNSDlg::OnSensorconnectionWheelactuator)
	ON_COMMAND(ID_GUIINFORMATION_SETROBOTPOSE, &CKUNSDlg::OnGuiinformationSetrobotpose)
	ON_COMMAND(ID_GUIINFORMATION_RENDERMAP, &CKUNSDlg::OnGuiinformationRendermap)
	ON_COMMAND(ID_GUIINFORMATION_RENDERLASER, &CKUNSDlg::OnGuiinformationRenderlaser)
	ON_BN_CLICKED(IDC_SAVE_WAYPOINT_BUTTON, &CKUNSDlg::OnBnClickedSaveWaypointButton)
	ON_WM_DESTROY()
	ON_COMMAND(ID_GUIINFORMATION_RENDERCADMAP, &CKUNSDlg::OnGuiinformationRendercadmap)
	ON_COMMAND(ID_BEHAVIOR_GOTOGOAL, &CKUNSDlg::OnBehaviorGotogoal)
	ON_COMMAND(ID_GUIINFORMATION_SETGOALPOSE, &CKUNSDlg::OnGuiinformationSetgoalpose)
	ON_COMMAND(ID_GUIINFORMATION_RENDERPATH, &CKUNSDlg::OnGuiinformationRenderpath)
	ON_COMMAND(ID_LOADDATA_CADMAP, &CKUNSDlg::OnLoaddataCadmap)
	ON_COMMAND(ID_SENSORCONNECTION_KINECT, &CKUNSDlg::OnSensorconnectionKinect)
	ON_COMMAND(ID_GUIINFORMATION_RENDERKINECT, &CKUNSDlg::OnGuiinformationRenderkinect)
	ON_COMMAND(ID_VIRTUALBEHAVIOR_GOTOGOAL, &CKUNSDlg::OnVirtualbehaviorGotogoal)
	ON_COMMAND(ID_VIRTUALBEHAVIOR_WANDERINGOBSTACLE, &CKUNSDlg::OnVirtualbehaviorWanderingobstacle)
	ON_COMMAND(ID_GUIINFORMATION_SETOBSTACLEAREAS, &CKUNSDlg::OnGuiinformationSetobstacleareas)
	ON_COMMAND(ID_GUIINFORMATION_CLEAROBSTACLEAREAS, &CKUNSDlg::OnGuiinformationClearobstacleareas)
	ON_COMMAND(ID_BEHAVIOR_AU, &CKUNSDlg::OnBehaviorAu)
	ON_LBN_SELCHANGE(IDC_MAP_LIST, &CKUNSDlg::OnLbnSelchangeMapList)
	ON_COMMAND(ID_SENSORCONNECTION_RPLIDAR, &CKUNSDlg::OnSensorconnectionRplidar)
	ON_BN_CLICKED(IDC_PATTERN_CHECK1, &CKUNSDlg::OnBnClickedPatternCheck1)
	ON_BN_CLICKED(IDC_PATTERN_CHECK2, &CKUNSDlg::OnBnClickedPatternCheck2)
	ON_BN_CLICKED(IDC_PATTERN_CHECK3, &CKUNSDlg::OnBnClickedPatternCheck3)
	ON_BN_CLICKED(IDC_PATTERN_CHECK4, &CKUNSDlg::OnBnClickedPatternCheck4)
	ON_COMMAND(ID_GUIINFORMATION_SETOBSTACLEGOALS, &CKUNSDlg::OnGuiinformationSetobstaclegoals)
	ON_COMMAND(ID_VIRTUALBEHAVIOR_WANDERINGSTART, &CKUNSDlg::OnVirtualbehaviorWanderingstart)
END_MESSAGE_MAP()


// CKUNSDlg 메시지 처리기

BOOL CKUNSDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();
	
	//_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF|_CRTDBG_ALLOC_MEM_DF);

	// 시스템 메뉴에 "정보..." 메뉴 항목을 추가합니다.

	// IDM_ABOUTBOX는 시스템 명령 범위에 있어야 합니다.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 이 대화 상자의 아이콘을 설정합니다. 응용 프로그램의 주 창이 대화 상자가 아닐 경우에는
	//  프레임워크가 이 작업을 자동으로 수행합니다.
	SetIcon(m_hIcon, TRUE);			// 큰 아이콘을 설정합니다.
	SetIcon(m_hIcon, FALSE);		// 작은 아이콘을 설정합니다.

	// TODO: 여기에 추가 초기화 작업을 추가합니다.

//	this->MoveWindow(0,0,840,700);
	this->MoveWindow(0,0,1050,700);


	m_KUNSUI3DDlg.Create(IDD_KUNSUI3D_DIALOG, this);
	m_KUNSUI3DDlg.MoveWindow(5, 5, 810, 600);
	m_KUNSUI3DDlg.ShowWindow(SW_SHOW);

	
	m_XPosEditCtrl.MoveWindow(10,610,100,25);
	m_YPosEditCtrl.MoveWindow(120,610,100,25);
	m_TPosEditCtrl.MoveWindow(230,610,100,25);
	m_MapListBox.MoveWindow(830,5,200,400);

	m_XPosEditCtrl.SetWindowText(_T("X POS:0 m"));
	m_YPosEditCtrl.SetWindowText(_T("Y POS:0 m"));
	m_TPosEditCtrl.SetWindowText(_T("Theta :0 Deg"));
	
	if(!KuRobotParameter::getInstance()->initialize()){exit(1);}
	if(!KuPOIMapParameter::getInstance()->initialize()){exit(1);}

	CString str;
	str="IH_1F";
	m_MapListBox.AddString(str);
	str="IH_4F";
	m_MapListBox.AddString(str);
	str="IH_6F";
	m_MapListBox.AddString(str);
	str="IH_1F";
	m_MapListBox.SelectString(0,str);

 	m_KuThread.start(doThread,this,200); //메인 스레드 시작				

 	MobileSupervisor::getInstance()->loadMap();
 	SensorSupervisor::getInstance()->DataRecodingNPlay();

	KuDrawingInfo::getInstance()->setRenderMapflag(true);
	KuDrawingInfo::getInstance()->setRenderPathflag(true);
	KuDrawingInfo::getInstance()->setRenderLaserflag(true);

	CMenu* pMainMenu = GetMenu();
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERMAP, MF_CHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERPATH, MF_CHECKED);
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERLASER, MF_CHECKED);
	SetTimer(0,200, NULL);




	return TRUE;  // 포커스를 컨트롤에 설정하지 않으면 TRUE를 반환합니다.
}

void CKUNSDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

void CKUNSDlg::doThread(void* arg)
{
	CKUNSUI3DDlg* p3DMapDlg = (CKUNSUI3DDlg*)arg;
	CKUNSDlg* pKUNSDlg = (CKUNSDlg*)arg;

	pKUNSDlg->m_CriticalSection.Lock();
	p3DMapDlg->UpdateWindow();
	pKUNSDlg->m_CriticalSection.Unlock();

}
// 대화 상자에 최소화 단추를 추가할 경우 아이콘을 그리려면
//  아래 코드가 필요합니다. 문서/뷰 모델을 사용하는 MFC 응용 프로그램의 경우에는
//  프레임워크에서 이 작업을 자동으로 수행합니다.

void CKUNSDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 그리기를 위한 디바이스 컨텍스트입니다.

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 클라이언트 사각형에서 아이콘을 가운데에 맞춥니다.
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 아이콘을 그립니다.
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 사용자가 최소화된 창을 끄는 동안에 커서가 표시되도록 시스템에서
//  이 함수를 호출합니다.
HCURSOR CKUNSDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



BOOL CKUNSDlg::PreTranslateMessage(MSG* pMsg)
{
	// TODO: 여기에 특수화된 코드를 추가 및/또는 기본 클래스를 호출합니다.

	double dGRID=100.0; //mm 단위 10cm격자
	double dTVelIncrement=200;
	double dRVelIncrement =20;
	switch(pMsg->message){

	case WM_KEYDOWN:{
		switch(pMsg->wParam) {			 
			// 맵 확대,축소
		case VK_OEM_COMMA :
			if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
				KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
				RobotPos.setThetaDeg(RobotPos.getThetaDeg()+1);	
				KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
				if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);
			}

			break;
		case VK_OEM_PERIOD :
			if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
				KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
				RobotPos.setThetaDeg(RobotPos.getThetaDeg()-1);	
				KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
				if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);
			}
			break;
		case VK_OEM_PLUS:
			m_KUNSUI3DDlg.Zoom_In(2.);
			return TRUE;

		case VK_OEM_MINUS:
			m_KUNSUI3DDlg.Zoom_Out(2.);
			return TRUE;

		case VK_UP:
			{ 
				if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
					KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
					RobotPos.setY(RobotPos.getY()+dGRID);	
					KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
					if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);
				}else{ //실제로 로봇을 구동한다.
					// 로봇 컨트롤
// 					double dTVel = TETRAWheelActuatorInterface::getInstance()->getTVel();
// 					TETRAWheelActuatorInterface::getInstance()->moveByTRVelocity(dTVel+dTVelIncrement,0.0);
					PioneerWheelActuatorInterface::getInstance()->moveByTRVelocity(dTVelIncrement,0.0);
				
				}
				break;

			}
		case VK_DOWN:
			{
				if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
					KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
					RobotPos.setY(RobotPos.getY()-dGRID);	
					KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
					if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);
				}else{ //실제로 로봇을 구동한다.
// 					double dTVel = TETRAWheelActuatorInterface::getInstance()->getTVel();
// 					TETRAWheelActuatorInterface::getInstance()->moveByTRVelocity(dTVel - dTVelIncrement,0.0);
					PioneerWheelActuatorInterface::getInstance()->moveByTRVelocity( -dTVelIncrement,0.0);

				}
				break;
			}

		case VK_LEFT:
			{
				if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
					KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
					RobotPos.setX(RobotPos.getX()-dGRID);	
					KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
					if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);
				}else{ //실제로 로봇을 구동한다.
// 					double dRVel = TETRAWheelActuatorInterface::getInstance()->getRVel();
// 					double dTVel = TETRAWheelActuatorInterface::getInstance()->getTVel();
// 					TETRAWheelActuatorInterface::getInstance()->moveByTRVelocity(dTVel,dRVel+dRVelIncrement);
					PioneerWheelActuatorInterface::getInstance()->moveByTRVelocity(0,dRVelIncrement);
				}
				break;
			}

		case VK_RIGHT:
			{
				if(m_bSetRobotPos){ //화면상에서 로봇의 위치좌표 정보만을 변화시켜준다. 로봇은 실제로 움직이지 않는다.
					KuPose RobotPos = KuDrawingInfo::getInstance()->getRobotPos();
					RobotPos.setX(RobotPos.getX()+dGRID);	
					KuDrawingInfo::getInstance()->setRobotPos(RobotPos);
					if(m_bexecuteflag)m_pLocalizer->setRobotPos(RobotPos);
				}else{ //실제로 로봇을 구동한다.
// 					double dRVel = TETRAWheelActuatorInterface::getInstance()->getRVel();
// 					double dTVel = TETRAWheelActuatorInterface::getInstance()->getTVel();
// 					TETRAWheelActuatorInterface::getInstance()->moveByTRVelocity(dTVel,dRVel-dRVelIncrement);
					PioneerWheelActuatorInterface::getInstance()->moveByTRVelocity(0,-dRVelIncrement);
				}

				break;
			}

		case VK_SHIFT:
			{
				PioneerWheelActuatorInterface::getInstance()->stop();				
				return TRUE;
			}

		case 65: //a키 자율 주행 셋팅

			break;
		case 77: //m키 수동 주행 셋팅


			break; 
		case 48: case 49: case 50: case 51: case 52: case 53: case 54: case 55: case 56: case 57: return false;  
		}

		return TRUE;
					}
	}

	return FALSE;
}

void CKUNSDlg::ManualControl (double dt)
{
	DWORD buttons = 0;
	double linearVel = 0.;
	double steerAngle = 0.;
	
	if (JoystickControl (linearVel, steerAngle, buttons)) {
		PioneerWheelActuatorInterface::getInstance()->moveByTRVelocity(linearVel*500,steerAngle*500);
	}
}

bool CKUNSDlg::JoystickControl (double &linearVel, double &steerAngle, DWORD &buttons)
{
	if (0 < joyGetNumDevs ()) {
		const double deadZoneX = 0.1;
		const double deadZoneY = 0.2;

		UINT joystickid = JOYSTICKID1;
		JOYINFOEX joyinfo;

		memset (&joyinfo, 0, sizeof(JOYINFOEX));
		joyinfo.dwSize = sizeof(JOYINFOEX);
		joyinfo.dwFlags = JOY_RETURNALL;

		if (joyGetPosEx(joystickid, &joyinfo) == JOYERR_NOERROR) {
			if(joyinfo.dwButtons == 192)
			{
				double vel = (32768 - (long)joyinfo.dwYpos)/32768.;
				double ang = (32768 - (long)joyinfo.dwVpos)/32768.;
				double thr = (65536 - (long)joyinfo.dwZpos)/65536.;

				if (vel > deadZoneX)       vel -= deadZoneX;
				else if (vel < -deadZoneX) vel += deadZoneX;
				else                       vel  = 0.;

				if (ang > deadZoneY)       ang -= deadZoneY;
				else if (ang < -deadZoneY) ang += deadZoneY;
				else                       ang  = 0.;

				linearVel = thr*vel/(1. - deadZoneX);
				steerAngle = ang/(1. - deadZoneY);
				buttons = joyinfo.dwButtons;
				return true;
			}
		}
	}
	return false;
}

void CKUNSDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	//ManualControl(0.1);


	KuPose RobotPos=KuDrawingInfo::getInstance()->getRobotPos();
	
	CString strRobotPos;
	strRobotPos.Format(_T("X Pos: %.2fm"),RobotPos.getXm());
	m_XPosEditCtrl.SetWindowText(strRobotPos);
	strRobotPos.Format(_T("Y Pos: %.2fm"),RobotPos.getYm());
	m_YPosEditCtrl.SetWindowText(strRobotPos);
	strRobotPos.Format(_T("Theata: %.2fDeg"),RobotPos.getThetaDeg());
	m_TPosEditCtrl.SetWindowText(strRobotPos);

	if(m_bexecuteflag==false)
	{
		// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.	
		if( KuRobotParameter::getInstance()->getDataRecoding()=="yes" ){ //데이터 레코딩을 수행하는 경우이다.
			SensorSupervisor::getInstance()->readSensorData();
		}
		else
		{
			//SensorSupervisor::getInstance()->readOnlySensorData();
			SensorSupervisor::getInstance()->readSensorData();
		}
		int_1DArray nLaserData181 = SensorSupervisor::getInstance()->getURG04LXLaserData();
		KuPose DelEncoder =SensorSupervisor::getInstance()->getEncoderDelPos();
// 		KuPose* pKinectDataPos = SensorSupervisor::getInstance()->getKinectDataPos();
// 		KuDrawingInfo::getInstance()->setKinectGlobal3DPos(pKinectDataPos);

		int_1DArray nRplidarData181 = SensorSupervisor::getInstance()->getRplidarData();

		KuDrawingInfo::getInstance()->setLaserData181(nLaserData181);

		KuDrawingInfo::getInstance()->setRplidarData181(nRplidarData181);

		if(KuDrawingInfo::getInstance()->getRenderKinect3DCloudFlag( ))
		{
			IplImage * IplKinectCamera = SensorSupervisor::getInstance()->getKinectImageData();
			KuPose* pKinectDataPos = SensorSupervisor::getInstance()->getKinectDataPos();

			KuDrawingInfo::getInstance()->setKinectGlobal3DPos(pKinectDataPos);
			KuDrawingInfo::getInstance()->setKinectImageData(IplKinectCamera);
		}

		
		double dX = RobotPos.getX() + DelEncoder.getX() * cos(RobotPos.getThetaRad()) + 
			DelEncoder.getY() * sin(-RobotPos.getThetaRad());
		double dY = RobotPos.getY() + DelEncoder.getX() * sin(RobotPos.getThetaRad()) + 
			DelEncoder.getY() * cos(RobotPos.getThetaRad());
		double dThetaDeg = RobotPos.getThetaDeg() + DelEncoder.getThetaDeg();
		// pose update	
		RobotPos.setX(dX);
		RobotPos.setY(dY);
		RobotPos.setThetaDeg(dThetaDeg);
	
		KuDrawingInfo::getInstance()->setRobotPos(RobotPos);

		


		switch(m_nBehaviorName){
		case KuCommandMessage::HYBRID_MAP_BUILDING_BH:
			CheckMenuItem(GetMenu()->GetSafeHmenu(), ID_BEHAVIOR_MAPBUILDING, MF_UNCHECKED);
			break;
		case KuCommandMessage::GOTOGOAL_BH:
			CheckMenuItem(GetMenu()->GetSafeHmenu(), ID_BEHAVIOR_GOTOGOAL, MF_UNCHECKED);
			break;
		case KuCommandMessage::VIRTUAL_GOTOGOAL_BH:
			CheckMenuItem(GetMenu()->GetSafeHmenu(), ID_VIRTUALBEHAVIOR_GOTOGOAL, MF_UNCHECKED);
			break;
		case KuCommandMessage::WANDERING_OBSTACLE_PR:
			CheckMenuItem(GetMenu()->GetSafeHmenu(), ID_VIRTUALBEHAVIOR_WANDERINGOBSTACLE, MF_UNCHECKED);
			break;

		default:break;
		}
	}
	else
	{
		KuCommandMessage CMessage;
		CMessage.setBehaviorName(m_nBehaviorName);
		if(!MobileSupervisor::getInstance()->getBehaviorStates(CMessage))
			m_bexecuteflag=false;
	}
	::InvalidateRect(m_KUNSUI3DDlg.GetSafeHwnd(), 0,FALSE);
	CDialogEx::OnTimer(nIDEvent);
}


BOOL CKUNSDlg::OnMouseWheel(UINT nFlags, short zDelta, CPoint pt)
{
	// TODO: 여기에 메시지 처리기 코드를 추가 및/또는 기본값을 호출합니다.
	if(zDelta > 0) m_KUNSUI3DDlg.Zoom_In(0.5);
	else m_KUNSUI3DDlg.Zoom_Out(0.5);
	return CDialogEx::OnMouseWheel(nFlags, zDelta, pt);
}

void CKUNSDlg::OnBehaviorMapbuilding()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_BEHAVIOR_MAPBUILDING, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_MAPBUILDING, MF_UNCHECKED);
		KuCommandMessage CMessage;
		CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::HYBRID_MAP_BUILDING_BH);
		MobileSupervisor::getInstance()->execute(CMessage);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_MAPBUILDING, MF_CHECKED);
		m_bexecuteflag=true;
		KuDrawingInfo::getInstance()->setRenderBuildingMapflag(true);
		KuCommandMessage CMessage;
		m_nBehaviorName=KuCommandMessage::HYBRID_MAP_BUILDING_BH;
		CMessage.setCommandName(KuCommandMessage::START_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::HYBRID_MAP_BUILDING_BH);
		CMessage.setRobotPos(KuDrawingInfo::getInstance()->getRobotPos());
		CMessage.setMapSizeXmYm(KuRobotParameter::getInstance()->getMapSizeXm(),KuRobotParameter::getInstance()->getMapSizeYm());
		MobileSupervisor::getInstance()->execute(CMessage);
		m_pLocalizer=MobileSupervisor::getInstance()->getLocalizer(CMessage);
	} 	
}


void CKUNSDlg::OnSensorconnectionHokuyoutm()
{
	printf("connect Laser\n");
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SENSORCONNECTION_HOKUYOUTM, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_HOKUYOUTM, MF_UNCHECKED);
		HokuyoURG04LXInterface::getInstance()->disconnectLaserScanner();
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_HOKUYOUTM, MF_CHECKED);
		if(!SensorSupervisor::getInstance()->connectionHokuyoutm()) 	
			CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_HOKUYOUTM, MF_UNCHECKED);
	} 	
}


void CKUNSDlg::OnSensorconnectionWheelactuator()
{
	printf("connect Wheel\n");
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SENSORCONNECTION_WHEELACTUATOR, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_WHEELACTUATOR, MF_UNCHECKED);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_WHEELACTUATOR, MF_CHECKED);
		if(!SensorSupervisor::getInstance()->connectionWheelactuator()) CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_WHEELACTUATOR, MF_UNCHECKED);
	} 		
}


void CKUNSDlg::OnGuiinformationSetrobotpose()
{
	printf("set Robot Pose\n");
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_SETROBOTPOSE, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETROBOTPOSE, MF_UNCHECKED);
		m_KUNSUI3DDlg.setRobotPosFlag( false );
		m_bSetRobotPos=false;

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETROBOTPOSE, MF_CHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETGOALPOSE, MF_UNCHECKED);


		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETOBSTACLEGOALS, MF_UNCHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETGOALPOSE, MF_UNCHECKED);
		//CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETROBOTPOSE, MF_UNCHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETOBSTACLEAREAS, MF_UNCHECKED);

		//m_KUNSUI3DDlg.setRobotPosFlag( false );
		m_KUNSUI3DDlg.setGoalPosFlag( false );
		m_KUNSUI3DDlg.setObsAreaPosFlag( false );		
		m_KUNSUI3DDlg.setObsAreaGoalPosFlag( false );

		m_KUNSUI3DDlg.setRobotPosFlag( true );
		//m_KUNSUI3DDlg.setGoalPosFlag( false );
		m_bSetRobotPos=true;
	} 	
}

void CKUNSDlg::OnGuiinformationRendermap()
{
	printf("Render Mape\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_RENDERMAP, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERMAP, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setRenderMapflag(false);

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERMAP, MF_CHECKED);
		KuDrawingInfo::getInstance()->setRenderMapflag(true);


	} 	
}


void CKUNSDlg::OnGuiinformationRenderlaser()
{
	printf("Render Laser\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_RENDERLASER, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERLASER, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setRenderLaserflag(false);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERLASER, MF_CHECKED);
		KuDrawingInfo::getInstance()->setRenderLaserflag(true);

	} 	
}

void CKUNSDlg::OnDestroy()
{
	m_KuThread.terminate();
	KuCommandMessage CMessage;
	CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
	CMessage.setBehaviorName(KuCommandMessage::HYBRID_MAP_BUILDING_BH);
	MobileSupervisor::getInstance()->execute(CMessage);
	CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
	CMessage.setBehaviorName(KuCommandMessage::PATH_TEACHING_BH);
	MobileSupervisor::getInstance()->execute(CMessage);	
	AfxGetMainWnd()->SendMessage(WM_CLOSE);
	CDialog::OnDestroy();

	PostQuitMessage(WM_QUIT);

	DWORD processID = 0;
	GetWindowThreadProcessId(this->m_hWnd, &processID);
	TerminateProcess(OpenProcess(PROCESS_ALL_ACCESS,FALSE,processID),0);

	// TODO: 여기에 메시지 처리기 코드를 추가합니다.
}


void CKUNSDlg::OnGuiinformationRendercadmap()
{
	printf("Render CAD Map\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_RENDERCADMAP, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERCADMAP, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setRenderCADMapflag(false);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERCADMAP, MF_CHECKED);
		KuDrawingInfo::getInstance()->setRenderCADMapflag(true);
	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CKUNSDlg::OnBehaviorGotogoal()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_BEHAVIOR_GOTOGOAL, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_GOTOGOAL, MF_UNCHECKED);
		m_bexecuteflag=false;
		KuCommandMessage CMessage;
		CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::GOTOGOAL_BH);
		MobileSupervisor::getInstance()->execute(CMessage);

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_GOTOGOAL, MF_CHECKED);
		m_bexecuteflag=true;
		KuCommandMessage CMessage;
		m_nBehaviorName=KuCommandMessage::GOTOGOAL_BH;
		CMessage.setCommandName(KuCommandMessage::START_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::GOTOGOAL_BH);
		CMessage.setGoalPos(KuDrawingInfo::getInstance()->getGoalPos());
		CMessage.setRobotPos(KuDrawingInfo::getInstance()->getRobotPos());		 
		MobileSupervisor::getInstance()->execute(CMessage);
		m_pLocalizer=MobileSupervisor::getInstance()->getLocalizer(CMessage);

	} 	
}


void CKUNSDlg::OnGuiinformationSetgoalpose()
{
	printf("set Goal Pose\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_SETGOALPOSE, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETGOALPOSE, MF_UNCHECKED);
		m_KUNSUI3DDlg.setGoalPosFlag( false );
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETGOALPOSE, MF_CHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETOBSTACLEGOALS, MF_UNCHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETROBOTPOSE, MF_UNCHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETOBSTACLEAREAS, MF_UNCHECKED);

		m_KUNSUI3DDlg.setObsAreaPosFlag( false );		
		m_KUNSUI3DDlg.setObsAreaGoalPosFlag( false );
		m_KUNSUI3DDlg.setRobotPosFlag( false );
		m_KUNSUI3DDlg.setGoalPosFlag( true );
		m_bSetRobotPos=false;

	} 	
}


void CKUNSDlg::OnGuiinformationRenderpath()
{
	printf("Render Path\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_RENDERPATH, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERPATH, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setRenderPathflag(false);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERPATH, MF_CHECKED);
		KuDrawingInfo::getInstance()->setRenderPathflag(true);
	} 	
}


void CKUNSDlg::OnLoaddataCadmap()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	MobileSupervisor::getInstance()->loadCADMap();
}

void CKUNSDlg::OnBnClickedSaveWaypointButton()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	KuDrawingInfo::getInstance()->setWayPointflag(true);
}

void CKUNSDlg::OnSensorconnectionKinect()
{
	printf("connect Kinect\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SENSORCONNECTION_KINECT, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_KINECT, MF_UNCHECKED);
		KinectSensorInterface::getInstance()->terminate();
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_KINECT, MF_CHECKED);
		if(!SensorSupervisor::getInstance()->connectionKinect())	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_KINECT, MF_UNCHECKED);

	} 	
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
}


void CKUNSDlg::OnGuiinformationRenderkinect()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_RENDERXTION, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERXTION, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setRenderKinect3DCloudFlag(false) ;

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_RENDERXTION, MF_CHECKED);
		KuDrawingInfo::getInstance()->setRenderKinect3DCloudFlag(true) ;
		printf("Render Kinect\n");
	} 	
}

void CKUNSDlg::OnVirtualbehaviorGotogoal()
{

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_VIRTUALBEHAVIOR_GOTOGOAL, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_VIRTUALBEHAVIOR_GOTOGOAL, MF_UNCHECKED);
		m_bexecuteflag=false;
		KuCommandMessage CMessage;
		CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::VIRTUAL_GOTOGOAL_BH);
		MobileSupervisor::getInstance()->execute(CMessage);

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_VIRTUALBEHAVIOR_GOTOGOAL, MF_CHECKED);
		m_bexecuteflag=true;
		// TODO: Add your command handler code here
		KuCommandMessage CMessage;
		m_nBehaviorName=KuCommandMessage::VIRTUAL_GOTOGOAL_BH;
		CMessage.setCommandName(KuCommandMessage::START_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::VIRTUAL_GOTOGOAL_BH);
		CMessage.setBehaviorPeriod(100);
		CMessage.setRobotPos(KuDrawingInfo::getInstance()->getRobotPos());
		CMessage.setGoalPos(KuDrawingInfo::getInstance()->getGoalPos());
		CMessage.setMapSizeXmYm(KuRobotParameter::getInstance()->getMapSizeXm(),KuRobotParameter::getInstance()->getMapSizeYm());
		MobileSupervisor::getInstance()->execute(CMessage);

	} 	
}


void CKUNSDlg::OnVirtualbehaviorWanderingobstacle()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_VIRTUALBEHAVIOR_WANDERINGOBSTACLE, MF_BYCOMMAND); 
	CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_VIRTUALBEHAVIOR_WANDERINGOBSTACLE, MF_CHECKED);
	
	// TODO: Add your command handler code here
	KuCommandMessage CMessage;
	m_nBehaviorName=KuCommandMessage::WANDERING_OBSTACLE_PR;
	CMessage.setCommandName(KuCommandMessage::START_THREAD);
	CMessage.setBehaviorName(KuCommandMessage::WANDERING_OBSTACLE_PR);
	CMessage.setBehaviorPeriod(100);
	CMessage.setRobotPos(KuDrawingInfo::getInstance()->getRobotPos());
	CMessage.setGoalPos(KuDrawingInfo::getInstance()->getGoalPos());
	CMessage.setMapSizeXmYm(KuRobotParameter::getInstance()->getMapSizeXm(),KuRobotParameter::getInstance()->getMapSizeYm());
	MobileSupervisor::getInstance()->executesim(CMessage);
}


void CKUNSDlg::OnGuiinformationSetobstacleareas()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_SETOBSTACLEAREAS, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETOBSTACLEAREAS, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setObstacleAreasFlag(false);
		m_KUNSUI3DDlg.setObsAreaPosFlag( false );

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETOBSTACLEAREAS, MF_CHECKED);

		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETOBSTACLEGOALS, MF_UNCHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETGOALPOSE, MF_UNCHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETROBOTPOSE, MF_UNCHECKED);
		//CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETOBSTACLEAREAS, MF_UNCHECKED);

		m_KUNSUI3DDlg.setRobotPosFlag( false );
		m_KUNSUI3DDlg.setGoalPosFlag( false );
		//m_KUNSUI3DDlg.setObsAreaPosFlag( false );		
		m_KUNSUI3DDlg.setObsAreaGoalPosFlag( false );

		KuDrawingInfo::getInstance()->setObstacleAreasFlag(true) ;
		m_KUNSUI3DDlg.setObsAreaPosFlag( true );
		
		printf("Set Obstacle Areas \n");
	} 	
}


void CKUNSDlg::OnGuiinformationClearobstacleareas()
{
	// TODO: 여기에 명령 처리기 코드를 추가합니다.
	KuDrawingInfo::getInstance()->clearObstacleAreas();
}


void CKUNSDlg::OnBehaviorAu()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_BEHAVIOR_AU, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_AU, MF_UNCHECKED);
		m_bexecuteflag=false;
		KuCommandMessage CMessage;
		CMessage.setCommandName(KuCommandMessage::TERMINATE_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::AUTONOMOUS_GOTOGOAL);
		MobileSupervisor::getInstance()->execute(CMessage);

	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_BEHAVIOR_AU, MF_CHECKED);
		m_bexecuteflag=true;
		// TODO: Add your command handler code here
		KuCommandMessage CMessage;
		m_nBehaviorName=KuCommandMessage::AUTONOMOUS_GOTOGOAL;
		CMessage.setCommandName(KuCommandMessage::START_THREAD);
		CMessage.setBehaviorName(KuCommandMessage::AUTONOMOUS_GOTOGOAL);
		CMessage.setBehaviorPeriod(100);
		CMessage.setRobotPos(KuDrawingInfo::getInstance()->getRobotPos());
		CMessage.setGoalPos(KuDrawingInfo::getInstance()->getGoalPos());
		CMessage.setMapSizeXmYm(KuRobotParameter::getInstance()->getMapSizeXm(),KuRobotParameter::getInstance()->getMapSizeYm());
		MobileSupervisor::getInstance()->execute(CMessage);

	} 	
}


void CKUNSDlg::OnLbnSelchangeMapList()
{
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
	int index;
	CString str;
	index = m_MapListBox.GetCurSel();
	m_MapListBox.GetText(index,str);
	int totCount;
	totCount = m_MapListBox.GetCount();

	//PIOMapPara PIOMapData;
	//PIOMapData=KuPIOMapParameter::getInstance()->getPIOMapData(index);
	MobileSupervisor::getInstance()->setCurIndex(index);
}


void CKUNSDlg::OnSensorconnectionRplidar()
{
	printf("connect Rplidar\n");
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_SENSORCONNECTION_RPLIDAR, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_RPLIDAR, MF_UNCHECKED);
		//HokuyoURG04LXInterface::getInstance()->disconnectLaserScanner();
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_RPLIDAR, MF_CHECKED);
		if(!SensorSupervisor::getInstance()->connectionRplidar()) 	
			CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_SENSORCONNECTION_RPLIDAR, MF_UNCHECKED);
	} 	
}


void CKUNSDlg::OnBnClickedPatternCheck1()
{
	CButton *pChkBox1 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK1);
	int ChkState = pChkBox1->GetCheck();
	 
	if(ChkState==BST_CHECKED)
	{
		m_bPatternCheck1=true;
		CButton *pChkBox2 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK2);
		pChkBox2->SetCheck(FALSE);
		CButton *pChkBox3 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK3);
		pChkBox3->SetCheck(FALSE);
		CButton *pChkBox4 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK4);
		pChkBox4->SetCheck(FALSE);
		KuDrawingInfo::getInstance()->setPatternID(1);
	}
	else
	{
		m_bPatternCheck1=false;
	}
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CKUNSDlg::OnBnClickedPatternCheck2()
{
	CButton *pChkBox2 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK2);
	int ChkState = pChkBox2->GetCheck();

	if(m_bPatternCheck2==false)
	{
		m_bPatternCheck2=true;
		CButton *pChkBox1 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK1);
		pChkBox1->SetCheck(FALSE);
		CButton *pChkBox3 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK3);
		pChkBox3->SetCheck(FALSE);
		CButton *pChkBox4 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK4);
		pChkBox4->SetCheck(FALSE);
		KuDrawingInfo::getInstance()->setPatternID(2);

	}
	else
	{
		m_bPatternCheck2=false;
	}
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CKUNSDlg::OnBnClickedPatternCheck3()
{
	CButton *pChkBox3 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK3);
	int ChkState = pChkBox3->GetCheck();

	if(m_bPatternCheck3==false)
	{
		m_bPatternCheck3=true;
		CButton *pChkBox2 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK2);
		pChkBox2->SetCheck(FALSE);
		CButton *pChkBox1 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK1);
		pChkBox1->SetCheck(FALSE);
		CButton *pChkBox4 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK4);
		pChkBox4->SetCheck(FALSE);
		KuDrawingInfo::getInstance()->setPatternID(3);

	}
	else
	{
		m_bPatternCheck3=false;
	}
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CKUNSDlg::OnBnClickedPatternCheck4()
{
	CButton *pChkBox4 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK3);
	int ChkState = pChkBox4->GetCheck();

	if(m_bPatternCheck4==false)
	{
		m_bPatternCheck4=true;
		CButton *pChkBox2 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK2);
		pChkBox2->SetCheck(FALSE);
		CButton *pChkBox3 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK3);
		pChkBox3->SetCheck(FALSE);
		CButton *pChkBox1 = (CButton*)GetDlgItem(IDC_PATTERN_CHECK1);
		pChkBox1->SetCheck(FALSE);
		KuDrawingInfo::getInstance()->setPatternID(4);

	}
	else
	{
		m_bPatternCheck4=false;
	}
	// TODO: 여기에 컨트롤 알림 처리기 코드를 추가합니다.
}


void CKUNSDlg::OnGuiinformationSetobstaclegoals()
{
	printf("set Goal Pose\n");

	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_GUIINFORMATION_SETOBSTACLEGOALS, MF_BYCOMMAND); 
	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETOBSTACLEGOALS, MF_UNCHECKED);
		m_KUNSUI3DDlg.setObsAreaGoalPosFlag( false );
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETOBSTACLEGOALS, MF_CHECKED);

		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETGOALPOSE, MF_UNCHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETROBOTPOSE, MF_UNCHECKED);
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_GUIINFORMATION_SETOBSTACLEAREAS, MF_UNCHECKED);

		m_KUNSUI3DDlg.setRobotPosFlag( false );
		m_KUNSUI3DDlg.setGoalPosFlag( false );
		m_KUNSUI3DDlg.setObsAreaPosFlag( false );		
		m_KUNSUI3DDlg.setObsAreaGoalPosFlag( true );

		m_bSetRobotPos=false;

	} 	
}


void CKUNSDlg::OnVirtualbehaviorWanderingstart()
{
	CMenu* pMainMenu = GetMenu();
	int nMenuState = GetMenuState(pMainMenu->GetSafeHmenu(), (UINT) ID_VIRTUALBEHAVIOR_WANDERINGSTART, MF_BYCOMMAND); 

	if (nMenuState == MF_CHECKED){ //메뉴가 체크되어 있는 상태이다.
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_VIRTUALBEHAVIOR_WANDERINGSTART, MF_UNCHECKED);
		KuDrawingInfo::getInstance()->setWanderingflag(false);
	}
	else{ //메뉴가 체크 안된 상태이기 때문에 체크표시 해주고 
		CheckMenuItem(pMainMenu->GetSafeHmenu(), ID_VIRTUALBEHAVIOR_WANDERINGSTART, MF_CHECKED);			
		KuDrawingInfo::getInstance()->setWanderingflag(true);

	} 	
}
