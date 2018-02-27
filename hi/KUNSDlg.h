// KUNSDlg.h : ��� ����
//

#pragma once
#include "./src/KUNSGUI/KUNSUI3DDlg.h"
#include "./src/MobileSupervisor/MobileSupervisor.h"
#include"./src/MobileSupervisor/KuCommandMessage.h"
#include"./src/MobileSupervisor/MobileSupervisor.h"
#include "./src/Sensor/WheelActuatorInterface/PioneerWheelActuatorInterface.h"
#include "./src/Sensor/SensorSupervisor.h"
#include "./src/MobileSupervisor/KuRobotParameter.h"
#include "./src/KUNSUtil/KUNSThread/KuThread.h"
#include "./src/KUNSUtil/KUNSCriticalSection/KuCriticalSection.h"
#include "./src/MobileSupervisor/KuPOIMapParameter.h"



// CKUNSDlg ��ȭ ����
class CKUNSDlg : public CDialogEx
{
// �����Դϴ�.
public:
	CKUNSDlg(CWnd* pParent = NULL);	// ǥ�� �������Դϴ�.

// ��ȭ ���� �������Դϴ�.
	enum { IDD = IDD_KUNS_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �����Դϴ�.


// �����Դϴ�.
protected:
	HICON m_hIcon;
	void ManualControl (double dt);
	bool JoystickControl (double &linearVel, double &steerAngle, DWORD &buttons);
	// ������ �޽��� �� �Լ�
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
private:
	CCriticalSection m_CriticalSection;
	CStatic m_CamInfoPicCtrl;
	CStatic m_CamInfoPicCtrl2;

private:
	CEdit m_XPosEditCtrl; //�κ��� ��ġ������ ��Ÿ���� edit control
	CEdit m_YPosEditCtrl; //�κ��� ��ġ������ ��Ÿ���� edit control
	CEdit m_TPosEditCtrl; //�κ��� ��ġ������ ��Ÿ���� edit control
	CListBox m_MapListBox;
private:
	KuThread m_KuThread;
	static void doThread(void* arg);
private:
	CKUNSUI3DDlg m_KUNSUI3DDlg;
	Localizer * m_pLocalizer;
	int m_nBehaviorName;
	int m_nSetGoalPosID,m_nSetRobotPosID;
	int m_nBhID;
	bool m_bexecuteflag;
	bool m_bSetRobotPos;
	bool m_bSetGoalPos;
	bool m_bTeachingPathflag;
	bool m_bMapBuildingflag;
	bool m_bLaserConflag;
	bool m_bWheelConflag;
	bool m_bKinectConflag;
	bool m_bCameraConflag;
	bool m_bMapRenderingflag;
	double m_dX;
	double m_dY;
	double m_dThetaDeg;
	double m_dTheta;
	bool m_bPatternCheck1;
	bool m_bPatternCheck2;
	bool m_bPatternCheck3;
	bool m_bPatternCheck4;

public:
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnBehaviorMapbuilding();
	afx_msg void OnSensorconnectionHokuyoutm();
	afx_msg void OnSensorconnectionWheelactuator();
	afx_msg void OnGuiinformationSetrobotpose();
	afx_msg void OnGuiinformationRendermap();
	afx_msg void OnGuiinformationRenderlaser();
	afx_msg void OnDestroy();
	afx_msg void OnGuiinformationRendercadmap();
	afx_msg void OnBehaviorGotogoal();
	afx_msg void OnGuiinformationSetgoalpose();
	afx_msg void OnGuiinformationRenderpath();
	afx_msg void OnLoaddataCadmap();
	afx_msg void OnBnClickedSaveWaypointButton();
	afx_msg void OnSensorconnectionKinect();
	afx_msg void OnGuiinformationRenderkinect();
	afx_msg void OnVirtualbehaviorGotogoal();
	afx_msg void OnVirtualbehaviorWanderingobstacle();
	afx_msg void OnGuiinformationSetobstacleareas();
	afx_msg void OnGuiinformationClearobstacleareas();
	afx_msg void OnBehaviorAu();
	afx_msg void OnLbnSelchangeMapList();
	afx_msg void OnSensorconnectionRplidar();
	afx_msg void OnBnClickedPatternCheck1();
	afx_msg void OnBnClickedPatternCheck2();
	afx_msg void OnBnClickedPatternCheck3();
	afx_msg void OnBnClickedPatternCheck4();
	afx_msg void OnGuiinformationSetobstaclegoals();
	afx_msg void OnVirtualbehaviorWanderingstart();
};
