
// KUNS.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once

#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.


// CKUNSApp:
// �� Ŭ������ ������ ���ؼ��� KUNS.cpp�� �����Ͻʽÿ�.
//

class CKUNSApp : public CWinApp
{
public:
	CKUNSApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};

extern CKUNSApp theApp;