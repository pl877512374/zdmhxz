
// LaseAutoCorrect.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CLaseAutoCorrectApp:
// �йش����ʵ�֣������ LaseAutoCorrect.cpp
//

class CLaseAutoCorrectApp : public CWinApp
{
public:
	CLaseAutoCorrectApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CLaseAutoCorrectApp theApp;