#pragma once
#include "Utils/TeeChart/tchart.h"


// CRealDisplay �Ի���

class CRealDisplay : public CDialogEx
{
	DECLARE_DYNAMIC(CRealDisplay)

public:
	CRealDisplay(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CRealDisplay();

// �Ի�������
	enum { IDD = IDD_DIALOG_REALDISPLAY };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	CTChart m_ChartReal;
};
