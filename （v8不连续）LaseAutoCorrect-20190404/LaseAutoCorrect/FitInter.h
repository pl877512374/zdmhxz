#pragma once
#include "Utils/TeeChart/tchart.h"

// CFitInter �Ի���

class CFitInter : public CDialogEx
{
	DECLARE_DYNAMIC(CFitInter)

public:
	CFitInter(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CFitInter();

// �Ի�������
	enum { IDD = IDD_DIALOG_PolyFIt };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	CTChart m_ChartFit;
};
