#pragma once
#include "Utils/TeeChart/tchart.h"


// CRealDisplay 对话框

class CRealDisplay : public CDialogEx
{
	DECLARE_DYNAMIC(CRealDisplay)

public:
	CRealDisplay(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CRealDisplay();

// 对话框数据
	enum { IDD = IDD_DIALOG_REALDISPLAY };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	CTChart m_ChartReal;
};
