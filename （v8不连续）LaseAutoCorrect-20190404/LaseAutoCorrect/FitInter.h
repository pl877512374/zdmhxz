#pragma once
#include "Utils/TeeChart/tchart.h"

// CFitInter 对话框

class CFitInter : public CDialogEx
{
	DECLARE_DYNAMIC(CFitInter)

public:
	CFitInter(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CFitInter();

// 对话框数据
	enum { IDD = IDD_DIALOG_PolyFIt };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	CTChart m_ChartFit;
};
