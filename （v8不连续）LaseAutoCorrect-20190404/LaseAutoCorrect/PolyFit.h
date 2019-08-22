#pragma once


// CPolyFit 对话框

class CPolyFit : public CDialogEx
{
	DECLARE_DYNAMIC(CPolyFit)

public:
	CPolyFit(CWnd* pParent = NULL);   // 标准构造函数
	virtual ~CPolyFit();

// 对话框数据
	enum { IDD = IDD_DIALOG_PolyFIt };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
};
