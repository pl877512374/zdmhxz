#pragma once


// CPolyFit �Ի���

class CPolyFit : public CDialogEx
{
	DECLARE_DYNAMIC(CPolyFit)

public:
	CPolyFit(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CPolyFit();

// �Ի�������
	enum { IDD = IDD_DIALOG_PolyFIt };

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
};
