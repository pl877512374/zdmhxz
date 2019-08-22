// RealDisplay.cpp : 实现文件
//

#include "stdafx.h"
#include "LaseAutoCorrect.h"
#include "RealDisplay.h"
#include "afxdialogex.h"


// CRealDisplay 对话框

IMPLEMENT_DYNAMIC(CRealDisplay, CDialogEx)

CRealDisplay::CRealDisplay(CWnd* pParent /*=NULL*/)
	: CDialogEx(CRealDisplay::IDD, pParent)
{

}

CRealDisplay::~CRealDisplay()
{
}

void CRealDisplay::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_TCHART_RealDraw, m_ChartReal);
}


BEGIN_MESSAGE_MAP(CRealDisplay, CDialogEx)
END_MESSAGE_MAP()


// CRealDisplay 消息处理程序
