// FitInter.cpp : 实现文件
//

#include "stdafx.h"
#include "LaseAutoCorrect.h"
#include "FitInter.h"
#include "afxdialogex.h"


// CFitInter 对话框

IMPLEMENT_DYNAMIC(CFitInter, CDialogEx)

CFitInter::CFitInter(CWnd* pParent /*=NULL*/)
	: CDialogEx(CFitInter::IDD, pParent)
{

}

CFitInter::~CFitInter()
{
}

void CFitInter::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_TCHART_PolyFit, m_ChartFit);
}


BEGIN_MESSAGE_MAP(CFitInter, CDialogEx)
END_MESSAGE_MAP()


// CFitInter 消息处理程序
