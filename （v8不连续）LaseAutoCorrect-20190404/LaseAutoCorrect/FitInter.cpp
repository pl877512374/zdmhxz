// FitInter.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "LaseAutoCorrect.h"
#include "FitInter.h"
#include "afxdialogex.h"


// CFitInter �Ի���

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


// CFitInter ��Ϣ�������
