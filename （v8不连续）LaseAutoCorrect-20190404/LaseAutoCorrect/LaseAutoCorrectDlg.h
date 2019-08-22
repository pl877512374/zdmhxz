
// LaseAutoCorrectDlg.h : 头文件
//
#include "RealDisplay.h"
#include "FitInter.h"

#include "Utils/NetWork/NetworkUtils.h"

#define WM_NET_RECV WM_USER+100
#define WM_NET_SEND  WM_USER+101
#define WM_COM_PARSE  WM_USER+102

//激光参数查询参数设置消息响应 
#define WM_LASER_GETPARAM WM_USER+103
#define WM_LASER_SETPARAM WM_USER+104

//光阑控制消息响应
#define WM_GL_OPEN  WM_USER+105
#define WM_GL_CLOSE WM_USER+106
#define WM_GL_ADJUST WM_USER+107

//提示图片更改
#define WM_IMG_CHANGE WM_USER+108

//切换子选项卡
#define WM_TAB_CHANGE WM_USER+109
#define WM_FIT_MAINTH WM_USER+110
#define WM_CUT_PICTURE WM_USER+111
#define WM_GET_SPLMD WM_USER+112 //单点测距数据 710A

#define WM_RECVDATA WM_USER+113
#define WM_GL_ALARM WM_USER+114

#define WM_GET_711SPLMD WM_USER+115 //单点测距数据 711
#define WM_NET_RECV_711 WM_USER+116
#define WM_NET_RECV_715 WM_USER+117

#define WM_MOTOSTATE WM_USER+118  //712中光阑当前状态更新
#define WM_GL_STOPMOVE WM_USER+119  //712中停止光阑

#define  arrayXYsize  100
//#define  MK_ORG0 190
//#define  MK_ORG1 300
//激光器协议命令号
#define H02_Cmd89 0x89
#define H02_CmdA0 0xA0
#define H02_CmdA3 0xA3
#define H02_CmdA4 0xA4
#define H02_CmdA5 0x03
#define H02_CmdA8 0xA8
#define H02_CmdA9 0xA9
#define H02_CmdB2 0x01
#define H02_CmdB8 0x05
//自动修正步骤
#define STEP_IDLE  0x00 //未开启状态
#define STEP_START 0x01 //初始状态
#define STEP_PARAM 0x02   //检查参数设置是否合理
#define STEP_ORIGMK 0x03  //确保初始脉宽满足要求
#define STEP_ACQMK 0x04 //采集原始数据 依次得到各个脉宽下的原始数据
#define STEP_FITMK 0x05 //拟合插值阶段
#define STEP_DOWNTB 0x06 //下载到下位机
#define STEP_CHECKTB 0x07//检验下载的修正表
#define STEP_FINALCH 0x08 //最终检验
#define STEP_RESTART 0x09//重启激光器主控板
#define STEP_MINMK 0x0E//检查光阑关闭时脉宽为0
#define STEP_MAXMK 0x0F//检查光阑张开时脉宽大于190
#define STEP_BEFORECHECKOPEN 0x20   //一键自动验证前的张开光阑

#define TIMER_GETPARAM 0x0A
#define TIMER_GLADJUST 0x0B
#define TIMER_RESTART 0x0C
#define TIMER_GETLMD 0x0D
#define TIMER_ORIGMK 0x10
#define TIMER_INPUTDIST 0x11
#define TIMER_OPENING 0x12   //712张开计时
#define TIMER_712COMSEND 0x13   //712张开计时
#define TIMER_CLOSING 0x14   //712闭合计时

#pragma once
#include "afxcmn.h"
#include "afxwin.h"
#include <vector> 
// CLaseAutoCorrectDlg 对话框
class CLaseAutoCorrectDlg : public CDialogEx
{
// 构造
public:
	CLaseAutoCorrectDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
	enum { IDD = IDD_LASEAUTOCORRECT_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	//CIPAddressCtrl m_HostIpAddr;
	CIPAddressCtrl m_DestIp;
	UINT m_DestPort;
	CTabCtrl m_Tab;
	CRealDisplay m_prop1;
	CFitInter m_prop2;
	CComboBox m_LaserType;
	CComboBox m_PorType;
	CComboBox m_ComIndex;
	CComboBox m_ComBuand;
	CString m_sLog;
	CRichEditCtrl m_RichEdit;
	NET_CONNECTION m_NetConnection;//网络连接结构体
	UINT m_IntervNum;
	UINT m_ChartNum;
	UINT m_MkCalNum;
	UINT m_MkAvg;
	char m_cSendBufData[500];
	int m_nSendBufDataSize;
	UINT m_ApdPress;
	UINT m_ApdTemp;
	UINT m_ApdXs;
	CString m_MacAddr;
	CString m_SetMAC;
	UINT m_slopeSteps;
	CComboBox m_slopDir;
	CString m_SavePath;
	CString m_OrigDataPath;
	CString m_FitDataPath;
	CString m_OperTxtPath;
public:
	void LoadGui(void);
	void InitSysParam(void);
	void RecordVerifyMK(int CurMK);
	afx_msg void OnBnClickedButtonDisconnet();
	afx_msg void OnBnClickedButtonConnet();
	afx_msg void OnTcnSelchangeTab(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg BOOL OnDeviceChange(UINT nEventType, DWORD dwData);  
	//int GetComListFromReg( CComboBox * CCombox );
	void WriteLog(CRichEditCtrl *pREC, CString sLog, unsigned char *pbuf, unsigned short u16datalen);
	//BOOL GetHostAddress(CIPAddressCtrl *pIPAddr);
	afx_msg void OnBnClickedButtonOpencom();
	afx_msg void OnBnClickedButtonClosecom();
	afx_msg LRESULT OnRecvNetData(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnRecvNetData711(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnRecvNetData715(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnSendNetData(WPARAM wParam, LPARAM lParam);
	//激光参数查询和设置
	afx_msg LRESULT OnLaserGetParam(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnLaserSetParam(WPARAM wParam, LPARAM lParam);
	//光阑调整消息
	afx_msg LRESULT OnGlOpen(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnGlClsoe(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnGlAdjust(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnComParse(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnGlAlarm(WPARAM wParam, LPARAM lParam);
	//选项卡切换消息
	afx_msg LRESULT OnSelfChangeTab(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnFitInMainHandle(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnCutPicture(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnGet710ALMD(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnGet711LMD(WPARAM wParam, LPARAM lParam);

	afx_msg LRESULT OnWriteLog(WPARAM wParam,LPARAM lParam);
	afx_msg LRESULT OnMotoState(WPARAM wParam,LPARAM lParam);

	void HFA_ParseCmd02(char *pcBuf, int nBufSize);//查询设备参数解析
	void H02_ParseCmd54(char *pcBuf, int nBufSize);//查询设备参数解析
	void H711_ParseCmd55(char *pcBuf, int nBufSize);//查询设备参数解析
	void H711_ParseCmd68(char *pcBuf, int nBufSize);//APD参数查询解析
	void H711_ParseCmd62(char *pcBuf, int nBufSize);//生产设置下的功能参数解析
	void H715_ParseHL(char *pcBuf, int nBufSize);
	void H715_ParseAPD(char *pcBuf, int nBufSize);
	void H712_ParseAPD(char *pcBuf, int nBufSize);
	void H712_ParseMAC(char *pcBuf, int nBufSize);
	//void H712_ParseTempSate(char *pcBuf, int nBufSize);

	void H02_SetCmd54(void);//设置设备参数
	void H02_GetCmd54(void);//获取设备参数
	void H02_Restart88(void);//重启激光主控板
	void HFA_Restart0C(void);//重启710A激光主控板
	void H02_Restart47(void);//重启711激光主控板

	void HFA_GetCmd02(void);//获取设备参数
	void HFA_SetCmd02(void);//设置设备参数
	void H711_GetCmd55(void);//获取设备参数
	void H711_GetCmd68(void);//APD参数查询
	void H711_GetCmd62(void);//生产设置下的功能参数

	void H711_SetCmd55(void);//设置设备参数
	void H711_SetCmd68(void);//设置APD参数
	void H711_SetCmd65(void);//设置生产参数
	void H715_SetHL(void);
	void H715_GetAPD(void);
	void H715_GetHL(void);
	void H715_GetMAC(void);
	void H712_GetAPD(void);
	void H712_GetMAC(void);
	void H712_SetAPD(void);
	//void H712_GetState(void);

	afx_msg void OnBnClickedButtonGetparam();
	afx_msg void OnBnClickedButtonSetparam();
	char ConvertHexChar(char ch);
	void UDP_DrawWave(char *pcDataBuf,int nBufCount,int nChannel);
	void UDP_DrawWave711(char *pcDataBuf,int nBufCount,int nChannel);
	afx_msg void OnBnClickedButtonOpenslot();
	afx_msg void OnBnClickedButtonCloseslop();
	afx_msg void OnBnClickedButtonMoveslop();
	afx_msg void OnOneKeyTestCorrect();
	afx_msg void OnOnekeyTestFinish();
	afx_msg void OnBnClickedButtonAutocorrect();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	void StartTimer(UINT &nTimer,UINT nIDEvent,UINT nElapse);
	void StopTimer(UINT &nTimer);
	afx_msg void OnEnChangeEditIntevnum();
	afx_msg void OnEnChangeEditChartnum();
	afx_msg void OnEnChangeEditMkcalnum();
	long UDP_DataSum(char *pcDataBuf, int nFirstByte,int nType);
	long UDP_360DataSum711(char *pcDataBuf, int nFirstByte);
	long UDP_DataSum711(char *pcDataBuf, int nFirstByte);
	void ResetEvents(void);
	void OnCreateFolder(CString fileName);  //创建文件夹
	void OnCreateFile(CString fileName,int nType);  //创建文件
	void InitAqcDataTarget(void);
	void SaveOrigData(void);
	int FitPro(CString TxtName);
	void InitAutoCorrectTask(void);
	void DispTips(UINT nResID,CString Tips);
	bool LoadImageFromResource(IN CImage* pImage,IN UINT nResID, IN LPCSTR lpTyp);
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	afx_msg LRESULT OnChangeStep(WPARAM wParam, LPARAM lParam);
	void SendCheckCorrectTb();
	void SendCheckCorrectTb710A();
	void SendCheckCorrectTb711();
	void GetLmdSinglePoint();
	void UDP_DrawWave_710A(char *pcDataBuf, int nBufCount,int nChannel);
	void Get711LmdSinglePoint();
	afx_msg void OnCbnSelchangeComboLasertype();

	CComboBox m_ChannelSelect;
	afx_msg void OnBnClickedButton1();
	UINT m_AveError;
	UINT m_AllSingleError;
	UINT m_CheckRangeMINValue;
	afx_msg void OnBnClickedButton2();
	void TraverseDir(CString& dir, std::vector<CString>& vec);
	afx_msg void OnBnClickedButton3();
	UINT m_InitialSpeed;
	UINT m_WorkingSpeed;
	CString m_CurrentState;
	afx_msg void OnBnClickedButtonSetinispeed();
	afx_msg void OnBnClickedButtonSetworkspeed();
	afx_msg void OnBnClickedButtonGetstate();
	float m_RealDist;
	float m_APDStateTemp;
	afx_msg void OnBnClickedButtonDistconfirm();
protected:
	afx_msg LRESULT OnGlStopmove(WPARAM wParam, LPARAM lParam);
public:
	afx_msg void OnEnChangeEditAveerror();
	afx_msg void OnEnChangeEditAllsingleerror();
	afx_msg void OnEnChangeEditCheckrangeminvalue();
};
