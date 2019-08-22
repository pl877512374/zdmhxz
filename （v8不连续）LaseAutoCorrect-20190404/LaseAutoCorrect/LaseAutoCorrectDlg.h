
// LaseAutoCorrectDlg.h : ͷ�ļ�
//
#include "RealDisplay.h"
#include "FitInter.h"

#include "Utils/NetWork/NetworkUtils.h"

#define WM_NET_RECV WM_USER+100
#define WM_NET_SEND  WM_USER+101
#define WM_COM_PARSE  WM_USER+102

//���������ѯ����������Ϣ��Ӧ 
#define WM_LASER_GETPARAM WM_USER+103
#define WM_LASER_SETPARAM WM_USER+104

//����������Ϣ��Ӧ
#define WM_GL_OPEN  WM_USER+105
#define WM_GL_CLOSE WM_USER+106
#define WM_GL_ADJUST WM_USER+107

//��ʾͼƬ����
#define WM_IMG_CHANGE WM_USER+108

//�л���ѡ�
#define WM_TAB_CHANGE WM_USER+109
#define WM_FIT_MAINTH WM_USER+110
#define WM_CUT_PICTURE WM_USER+111
#define WM_GET_SPLMD WM_USER+112 //���������� 710A

#define WM_RECVDATA WM_USER+113
#define WM_GL_ALARM WM_USER+114

#define WM_GET_711SPLMD WM_USER+115 //���������� 711
#define WM_NET_RECV_711 WM_USER+116
#define WM_NET_RECV_715 WM_USER+117

#define WM_MOTOSTATE WM_USER+118  //712�й�����ǰ״̬����
#define WM_GL_STOPMOVE WM_USER+119  //712��ֹͣ����

#define  arrayXYsize  100
//#define  MK_ORG0 190
//#define  MK_ORG1 300
//������Э�������
#define H02_Cmd89 0x89
#define H02_CmdA0 0xA0
#define H02_CmdA3 0xA3
#define H02_CmdA4 0xA4
#define H02_CmdA5 0x03
#define H02_CmdA8 0xA8
#define H02_CmdA9 0xA9
#define H02_CmdB2 0x01
#define H02_CmdB8 0x05
//�Զ���������
#define STEP_IDLE  0x00 //δ����״̬
#define STEP_START 0x01 //��ʼ״̬
#define STEP_PARAM 0x02   //�����������Ƿ����
#define STEP_ORIGMK 0x03  //ȷ����ʼ��������Ҫ��
#define STEP_ACQMK 0x04 //�ɼ�ԭʼ���� ���εõ����������µ�ԭʼ����
#define STEP_FITMK 0x05 //��ϲ�ֵ�׶�
#define STEP_DOWNTB 0x06 //���ص���λ��
#define STEP_CHECKTB 0x07//�������ص�������
#define STEP_FINALCH 0x08 //���ռ���
#define STEP_RESTART 0x09//�������������ذ�
#define STEP_MINMK 0x0E//�������ر�ʱ����Ϊ0
#define STEP_MAXMK 0x0F//�������ſ�ʱ�������190
#define STEP_BEFORECHECKOPEN 0x20   //һ���Զ���֤ǰ���ſ�����

#define TIMER_GETPARAM 0x0A
#define TIMER_GLADJUST 0x0B
#define TIMER_RESTART 0x0C
#define TIMER_GETLMD 0x0D
#define TIMER_ORIGMK 0x10
#define TIMER_INPUTDIST 0x11
#define TIMER_OPENING 0x12   //712�ſ���ʱ
#define TIMER_712COMSEND 0x13   //712�ſ���ʱ
#define TIMER_CLOSING 0x14   //712�պϼ�ʱ

#pragma once
#include "afxcmn.h"
#include "afxwin.h"
#include <vector> 
// CLaseAutoCorrectDlg �Ի���
class CLaseAutoCorrectDlg : public CDialogEx
{
// ����
public:
	CLaseAutoCorrectDlg(CWnd* pParent = NULL);	// ��׼���캯��

// �Ի�������
	enum { IDD = IDD_LASEAUTOCORRECT_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��


// ʵ��
protected:
	HICON m_hIcon;

	// ���ɵ���Ϣӳ�亯��
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
	NET_CONNECTION m_NetConnection;//�������ӽṹ��
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
	//���������ѯ������
	afx_msg LRESULT OnLaserGetParam(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnLaserSetParam(WPARAM wParam, LPARAM lParam);
	//����������Ϣ
	afx_msg LRESULT OnGlOpen(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnGlClsoe(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnGlAdjust(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnComParse(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnGlAlarm(WPARAM wParam, LPARAM lParam);
	//ѡ��л���Ϣ
	afx_msg LRESULT OnSelfChangeTab(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnFitInMainHandle(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnCutPicture(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnGet710ALMD(WPARAM wParam, LPARAM lParam);
	afx_msg LRESULT OnGet711LMD(WPARAM wParam, LPARAM lParam);

	afx_msg LRESULT OnWriteLog(WPARAM wParam,LPARAM lParam);
	afx_msg LRESULT OnMotoState(WPARAM wParam,LPARAM lParam);

	void HFA_ParseCmd02(char *pcBuf, int nBufSize);//��ѯ�豸��������
	void H02_ParseCmd54(char *pcBuf, int nBufSize);//��ѯ�豸��������
	void H711_ParseCmd55(char *pcBuf, int nBufSize);//��ѯ�豸��������
	void H711_ParseCmd68(char *pcBuf, int nBufSize);//APD������ѯ����
	void H711_ParseCmd62(char *pcBuf, int nBufSize);//���������µĹ��ܲ�������
	void H715_ParseHL(char *pcBuf, int nBufSize);
	void H715_ParseAPD(char *pcBuf, int nBufSize);
	void H712_ParseAPD(char *pcBuf, int nBufSize);
	void H712_ParseMAC(char *pcBuf, int nBufSize);
	//void H712_ParseTempSate(char *pcBuf, int nBufSize);

	void H02_SetCmd54(void);//�����豸����
	void H02_GetCmd54(void);//��ȡ�豸����
	void H02_Restart88(void);//�����������ذ�
	void HFA_Restart0C(void);//����710A�������ذ�
	void H02_Restart47(void);//����711�������ذ�

	void HFA_GetCmd02(void);//��ȡ�豸����
	void HFA_SetCmd02(void);//�����豸����
	void H711_GetCmd55(void);//��ȡ�豸����
	void H711_GetCmd68(void);//APD������ѯ
	void H711_GetCmd62(void);//���������µĹ��ܲ���

	void H711_SetCmd55(void);//�����豸����
	void H711_SetCmd68(void);//����APD����
	void H711_SetCmd65(void);//������������
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
	void OnCreateFolder(CString fileName);  //�����ļ���
	void OnCreateFile(CString fileName,int nType);  //�����ļ�
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
