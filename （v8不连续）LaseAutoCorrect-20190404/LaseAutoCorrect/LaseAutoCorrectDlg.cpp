
// LaseAutoCorrectDlg.cpp : ʵ���ļ�
//

#include "stdafx.h"
#include "LaseAutoCorrect.h"
#include "LaseAutoCorrectDlg.h"
#include "afxdialogex.h"
#include "Utils/NetWork/NetworkUtils.h"
//#include "Utils/SerialPort/CommonCom.h"
#include "Utils/SerialPort/SerialUtils.h"
//#include "Utils/File/FileUtils.h"
#include "Utils/File/CDMLogInfo.h"
#include "Utils/TeeChart/axis.h"
#include "Utils/TeeChart/axes.h"
#include "Utils/TeeChart/scroll.h"
#include "Utils/TeeChart/series.h"
#include "Utils/TeeChart/valuelist.h"
#include "Utils/TeeChart/toollist.h"
#include "Utils/TeeChart/tools.h"
#include "Utils/TeeChart/annotationtool.h"
#include "Utils/TeeChart/zoom.h"
#include "Utils/TeeChart/titles.h"
#include "Utils/TeeChart/export.h"
#include "PID.h"
#include "CRC16.h"
#include "Interp.h"
#include <fstream>
#include "io.h"
using namespace std;
#import "c:\program files\common files\system\ado\msado15.dll"  no_namespace rename("EOF", "adoEOF")
#include <string>
#include <tchar.h>
#include<iostream>

#include <afxinet.h>
#ifdef _DEBUG
#define new DEBUG_NEW
#endif

extern int xorflag;
int okAvg;
typedef struct tagCheckMKForm
{
	int ChMkMin;//����¼���������ֵ
	int ChMkMax;//����¼��������Сֵ
	int ChMkOffset;//����¼������ƫ�Χ
	int ReferenceValue;//�ο���ֵ
	int ValueJudgeCnt;//���ڼ���ë�̵㣬��������ֵ����300�ĵ�
	int ValueJudge;//����֤�е����Ƿ�ϸ���ж�
	int AveValueJudgeCnt;//���ڼ�����ֵ�����֡��
	int AveValueJudge;//������֤�о�ֵ�Ƿ�ϸ���ж�
	int ChMkRegionMin;//��֤��������ֵ����Сֵ
	int ChMkRegionMax;//��֤��������ֵ�����ֵ
	int ChMkGetReference;//��֤�ο���׼ֵ��ȡ����С���������ڴ�ֵ������ȫ����ƽ���õ��ο�ֵ
	int AveValueError;//ƽ��ֵ��֤��ʽ�е�������ı�׼ֵ
	int SingleValueError;//������֤��ʽ�е�������ı�׼ֵ
	int SingleErrorMAX; //��������֤����в�ֵ�����ֵ
	int SingleErrorMIN; //��������֤����в�ֵ����Сֵ
}CheckMKForm;
CheckMKForm g_CheckMKForm;

typedef struct tagSysDevParam
{
	UINT8 LaserType;
	BOOL NetConnected;
	UINT8 LaserPor;
	int nPort;
	int nBuad;
	BOOL ComConnected;
}SysDevParam;
SysDevParam g_SysDevParam;

typedef struct tagSYSParameter
{
	UINT16	u16FlashProgramFlag;		//Flash������Ч��־λ������λΪFlashDataEnableʱ��������Ч
	UINT16	u16FlashDataLength;		//Flash����Ч���ݣ����֣��ĳ��ȣ�����Flash������Ч��־λ

	UINT16 	u16NCdata[10];			//Ԥ��20���ֽڣ�

	UINT16	u16Div_AngleNum;		//����һ�ܱ��ȷ�Ϊ���ٵȷݣ�720����0.5�Ȳ���
	//���Ϊ0.5�Ȳ�����ÿȦ����720�����塣36K
	UINT16	u16Dot_PerAngle;			//����ÿ���ٲ�ɨ��һ����.��������Ƶ��Ϊ12Kʱ������Ϊ3,36KʱΪ1

	UINT16  u16ZeroStart;				//�����ʼλ��
	UINT16  u16ZeroEnd;				//�����ֹλ��
	UINT16	u16ZeroBufLengrh;		//�����Ч���ݳ���

	UINT16	u16ZeroLength;			//�����ʱȡ���ٸ�����ƽ��
	UINT16	u16ZeroTimerMin;			//Ѱ�������Сֵ�����ޣ����ڸ�ֵ��Ϊ������Сֵ�㡣��λpS

	UINT16	u16ScanStart;				//ɨ����ʼλ��
	UINT16	u16ScanEnd;				//ɨ����ֹλ��
	UINT16	u16ScanBufLength;		//ɨ��ĵ�����

	UINT16	u16APDHvValue;		    //APD��ѹֵ��
	UINT16	u16APDTemperValue;		//APD�¶�ֵ��
	UINT16	u16APDHV_OP_Ratio;		//APD��ѹϵ������/1000��

	UINT16 	u16NCdata1[7];			//Ԥ��14���ֽڣ�

	//���ڴ������
	UINT16 	u16My_Phy_Addr[6];		//�����ַ
	UINT16 	u16My_Sub_Mask[4]; 		//��������
	UINT16 	u16My_IP_Addr[4];		//�豸IP��ַ
	UINT16	u16My_Gateway_IP[4];		//���ص�ַ

	UINT16	u16My_port;				//�豸��ʹ�õĶ˿ں�

	UINT16 	u16Ser_Ip[4];				//������IP��ַ
	UINT16	u16Ser_port;				//��������ʹ�õĶ˿ں�
} SYSParameter;
SYSParameter LaserParam;

typedef struct tagSYSParameter711
{
//ϵͳ��Ϣ
	UINT16 m_u16EquipNo[2];
	UINT16 m_u16SysMode_SingleOrScanf;
	// APD���� 
	UINT16 m_u16APDVSet;								//�������Ե�ѹ����to fpga
	UINT16 m_u16APDTemperValue;					//APD�¶�ֵ
	UINT16 m_u16APDHV_OP_Ratio;					//APD��ѹϵ��(/1000)	
	
	UINT16 m_u16ZeroDisc;
	UINT16 m_u16OverallDisDif;//����ƫ�ƾ���	
	UINT16 m_u16OverallDisDif_L;

	UINT16 m_u16Channel_Sel;	
}SYSParameter711;
SYSParameter711 LaserParam711;

typedef struct tagSYSParameter712
{
	// APD���� 
	UINT16 m_u16APDVSet;								//�������Ե�ѹ����to fpga
	UINT16 m_u16APDTemperValue;					//APD�¶�ֵ
	UINT16 m_u16APDHV_OP_Ratio;					//APD��ѹϵ��(/1000)	
}SYSParameter712;
SYSParameter712 LaserParam712;

typedef struct tagAutoCorretInfo
{
	UINT8 nStepIndx;//�������в�������
	UINT8 nTaskCode;//00 δ���� ;1 �����ɹ�; 2 �Զ���������
	UINT8 nParamChCode;//01�������ͨ��  02 �������ʧ�� 03������鳬ʱ  
	UINT8 nOrigMkChCode;//01��ʼ������ͨ�� 02��ʼ������δͨ��  03 ��ʼ�����鳬ʱ 04 �����ſ���� 05 �����ſ�ʧ�� 06 �����ſ���ʱ
	UINT8 nAqcMkCode;//01�ɼ�����ԭʼ������� 02�ɼ�����ԭʼʱ��Ҫ��������;03 �ɼ�ԭʼ��������ʱ����������� 04 �ɼ�ԭʼ����ʱ����������λ��ֱ��ͳ�Ƽ��� 05 ����������ʱ 
	UINT8 nFitMkCode;//01�������ɹ� 02�������ʧ��
	UINT8 nDownTbCode;//01���������سɹ� 02����������ʧ�� 03���������س�ʱ
	UINT8 nCheckTbCode;//01��������������ȷ��ͨ��  02��������������ȷ��δͨ�� 03 ����������������
	UINT8 nChDownTbCnt;//���������ʧ��ʱ ��������������Ĵ��� �������������֤������ ����ֹ
	UINT8 nFinalCkCode;//01����һ����֤ͨ�� 02����һ����֤δͨ�� 03 ��λ��׼���׵����Բ�����  04 �������ú�����ر�   05 �رչ�����ʱ
	UINT8 nNetReSendCnt;//�����ط�����
	UINT8 nComReSendCnt;//�����ط�����
	UINT8 nWaitInputDistCnt; //�ȴ����������Ϣ����
}AutoCorretInfo;

AutoCorretInfo AutoCorretRegInfo;
int BeginValue=0;  //��¼��ȡ��ʼ������ֵ

typedef struct tagAqcTarget
{
	int SpCnt;
	int KeyValue[43];
	int DevValue[43];
	int AqcDone[43];
	int Step[43];
	int AqcIndx;
}AqcTarget;

AqcTarget g_AcqTartetInfo;
PID g_Pid;
char xitongcanshu[50]={0};
char netmac[21]={0};
//���ڷ���ָ��
unsigned char a_ucTemp1[13] = {0xFF,0xAA,0x00,0x0A,0x09,0x01,0xAA,0x00,0x00,0x00,0x93,0x20,0xEE};              //�ſ�����
unsigned char a_ucTemp2[13] = {0xFF,0xAA,0x00,0x0A,0x09,0x02,0xAA,0x00,0x00,0x00,0x7D,0xF2,0xEE};              //�պϹ���
unsigned char a_ucTemp3[15] = {0xFF,0xAA,0x00,0x0C,0x09,0x03,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xEE};    //΢������
unsigned char a_ucTemp4[12] = {0xFF,0xAA,0x00,0x09,0x09,0x04,0xAA,0x00,0x00,0x50,0x32,0xEE};                   //һ����֤
unsigned char a_ucTemp5[12] = {0xFF,0xAA,0x00,0x09,0x09,0x05,0xAA,0x00,0x00,0x26,0x86,0xEE};                   //��֤����
unsigned char a_ucTemp6[13] = {0xFF,0xAA,0x00,0x0A,0x09,0x06,0xAA,0x00,0x00,0x00,0xF4,0xF4,0xEE};              //�������ٹر�
unsigned char a_ucTemp7[13] = {0xFF,0xAA,0x00,0x0A,0x09,0x07,0xAA,0x00,0x00,0x00,0x5E,0xA5,0xEE};              //���������ſ�
unsigned char a_ucTemp8[12] = {0xFF,0xAA,0x00,0x09,0x09,0x08,0xAA,0x00,0x00,0x1F,0x00,0xEE};              //����ָ��
//712ʹ��
unsigned char a_ucTemp9[9] = {0x01,0x27,0x04,0x00,0x00,0x00,0x00,0x00,0x00};   //΢������
unsigned char a_ucTemp10[6] = {0x01,0x28,0x01,0x10,0x00,0x3A};   //�����ſ��˶�
unsigned char a_ucTemp11[6] = {0x01,0x28,0x01,0x20,0x00,0x4A};   //�����պ��˶�
unsigned char a_ucTemp12[6] = {0x01,0x29,0x01,0x20,0x00,0x4B};   //��ֹͣ�˶�
unsigned char a_ucTemp13[9] = {0x01,0x23,0x04,0x00,0x00,0x00,0x00,0x00,0x00};   //���ó��ٶ�
unsigned char a_ucTemp14[9] = {0x01,0x24,0x04,0x00,0x00,0x00,0x00,0x00,0x00};   //���������ٶ�
unsigned char a_ucTemp15[5] = {0x01,0x30,0x00,0x00,0x31};   //��ѯ����״̬

// MFC�ٽ��������
CCriticalSection g_clsCriticalSection;
CRITICAL_SECTION g_netcs;

//����ԭʼ���ݻ�������
#define JGMAX_CNT 200
#define JGMAX_LEN 20000
char g_nJGdata[3][JGMAX_CNT][JGMAX_LEN]={0};
int g_nJGRecCnt=0;
int g_nJGProCnt=0;
int g_nMatch=0;

//����Net�����õ�ȫ�ֻ���
char g_nNetParseBuff[10000]={0};
int  g_nNetParseBuffLen=0;
BYTE g_nComParseBuff[200]={0};
int g_nComParseBuffLen=0;
char g_nNetSendBuff[200]={0};
int g_nNetSendBuffLen=0;
bool b711AFirstClose=true;
//711��
int g_ChannelSelect = 0; //ͨ��ѡ�񣬷�Ϊ����ֵ�͵���ֵ
bool g_CheckAPD=false;
int jg1=0;
int j2=0;
int j3=0;
char ShaoXie_zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x0D,0x00,0x00};
int shaoxie=34;

//�����ж��������ֵ�Ƿ�����Ҫ��
int  MK_ORG0=190;
int  MK_ORG1=300;

//712��
//int g_nAPDStateTemp=-100;
int g_nReferUpEdge = 0;  //���ݾ������Ĳο���ʱֵ
bool g_bMoveFlag = false; //�����Ƿ��˶���־λ
bool g_bOpenflag = false;  //�ſ��ж��Ƿ����
bool g_bCloseflag = false;  //�ر��ж��Ƿ���С
int g_nComComandStat = 0;   //���ڷ��͵�ָ��״̬��0:��״̬��1:�ſ�״̬��2:ֹͣ״̬; 3:�ر�״̬
int g_nContinuousMoveState = 0;   //�����˶���־λ����Ӧָֹͣ��֮ǰ��״̬��  0:��״̬�� 1���ſ�״̬�� 2���ر�״̬
bool g_bCloseSpeedChange = false;    //�ж��Ƿ���Ҫ�ı�ر��ٶ�  ture����ʾ�ȴ��ı��ٶ�

//�������߳�
UINT WriteData(LPVOID lpParam);
UINT ConnectThreadProc(LPVOID lpParam);
UINT DisConThreadProc(LPVOID lpParam);
void OnNetRecv(LPVOID lpParam, char *pDataBuf, int nDataBufSize);
//UINT ComParseThread(LPVOID lpParam);
BOOL bComParseTerm=FALSE;

UINT AutoCheckThreadProc(LPVOID lpParam);  //������
UINT AutoCorrThreadProc(LPVOID lpParam);
BOOL bAutoCorrTerm=FALSE;
UINT TargetDataAqcThread(LPVOID lpParam);
BOOL bTargetAqcTerm=FALSE;

UINT AqcOrgThreadProc(LPVOID lpParam);
BOOL bAqcOrgTerm=FALSE;


int g_nReadbuf[20000] = {0};  //��д����
int g_nReadbuflen = 0; //ʵ�ʳ���
int g_nCurMKValue[800] = {0};//�洢��ǰ�ڴ��������������ֵ

void OnComRecData(LPVOID lpParam, BYTE *pDataBuf, int nDataBufSize);
BOOL Com_Send(BYTE *out_buff,int len);

UINT DownLoadCorrectTableThread(LPVOID lpParam);
int SetPacket(char* sendpacket,int* datapacket,int len,int num);  //Դ���ݵĳ��� ����
int SetPacket711(char index,char* sendpacket,int* datapacket,int len,int num);  //Դ���ݵĳ��� ����
int SetPacket712(char* sendpacket,int* datapacket,int len,int num);  //Դ���ݵĳ��� ����
short bccW_packs=0;//711�������У��
int SetPacket710A(char* sendpacket,int* datapacket,int len,int num);  //Դ���ݵĳ��� ����
UINT TIMERGETPARMA=NULL;
UINT TIMERGLADJUST=NULL;
UINT TIMERRESTART=NULL;
UINT TIMERGETLMD=NULL;
UINT TIMERORIGMK=NULL;
UINT TIMERINPUTDIST=NULL;
UINT TIMEROPENING=NULL;
UINT TIMER712COMSEND=NULL;
UINT TIMERCLOSING=NULL;
CTime TimerRestart;

int Recv710AMKDataCnt;//��¼����710A�������ݵĴ�����3��û�ӵ���Ҫ�ط���ȡָ��
int Recv711MKDataCnt;//��¼����711�������ݵĴ�����3��û�ӵ���Ҫ�ط���ȡָ��

int abcd=0;

//���廭ͼ��صı���
int g_nRevcNum1 = 0 ;  //�յ���1�Ű����� 
int g_nRevcNum2 = 0 ;  //�յ���2�Ű�����
int g_nRevcNum3 = 0 ;  //�յ���3�Ű�����
long g_nRevcNumcCheck = 0;
long g_nRevcNumcCheck1 = 0;
long g_nRevcNumcCheck2 = 0;
long g_nRevcNumcCheck3=0;//���byYSS
int avgdata(0),avgdata1(0),avgdata2(0);  

char g_recv1[2000]={0};
int  g_recv1len=0;
char g_recv2[2000]={0};
int  g_recv2len=0;
char g_recv3[2000]={0};
int  g_recv3len=0;

int g_bCountStart=0;//ͳ�Ƽ�������������ֵ��ʶλ

int g_nCacuNum1=0;
int g_nCacuNum2=0;
long  g_nAvgStop1 = 0L; //ÿn����������ƽ��ֵ
long g_nAvgStop2 = 0; //ÿn�����½���ƽ��ֵ
long g_nSumStop1 = 0; //ÿn���������صĺ�
long g_nSumStop2 = 0; //ÿn�����½��صĺ�
int g_nReviseNum = 0;  //ÿ�������������
int g_nFirstData = 0;//��һ��������������
int g_nInterpNum = 0;  //������ϵ������С
CArray<int,int> g_array1; //�����������
CArray<int,int> g_array2; //��������׼��ֵ
int g_datax[arrayXYsize] = {0};//��ȡ��Ͻ���������
int g_datay[arrayXYsize] = {0};
int g_nXY[100][2] = {0};  //��ʱ�������
int g_nXYlen = 0;     //����ĳ���


CEvent Event_START;//��ʼ�Զ�����
CEvent Event_PARAM;//�������
CEvent Even_ORIGMK;//��ʼ������;
CEvent Event_AQCMK;//����ɼ�
CEvent Event_ALIVE;//�����ֹ��ʱ
CEvent Event_FITMK;//������ϲ�ֵ
CEvent Event_DOWNTB;//����������
CEvent Event_CHECKTB;//���������
CEvent Event_FINALCK;//���ռ��
CEvent Event_OVEROPEN; //����Զ�����ǰ�ſ�����
CEvent Event_STOP; //ֹͣ�Զ�����
HANDLE Event_List[11] = {Event_START,Event_PARAM,Even_ORIGMK,Event_AQCMK,Event_ALIVE,Event_FITMK,Event_DOWNTB,Event_CHECKTB,Event_FINALCK,Event_OVEROPEN,Event_STOP};

HANDLE g_eventdl = CreateEvent(NULL,TRUE,FALSE,NULL);//�����¼�
int g_nNum = 0;
BOOL g_flag = FALSE;
CCDMLogInfo* m_pMyLog = NULL;
std::vector<CString> filenames;
CString g_cover="1";
BOOL g_nameadd=FALSE;


// ����Ӧ�ó��򡰹��ڡ��˵���� CAboutDlg �Ի���

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// �Ի�������
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

// ʵ��
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CLaseAutoCorrectDlg �Ի���

CLaseAutoCorrectDlg::CLaseAutoCorrectDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CLaseAutoCorrectDlg::IDD, pParent)
	, m_sLog(_T(""))
	,m_SavePath(_T(""))
	, m_DestPort(0)
	, m_IntervNum(0)
	, m_ChartNum(0)
	, m_MkCalNum(0)
	, m_MkAvg(0)
	, m_ApdPress(0)
	, m_ApdTemp(0)
	, m_ApdXs(0)
	, m_MacAddr(_T(""))
	,m_OrigDataPath(_T(""))
	,m_FitDataPath(_T(""))
	, m_slopeSteps(0)
	,m_OperTxtPath(_T(""))
	, m_SetMAC(_T("001234000"))
	, m_AveError(0)
	, m_AllSingleError(0)
	, m_CheckRangeMINValue(0)
	, m_InitialSpeed(500)
	, m_WorkingSpeed(1500)
	, m_CurrentState(_T(""))
	, m_RealDist(0)
	, m_APDStateTemp(0)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CLaseAutoCorrectDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_TAB, m_Tab);
	DDX_Control(pDX, IDC_COMBO_LaserType, m_LaserType);
	DDX_Control(pDX, IDC_COMBO_ProtType, m_PorType);
	DDX_Control(pDX, IDC_COMBO_ComIndex, m_ComIndex);
	DDX_Control(pDX, IDC_COMBO_ComBaund, m_ComBuand);
	DDX_Control(pDX, IDC_RICHEDIT_DATADISPLAY, m_RichEdit);
	DDX_Control(pDX, IDC_IPADDRESSDest, m_DestIp);
	DDX_Text(pDX, IDC_EDIT_DestPort, m_DestPort);
	DDX_Text(pDX, IDC_EDIT_IntevNum, m_IntervNum);
	DDX_Text(pDX, IDC_EDIT_ChartNum, m_ChartNum);
	DDX_Text(pDX, IDC_EDIT_MkCalNum, m_MkCalNum);
	DDX_Text(pDX, IDC_EDIT_MkAvg, m_MkAvg);
	DDX_Text(pDX, IDC_EDIT_ApdPress, m_ApdPress);
	DDX_Text(pDX, IDC_EDIT_ApdTemp, m_ApdTemp);
	DDX_Text(pDX, IDC_EDIT_ApdXs, m_ApdXs);
	DDX_Text(pDX, IDC_EDIT_MacAddr, m_MacAddr);
	DDX_Text(pDX, IDC_EDIT_SlopSteps, m_slopeSteps);
	DDX_Control(pDX, IDC_COMBO_SlopDir, m_slopDir);
	DDX_Text(pDX, IDC_EDIT_MACSET, m_SetMAC);
	DDX_Control(pDX, IDC_COMBO_ChannelSelect, m_ChannelSelect);
	DDX_Text(pDX, IDC_EDIT_AveError, m_AveError);
	DDX_Text(pDX, IDC_EDIT_AllSingleError, m_AllSingleError);
	DDX_Text(pDX, IDC_EDIT_CheckRangeMINValue, m_CheckRangeMINValue);
	DDX_Text(pDX, IDC_EDIT_InitialSpeed, m_InitialSpeed);
	DDX_Text(pDX, IDC_EDIT_WorkingSpeed, m_WorkingSpeed);
	DDX_Text(pDX, IDC_EDIT_CurrentState, m_CurrentState);
	DDX_Text(pDX, IDC_EDIT_RealDist, m_RealDist);
	DDX_Text(pDX, IDC_EDIT_APDStateTemp, m_APDStateTemp);
}

BEGIN_MESSAGE_MAP(CLaseAutoCorrectDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_DEVICECHANGE()
	ON_WM_QUERYDRAGICON()
	ON_MESSAGE(WM_NET_RECV,OnRecvNetData)
	ON_MESSAGE(WM_NET_RECV_711,OnRecvNetData711)
	ON_MESSAGE(WM_NET_RECV_715,OnRecvNetData715)
	ON_MESSAGE(WM_NET_SEND,OnSendNetData)
	ON_MESSAGE(WM_COM_PARSE,OnComParse)
	ON_MESSAGE(WM_LASER_GETPARAM,OnLaserGetParam)
	ON_MESSAGE(WM_LASER_SETPARAM,OnLaserSetParam)
	ON_MESSAGE(WM_GL_OPEN,OnGlOpen)
	ON_MESSAGE(WM_GL_CLOSE,OnGlClsoe)
	ON_MESSAGE(WM_GL_ADJUST,OnGlAdjust)
	ON_MESSAGE(WM_GL_ALARM,OnGlAlarm)
	ON_MESSAGE(WM_IMG_CHANGE,OnChangeStep)
	ON_MESSAGE(WM_TAB_CHANGE,OnSelfChangeTab)
	ON_MESSAGE(WM_FIT_MAINTH,OnFitInMainHandle)
	ON_MESSAGE(WM_CUT_PICTURE,OnCutPicture)
	ON_MESSAGE(WM_GET_SPLMD,OnGet710ALMD)
	ON_MESSAGE(WM_GET_711SPLMD,OnGet711LMD)
	ON_MESSAGE(WM_RECVDATA,OnWriteLog)
	ON_MESSAGE(WM_MOTOSTATE,OnMotoState)
	
	ON_BN_CLICKED(IDC_BUTTON_DisConNet, &CLaseAutoCorrectDlg::OnBnClickedButtonDisconnet)
	ON_BN_CLICKED(IDC_BUTTON_ConNet, &CLaseAutoCorrectDlg::OnBnClickedButtonConnet)
	ON_NOTIFY(TCN_SELCHANGE, IDC_TAB, &CLaseAutoCorrectDlg::OnTcnSelchangeTab)
	ON_BN_CLICKED(IDC_BUTTON_OpenCom, &CLaseAutoCorrectDlg::OnBnClickedButtonOpencom)
	ON_BN_CLICKED(IDC_BUTTON_CloseCom, &CLaseAutoCorrectDlg::OnBnClickedButtonClosecom)
	ON_BN_CLICKED(IDC_BUTTON_GetParam, &CLaseAutoCorrectDlg::OnBnClickedButtonGetparam)
	ON_BN_CLICKED(IDC_BUTTON_SetParam, &CLaseAutoCorrectDlg::OnBnClickedButtonSetparam)
	ON_BN_CLICKED(IDC_BUTTON_OpenSlot, &CLaseAutoCorrectDlg::OnBnClickedButtonOpenslot)
	ON_BN_CLICKED(IDC_BUTTON_CloseSlop, &CLaseAutoCorrectDlg::OnBnClickedButtonCloseslop)
	ON_BN_CLICKED(IDC_BUTTON_MoveSlop, &CLaseAutoCorrectDlg::OnBnClickedButtonMoveslop)
	ON_BN_CLICKED(IDC_BUTTON_AutoCorrect, &CLaseAutoCorrectDlg::OnBnClickedButtonAutocorrect)
	ON_WM_TIMER()
	ON_EN_CHANGE(IDC_EDIT_IntevNum, &CLaseAutoCorrectDlg::OnEnChangeEditIntevnum)
	ON_EN_CHANGE(IDC_EDIT_ChartNum, &CLaseAutoCorrectDlg::OnEnChangeEditChartnum)
	ON_EN_CHANGE(IDC_EDIT_MkCalNum, &CLaseAutoCorrectDlg::OnEnChangeEditMkcalnum)
	ON_WM_CTLCOLOR()
	ON_WM_ERASEBKGND()
	ON_CBN_SELCHANGE(IDC_COMBO_LaserType, &CLaseAutoCorrectDlg::OnCbnSelchangeComboLasertype)
	ON_BN_CLICKED(IDC_BUTTON1, &CLaseAutoCorrectDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON2, &CLaseAutoCorrectDlg::OnBnClickedButton2)
	ON_BN_CLICKED(IDC_BUTTON3, &CLaseAutoCorrectDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON_SetIniSpeed, &CLaseAutoCorrectDlg::OnBnClickedButtonSetinispeed)
	ON_BN_CLICKED(IDC_BUTTON_SetWorkSpeed, &CLaseAutoCorrectDlg::OnBnClickedButtonSetworkspeed)
	ON_BN_CLICKED(IDC_BUTTON_GetState, &CLaseAutoCorrectDlg::OnBnClickedButtonGetstate)
	ON_BN_CLICKED(IDC_BUTTON_DISTCONFIRM, &CLaseAutoCorrectDlg::OnBnClickedButtonDistconfirm)
	ON_MESSAGE(WM_GL_STOPMOVE, &CLaseAutoCorrectDlg::OnGlStopmove)
	ON_EN_CHANGE(IDC_EDIT_AveError, &CLaseAutoCorrectDlg::OnEnChangeEditAveerror)
	ON_EN_CHANGE(IDC_EDIT_AllSingleError, &CLaseAutoCorrectDlg::OnEnChangeEditAllsingleerror)
	ON_EN_CHANGE(IDC_EDIT_CheckRangeMINValue, &CLaseAutoCorrectDlg::OnEnChangeEditCheckrangeminvalue)
END_MESSAGE_MAP()


// CLaseAutoCorrectDlg ��Ϣ�������

BOOL CLaseAutoCorrectDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// ��������...���˵�����ӵ�ϵͳ�˵��С�

	// IDM_ABOUTBOX ������ϵͳ���Χ�ڡ�
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// ���ô˶Ի����ͼ�ꡣ��Ӧ�ó��������ڲ��ǶԻ���ʱ����ܽ��Զ�
	//  ִ�д˲���
	SetIcon(m_hIcon, TRUE);			// ���ô�ͼ��
	SetIcon(m_hIcon, FALSE);		// ����Сͼ��

	LoadGui();
	InitSysParam();
	// TODO: �ڴ���Ӷ���ĳ�ʼ������

	return TRUE;  // ���ǽ��������õ��ؼ������򷵻� TRUE
}

void CLaseAutoCorrectDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// �����Ի��������С����ť������Ҫ����Ĵ���
//  �����Ƹ�ͼ�ꡣ����ʹ���ĵ�/��ͼģ�͵� MFC Ӧ�ó���
//  �⽫�ɿ���Զ���ɡ�

void CLaseAutoCorrectDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // ���ڻ��Ƶ��豸������

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// ʹͼ���ڹ����������о���
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// ����ͼ��
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//���û��϶���С������ʱϵͳ���ô˺���ȡ�ù��
//��ʾ��
HCURSOR CLaseAutoCorrectDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CLaseAutoCorrectDlg::OnBnClickedButtonDisconnet()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	UpdateData(TRUE);
	Recv710AMKDataCnt=0;
	Recv711MKDataCnt=0;
	CWinThread *pThread;
	pThread = ::AfxBeginThread(DisConThreadProc,this);//���������߳�
	if(pThread == NULL)
	{
		return;
	}
}


void CLaseAutoCorrectDlg::OnBnClickedButtonConnet()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	UpdateData(TRUE);
	memset(g_nJGdata,0,sizeof(g_nJGdata));
	CWinThread *pThread;
	pThread = ::AfxBeginThread(ConnectThreadProc,this);//���������߳�
	if(pThread == NULL)
	{
		return;
	}
}

void CLaseAutoCorrectDlg::LoadGui()
{
	CRect rc;
	m_Tab.InsertItem(0,"ʵʱ��ʾ");
	m_Tab.InsertItem(1,"��ϲ�ֵ");

	m_prop1.Create(IDD_DIALOG_REALDISPLAY, &m_Tab);
	m_prop2.Create(IDD_DIALOG_PolyFIt,&m_Tab);

	m_Tab.GetClientRect(rc);
	rc.top += 22;
	rc.bottom -= 2;
	rc.left += 2;
	rc.right -= 3;

	m_prop1.MoveWindow(&rc);
	m_prop2.MoveWindow(&rc);

	m_prop1.ShowWindow(SW_SHOW);
	m_prop2.ShowWindow(SW_HIDE);
}

void CLaseAutoCorrectDlg::OnTcnSelchangeTab(NMHDR *pNMHDR, LRESULT *pResult)
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	*pResult = 0;
	int nSel=0;
	nSel=m_Tab.GetCurSel();
	switch(nSel)
	{
	case 0:
		m_prop1.ShowWindow(SW_SHOW);
		m_prop2.ShowWindow(SW_HIDE);
		break;
	case 1:
		m_prop1.ShowWindow(SW_HIDE);
		m_prop2.ShowWindow(SW_SHOW);
		break;
	default:
		m_prop1.ShowWindow(SW_SHOW);
		m_prop2.ShowWindow(SW_HIDE);
		break;
	}
}
//��ʼ���������
void CLaseAutoCorrectDlg::InitSysParam()
{
	//ȫ�ֱ�����ʼ��;
	memset(&g_SysDevParam,0,sizeof(SysDevParam));

	////��䱾��IP�Ͷ˿�
	//if(!GetHostAddress(&m_HostIpAddr))
	//{
	//	m_HostIpAddr.SetAddress(192,168,0,76);
	//}

	//��ʼ��Ŀ��ip�Ͷ˿�
	m_DestIp.SetAddress(192,168,0,2);
	m_DestPort=2110;
	m_LaserType.SetCurSel(7);
	g_SysDevParam.LaserType=6;
	m_PorType.SetCurSel(1);
	g_SysDevParam.LaserPor=1;
	m_ComBuand.SetCurSel(4);
	g_SysDevParam.nBuad=9600;
	m_ComIndex.SetCurSel(0);
	g_SysDevParam.nPort=-1;

	//712��ʼ���Ľ���
	GetDlgItem(IDC_COMBO_ChannelSelect)->ShowWindow(FALSE);
	GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("��ǰMAC��ַ");
	GetDlgItem(IDC_STATIC_MACSET)->ShowWindow(FALSE);
	GetDlgItem(IDC_EDIT_MACSET)->ShowWindow(FALSE);

	//���ڳ�ʼ��
	//m_ComIndex.Clear();
	//if(0==GetComListFromReg(&m_ComIndex))
	//{
	//	m_ComIndex.AddString("COM1");
	//	m_ComIndex.AddString("COM2");
	//	m_ComIndex.AddString("COM3");
	//	m_ComIndex.SetCurSel(0);
	//	
	//}
	//m_ComIndex.GetWindowTextA(g_SysDevParam.ComName,7);
	memset(&m_NetConnection,0,sizeof(m_NetConnection));

	GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(TRUE);
	GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(FALSE);

	GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(TRUE);
	GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(FALSE);
	GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(FALSE);

	m_slopDir.SetCurSel(0);
	m_slopeSteps=10;


	///��ȡ�ϸ��׼�����ļ�
	CString lpPath = "";
	CString path = "";
	GetModuleFileName(NULL, path.GetBufferSetLength(MAX_PATH + 1), MAX_PATH);
	path.ReleaseBuffer();
	int pos = path.ReverseFind('\\');
	path = path.Left(pos);
	lpPath = path + _T("\\") + "setup.ini";
	fstream _file;
	_file.open(lpPath, ios::in);
	LPTSTR l_getAveError = new char[50];
	LPTSTR l_getAllSingleError = new char[50];
	LPTSTR l_getCheckRangeMINValue = new char[50];
	CString AveError, AllSingleError, CheckRangeMINValue;
	if (!_file)
	{
		::AfxMessageBox(_T("�����ļ���ʧ�ܡ�"));
	}
	else
	{
		memset(l_getAveError, 0, sizeof(l_getAveError));
		GetPrivateProfileString("CONFIG", "AveError", "", l_getAveError, 50, lpPath);
		GetPrivateProfileString("CONFIG", "AllSingleError", "", l_getAllSingleError, 50, lpPath);
		GetPrivateProfileString("CONFIG", "CheckRangeMINValue", "", l_getCheckRangeMINValue, 50, lpPath);
		AveError.Format(_T("%s"), l_getAveError);
		AllSingleError.Format(_T("%s"), l_getAllSingleError);
		CheckRangeMINValue.Format(_T("%s"), l_getCheckRangeMINValue);
		m_AveError = _ttoi(AveError);
		m_AllSingleError = _ttoi(AllSingleError);
		m_CheckRangeMINValue = _ttoi(CheckRangeMINValue);
	}

	//���Ժͻ滭��ʼ��
	m_IntervNum=5;
	m_ChartNum=100;
	m_MkCalNum=50;
	m_prop1.m_ChartReal.GetAxis().GetBottom().SetMinMax(0,m_ChartNum*300);
	m_prop1.m_ChartReal.GetAxis().GetLeft().SetAutomatic(TRUE);
	UpdateData(FALSE);
	//�������ڽ��ս����߳�
	//CWinThread *pThread;
	//pThread = ::AfxBeginThread(ComParseThread,this);//���������߳�
	//if(pThread == NULL)
	//{
	//	bComParseTerm=FALSE;
	//	return;
	//}
	//bComParseTerm=TRUE;

	DispTips(IDB_PNG0,"����״̬");

}

BOOL CLaseAutoCorrectDlg::OnDeviceChange(UINT nEventType,DWORD dwData)
{
	//0x4d36e978L, 0xe325, 0x11ce, 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18
	//DEV_BROADCAST_DEVICEINTERFACE* dbd = (DEV_BROADCAST_DEVICEINTERFACE*) dwData;

	DEV_BROADCAST_HDR* dhr = (DEV_BROADCAST_HDR *)dwData;
	TRACE("--nEventType--:%d\n", nEventType);
	switch (nEventType)
	{
	case DBT_DEVICEREMOVECOMPLETE://�Ƴ��豸
		TRACE("--DEVICE REMOVE--\n");
#if 1
		if(dhr->dbch_devicetype == DBT_DEVTYP_PORT) 
		{ 
			PDEV_BROADCAST_PORT lpdbv = (PDEV_BROADCAST_PORT)dhr; 
			int len = strlen(lpdbv->dbcp_name);
			CString name(lpdbv->dbcp_name);//COM8
			int port = 0;
			sscanf_s(name.GetBuffer(0), "COM%d", &port);
		} 
#endif
		TRACE("--DEVICE REMOVE--end\n");
		break;
	case DBT_DEVICEARRIVAL://����豸
		TRACE("--DEVICE ARRIVAL--\n");
#if 1		
		if(dhr->dbch_devicetype == DBT_DEVTYP_PORT) 
		{ 
			PDEV_BROADCAST_PORT lpdbv = (PDEV_BROADCAST_PORT)dhr; 
			int len = strlen(lpdbv->dbcp_name);
			CString name(lpdbv->dbcp_name);//COM8
			int port = 0;
			sscanf_s(name.GetBuffer(0), "COM%d", &port);
		} 
#endif
		break;
	default:
		break;
	}

	return TRUE;

}

afx_msg LRESULT CLaseAutoCorrectDlg::OnWriteLog(WPARAM wParam,LPARAM lParam)
{
	int b_Show = (int)wParam;
	CString sLogTemp=(char*)lParam; 
	CString strLog=""; 
	CString TxtLogPath="";
	CFile sfile;
	strLog= CTime::GetCurrentTime().Format("%H:%M:%S") + " " + sLogTemp + "\r\n";
	if(m_pMyLog)	//д����־
		m_pMyLog->SetNotify(strLog.GetBuffer(strLog.GetLength()));
	if (b_Show == 0)//Ϊ�˼���UI����Ӧ��������Щ����Ҫ�����ݾ�ֻ��¼����־������UI����ʾ pl 0604
	{
		m_RichEdit.SetSel(-1, -1);
		m_RichEdit.ReplaceSel(strLog);
		m_RichEdit.PostMessage(WM_VSCROLL, SB_BOTTOM, 0);
		if (m_RichEdit.GetLineCount() > 100)
		{
			m_RichEdit.SetSel(0, -1);
			m_RichEdit.ReplaceSel("");
		}
	}
	return   0L;
}

void CLaseAutoCorrectDlg::WriteLog(CRichEditCtrl *pREC, CString sLogTemp, unsigned char *pbuf=NULL, unsigned short u16datalen=0)
{
	CString strLog=""; 
	CString TxtLogPath="";
	CFile sfile;
	strLog= CTime::GetCurrentTime().Format("%H:%M:%S") + " " + sLogTemp + "\r\n";

	pREC->SetSel(-1, -1);		
	pREC->ReplaceSel(strLog);//��RichEditCtrl�������ַ���

	pREC->PostMessage(WM_VSCROLL, SB_BOTTOM,0);

	if(pREC->GetLineCount()>100)
	{
		pREC->SetSel(0, -1);		
		pREC->ReplaceSel("");
	}
}

//�����̺߳���
UINT ConnectThreadProc(LPVOID lpParam)
{
	CString strTemp="";
	CLaseAutoCorrectDlg *pDlg = (CLaseAutoCorrectDlg*)lpParam;
	g_SysDevParam.LaserPor=pDlg->m_PorType.GetCurSel();
	if (!(pDlg->m_NetConnection.bConnected))
	{
		//����ṹ�������ֵ
//		pDlg->m_HostIpAddr.GetAddress(pDlg->m_NetConnection.dwServerIP);
		pDlg->m_NetConnection.unServerPort = 3000;
		pDlg->m_NetConnection.lpWnd = lpParam;
		pDlg->m_NetConnection.lpRecvFun = (LPVOID)OnNetRecv;	


		if (g_SysDevParam.LaserPor== 1)   //TCPģʽ
		{
			strTemp = "��������...";
			//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
			if (pDlg->m_LaserType.GetCurSel()==5) 
			{
				g_SysDevParam.LaserType=4;//711A
			}
			else if(pDlg->m_LaserType.GetCurSel()==6)//717
			{
				g_SysDevParam.LaserType=5;
			}
			else if(pDlg->m_LaserType.GetCurSel()==7)//712
			{
				g_SysDevParam.LaserType=6;    //712
			}
			else
			{
				g_SysDevParam.LaserType=3;    //711
			}
			pDlg->m_DestIp.GetAddress(pDlg->m_NetConnection.dwServerIP);
			pDlg->m_NetConnection.unServerPort = pDlg->m_DestPort;
			::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
			if (ConnectServer(&(pDlg->m_NetConnection)))
			{
				strTemp = "TCP�������ӳɹ�";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				pDlg->GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(FALSE);
				pDlg->GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(TRUE);
				pDlg->GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(FALSE);
				g_SysDevParam.NetConnected=TRUE;
				//��ʼ��ͼ
				pDlg->m_prop1.m_ChartReal.Series(0).Clear();
				pDlg->m_prop1.m_ChartReal.Series(1).Clear();
				pDlg->m_prop1.m_ChartReal.Series(2).Clear();
				Sleep(1000);
				g_nRevcNum3=0;
				g_nJGProCnt=0;
				g_nJGRecCnt =0;
				//�������ջ�ͼ�߳�
				bAqcOrgTerm=TRUE;
				::AfxBeginThread(AqcOrgThreadProc,lpParam);  //�������տ����߳�
				pDlg->Get711LmdSinglePoint();
			}
			else
			{
				strTemp = "TCP��������ʧ��";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				pDlg->GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(TRUE);
				pDlg->GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(FALSE);
				g_SysDevParam.NetConnected=FALSE;
			}
		}
		else if (g_SysDevParam.LaserPor == 0)
		{
			//pDlg->m_NetConnection.lpRecvFun = (LPVOID)OnNetRecv;
			if (UDP_Connect(&(pDlg->m_NetConnection)))
			{
				strTemp = "UDP�������ӳɹ�";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				pDlg->GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(FALSE);
				pDlg->GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(TRUE);
				g_SysDevParam.NetConnected=TRUE;
				if (pDlg->m_LaserType.GetCurSel()<3)     //��711����Ϊ711��tcpģʽ
				{
					g_SysDevParam.LaserType=1;     //710��710B��710C
				}
				else
				{
					g_SysDevParam.LaserType=2;     //710A
					::PostMessageA(pDlg->GetSafeHwnd(),WM_GET_SPLMD,0,0);
				}
				//��ʼ��ͼ
				pDlg->m_prop1.m_ChartReal.Series(0).Clear();
				pDlg->m_prop1.m_ChartReal.Series(1).Clear();
				pDlg->m_prop1.m_ChartReal.Series(2).Clear();
				g_nRevcNum3=0;
				g_nJGProCnt=0;
				g_nJGRecCnt =0;
				//�������ջ�ͼ�߳�
				bAqcOrgTerm=TRUE;
				::AfxBeginThread(AqcOrgThreadProc,lpParam);//���������߳�
			}
			else
			{
				strTemp = "UDP��������ʧ��";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				pDlg->GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(TRUE);
				pDlg->GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(FALSE);
				g_SysDevParam.NetConnected=FALSE;
			}
		}
	}
	else
	{
		strTemp = "������������Ͽ�����";
		::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	}
	//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
	//::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	return 0;
}

//�Ͽ������̺߳���
UINT DisConThreadProc(LPVOID lpParam)
{
	CString strTemp="";
	CLaseAutoCorrectDlg *pDlg = (CLaseAutoCorrectDlg*)lpParam;
	if (pDlg->m_NetConnection.bConnected)
	{
		if(DisconnectServer(&(pDlg->m_NetConnection)))
		{
			strTemp = "�������ӶϿ��ɹ�";
			pDlg->GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(TRUE);
			pDlg->GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(FALSE);
			if (g_SysDevParam.LaserType==2 || g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==4)
			{
				pDlg->StopTimer(TIMERGETLMD);
			}
			g_SysDevParam.NetConnected=FALSE;
			g_SysDevParam.LaserType=0;
		}
		else
		{
			strTemp = "�������ӶϿ�ʧ��";
			pDlg->GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(FALSE);
			pDlg->GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(TRUE);
		}

		pDlg->m_NetConnection.Init();
	}
	else
	{
		strTemp = "�����������ѶϿ�";
		pDlg->GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(TRUE);
			pDlg->GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(FALSE);
	}
	//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
	::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	return 0;
}


//BOOL CLaseAutoCorrectDlg::GetHostAddress(CIPAddressCtrl *pIPAddr)
//{
//	WSADATA wsadata;
//	CString strIp="";
//	if(0 != WSAStartup(MAKEWORD(2, 2), &wsadata))   //��ʼ��
//	{
//		return FALSE;
//	}
//	char szHostName[MAX_PATH + 1];
//	gethostname(szHostName, MAX_PATH);  //�õ��������
//	hostent *p = gethostbyname(szHostName); //�Ӽ�������õ�������Ϣ
//	if(p == NULL)
//	{
//		return FALSE;
//	}
//	strIp=inet_ntoa(*(struct in_addr *)p->h_addr_list[0]);
//	if (strIp=="127.0.0.1")
//	{
//		strIp="192.168.0.76";
//	}
//	DWORD dwIP=ntohl(inet_addr(strIp));
//	pIPAddr->SetAddress(dwIP);
//	WSACleanup();               //�ͷ�Winsock API
//	return TRUE;
//}

void CLaseAutoCorrectDlg::OnBnClickedButtonOpencom()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������

	SerPortPar serPortPar;
	int nPort; //���ں�
	int nBaud; //������
	CString strTemp="";
	int nRes=0;
	
	nPort = m_ComIndex.GetCurSel()+1;
	g_SysDevParam.nPort=nPort;
	m_ComBuand.GetLBText(m_ComBuand.GetCurSel(),strTemp);//������char����ת����int����
	nBaud = atoi(strTemp);
	g_SysDevParam.nBuad=nBaud;

	serPortPar.nPort = nPort;
	serPortPar.nBaud = nBaud;
	serPortPar.lpRecvFun = (LPVOID)OnComRecData;
	serPortPar.lpParam = (LPVOID)this;

	nRes=OpenComm(&serPortPar);
	//nRes=Com_Open(nbaund,ComName);
	if(-1==nRes)
	{
		//��ʧ��
		strTemp="���ڴ�ʧ��";
		GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(TRUE);
		GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(FALSE);
	}
	else if (0==nRes)
	{
		//�����Ѵ�
		strTemp="���ڱ�ռ�û򲻴���";
		GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(TRUE);
		GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(TRUE);
		
	}
	else if (1==nRes)
	{
		//�򿪳ɹ�
		strTemp="���ڴ򿪳ɹ�";
		GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(FALSE);
		GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(TRUE);
		g_SysDevParam.ComConnected=TRUE;
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}


void CLaseAutoCorrectDlg::OnBnClickedButtonClosecom()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	int nRes=0;
	//nRes=Com_Close(fd);
	nRes=CloseComm(g_SysDevParam.nPort);
	if (1==nRes)
	{
		//�����ѹر�
		strTemp="�����ѱ��ر�";
		GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(TRUE);
		GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(FALSE);
		g_SysDevParam.nBuad=9600;
		g_SysDevParam.nPort=-1;
	}
	else if (0==nRes)
	{
		//���ڹرճɹ�
		strTemp="���ڹرճɹ�";
		GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(TRUE);
		GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(FALSE);
		g_SysDevParam.ComConnected=FALSE;
	}
	else if (-1==nRes)
	{
		//���ڹر�ʧ��
		strTemp="�����ѱ��ر�";
		GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(FALSE);
		GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(TRUE);
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

int aa = 0;
int bb = 0;
int cc = 0;
void OnNetRecv(LPVOID lpParam, char *pDataBuf, int nDataBufSize)
{
	//EnterCriticalSection(&g_netcs);
	CLaseAutoCorrectDlg *pDlg = (CLaseAutoCorrectDlg *) lpParam;
	if (nDataBufSize>0)
	{
		if (g_SysDevParam.LaserPor == 1)
		{
			EnterCriticalSection(&g_netcs);
			if(g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==4||(g_SysDevParam.LaserType==5&&pDataBuf[1]==(char)0xAA))
			{
				if(newBCC711_Check(pDataBuf,nDataBufSize+4))
				{
					if(pDataBuf[22]==0x01)
					{
						if (pDataBuf[24] == 0x01)
						{
							aa++;
						}
						if (pDataBuf[24] == 0x02)
						{
							bb++;
						}
						if (pDataBuf[24] == 0x03)
						{
							cc++;
							if (aa != bb || aa != cc || bb != cc || aa >=100)
							{
								aa = 0;
								bb = 0;
								cc = 0;
							}
						}
					}
					memcpy(g_cNetRecvBuf[g_nNetRecvInd],pDataBuf,nDataBufSize+4);
					g_nNetRecvSize[g_nNetRecvInd] = nDataBufSize;

					pDlg->PostMessage(WM_NET_RECV_711,g_nNetRecvInd);
					g_nNetRecvInd = (g_nNetRecvInd+1)%NET_BUF_NUM;
				}
				else
				{
					xorflag = 1;
				}
			}
			else if(g_SysDevParam.LaserType==5&&pDataBuf[1]==(char)0xFF)
			{
				memcpy(g_cNetRecvBuf[g_nNetRecvInd],pDataBuf,nDataBufSize);
				g_nNetRecvSize[g_nNetRecvInd] = nDataBufSize;
				pDlg->PostMessage(WM_NET_RECV_715,g_nNetRecvInd);
				g_nNetRecvInd = (g_nNetRecvInd+1)%NET_BUF_NUM;
			}
			else if(g_SysDevParam.LaserType==6)
			{
				if(newBCC711_Check(pDataBuf,nDataBufSize+4))
				{
					memcpy(g_cNetRecvBuf[g_nNetRecvInd],pDataBuf,nDataBufSize+4);
					g_nNetRecvSize[g_nNetRecvInd] = nDataBufSize;
					pDlg->PostMessage(WM_NET_RECV_711,g_nNetRecvInd);
					g_nNetRecvInd = (g_nNetRecvInd+1)%NET_BUF_NUM;
				}
			}
			LeaveCriticalSection(&g_netcs); 
		} 
		else
		{
			memcpy(g_cNetRecvBuf[g_nNetRecvInd],pDataBuf,nDataBufSize);
			g_nNetRecvSize[g_nNetRecvInd] = nDataBufSize;
			pDlg->PostMessage(WM_NET_RECV,g_nNetRecvInd);
			g_nNetRecvInd = (g_nNetRecvInd+1)%NET_BUF_NUM;
		}
	}
	//LeaveCriticalSection(&g_netcs);
}

//���ݴ����������̺߳���
LRESULT CLaseAutoCorrectDlg::OnRecvNetData(WPARAM wParam, LPARAM lParam)
{
	int nBufID = (int)wParam;
	int k=0;
	int nDlThred = 0;
	int nSglThred = 0;
	int nTmp=0;
	CString m_sData = "";
	CString strTmp="";
	CString strTemp="";
	long int nSum = 0L;//ÿ֡�ĺ�
	float DeltU=0;
	if (m_IntervNum <= 0)
	{
		m_IntervNum = 5;
	}
	if (m_ChartNum<=0)
	{
		m_ChartNum = 100;
	}
	if ( m_MkCalNum <= 0)
	{
		m_MkCalNum = 100;
	}
	

	int temp_length=g_nNetRecvSize[nBufID];
	int temp_head=0;
	int temp_tail=g_nNetRecvSize[nBufID]-1;
	int frame_length=0;

	while(temp_length > 0 && temp_head != temp_tail)
	{
		if (g_cNetRecvBuf[nBufID][temp_head]== (char)0xFF && g_cNetRecvBuf[nBufID][temp_head+1] == (char)0xAA)
		{
			//���յ�FF AA��ʽ ����������ѯ�͵�����ֵ
			frame_length= (((byte)g_cNetRecvBuf[nBufID][temp_head+2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][temp_head+3];
			frame_length = frame_length+4;
			if (frame_length <= temp_length)
			{
				if (g_cNetRecvBuf[nBufID][temp_head+4]==0x02 && g_cNetRecvBuf[nBufID][temp_head+5]==0x04)
				{
					//���յ�������ѯ�ظ�
					g_nNetParseBuffLen = frame_length;
					memcpy(g_nNetParseBuff,&g_cNetRecvBuf[nBufID][temp_head],g_nNetParseBuffLen);
					HFA_ParseCmd02(g_nNetParseBuff,g_nNetParseBuffLen);
				}
				else if ((g_cNetRecvBuf[nBufID][temp_head+4] == 0x02)&&(g_cNetRecvBuf[nBufID][temp_head+5] == 0x0A))
				{
					if(AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
					{
						AutoCorretRegInfo.nComReSendCnt=0;
						StopTimer(TIMERGLADJUST);
					}
					switch(g_cNetRecvBuf[nBufID][temp_head+8])
					{			
					case (char)0x0A:       //A9BAH��ʾ��ǰ��������д�����������Ҵ����������ǰ20�����ݹ���⡣;
						//WriteLog(&m_RichEdit,"��д������ɹ���ǰ20���������£�");
						strTemp="��д������ɹ�";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						strTemp="ǰ20���������£�\n";
						for (k=0;k<20;k++)
						{
							nTmp = (BYTE)g_cNetRecvBuf[nBufID][temp_head+2*k+9] | ((BYTE)g_cNetRecvBuf[nBufID][temp_head+2*k+10]<<8);
							strTmp.Format("%d\t ",nTmp);
							strTemp += strTmp;
							if (nTmp&g_nReadbuf[k])
							{
								AutoCorretRegInfo.nCheckTbCode=1;
							}
							else
							{
								if (AutoCorretRegInfo.nChDownTbCnt>3)
								{
									AutoCorretRegInfo.nCheckTbCode=2;
								}
								else
								{
									AutoCorretRegInfo.nCheckTbCode=3;//����������������
								}
								break;
							}
						}
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						if (AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
						{
							SetEvent(Event_CHECKTB);
						}
						break;
					case (char)0x0B:    
						//WriteLog(&m_RichEdit,"��δ��д��������");
						strTemp="��δ��д��������";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						break;
					case (char)0x0C:
						//WriteLog(&m_RichEdit,"��������д����ȷ");
						strTemp="��������д����ȷ";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						break;
					default:
						//WriteLog(&m_RichEdit,"��������д���ָ��ظ���Ч");
						strTemp="��������д���ָ��ظ���Ч";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						break;
					}
				}
				else if ((g_cNetRecvBuf[nBufID][temp_head+4] == 0x02)&&(g_cNetRecvBuf[nBufID][temp_head+5] == 0x0B))
				{
					switch(g_cNetRecvBuf[nBufID][temp_head+10])
					{
					case (char)0x01:        //�ɹ�
						g_nNum = (int)g_cNetRecvBuf[nBufID][temp_head+6];
						g_flag = TRUE;
						SetEvent(g_eventdl);
						strTmp.Format("%d",g_nNum);
						strTemp="��"+strTmp+"�����ճɹ�";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						//WriteLog(&m_RichEdit,"�յ��ظ�����"+strTmp+"�����ͳɹ�\r\n");
						break;
					case  (char)0x02:       //ʧ��
						g_nNum = g_cNetRecvBuf[nBufID][temp_head+6];
						g_flag = FALSE;
						SetEvent(g_eventdl);
						strTmp.Format("%d",g_nNum);
						strTemp="��"+strTmp+"������ʧ��";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						//WriteLog(&m_RichEdit,"�յ��ظ�����"+strTmp+"������ʧ��\r\n");
						break;
					default:
						break;
					}	
				}
				else if ((g_cNetRecvBuf[nBufID][temp_head+4] == 0x02)&&(g_cNetRecvBuf[nBufID][temp_head+5] == 0x0C))
				{
					strTemp = "���������ذ������ɹ�";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				}
				temp_head = temp_head + frame_length;
				temp_length = temp_length - frame_length;
			}
			else
			{
				frame_length = frame_length-4;
				if (frame_length <= temp_length)
				{
					if (g_cNetRecvBuf[nBufID][temp_head+4]==0x03&&g_cNetRecvBuf[nBufID][temp_head+5]==0x03)
					{
						switch (g_cNetRecvBuf[nBufID][temp_head+6])
						{
						case (char)0x01:
							{    
								if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��m_nInterval�����ݰ���ʾһ��
								{  

									g_recv1len = frame_length;

									memcpy(g_nJGdata[0][g_nJGRecCnt],&g_cNetRecvBuf[nBufID][temp_head],g_recv1len);

									g_nMatch=1;//�յ���һ������

								}
							}
							break;
						case (char)0x02:
							{
								if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��5�����ݰ���ʾһ��
								{

									g_recv2len = frame_length;

									memcpy(g_nJGdata[1][g_nJGRecCnt],&g_cNetRecvBuf[nBufID][temp_head],g_recv2len);
									if (g_nMatch==1)
									{
										g_nMatch=2;
									}
								}
								break;
							}
						case (char)0x03:
							{
								if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��5�����ݰ���ʾһ��
								{
									if((g_nRevcNum3%(m_ChartNum*m_IntervNum) == 0))	
									{
										m_prop1.m_ChartReal.Series(0).Clear();
										m_prop1.m_ChartReal.Series(1).Clear();
										m_prop1.m_ChartReal.Series(2).Clear();
										g_nRevcNum3=0;
									}
									g_recv3len = frame_length;
									memcpy(g_nJGdata[2][g_nJGRecCnt],&g_cNetRecvBuf[nBufID][temp_head],g_recv3len);

									if (g_nMatch==2)
									{
										g_nMatch=0;
										//��ͼ

										UDP_DrawWave_710A(g_nJGdata[0][g_nJGRecCnt],1,1);
										UDP_DrawWave_710A(g_nJGdata[1][g_nJGRecCnt],1,2);
										UDP_DrawWave_710A(g_nJGdata[2][g_nJGRecCnt],1,3);
										if (AutoCorretRegInfo.nFinalCkCode==3)
										{
											g_nRevcNumcCheck1++;
										}

										//��ŵ���
										g_clsCriticalSection.Lock();
										g_nJGRecCnt=g_nJGRecCnt+1;
										g_nJGRecCnt=g_nJGRecCnt%JGMAX_CNT;
										g_clsCriticalSection.Unlock();
									}
								}
								g_nRevcNum3++;
								Recv710AMKDataCnt=0;
								break;
							}
						}
					}
					temp_head = temp_head + frame_length;
					temp_length = temp_length - frame_length;
				} 
				else
				{
					break;
				}
			}	
		}
		else if (g_cNetRecvBuf[nBufID][temp_head]== (char)0xFF && g_cNetRecvBuf[nBufID][temp_head+1] == (char)0xFF)
		{
			//strTemp.Format("%d,%d,%d",jg1,j2,j3);
			//::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
			//FF FF��ͷ�� ��λ���ϴ�����������
			frame_length= (((byte)g_cNetRecvBuf[nBufID][temp_head+5] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][temp_head+4];
			if (frame_length <= temp_length)
			{
				if (g_cNetRecvBuf[nBufID][temp_head+frame_length-1]== (char)0xFF)
				{
					switch (g_cNetRecvBuf[nBufID][temp_head+2])
					{
					case (char)0x01:
						{    
							jg1++;
							if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��m_nInterval�����ݰ���ʾһ��
							{  
								g_recv1len = frame_length;
								memcpy(g_nJGdata[0][g_nJGRecCnt],&g_cNetRecvBuf[nBufID][temp_head],g_recv1len);				
								g_nMatch=1;//�յ���һ������
							}
						}
						break;
					case (char)0x02:
						{
							j2++;
							if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��5�����ݰ���ʾһ��
							{
								g_recv2len = frame_length;
								memcpy(g_nJGdata[1][g_nJGRecCnt],&g_cNetRecvBuf[nBufID][temp_head],g_recv2len);
								if (g_nMatch==1)
								{
									g_nMatch=2;
								}
							}
							break;
						}
					case (char)0x03:
						{
							j3++;
							if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��5�����ݰ���ʾһ��
							{
								if((g_nRevcNum3%(m_ChartNum*m_IntervNum) == 0))	
								{
									m_prop1.m_ChartReal.Series(0).Clear();
									m_prop1.m_ChartReal.Series(1).Clear();
									m_prop1.m_ChartReal.Series(2).Clear();
									g_nRevcNum3=0;
								}

								g_recv3len = frame_length;
								memcpy(g_nJGdata[2][g_nJGRecCnt],&g_cNetRecvBuf[nBufID][temp_head],g_recv3len);	
								if (g_nMatch==2)
								{
									g_nMatch=0;
									//��ͼ
									UDP_DrawWave(g_nJGdata[0][g_nJGRecCnt],1,1);
									UDP_DrawWave(g_nJGdata[1][g_nJGRecCnt],1,2);
									UDP_DrawWave(g_nJGdata[2][g_nJGRecCnt],1,3);
									if (AutoCorretRegInfo.nFinalCkCode==3)
									{
										g_nRevcNumcCheck1++;
									}

									//��ŵ���
									g_clsCriticalSection.Lock();
									g_nJGRecCnt=g_nJGRecCnt+1;
									g_nJGRecCnt=g_nJGRecCnt%JGMAX_CNT;
									g_clsCriticalSection.Unlock();
								}
							}
							g_nRevcNum3++;
							if (g_SysDevParam.LaserPor== 1)   //TCPģʽ����711��
							{
								Recv711MKDataCnt=0;
							}
							break;
						}
					}
					temp_head = temp_head + frame_length;
					temp_length = temp_length - frame_length;
				}
				else
				{
					break;
				}
			}
			else
			{
				break;
			}
		}
		else
		{
			temp_head = temp_head + 1;
			if(temp_head == temp_tail + 1)
			{
				break;
			}
			temp_length--;
		}
	}
	
	return 0;
}

//���ݴ����������̺߳���    711��712
LRESULT CLaseAutoCorrectDlg::OnRecvNetData711(WPARAM wParam, LPARAM lParam)
{
	int nBufID = (int)wParam;
	int k=0;
	int nDlThred = 0;
	int nSglThred = 0;
	short nTmp=0;
	CString m_sData = "";
	CString strTmp="";
	CString strTemp="";
	long int nSum = 0L;//ÿ֡�ĺ�
	float DeltU=0;
	if (m_IntervNum <= 0)
	{
		m_IntervNum = 5;
	}
	if (m_ChartNum<=0)
	{
		m_ChartNum = 100;
	}
	if ( m_MkCalNum <= 0)
	{
		m_MkCalNum = 100;
	}

	int frame_length=0;
	if (g_cNetRecvBuf[nBufID][0] == (char)0xff && 
		g_cNetRecvBuf[nBufID][1] == (char)0xaa &&
		g_cNetRecvBuf[nBufID][12] == (char)0x00 && 
		(g_cNetRecvBuf[nBufID][13] == (char)0x04||g_cNetRecvBuf[nBufID][13] == (char)0x06||
		 g_cNetRecvBuf[nBufID][13] == (char)0x05||g_cNetRecvBuf[nBufID][13] == (char)0x07))     
	{
		frame_length= (((byte)g_cNetRecvBuf[nBufID][2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][3] + 4;
		if(g_cNetRecvBuf[nBufID][22]==0x05)
		{
			switch((unsigned char)(g_cNetRecvBuf[nBufID][23]))
			{	
			case H02_CmdA0://��λ������
				{
					strTemp = "����ָ����ִ���";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				}
				break;
			case H02_CmdA3://��λ������
				{
					strTemp = "�����豸�����ɹ�";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(FALSE);
				}
				break;
			case H02_Cmd89://��λ������
				{
					strTemp = "���������ذ������ɹ�";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					//g_SysDevParam.NetConnected=FALSE;
					//m_NetConnection.bConnected=0;
					//OnBnClickedButtonDisconnet();
				}
				break;
			case H02_CmdA4:
				{
					g_nNetParseBuffLen = frame_length;
					memcpy(g_nNetParseBuff,&g_cNetRecvBuf[nBufID],g_nNetParseBuffLen);
					H02_ParseCmd54(g_nNetParseBuff, g_nNetParseBuffLen);
				}
				break;
			case H02_CmdB2:
				{
					g_nNetParseBuffLen = frame_length;
					memcpy(g_nNetParseBuff,&g_cNetRecvBuf[nBufID],g_nNetParseBuffLen);
					H711_ParseCmd62(g_nNetParseBuff, g_nNetParseBuffLen);
				}
				break;
			case 0xB5:
				{
					strTemp="�������óɹ�";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(FALSE);
				}
				break;
			case 0x02:
				if (g_SysDevParam.LaserType==6)  //��ȡMAC
				{
					g_nNetParseBuffLen = frame_length;
					memcpy(g_nNetParseBuff,&g_cNetRecvBuf[nBufID],g_nNetParseBuffLen);
					H712_ParseMAC(g_nNetParseBuff, g_nNetParseBuffLen);
				}
				else
				{
					//���øߵ���ֵͨ���ɹ�֮��
					if((BYTE)g_cNetRecvBuf[nBufID][29]==0x01&&this->m_NetConnection.bConnected)
					{
						strTemp="�������óɹ�";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						OnBnClickedButtonDisconnet();
					}
				}
				break;
			case 0x04:
				{
					//712APD��ѯ�ظ�
					g_nNetParseBuffLen = frame_length;
					memcpy(g_nNetParseBuff,&g_cNetRecvBuf[nBufID],g_nNetParseBuffLen);
					H712_ParseAPD(g_nNetParseBuff, g_nNetParseBuffLen);
				}
				break;
			case 0x05:
				{
					g_nNetParseBuffLen = frame_length;
					memcpy(g_nNetParseBuff,&g_cNetRecvBuf[nBufID],g_nNetParseBuffLen);
					//H711_ParseCmd68(g_nNetParseBuff, g_nNetParseBuffLen);
					if((BYTE)g_cNetRecvBuf[nBufID][29]==0x01)
					{
						strTemp="APD�������óɹ�";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					}
				}
				break;
			case H02_CmdA5:
				{
					g_nNetParseBuffLen = frame_length;
					memcpy(g_nNetParseBuff,&g_cNetRecvBuf[nBufID],g_nNetParseBuffLen);
					H711_ParseCmd55(g_nNetParseBuff, g_nNetParseBuffLen);
				}
				break;
			case 0x0C: //��λ���ظ���������������ͳɹ�
				switch ((BYTE)g_cNetRecvBuf[nBufID][29])
				{
				case 0x01:        //�ɹ�
					g_nNum = (g_cNetRecvBuf[nBufID][26]<<8)+g_cNetRecvBuf[nBufID][27];
					strTmp.Format("%d",g_nNum);
					strTemp="��"+strTmp+"�����ճɹ�";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					g_flag = TRUE;
					SetEvent(g_eventdl);						
					break;
				case 0x02:       //ʧ��
					g_nNum = (g_cNetRecvBuf[nBufID][26]<<8)+g_cNetRecvBuf[nBufID][27];
					strTmp.Format("%d",g_nNum);
					strTemp="��"+strTmp+"������ʧ��\r\n";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					g_flag = FALSE;
					SetEvent(g_eventdl);
					break;
				default:
					break;
				}	
				break;
			case (char)0x0D://������д������
				if((BYTE)g_cNetRecvBuf[nBufID][29]==0x01)
				{
					strTemp="��������д�ɹ�";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					g_flag = TRUE;
					SetEvent(g_eventdl);
				}
				else if((BYTE)g_cNetRecvBuf[nBufID][29]==0x02)
				{
					strTemp="��������дʧ��";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					g_flag = FALSE;
					SetEvent(g_eventdl);
				}
				break;
			case (char)0x0E://�����������д���
				if((BYTE)g_cNetRecvBuf[nBufID][26]==0x01)
				{
					AutoCorretRegInfo.nCheckTbCode=1;
					strTemp="����������д��ǰ10���������£�\n";
					for (k=0;k<10;k++)
					{
						nTmp = (((g_cNetRecvBuf[nBufID][29+k*2])<<8) & 0xff00) +(g_cNetRecvBuf[nBufID][30+k*2] & 0xff);
						strTmp.Format("%d\t ",nTmp);
						strTemp += strTmp;
						if (nTmp&g_nReadbuf[k])
						{
							AutoCorretRegInfo.nCheckTbCode=1;
						}
						else
						{
							strTemp += "����ѯ��֤��һ��";
							if (AutoCorretRegInfo.nChDownTbCnt>3)
							{
								AutoCorretRegInfo.nCheckTbCode=2;
							}
							else
							{
								AutoCorretRegInfo.nCheckTbCode=3;//����������������
							}
							break;
						}
					}
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				}
				else if((BYTE)g_cNetRecvBuf[nBufID][26]==0x02)
				{
					strTemp="������δ��д";
					if (AutoCorretRegInfo.nChDownTbCnt>3)
					{
						AutoCorretRegInfo.nCheckTbCode=2;
					}
					else
					{
						AutoCorretRegInfo.nCheckTbCode=3;//����������������
					}
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				}
				if (AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
				{
					SetEvent(Event_CHECKTB);
				}
				break;
			case H02_CmdA9:
				if(AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
				{
					AutoCorretRegInfo.nComReSendCnt=0;
					StopTimer(TIMERGLADJUST);
				}
				switch((BYTE)g_cNetRecvBuf[nBufID][11])
				{
				case 0xba:       //A9BAH��ʾ��ǰ��������д�����������Ҵ����������ǰ20�����ݹ���⡣;
					//WriteLog(&m_RichEdit,"��д������ɹ���ǰ20���������£�");

					strTemp="ǰ20���������£�\n";
					for (k=0;k<20;k++)
					{
						nTmp = (BYTE)g_cNetRecvBuf[nBufID][2*k+24] | ((BYTE)g_cNetRecvBuf[nBufID][2*k+25]<<8);
						strTmp.Format("%d\t ",nTmp);

						strTemp += strTmp;
						if (nTmp&g_nReadbuf[k])
						{
							AutoCorretRegInfo.nCheckTbCode=1;
						}
						else
						{
							if (AutoCorretRegInfo.nChDownTbCnt>3)
							{
								AutoCorretRegInfo.nCheckTbCode=2;
							}
							else
							{
								AutoCorretRegInfo.nCheckTbCode=3;//����������������
							}
							break;
						}
					}
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					if (g_ChannelSelect==0)    //����ֵ
					{
						strTemp="��д����ֵ������ɹ�";
					} 
					else
					{
						strTemp="��д����ֵ������ɹ�";
					}

					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					if (AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
					{
						SetEvent(Event_CHECKTB);
					}
					break;
				case 0xbb:    
					//WriteLog(&m_RichEdit,"��δ��д��������");
					strTemp="��δ��д��������";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					break;
				case 0xbc:
					//WriteLog(&m_RichEdit,"��������д����ȷ");
					strTemp="��������д����ȷ";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					break;
				default:
					//WriteLog(&m_RichEdit,"��������д���ָ��ظ���Ч");
					strTemp="��������д���ָ��ظ���Ч";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					break;
				}
			}
		}
		else if(g_cNetRecvBuf[nBufID][22]==0x01)
		{
			if(g_cNetRecvBuf[nBufID][23]==0x02)//������������
			{
				if(g_SysDevParam.LaserType== 3 || (g_SysDevParam.LaserType== 6))
				{
					switch (g_cNetRecvBuf[nBufID][24])
					{
					case (char)0x01:
						{   
							g_recv1len=(((byte)g_cNetRecvBuf[nBufID][2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][3]+4;
							jg1++;
							if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��m_nInterval�����ݰ���ʾһ��
							{
								memcpy(g_nJGdata[0][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv1len);				
								g_nMatch=1;//�յ���һ������
							}
						}
						break;
					case (char)0x02:
						{
							g_recv2len=(((byte)g_cNetRecvBuf[nBufID][2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][3]+4;
							j2++;
							if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��5�����ݰ���ʾһ��
							{
								memcpy(g_nJGdata[1][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv2len);
								if (g_nMatch==1)
								{
									g_nMatch=2;
								}
							}
							break;
						}
					case (char)0x03:
						{
							g_recv3len=(((byte)g_cNetRecvBuf[nBufID][2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][3]+4;
							j3++;
							if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��5�����ݰ���ʾһ��
							{
								if((g_nRevcNum3%(m_ChartNum*m_IntervNum) == 0))	
								{
									m_prop1.m_ChartReal.Series(0).Clear();
									m_prop1.m_ChartReal.Series(1).Clear();
									m_prop1.m_ChartReal.Series(2).Clear();
									g_nRevcNum3=0;
								}

								memcpy(g_nJGdata[2][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv3len);	
								if (g_nMatch==2)
								{
									g_nMatch=0;
									//��ͼ
									UDP_DrawWave711(g_nJGdata[0][g_nJGRecCnt],1,1);
									UDP_DrawWave711(g_nJGdata[1][g_nJGRecCnt],1,2);
									UDP_DrawWave711(g_nJGdata[2][g_nJGRecCnt],1,3);
									if (AutoCorretRegInfo.nFinalCkCode==3)
									{
										g_nRevcNumcCheck1++;
									}

									//��ŵ���
									g_clsCriticalSection.Lock();
									g_nJGRecCnt=g_nJGRecCnt+1;
									g_nJGRecCnt=g_nJGRecCnt%JGMAX_CNT;
									g_clsCriticalSection.Unlock();
								}
							}
							g_nRevcNum3++;
							if (g_SysDevParam.LaserPor== 1)   //TCPģʽ����711��
							{
								Recv711MKDataCnt=0;
							}
							break;
						}
					}
				}
				else if(g_SysDevParam.LaserType== 4)
				{
					switch (g_cNetRecvBuf[nBufID][26])
					{
					case (char)0x01:
						{   
							g_recv1len=(((byte)g_cNetRecvBuf[nBufID][2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][3]+4;
							jg1++;
							if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��m_nInterval�����ݰ���ʾһ��
							{
								memcpy(g_nJGdata[0][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv1len);				
								g_nMatch=1;//�յ���һ������
							}
						}
						break;
					case (char)0x02:
						{
							g_recv2len=(((byte)g_cNetRecvBuf[nBufID][2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][3]+4;
							j2++;
							if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��5�����ݰ���ʾһ��
							{
								memcpy(g_nJGdata[1][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv2len);
								if (g_nMatch==1)
								{
									g_nMatch=2;
								}
							}
							break;
						}
					case (char)0x03:
						{
							g_recv3len=(((byte)g_cNetRecvBuf[nBufID][2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][3]+4;
							j3++;
							if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��5�����ݰ���ʾһ��
							{
								if((g_nRevcNum3%(m_ChartNum*m_IntervNum) == 0))	
								{
									m_prop1.m_ChartReal.Series(0).Clear();
									m_prop1.m_ChartReal.Series(1).Clear();
									m_prop1.m_ChartReal.Series(2).Clear();
									g_nRevcNum3=0;
								}

								memcpy(g_nJGdata[2][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv3len);	
								if (g_nMatch==2)
								{
									g_nMatch=0;
									//��ͼ
									UDP_DrawWave711(g_nJGdata[0][g_nJGRecCnt],1,1);
									UDP_DrawWave711(g_nJGdata[1][g_nJGRecCnt],1,2);
									UDP_DrawWave711(g_nJGdata[2][g_nJGRecCnt],1,3);
									if (AutoCorretRegInfo.nFinalCkCode==3)
									{
										g_nRevcNumcCheck1++;
									}

									//��ŵ���
									g_clsCriticalSection.Lock();
									g_nJGRecCnt=g_nJGRecCnt+1;
									g_nJGRecCnt=g_nJGRecCnt%JGMAX_CNT;
									g_clsCriticalSection.Unlock();
								}
							}
							g_nRevcNum3++;
							if (g_SysDevParam.LaserPor== 1)   //TCPģʽ����711��
							{
								Recv711MKDataCnt=0;
							}
							break;
						}
					}
				}
			}
		}
		else if(g_cNetRecvBuf[nBufID][22]==0x07)
		{
			if(g_cNetRecvBuf[nBufID][23]==0x01)
			{
				g_nNetParseBuffLen = frame_length;
				memcpy(g_nNetParseBuff,&g_cNetRecvBuf[nBufID],g_nNetParseBuffLen);
				H715_ParseHL(g_nNetParseBuff, g_nNetParseBuffLen);
			}
			else if(g_cNetRecvBuf[nBufID][23]==0x02)
			{
				strTemp="���ܲ������óɹ�";
				::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
			}
			else if(g_cNetRecvBuf[nBufID][23]==0x04)
			{
				g_nNetParseBuffLen = frame_length;
				memcpy(g_nNetParseBuff,&g_cNetRecvBuf[nBufID],g_nNetParseBuffLen);
				H715_ParseAPD(g_nNetParseBuff, g_nNetParseBuffLen);
			}
		}
	}
	return 0;
}

LRESULT CLaseAutoCorrectDlg::OnRecvNetData715(WPARAM wParam, LPARAM lParam)
{
	int nBufID = (int)wParam;
	int k=0;
	int nDlThred = 0;
	int nSglThred = 0;
	int nTmp=0;
	CString m_sData = "";
	CString strTmp="";
	CString strTemp="";
	long int nSum = 0L;//ÿ֡�ĺ�
	float DeltU=0;
	if (m_IntervNum <= 0)
	{
		m_IntervNum = 5;
	}
	if (m_ChartNum<=0)
	{
		m_ChartNum = 100;
	}
	if ( m_MkCalNum <= 0)
	{
		m_MkCalNum = 100;
	}

	int frame_length=0;
	if (g_cNetRecvBuf[nBufID][0] == (char)0xff && 
		g_cNetRecvBuf[nBufID][1] == (char)0xff)     
	{
		frame_length= (((byte)g_cNetRecvBuf[nBufID][5] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][4];
		switch (g_cNetRecvBuf[nBufID][2])
		{
		case (char)0x01:
			{   
				g_recv1len=frame_length;
				jg1++;
				if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��m_nInterval�����ݰ���ʾһ��
				{
					memcpy(g_nJGdata[0][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv1len);				
					g_nMatch=1;//�յ���һ������
				}
			}
			break;
		case (char)0x02:
			{
				g_recv2len=frame_length;
				j2++;
				if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��5�����ݰ���ʾһ��
				{
					memcpy(g_nJGdata[1][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv2len);
					if (g_nMatch==1)
					{
						g_nMatch=2;
					}
				}
				break;
			}
		case (char)0x03:
			{
				g_recv3len=frame_length;
				j3++;
				if (g_nRevcNum3%m_IntervNum == 0)          //ÿ��5�����ݰ���ʾһ��
				{
					if((g_nRevcNum3%(m_ChartNum*m_IntervNum) == 0))	
					{
						m_prop1.m_ChartReal.Series(0).Clear();
						m_prop1.m_ChartReal.Series(1).Clear();
						m_prop1.m_ChartReal.Series(2).Clear();
						g_nRevcNum3=0;
					}

					memcpy(g_nJGdata[2][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv3len);	
					if (g_nMatch==2)
					{
						g_nMatch=0;
						//��ͼ
						UDP_DrawWave711(g_nJGdata[0][g_nJGRecCnt],1,1);
						UDP_DrawWave711(g_nJGdata[1][g_nJGRecCnt],1,2);
						UDP_DrawWave711(g_nJGdata[2][g_nJGRecCnt],1,3);
						if (AutoCorretRegInfo.nFinalCkCode==3)
						{
							g_nRevcNumcCheck1++;
						}

						//��ŵ���
						g_clsCriticalSection.Lock();
						g_nJGRecCnt=g_nJGRecCnt+1;
						g_nJGRecCnt=g_nJGRecCnt%JGMAX_CNT;
						g_clsCriticalSection.Unlock();
					}
				}
				g_nRevcNum3++;
				if (g_SysDevParam.LaserPor== 1)   //TCPģʽ����711��
				{
					Recv711MKDataCnt=0;
				}
				break;
			}
		}
	}
	return 0;
}
/*******************************************************
* �������ܣ���UDPÿ֡���ݵ�ĺ�
* ����������pcDataBuf:���ݻ�������nFirstByte:������ʼ�ֽ�
* �� �� ֵ�����ݺ�
********************************************************/
long CLaseAutoCorrectDlg::UDP_DataSum(char *pcDataBuf, int nFirstByte,int nType)
{
	long nSum = 0L;
	int i = 0;
	int j= 0;
	int nDataNum = 0;
	int nSunNum = 0;

	if (nType==1)
	{
		nDataNum = (pcDataBuf[7]<<8) + (BYTE)(pcDataBuf[6]);//���ݵ����
	}
	else if (nType==2)
	{
		nDataNum = (pcDataBuf[7]<<8) + (pcDataBuf[8]);//���ݵ����
	}

	i = nFirstByte;
	for (j = 1 ;j <=nDataNum;j++)  
	{
		g_nCurMKValue[j-1] = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
		if (g_nCurMKValue[j-1] > 10)
		{
			nSum = nSum + g_nCurMKValue[j-1];
			nSunNum++;
		}		
		i = i + 4;
	}

	if (nSunNum == 0)
	{
		return 0;
	}
	return nSum/nSunNum;
}

long CLaseAutoCorrectDlg::UDP_360DataSum711(char *pcDataBuf, int nFirstByte)
{
	long nSum = 0L;
	int i = 0;
	int j= 0;
	int nDataNum = 0;
	int nSunNum = 0;

	if(g_SysDevParam.LaserType==6)
	{
		nDataNum = (pcDataBuf[26]<<8) + (BYTE)(pcDataBuf[27]);//���ݵ����
		if (nDataNum==0)
		{
			return 0;
		}

		i = nFirstByte;
		for (j = 1 ;j <=nDataNum;j++)  
		{
			g_nCurMKValue[j-1] = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			if (g_nCurMKValue[j-1] > 10)
			{
				nSum = nSum + g_nCurMKValue[j-1];
				nSunNum++;
			}
			i = i + 4;
		}
	}
	else if(g_SysDevParam.LaserType==4)
	{
		nDataNum = (pcDataBuf[27]<<8) + (BYTE)(pcDataBuf[28]);//���ݵ����
		if (nDataNum==0)
		{
			return 0;
		}
		nFirstByte=29;
		i = nFirstByte + 240;
		for (j = 61 ;j <=nDataNum;j++)  
		{
			g_nCurMKValue[j-1] = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			if (g_nCurMKValue[j-1] > 10)
			{
				nSum = nSum + g_nCurMKValue[j-1];
				nSunNum++;
			}
			i = i + 4;
		}
	}
	else if(g_SysDevParam.LaserType==5)
	{
		nDataNum = (pcDataBuf[7]<<8) + (BYTE)(pcDataBuf[6]);//���ݵ����
		if (nDataNum==0)
		{
			return 0;
		}
		nFirstByte=8;
		i = nFirstByte + 240;
		for (j = 61 ;j <=nDataNum;j++)  
		{
			g_nCurMKValue[j-1] = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			if (g_nCurMKValue[j-1] > 10)
			{
				nSum = nSum + g_nCurMKValue[j-1];
				nSunNum++;
			}
			i = i + 4;
		}
	}
	if (nSunNum == 0)
	{
		return 0;
	}
	return nSum/nSunNum;
}

long CLaseAutoCorrectDlg::UDP_DataSum711(char *pcDataBuf, int nFirstByte)
{
	long nSum = 0L;
	int i = 0;
	int j= 0;
	int nDataNum = 0;
	int nSunNum = 0;
	if(g_SysDevParam.LaserType==6)
	{
		nDataNum = (pcDataBuf[26]<<8) + (BYTE)(pcDataBuf[27]);//���ݵ����
		if (nDataNum==0)
		{
			return 0;
		}
		i = nFirstByte;
		for (j = 1 ;j <=nDataNum;j++)  
		{
			g_nCurMKValue[j-1] = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			if (g_nCurMKValue[j-1] > 10)
			{
				nSum = nSum + g_nCurMKValue[j-1];
				nSunNum++;
			}		
			i = i + 4;
		}
	}
	else if(g_SysDevParam.LaserType==4)
	{
		nDataNum = (pcDataBuf[27]<<8) + (BYTE)(pcDataBuf[28]);//���ݵ����
		if (nDataNum==0)
		{
			return 0;
		}
		nFirstByte=29;
		i = nFirstByte;
		for (j = 1 ;j <=nDataNum;j++)  
		{
			g_nCurMKValue[j-1] = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			if (g_nCurMKValue[j-1] > 10)
			{
				nSum = nSum + g_nCurMKValue[j-1];
				nSunNum++;
			}		
			i = i + 4;
		}
	}
	else if(g_SysDevParam.LaserType==5)
	{
		nDataNum = (pcDataBuf[7]<<8) + (BYTE)(pcDataBuf[6]);//���ݵ����
		if (nDataNum==0)
		{
			return 0;
		}
		nFirstByte=8;
		i = nFirstByte;
		for (j = 1 ;j <=nDataNum;j++)  
		{
			g_nCurMKValue[j-1] = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			if (g_nCurMKValue[j-1] > 10)
			{
				nSum = nSum + g_nCurMKValue[j-1];
				nSunNum++;
			}		
			i = i + 4;
		}
	}
	if (nSunNum == 0)
	{
		return 0;
	}
	return nSum/nSunNum;
}

void CLaseAutoCorrectDlg::H02_ParseCmd54(char *pcBuf, int nBufSize)
{

	int nTmp = 0;
	int i=0;
	CString strTmp="";
	CString strTemp="";
	if (nBufSize != 137)
	{
		strTemp = "��ѯ�����ظ�����:";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		return;
	}
	else
	{
		nTmp=24;
		memcpy(&LaserParam,pcBuf+24,sizeof(LaserParam));
		
		m_ApdPress=LaserParam.u16APDHvValue;
		m_ApdTemp=LaserParam.u16APDTemperValue;
		m_ApdXs=LaserParam.u16APDHV_OP_Ratio;
	
		m_MacAddr="";

		for (i=0;i<6;i++)
		{
			strTmp.Format("%2.2X", (DWORD)LaserParam.u16My_Phy_Addr[i]);
			m_MacAddr+=strTmp;
		}

		strTemp = "��ѯ�豸�����ɹ�";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(TRUE);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
		{
			StopTimer(TIMERGETPARMA);
			if (LaserParam.u16APDHV_OP_Ratio>880||LaserParam.u16APDHV_OP_Ratio<600)
			{
				AutoCorretRegInfo.nParamChCode=2;
			}
			else
			{
				AutoCorretRegInfo.nParamChCode=1;
			}
			SetEvent(Event_PARAM);
		}
		UpdateData(FALSE);
	}
}

void CLaseAutoCorrectDlg::H712_ParseAPD(char *pcBuf, int nBufSize)
{
	int nTmp = 0;
	int i=0;
	CString strTmp="";
	//int tmp=0;
	CString strTemp="";
	if (nBufSize != 36)
	{
		strTemp = "��ѯAPD�����ظ�����:";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		return;
	}
	else
	{
		//if (g_nAPDStateTemp==-100) //����ǰAPD�¶�û�в鵽���򲻽���
		//{
		//	strTemp = "APD��ǰ�¶�δ�鵽";
		//	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		//	WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		//	return;
		//}
		memcpy(xitongcanshu,&pcBuf[26],50);
		LaserParam712.m_u16APDVSet=((((byte)pcBuf[26]&0xff)<<8) + ((byte)pcBuf[27]&0xff))/100;
		m_ApdPress = LaserParam712.m_u16APDVSet;

		LaserParam712.m_u16APDTemperValue=((((byte)pcBuf[28]&0xff)<<8) + ((byte)pcBuf[29]&0xff))/100;
		m_ApdTemp = LaserParam712.m_u16APDTemperValue;

		LaserParam712.m_u16APDHV_OP_Ratio=((((byte)pcBuf[30]&0xff)<<8) + ((byte)pcBuf[31]&0xff))*10;
		m_ApdXs = LaserParam712.m_u16APDHV_OP_Ratio;

		strTemp = "��ѯAPD�����ɹ�";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		if (AutoCorretRegInfo.nStepIndx==STEP_PARAM && bAutoCorrTerm)
		{
			StopTimer(TIMERGETPARMA);
			if (LaserParam712.m_u16APDHV_OP_Ratio>880||LaserParam712.m_u16APDHV_OP_Ratio<600)
			{
				AutoCorretRegInfo.nParamChCode=2;
			}
			else
			{
				AutoCorretRegInfo.nParamChCode=4;
			}
			SetEvent(Event_PARAM);
		}
		else
		{
			GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(TRUE);
		}
		UpdateData(FALSE);
	}
}

//void CLaseAutoCorrectDlg::H712_ParseTempSate(char *pcBuf, int nBufSize)
//{
//	int nTmp = 0;
//	int i=0;
//	CString strTmp="";
//	CString strTemp="";
//	if (nBufSize != 77)
//	{
//		strTemp = "��ѯ�����ظ�����:";
//		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
//		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
//		return;
//	}
//	else
//	{
//		short int tmp = (short int)((pcBuf[33]<<8)+(pcBuf[34]));
//		g_nAPDStateTemp = tmp;
//		m_APDStateTemp = tmp/100.0;
//		UpdateData(FALSE);
//	}
//}

void CLaseAutoCorrectDlg::H712_ParseMAC(char *pcBuf, int nBufSize)
{
	int nTmp = 0;
	int i=0;
	CString strTmp="";
	CString strTemp="";
	if (nBufSize != 58)
	{
		strTemp = "��ѯ�����ظ�����:";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		return;
	}
	else
	{
		nTmp=32;
		m_MacAddr="";
		memcpy(netmac,pcBuf,21);
		m_MacAddr.Format("%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X",(pcBuf[40]&0xff),(pcBuf[41]&0xff),(pcBuf[42]&0xff),(pcBuf[43]&0xff),(pcBuf[44]&0xff),(pcBuf[45]&0xff));//MAC
		UpdateData(FALSE);
	}
}

void CLaseAutoCorrectDlg::H711_ParseCmd55(char *pcBuf, int nBufSize)
{
	int nTmp = 0;
	int i=0;
	CString strTmp="";
	//int tmp=0;
	CString strTemp="";
	if(g_SysDevParam.LaserType==4)
	{
		m_MacAddr="";
			memcpy(netmac,pcBuf,21);
			m_MacAddr.Format("%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X",(pcBuf[38]&0xff),(pcBuf[39]&0xff),(pcBuf[40]&0xff),(pcBuf[41]&0xff),(pcBuf[42]&0xff),(pcBuf[43]&0xff));//MAC
			UpdateData(FALSE);
	}
	else{
	if (nBufSize != 59)
	{
		strTemp = "��ѯ�����ظ�����:";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		return;
	}
	else
	{
		nTmp=32;
		m_MacAddr="";
		memcpy(netmac,pcBuf,21);
		//for (i=0;i<6;i++)
		//{
		//	if (i>3)
		//	{
		//		strTmp.Format("%2.2d", pcBuf[nTmp+i*2]);
		//	}
		//	else
		//	{
		//		strTmp.Format("%2.2X", pcBuf[nTmp+i*2]);
		//	}			
		//	m_MacAddr+=strTmp;
		//}

		//LaserParam711.m_u16APDVSet=((((byte)pcBuf[55]&0xff)<<8) + ((byte)pcBuf[54]&0xff))/100;
		//m_ApdPress = LaserParam711.m_u16APDVSet;

		//LaserParam711.m_u16APDTemperValue=((((byte)pcBuf[57]&0xff)<<8) + ((byte)pcBuf[56]&0xff))/100;
		//m_ApdTemp = LaserParam711.m_u16APDTemperValue;

		//LaserParam711.m_u16APDHV_OP_Ratio=((((byte)pcBuf[59]&0xff)<<8) + ((byte)pcBuf[58]&0xff))*10;
		//m_ApdXs = LaserParam711.m_u16APDHV_OP_Ratio;

		//strTemp = "��ѯ�豸�����ɹ�";
		//::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		////WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		//if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
		//{
		//	StopTimer(TIMERGETPARMA);
		//	if (m_ApdXs>880||m_ApdXs<600)
		//	{
		//		AutoCorretRegInfo.nParamChCode=2;
		//	}
		//	else
		//	{
		//		AutoCorretRegInfo.nParamChCode=1;
		//	}
		//	SetEvent(Event_PARAM);
		//}
		m_MacAddr.Format("%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X",(pcBuf[40]&0xff),(pcBuf[41]&0xff),(pcBuf[42]&0xff),(pcBuf[43]&0xff),(pcBuf[44]&0xff),(pcBuf[45]&0xff));//MAC
		UpdateData(FALSE);
	}
	}
}

void CLaseAutoCorrectDlg::H711_ParseCmd68(char *pcBuf, int nBufSize)
{
	int nTmp = 0;
	int i=0;
	CString strTmp="";
	//int tmp=0;
	CString strTemp="";
	if (nBufSize != 36)
	{
		strTemp = "��ѯAPD�����ظ�����:";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		return;
	}
	else
	{
		memcpy(xitongcanshu,&pcBuf[26],50);
		LaserParam711.m_u16APDVSet=((((byte)pcBuf[26]&0xff)<<8) + ((byte)pcBuf[27]&0xff))/100;
		m_ApdPress = LaserParam711.m_u16APDVSet;

		LaserParam711.m_u16APDTemperValue=((((byte)pcBuf[28]&0xff)<<8) + ((byte)pcBuf[29]&0xff))/100;
		m_ApdTemp = LaserParam711.m_u16APDTemperValue;

		LaserParam711.m_u16APDHV_OP_Ratio=((((byte)pcBuf[30]&0xff)<<8) + ((byte)pcBuf[31]&0xff))*10;
		m_ApdXs = LaserParam711.m_u16APDHV_OP_Ratio;

		strTemp = "��ѯAPD�����ɹ�";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
		{
			StopTimer(TIMERGETPARMA);
			if (m_ApdXs>880||m_ApdXs<600)
			{
				AutoCorretRegInfo.nParamChCode=2;
			}
			else
			{
				AutoCorretRegInfo.nParamChCode=4;  //���������������
				g_CheckAPD=true;
			}
			SetEvent(Event_PARAM);
		}
		UpdateData(FALSE);
	}
}

void CLaseAutoCorrectDlg::H711_ParseCmd62(char *pcBuf, int nBufSize)
{
	int nTmp = 0;
	int i=0;
	CString strTmp="";
	//int tmp=0;
	CString strTemp="";
	if (nBufSize != 90)
	{
		strTemp = "��ѯ���ܲ����ظ�����:";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		return;
	}
	else
	{
		memcpy(xitongcanshu,&pcBuf[26],50);
		LaserParam711.m_u16SysMode_SingleOrScanf=(byte)pcBuf[48]&0xff;

		//m_MacAddr="001234";
		/*LaserParam711.m_u16EquipNo[0] = (((byte)pcBuf[29]&0xff)<<8) + ((byte)pcBuf[30]&0xff);
		LaserParam711.m_u16EquipNo[1] = (byte)pcBuf[28]&0xff;
		strTmp.Format("%d",((LaserParam711.m_u16EquipNo[1]<<16)+LaserParam711.m_u16EquipNo[0]));
		m_MacAddr=strTmp;*/
		
		LaserParam711.m_u16Channel_Sel=(byte)pcBuf[50]&0xff;
		m_ChannelSelect.SetCurSel(LaserParam711.m_u16Channel_Sel);
		LaserParam711.m_u16ZeroDisc=(((byte)pcBuf[60]&0xff)<<8) + (byte)pcBuf[61]&0xff;
		LaserParam711.m_u16OverallDisDif = (((byte)pcBuf[70]&0xff)<<8) + (byte)pcBuf[71]&0xff;
		LaserParam711.m_u16OverallDisDif_L = (((byte)pcBuf[72]&0xff)<<8) + (byte)pcBuf[73]&0xff;
		if(m_ChannelSelect.GetCurSel()==1)
		{
		  m_AllSingleError=500;
		}

		strTemp = "��ѯϵͳ�����ɹ�";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(TRUE);
		if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
		{
			StopTimer(TIMERGETPARMA);
			if (LaserParam711.m_u16SysMode_SingleOrScanf==0 && LaserParam711.m_u16Channel_Sel == g_ChannelSelect)
			{
				AutoCorretRegInfo.nParamChCode=1;
			}
			else
			{
				AutoCorretRegInfo.nParamChCode=2;
			}
			SetEvent(Event_PARAM);
		}
		UpdateData(FALSE);
	}
}

void CLaseAutoCorrectDlg::H715_ParseHL(char *pcBuf, int nBufSize)
{
	int nTmp = 0;
	int i=0;
	CString strTmp="";
	//int tmp=0;
	CString strTemp="";
	memcpy(xitongcanshu,&pcBuf[26],11);
	LaserParam711.m_u16SysMode_SingleOrScanf=(byte)pcBuf[37]&0xff;	       LaserParam711.m_u16Channel_Sel=(byte)pcBuf[38]&0xff;
	m_ChannelSelect.SetCurSel(LaserParam711.m_u16Channel_Sel);
	if(m_ChannelSelect.GetCurSel()==1)
	{
		m_AllSingleError=500;
	}else{
		m_AllSingleError=300;
	}

	strTemp = "��ѯϵͳ�����ɹ�";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(TRUE);
	if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
	{
		StopTimer(TIMERGETPARMA);
		if (LaserParam711.m_u16SysMode_SingleOrScanf==0 && LaserParam711.m_u16Channel_Sel == g_ChannelSelect)
		{
			AutoCorretRegInfo.nParamChCode=1;
		}
		else
		{
			AutoCorretRegInfo.nParamChCode=2;
		}
		SetEvent(Event_PARAM);
	}
	UpdateData(FALSE);

}
void CLaseAutoCorrectDlg::H715_ParseAPD(char *pcBuf, int nBufSize)
{
	int nTmp = 0;
	int i=0;
	CString strTmp="";
	CString strTemp="";
	LaserParam711.m_u16APDVSet=((((byte)pcBuf[26]&0xff)<<8) + ((byte)pcBuf[27]&0xff))/100;
	m_ApdPress = LaserParam711.m_u16APDVSet;
	LaserParam711.m_u16APDTemperValue=((((byte)pcBuf[28]&0xff)<<8) + ((byte)pcBuf[29]&0xff))/100;
	m_ApdTemp = LaserParam711.m_u16APDTemperValue;

	LaserParam711.m_u16APDHV_OP_Ratio=((((byte)pcBuf[30]&0xff)<<8) + ((byte)pcBuf[31]&0xff))*10;
	m_ApdXs = LaserParam711.m_u16APDHV_OP_Ratio;

	strTemp = "��ѯAPD�����ɹ�";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
	{
		StopTimer(TIMERGETPARMA);
		if (m_ApdXs>880||m_ApdXs<600)
		{
			AutoCorretRegInfo.nParamChCode=2;
		}
		else
		{
			AutoCorretRegInfo.nParamChCode=4;  //���������������
			g_CheckAPD=true;
		}
		SetEvent(Event_PARAM);
	}
	UpdateData(FALSE);
}

void CLaseAutoCorrectDlg::OnBnClickedButtonGetparam()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	::SendMessage(this->GetSafeHwnd(),WM_LASER_GETPARAM,0,0);
	
}

void CLaseAutoCorrectDlg::OnBnClickedButtonSetparam()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	::SendMessageA(this->GetSafeHwnd(),WM_LASER_SETPARAM,0,0);
}

void CLaseAutoCorrectDlg::H02_SetCmd54()
{
	BYTE SysParamBuff[137]={0x02, 0x02,0x02, 0x02,0x00,0x00,0x00,0x80,0x73,0x53,0x4e,0x00,0x4c,0x4d,0x44,0x73,0x63,0x61,0x6e,0x64,0x61,0x74,0x61,0x20};
	int SysBuffLen=136;
	int nLen=0;
	int j=0;
	char ch[12]={0};
	CString strTemp="";

	UpdateData(TRUE);
	LaserParam.u16APDHvValue=m_ApdPress;
	LaserParam.u16APDTemperValue=m_ApdTemp;
	if (bAutoCorrTerm)
	{
		LaserParam.u16APDHV_OP_Ratio=800;
		m_ApdXs=800;
	}
	else
	{
		LaserParam.u16APDHV_OP_Ratio=m_ApdXs;
	}
	
	//nLen = m_MacAddr.GetLength();
	//if (nLen != 12)
	//{
	//	strTemp="MAC����12λ";
	//	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	//	return;
	//}
	nLen = m_SetMAC.GetLength();
	if (nLen != 12)
	{
		strTemp="MACӦ��Ϊ12λ";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		return;
	}
	for (j=0;j<nLen; j++)
	{
		//ch[j] = ConvertHexChar(m_MacAddr[j]);
		ch[j] = ConvertHexChar(m_SetMAC[j]);
	}
	LaserParam.u16My_Phy_Addr[0] = (ch[0]<<4)+ch[1];
	LaserParam.u16My_Phy_Addr[1] = (ch[2]<<4)+ch[3];
	LaserParam.u16My_Phy_Addr[2] = (ch[4]<<4)+ch[5];
	LaserParam.u16My_Phy_Addr[3] = (ch[6]<<4)+ch[7];
	LaserParam.u16My_Phy_Addr[4] = (ch[8]<<4)+ch[9];
	LaserParam.u16My_Phy_Addr[5] = (ch[10]<<4)+ch[11];
	memcpy(SysParamBuff+24,&LaserParam,sizeof(LaserParam));
	Add_BCC((char *)SysParamBuff,136);
	g_nNetSendBuffLen=137;
	memcpy(g_nNetSendBuff,SysParamBuff,g_nNetSendBuffLen);
	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="�������ò���ָ��";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

}

char CLaseAutoCorrectDlg::ConvertHexChar(char ch)
{
	if((ch>='0')&&(ch<='9'))
		return ch-0x30;
	else if((ch>='A')&&(ch<='F'))
		return ch-'A'+10;
	else if((ch>='a')&&(ch<='f'))
		return ch-'a'+10;
	else return (-1);
}

LONG CLaseAutoCorrectDlg::OnSendNetData(WPARAM wParam, LPARAM lParam)         //��Ӧ����ҳ�е������豸��������
{
	m_nSendBufDataSize = (int)wParam;	
	memcpy(m_cSendBufData, (char *)lParam, m_nSendBufDataSize);
	
	m_NetConnection.pcSendDataBuf = m_cSendBufData;
	m_NetConnection.nSendDataBufSize = m_nSendBufDataSize;

	if (g_SysDevParam.LaserPor == 1)                       //TCPЭ��
	{
		SendData(&m_NetConnection);
	}
	else if (g_SysDevParam.LaserPor ==0)                  //UDPЭ��
	{
		DWORD dwIP;
		m_DestIp.GetAddress(dwIP);
		
		SOCKADDR_IN addrTo;
		addrTo.sin_family=AF_INET;
		addrTo.sin_port=htons(m_DestPort);
		addrTo.sin_addr.S_un.S_addr=htonl(dwIP);             //�õ�Ŀ��IP�Ͷ˿ں�
		UDP_SendData(&m_NetConnection,&addrTo);
	}
	
	return 0L;
}

void CLaseAutoCorrectDlg::UDP_DrawWave(char *pcDataBuf, int nBufCount,int nChannel)
{

	int i,j;
	int nDataJi,nDataNum;  //������Yֵ����Ч���ݳ���
	int k =0 ;

	//������1����ռ2λ
	nDataNum = (pcDataBuf[7]<<8) + (BYTE)pcDataBuf[6]; //�����ݣ���λ�ں󣬵�λ��ǰ
	i = 8;


	int nNum = 0;

	for (j=(nBufCount-1)*nDataNum; j<nBufCount*nDataNum; j++)
	{
		nDataJi =0;
		//�˴������ֽ�Ҫ��0xFF���룬������Ϊ��ֵ
		nDataJi = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
		i+=4;
		switch (nChannel)
		{
		case 1:					
			m_prop1.m_ChartReal.Series(0).Add(nDataJi,"",0); //һ·
			break;
		case 2:
			//	m_twodata = nDataJi;			
			m_prop1.m_ChartReal.Series(1).Add(nDataJi,"",0);//��·
			break;
		case 3:			
			//		m_threedata = nDataJi;
			m_prop1.m_ChartReal.Series(2).Add(nDataJi,"",0);
			break;
		default:
			break;
		}

	}

}

void CLaseAutoCorrectDlg::UDP_DrawWave711(char *pcDataBuf, int nBufCount,int nChannel)
{

	int i,j;
	int nDataJi,nDataNum;  //������Yֵ����Ч���ݳ���
	int k =0 ;
	if(g_SysDevParam.LaserType==4)
	{
		//������1����ռ2λ
		nDataNum = (pcDataBuf[27]<<8) + (BYTE)pcDataBuf[28]; //�����ݣ���λ�ں󣬵�λ��ǰ
		i = 29;
		int nNum = 0;

		for (j=(nBufCount-1)*nDataNum; j<nBufCount*nDataNum; j++)
		{
			nDataJi =0;
			//�˴������ֽ�Ҫ��0xFF���룬������Ϊ��ֵ
			nDataJi = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			i+=4;
			switch (nChannel)
			{
			case 1:					
				m_prop1.m_ChartReal.Series(0).Add(nDataJi,"",0); //һ·
				break;
			case 2:
				//	m_twodata = nDataJi;			
				m_prop1.m_ChartReal.Series(1).Add(nDataJi,"",0);//��·
				break;
			case 3:			
				//		m_threedata = nDataJi;
				m_prop1.m_ChartReal.Series(2).Add(nDataJi,"",0);
				break;
			default:
				break;
			}
		}
	}
	else if(g_SysDevParam.LaserType==3 || g_SysDevParam.LaserType==6)
	{
		//������1����ռ2λ
		nDataNum = (pcDataBuf[26]<<8) + (BYTE)pcDataBuf[27]; //�����ݣ���λ�ں󣬵�λ��ǰ
		i = 28;


		int nNum = 0;

		for (j=(nBufCount-1)*nDataNum; j<nBufCount*nDataNum; j++)
		{
			nDataJi =0;
			//�˴������ֽ�Ҫ��0xFF���룬������Ϊ��ֵ
			nDataJi = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			i+=4;
			switch (nChannel)
			{
			case 1:					
				m_prop1.m_ChartReal.Series(0).Add(nDataJi,"",0); //һ·
				break;
			case 2:
				//	m_twodata = nDataJi;			
				m_prop1.m_ChartReal.Series(1).Add(nDataJi,"",0);//��·
				break;
			case 3:			
				//		m_threedata = nDataJi;
				m_prop1.m_ChartReal.Series(2).Add(nDataJi,"",0);
				break;
			default:
				break;
			}

		}
	}
	else if(g_SysDevParam.LaserType==5)
	{
		//������1����ռ2λ
		nDataNum = (pcDataBuf[7]<<8) + (BYTE)pcDataBuf[6]; //�����ݣ���λ�ں󣬵�λ��ǰ
		i = 8;


		int nNum = 0;

		for (j=(nBufCount-1)*nDataNum; j<nBufCount*nDataNum; j++)
		{
			nDataJi =0;
			//�˴������ֽ�Ҫ��0xFF���룬������Ϊ��ֵ
			nDataJi = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			i+=4;
			switch (nChannel)
			{
			case 1:					
				m_prop1.m_ChartReal.Series(0).Add(nDataJi,"",0); //һ·
				break;
			case 2:
				//	m_twodata = nDataJi;			
				m_prop1.m_ChartReal.Series(1).Add(nDataJi,"",0);//��·
				break;
			case 3:			
				//		m_threedata = nDataJi;
				m_prop1.m_ChartReal.Series(2).Add(nDataJi,"",0);
				break;
			default:
				break;
			}

		}
	}
}

void CLaseAutoCorrectDlg::UDP_DrawWave_710A(char *pcDataBuf, int nBufCount,int nChannel)
{

	int i,j;
	int nDataJi,nDataNum;  //������Yֵ����Ч���ݳ���
	int k =0 ;

	//��Ч�ֽ���
	nDataNum = (pcDataBuf[7]<<8) + (BYTE)pcDataBuf[8]; //�����ݣ���λ�ں󣬵�λ��ǰ
	i = 9;


	int nNum = 0;
	for (j=(nBufCount-1)*nDataNum; j<nBufCount*nDataNum; j++)
	{
		nDataJi =0;
		//�˴������ֽ�Ҫ��0xFF���룬������Ϊ��ֵ
		nDataJi = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
		i+=4;
		switch (nChannel)
		{
		case 1:					
			m_prop1.m_ChartReal.Series(0).Add(nDataJi,"",0); //һ·
			break;
		case 2:			
			m_prop1.m_ChartReal.Series(1).Add(nDataJi,"",0);//��·
			break;
		case 3:			
			m_prop1.m_ChartReal.Series(2).Add(nDataJi,"",0);
			break;
		default:
			break;
		}
	}
}

void CLaseAutoCorrectDlg::OnBnClickedButtonOpenslot()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	if (Com_Send(a_ucTemp11,6))
	{
		strTemp="���͹����ſ�ָ��ɹ�";
	}
	else
	{
		strTemp="���͹����ſ�ָ��ʧ��";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,1,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::OnBnClickedButtonCloseslop()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	if (Com_Send(a_ucTemp10,6))
	{
		strTemp="���͹����պ�ָ��ɹ�";
	}
	else
	{
		strTemp="���͹����պ�ָ��ʧ��";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,1,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::OnBnClickedButtonMoveslop()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	UINT16 CaclCRC=0;
	CString str="";
	GetDlgItem(IDC_EDIT_SlopSteps)->GetWindowTextA(str);
	m_slopeSteps=_ttoi(str);
	int step=m_slopeSteps;

	if(m_slopeSteps==0)
	{
		step=1;
		//UpdateData(FALSE);
		str.Format("%d",step);
		GetDlgItem(IDC_EDIT_SlopSteps)->SetWindowTextA((LPCTSTR)str);
	}

	if (m_slopDir.GetCurSel()==0)
	{
		strTemp="����"+str;
	}
	else if (m_slopDir.GetCurSel()==1)
	{
		step=-step;
		strTemp="����"+str;
	}
	else
	{
		strTemp="����"+str;
	}
	//if (m_slopeSteps>2000)
	//{
	//	m_slopeSteps=2000;
	//	//UpdateData(FALSE);
	//	str.Format("%d",m_slopeSteps);
	//	GetDlgItem(IDC_EDIT_SlopSteps)->SetWindowTextA((LPCTSTR)str);
	//}
	//else if(m_slopeSteps==0)
	

	a_ucTemp9[3]=((step>>24)&0xFF);
	a_ucTemp9[4]=((step>>16)&0xFF);
	a_ucTemp9[5]=((step>>8)&0xFF);
	a_ucTemp9[6]=(step&0xFF);

	CaclCRC=CRC712_Add(a_ucTemp9,7);

	a_ucTemp9[7]=(CaclCRC>>8)&0xFF;
	a_ucTemp9[8]=(CaclCRC)&0xFF;

	if (Com_Send(a_ucTemp9,9))
	{
		strTemp=strTemp+",���͹�������ָ��ɹ�";
	}
	else
	{
		strTemp=strTemp+",���͹�������ָ��ʧ��";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

//�����ã���˾�����������壬��Ӧ��Э��
/*void CLaseAutoCorrectDlg::OnBnClickedButtonMoveslop()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	UINT16 CaclCRC=0;
	//UpdateData(TRUE);
	CString str="";
	GetDlgItem(IDC_EDIT_SlopSteps)->GetWindowTextA(str);
	m_slopeSteps=_ttoi(str);

	if (m_slopDir.GetCurSel()==0)
	{
		a_ucTemp3[6]=0xAA;
		strTemp="����"+str;
	}
	else if (m_slopDir.GetCurSel()==1)
	{
		a_ucTemp3[6]=0xBB;
		strTemp="����"+str;
	}
	else
	{
		a_ucTemp3[6]=0xAA;
		strTemp="����"+str;
	}
	//if (m_slopeSteps>2000)
	//{
	//	m_slopeSteps=2000;
	//	//UpdateData(FALSE);
	//	str.Format("%d",m_slopeSteps);
	//	GetDlgItem(IDC_EDIT_SlopSteps)->SetWindowTextA((LPCTSTR)str);
	//}
	//else if(m_slopeSteps==0)
	if(m_slopeSteps==0)
	{
		m_slopeSteps=1;
		//UpdateData(FALSE);
		str.Format("%d",m_slopeSteps);
		GetDlgItem(IDC_EDIT_SlopSteps)->SetWindowTextA((LPCTSTR)str);
	}

	a_ucTemp3[7]=((m_slopeSteps>>8)&0xFF);
	a_ucTemp3[8]=(m_slopeSteps&0xFF);

	CaclCRC=CRC16_Cal(a_ucTemp3+2,10);

	a_ucTemp3[12]=(CaclCRC>>8)&0xFF;
	a_ucTemp3[13]=(CaclCRC)&0xFF;

	if (Com_Send(a_ucTemp3,15))
	{
		strTemp=strTemp+",���͹�������ָ��ɹ�";
	}
	else
	{
		strTemp=strTemp+",���͹�������ָ��ʧ��";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}*/

void CLaseAutoCorrectDlg::OnOneKeyTestCorrect()
{
	CString strTemp="";
	if (Com_Send(a_ucTemp4,12))
	{
		strTemp="����һ����֤�ɹ�";
	}
	else
	{
		strTemp="����һ����֤ʧ��";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	//WriteLog(&m_RichEdit,strTemp);
}

void CLaseAutoCorrectDlg::OnOnekeyTestFinish()
{
	CString strTemp="";
	if (Com_Send(a_ucTemp5,12))
	{
		strTemp="������֤�����ɹ�";
	}
	else
	{
		strTemp="������֤����ʧ��";
	}
	//WriteLog(&m_RichEdit,strTemp);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CommSendStop(LPVOID lpParam, int nStopType)
{
	CString strTemp="";
	strTemp.Format("Debug:Send-%d",nStopType);
	//CLaseAutoCorrectDlg *pMainDlg=(CLaseAutoCorrectDlg *)(AfxGetApp()->GetMainWnd());
	////pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
	//::SendMessageA(pMainDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

BOOL Com_Send(BYTE *out_buff,int len)
{
	BOOL nRes=FALSE;
 	SerPortPar serPortPar;
	CLaseAutoCorrectDlg *pMainDlg=(CLaseAutoCorrectDlg *)(AfxGetApp()->GetMainWnd());
	serPortPar.nPort = g_SysDevParam.nPort;
	serPortPar.nBaud=g_SysDevParam.nBuad;
	serPortPar.pDataBuf = out_buff;
	serPortPar.nDataBufSize = len;
	serPortPar.lpSendFun = (LPVOID)CommSendStop;
	serPortPar.lpParam = (LPVOID)pMainDlg;	
	nRes=SendComm(&serPortPar);
	return nRes;
}

void OnComRecData(LPVOID lpParam, BYTE *pDataBuf, int nDataBufSize)
{
	int i=0;
	CString strTemp="";
	CLaseAutoCorrectDlg *pDlg = (CLaseAutoCorrectDlg *)lpParam;

	//strTemp.Format("Debug:Recevie-%d",nDataBufSize);
	////pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
	//::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

	if (nDataBufSize == 2)
	{
		memcpy(g_nComParseBuff,pDataBuf,2);
		PostMessage(pDlg->GetSafeHwnd(),WM_COM_PARSE,g_nComParseBuff[1],0);
	}

	//for(i=0;i<nDataBufSize;i++)
	//{				
	//	//�ҵ�֡ͷ
	//	if(g_State==0)
	//	{
	//		if(pDataBuf[i]==0xFF)
	//		{
	//			g_State=1;
	//			gcnt=0;
	//		}
	//		else
	//		{
	//			g_State=0;
	//			gcnt=0;
	//		}
	//	}
	//	//��һ֡����֡ͷ���֣������ɸ�֡��֡β����֡��֡ͷ����һ�飬ȥ��ǰһ��ff
	//	else if(g_State == 1)
	//	{
	//		if (pDataBuf[i] != 0xAA)
	//		{
	//			g_State = 0;
	//			gcnt=0;
	//		}
	//		else 
	//		{	
	//			g_State=2;
	//			DataBuf[gcnt++] = 0xFF;
	//			DataBuf[gcnt++] = 0xAA;
	//		}
	//	}
	//	else if (g_State == 2)
	//	{
	//		DataBuf[gcnt++] = pDataBuf[i];

	//		if (gcnt>3)
	//		{
	//			g_FramLen=((DataBuf[2]<<8)+DataBuf[3]+3);

	//			if (gcnt==g_FramLen)
	//			{
	//				if (DataBuf[gcnt-1]==0xEE)
	//				{
	//					//������֡
	//					memcpy(g_nComParseBuff,DataBuf,g_FramLen);
	//					if (CRC16_Check(DataBuf,g_FramLen))
	//					{
	//						if (g_nComParseBuff[4]==0x09)
	//						{
	//							PostMessage(pDlg->GetSafeHwnd(),WM_COM_PARSE,g_nComParseBuff[5],0);
	//						}
	//					}
	//					else
	//					{
	//						strTemp="CRC16У��ʧ��";
	//						//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
	//						::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	//					}
	//					//g_nDataStoreLen[g_nDataBufWriteIndex] = gcnt;
	//					//g_nDataBufWriteIndex++;
	//					//g_nDataBufWriteIndex = g_nDataBufWriteIndex % RECV_DATA_BUF_SIZE;
	//					gcnt = 0;
	//					g_State=0;
	//					g_FramLen=0;
	//				}
	//			}
	//			else if (gcnt>400)
	//			{
	//				gcnt=0;
	//				g_State=0;
	//				g_FramLen=0;
	//			}
	//		}
	//	} 	
	//}
}

LRESULT CLaseAutoCorrectDlg::OnComParse(WPARAM wParam, LPARAM lParam)
{
	CString strTemp="";
	CString strTempState="";
	int nCmd=(int)wParam;
	switch(nCmd)
	{
	case 0x01:
		strTemp="�����˶���";
		strTempState="�˶���";
		::SendMessageA(this->GetSafeHwnd(),WM_MOTOSTATE,0,(LPARAM)(LPCTSTR)strTempState);
		break;
	case 0x02:
		strTemp="�����˶�ֹͣ";
		strTempState="ֹͣ";
		::SendMessageA(this->GetSafeHwnd(),WM_MOTOSTATE,0,(LPARAM)(LPCTSTR)strTempState);
		if (g_bMoveFlag==true)
		{
			strTemp="����΢�˶��ɹ�";
			g_bMoveFlag=false;
			StopTimer(TIMERGLADJUST);
			if (AutoCorretRegInfo.nAqcMkCode==2)
			{
				AutoCorretRegInfo.nAqcMkCode=3;
				SetEvent(Event_AQCMK);
			}
		}
		break;
	case 0x03:
		strTemp="�ִ�����λ";
		strTempState="����λ";
		::SendMessageA(this->GetSafeHwnd(),WM_MOTOSTATE,0,(LPARAM)(LPCTSTR)strTempState);
		if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
		{
			StopTimer(TIMERORIGMK);
			strTemp="�������ٹرճɹ�";
			AutoCorretRegInfo.nStepIndx=STEP_MINMK;
			g_bCountStart=1;//����ͳ���ж���С����
		}		
		break;
	case 0x04:
		strTemp="�ִ�����λ";
		strTempState="����λ";
		::SendMessageA(this->GetSafeHwnd(),WM_MOTOSTATE,0,(LPARAM)(LPCTSTR)strTempState);
		if (AutoCorretRegInfo.nStepIndx==STEP_MINMK)
		{
			StopTimer(TIMERORIGMK);
			strTemp="���������ſ��ɹ�";
			AutoCorretRegInfo.nStepIndx=STEP_MAXMK;
			g_bCountStart=1;//����ͳ���ж��������
		}
		else if(AutoCorretRegInfo.nStepIndx==STEP_FINALCH)
		{
			StopTimer(TIMERORIGMK);
			strTemp="��ʼ��֤����Ч��";
			AutoCorretRegInfo.nFinalCkCode=3;
			m_prop1.m_ChartReal.Series(0).Clear();
			m_prop1.m_ChartReal.Series(1).Clear();
			m_prop1.m_ChartReal.Series(2).Clear();
			g_nRevcNum3=0;
			SetEvent(Event_FINALCK);
		}
		break;
	case 0x10:
		strTemp="������Ӧ�ɹ�";
		if (g_nComComandStat==1)
		{
			//StopTimer(TIMEROPENING);
			StopTimer(TIMER712COMSEND);
			g_nComComandStat=0;
			g_nContinuousMoveState = 1;   //��ʾ��ǰ״̬Ϊ��ʼ�ſ�
		}
		else if (g_nComComandStat==2)
		{
			StopTimer(TIMER712COMSEND);
			g_nComComandStat=0;
			if (g_nContinuousMoveState == 1)
			{
				StopTimer(TIMEROPENING);
				g_nContinuousMoveState=0;
				if (AutoCorretRegInfo.nStepIndx==STEP_MAXMK)
				{
					g_bCountStart=1;   //��ʼͳ���Ƿ�ﵽ450ns
				}
				else if(AutoCorretRegInfo.nStepIndx==STEP_BEFORECHECKOPEN)
				{
					AutoCorretRegInfo.nStepIndx=STEP_FINALCH;
					strTemp="��ʼ��֤����Ч��";
					AutoCorretRegInfo.nFinalCkCode=3;
					m_prop1.m_ChartReal.Series(0).Clear();
					m_prop1.m_ChartReal.Series(1).Clear();
					m_prop1.m_ChartReal.Series(2).Clear();
					g_nRevcNum3=0;
					SetEvent(Event_FINALCK);
				}
			}
			else if (g_nContinuousMoveState == 2)
			{
				StopTimer(TIMERCLOSING);
				g_nContinuousMoveState = 0;
			}	
		}
		else if (g_nComComandStat==3)
		{
			StopTimer(TIMER712COMSEND);
			g_nComComandStat=0;
			g_nContinuousMoveState = 2;   //��ʾ��ǰ״̬Ϊ��ʼ�պ�
			g_bCountStart=3;    //�ɹ���ʼ�رպ�ʼ��֤
		}
		break;
	case 0x20:
		strTemp="������Ӧʧ��";
		break;
	default:
		strTemp="�����յ�δ֪�ظ�";
		break;
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,1,(LPARAM)(LPCTSTR)strTemp);
	return 0L;
}

afx_msg LRESULT CLaseAutoCorrectDlg::OnMotoState(WPARAM wParam,LPARAM lParam)
{
	m_CurrentState=(char*)lParam; 
	GetDlgItem(IDC_EDIT_CurrentState)->SetWindowTextA(m_CurrentState);
	return   0L;
}

void CLaseAutoCorrectDlg::OnBnClickedButtonAutocorrect()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	//�������ڽ��ս����߳�
	CWinThread *pThread;
	CString BtnCaption="";
	(GetDlgItem(IDC_BUTTON_AutoCorrect))->GetWindowTextA(BtnCaption);
	if (g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==4)
	{
		g_ChannelSelect = m_ChannelSelect.GetCurSel();
		g_CheckAPD=false;
		Get711LmdSinglePoint();
		if(g_ChannelSelect==1)
		{
			MK_ORG0=390;
			MK_ORG1=450;
		}
		else
		{
			MK_ORG0=190;
			MK_ORG1=300;
		}
	}
	else   //����712
	{
		MK_ORG0=450;
		MK_ORG1=500;
	}
	
	if (BtnCaption=="����һ������")
	{
		if (g_SysDevParam.NetConnected&&g_SysDevParam.ComConnected)
		{
			//��ʼ����ر���
			InitAutoCorrectTask();
			//���������߳�
			bAutoCorrTerm=TRUE;
			pThread = ::AfxBeginThread(AutoCorrThreadProc,this);//���������߳�
			if(pThread == NULL)
			{
				bAutoCorrTerm=FALSE;
				return;
			}
			SetEvent(Event_START);
			b711AFirstClose=true;

			//SetEvent(Event_FINALCK);//������
			//���Բ�����
			//SetEvent(Event_AQCMK);
			//AutoCorretRegInfo.nStepIndx=STEP_MAXMK;
			//BeginValue=450000;

			(GetDlgItem(IDC_BUTTON_AutoCorrect))->SetWindowTextA("ֹͣһ������");
		}
		else
		{
			AfxMessageBox(_T("������������״̬"),MB_OK|MB_ICONSTOP);
		}
	}
	else if (BtnCaption=="ֹͣһ������")
	{
		SetEvent(Event_STOP);
		/*bAutoCorrTerm=FALSE;
		bTargetAqcTerm=FALSE;
		InitAutoCorrectTask();
		(GetDlgItem(IDC_BUTTON_AutoCorrect))->SetWindowTextA("����һ������");*/
	}
}

UINT AutoCorrThreadProc(LPVOID lpParam)
{
	CLaseAutoCorrectDlg *pDlg = (CLaseAutoCorrectDlg*)lpParam;
	CString strTmp="";
	DWORD nRet =0;
	int nFitRes=0;
	CString Filename="";
	CString strTemp="";
	CWinThread *pThread=NULL;

	while (bAutoCorrTerm)
	{

		//{Event_START,Event_PARAM,Even_ORIMK,Event_AQCMK,Event_ALIVE,Event_FITMK,Event_DOWNTB,Event_CHECKTB,Event_FINALCK,Event_OVEROPEN,Event_STOP};
		nRet = WaitForMultipleObjects(11,Event_List,false,600*1000);	//�ȴ���ʱ

		if(WAIT_TIMEOUT == nRet)	//�ȴ��¼���ʱʱ�䵽
		{
			strTemp="�Զ�������ʱ��ֹ";
			//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,0);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
			SetEvent(Event_STOP);
		}
		else if(WAIT_OBJECT_0 == nRet)	//Event_START �������ź�״̬ �����Զ���������
		{
			//���뿪ʼ״̬ ��ȡ���� ��������
			strTemp="�����Զ���������";
			//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
			AutoCorretRegInfo.nStepIndx=STEP_START;//��������
			AutoCorretRegInfo.nTaskCode=1;//�����ɹ�
			::PostMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,1);
			Sleep(1000);
			SetEvent(Event_PARAM);
		}
		else if((WAIT_OBJECT_0 + 1) == nRet)	//Event_PARAM���ź� ����г�ʼ�������
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_START)
			{ 
				//g_AutoStep=STEP_PARAM;
				strTemp="����������׶�";
				//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,2);
				AutoCorretRegInfo.nStepIndx=STEP_PARAM;
				//���ò�����ѯ�����ð�ť
				pDlg->GetDlgItem(IDC_BUTTON_GetParam)->EnableWindow(FALSE);
				pDlg->GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(FALSE);
				::PostMessageA(pDlg->GetSafeHwnd(),WM_LASER_GETPARAM,0,0);
				pDlg->StartTimer(TIMERGETPARMA,TIMER_GETPARAM,5000);
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
			{
				if (AutoCorretRegInfo.nParamChCode==1)
				{
					strTemp="����������ͨ��";
					//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					
					//////////////////////////////////////////////////////////////////////////////////////////////
					SetEvent(Even_ORIGMK);  
				}
				else if (AutoCorretRegInfo.nParamChCode==2)
				{
					strTemp="������鲻��Ҫ��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if (AutoCorretRegInfo.nParamChCode==3)
				{
					strTemp="�����޷���ȡ��";
					//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if (AutoCorretRegInfo.nParamChCode==4)
				{
					if (g_SysDevParam.LaserType==6)   //712�ȴ�������ʵ����
					{
						//���������ļ��м��ļ�
						pDlg->OnCreateFolder(pDlg->m_MacAddr);

						CString name="���ͼ";
						pDlg->TraverseDir(CString(pDlg->m_SavePath), filenames);
						for(int i=0;i<filenames.size();i++)
						{
							if(filenames[i].Find(name)>=0)
							{
								g_nameadd=TRUE;
								int t=filenames[i].FindOneOf("(");
								if(t>0)
								{
									int hou=filenames[i].FindOneOf(")");
									CString fileindex= filenames[i].Mid(t+1,hou-t-1);
									int coverindex=atoi(fileindex); 
									coverindex++;
									if(coverindex> atoi(g_cover))
									{
										g_cover.Format("%d",coverindex); 
									}
								}

							}
						}

						if(g_nameadd)
						{
							pDlg->OnCreateFile(pDlg->m_MacAddr+"_ԭʼ����"+"("+g_cover+").txt",1);
							pDlg->OnCreateFile(pDlg->m_MacAddr+"_���Լ�¼"+"("+g_cover+").txt",2);
						}
						else
						{
							pDlg->OnCreateFile(pDlg->m_MacAddr+"_ԭʼ����.txt",1);
							pDlg->OnCreateFile(pDlg->m_MacAddr+"_���Լ�¼.txt",2);
						}

						//pDlg->OnCreateFile(pDlg->m_MacAddr+"_���Լ�¼.txt",2);
						m_pMyLog = CCDMLogInfo::GetInstance(pDlg->m_OperTxtPath.GetBuffer(pDlg->m_OperTxtPath.GetLength()));//��ȡ��־�ļ�
						strTemp=  "\r\n========" + CTime::GetCurrentTime().Format("%H:%M:%S") + "========\r\n";
						if(m_pMyLog)	//д����־
							m_pMyLog->SetNotify(strTemp.GetBuffer(strTemp.GetLength()));
						strTemp="�������ݱ����ļ�";
						//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
						::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

						AutoCorretRegInfo.nWaitInputDistCnt=0;
						strTmp="�����������Ϣ��Ȼ����[����ȷ��]��ť";
						::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						AfxMessageBox(_T("�����������Ϣ��Ȼ����[����ȷ��]��ť"),MB_OK);
						pDlg->StartTimer(TIMERINPUTDIST,TIMER_INPUTDIST,5000);
					}
					else
					{
						//strTemp="�������711����������";
						::PostMessageA(pDlg->GetSafeHwnd(),WM_LASER_GETPARAM,0,0);
						pDlg->StartTimer(TIMERGETPARMA,TIMER_GETPARAM,10000);
					}
				}
			}
		}
		else if ((WAIT_OBJECT_0 + 2) == nRet)//Even_ORIGMK���ź� ��ʼ��ʼ������  712Ϊ���ֵ���450ns
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
			{
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,3);
				//�ж���������Ƿ�Ϊ450ns
				strTemp="�ж���������Ƿ�Ϊ450ns";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				AutoCorretRegInfo.nStepIndx=STEP_MAXMK;
				g_bCountStart=1;   //��ʼͳ���Ƿ�ﵽ450ns
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_MAXMK)
			{
				if (AutoCorretRegInfo.nOrigMkChCode==2)
				{
					//δ��450ns
					//g_bCountStart=0;   //������㣬ֻ��ÿ��ͳ��ֱ������450ns������ֹͣ�ſ�
					g_bOpenflag=true;  //��ʼʵʱ�ж��ſ���ʱֹͣ
					strTemp="�������ֵδ��450ns���Զ��ſ�����";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_OPEN,0,0);
				}
				else if (AutoCorretRegInfo.nOrigMkChCode==1)
				{
					strTemp="�������ֵok";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					SetEvent(Event_AQCMK);
				}
				else if(AutoCorretRegInfo.nOrigMkChCode==3)
				{
					strTemp="�������ֵ�޷��ﵽ450ns";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if(AutoCorretRegInfo.nOrigMkChCode==4)
				{
					//ʶ��ﵽ450ns��ֹͣ�ſ�
					strTemp="ʶ��ﵽ450ns��ֹͣ�ſ�";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);
					//��ʼͳ���Ƿ�ﵽ450ns,����ֹͣ��Ӧ���޸ı�־λ
				}
			}
		}
		else if ((WAIT_OBJECT_0 + 3) == nRet) //Event_AQCMK ����ԭʼ���ݲɼ�
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_MAXMK)
			{
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,4);
				AutoCorretRegInfo.nStepIndx=STEP_ACQMK;
				//��ʼ���ɼ�Ŀ��
				pDlg->InitAqcDataTarget();
				bTargetAqcTerm=TRUE;
				pThread = ::AfxBeginThread(TargetDataAqcThread,lpParam);//����ԭʼ���ݲɼ��߳� ���Զ����������ź�
				SetEvent(Event_AQCMK);
				//��ʼ��������󣬽���ԭʼ���ݲɼ� ���Զ������������ź�
				strTemp="����ԭʼ���ݲɼ�";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);		
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_ACQMK)
			{
				if (AutoCorretRegInfo.nAqcMkCode==1)
				{
					pDlg->SaveOrigData();
					strTemp="����ԭʼ���ݳɹ�";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					SetEvent(Event_FITMK);
				}
				else if (AutoCorretRegInfo.nAqcMkCode==2)
				{
					if (g_Pid.e_0>0)
					{
						//������0 ����ת����
						if ((g_Pid.e_0>(g_AcqTartetInfo.KeyValue[g_AcqTartetInfo.AqcIndx-1]-g_AcqTartetInfo.KeyValue[g_AcqTartetInfo.AqcIndx])/2))   //g_AcqTartetInfo.KeyValue[g_AcqTartetInfo.AqcIndx]>16 && 
						{
							pDlg->m_slopDir.SetCurSel(1);
							pDlg->m_slopeSteps=(UINT)(2*g_Pid.e_0*g_AcqTartetInfo.Step[g_AcqTartetInfo.AqcIndx]/10)/1000;
							
						} 
						else
						{
							pDlg->m_slopDir.SetCurSel(1);
							pDlg->m_slopeSteps=(UINT)(g_Pid.e_0*g_AcqTartetInfo.Step[g_AcqTartetInfo.AqcIndx]/10)/1000;
						}
						if (abcd<0)
						{
							int aa=pDlg->m_slopeSteps;
							if (pDlg->m_slopeSteps>=(-abcd))
							{
								pDlg->m_slopeSteps=(-abcd)/2;
							}
							else if ((-abcd-aa)<((-abcd)/4))
							{
								pDlg->m_slopeSteps=(-abcd)/2;
							}			
						}
						else if(abcd != 0)
						{
							if (pDlg->m_slopeSteps > abcd)
							{
								pDlg->m_slopeSteps=abcd;
							}
						}	
						//���������
						if (g_AcqTartetInfo.KeyValue[g_AcqTartetInfo.AqcIndx] <= 120)    //711���ڵ�����ʱ������ÿ�ε������ֵΪ50��  (g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==4) && g_ChannelSelect==1 && 
						{
							if (pDlg->m_slopeSteps>30)
							{
								pDlg->m_slopeSteps=30;
							}
						}
						else
						{
							if (pDlg->m_slopeSteps>10000)
							{
								pDlg->m_slopeSteps=10000;
							}
						}
						abcd=pDlg->m_slopeSteps;
						strTmp.Format("%d",pDlg->m_slopeSteps);
						pDlg->GetDlgItem(IDC_EDIT_SlopSteps)->SetWindowTextA((LPCTSTR)strTmp);
						
					}
					else
					{
						//���С��0 ��ת����
						pDlg->m_slopDir.SetCurSel(0);
						pDlg->m_slopeSteps=(UINT)(-g_Pid.e_0*g_AcqTartetInfo.Step[g_AcqTartetInfo.AqcIndx]/10)/1000;
						//strTmp.Format("%d",pDlg->m_slopeSteps);
						//pDlg->GetDlgItem(IDC_EDIT_SlopSteps)->SetWindowTextA((LPCTSTR)strTmp);
						if (abcd>0)
						{
							int aa=pDlg->m_slopeSteps;
							if (pDlg->m_slopeSteps>=abcd)
							{
								pDlg->m_slopeSteps=abcd/2;
							}
							else if ((abcd-aa)<(abcd/4))
							{
								pDlg->m_slopeSteps=abcd/2;
							}						
						}
						else if(abcd != 0)
						{
							if (pDlg->m_slopeSteps > (-abcd))
							{
								pDlg->m_slopeSteps=(-abcd);
							}
						}
						//���������
						if (g_AcqTartetInfo.KeyValue[g_AcqTartetInfo.AqcIndx] <= 120)    //711���ڵ�����ʱ������ÿ�ε������ֵΪ50��  (g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==4) && g_ChannelSelect==1 && 
						{
							if (pDlg->m_slopeSteps>30)
							{
								pDlg->m_slopeSteps=30;
							}
						}
						else
						{
							if (pDlg->m_slopeSteps>10000)
							{
								pDlg->m_slopeSteps=10000;
							}
						}
						if(g_SysDevParam.LaserType==4&&g_AcqTartetInfo.AqcIndx==1&&b711AFirstClose)
						{
							pDlg->m_slopeSteps=20000;
							b711AFirstClose=false;
						}
						abcd=-pDlg->m_slopeSteps;
						strTmp.Format("%d",pDlg->m_slopeSteps);
						pDlg->GetDlgItem(IDC_EDIT_SlopSteps)->SetWindowTextA((LPCTSTR)strTmp);
						
					}
					AutoCorretRegInfo.nComReSendCnt=0;
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ADJUST,0,0);
				}
				else if (AutoCorretRegInfo.nAqcMkCode==3)
				{
					//����΢������� ���Լ������вɼ�
					g_bCountStart=1;
				}
				else if (AutoCorretRegInfo.nAqcMkCode==4)
				{
					//����������λ ����ֱ�Ӳɼ�������
					g_bCountStart=2;
				}
				else if (AutoCorretRegInfo.nAqcMkCode==5)
				{
					strTemp="��������ʧЧ��ʱ";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
			}
		}
		else if((WAIT_OBJECT_0 + 4) == nRet)//Event_ALIVE 
		{
			//Do nothing
			;
		}
		else if ((WAIT_OBJECT_0 + 5) == nRet)//Event_FITMK 
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_ACQMK)
			{
				AutoCorretRegInfo.nStepIndx=STEP_FITMK;
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,5);
				::SendMessage(pDlg->GetSafeHwnd(),WM_TAB_CHANGE,0,1);
				
				if(g_nameadd)
				{
					Filename=pDlg->m_MacAddr+"_��������"+"("+g_cover+").txt_MK";
				}
				else
				{
					Filename=pDlg->m_MacAddr+"_��������.txt_MK";
				}
				
				::SendMessage(pDlg->GetSafeHwnd(),WM_FIT_MAINTH,(WPARAM)&nFitRes,(LPARAM)&Filename);    //���
				
				if (nFitRes)
				{
					strTemp="��ϲ�ִֵ�����";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					AutoCorretRegInfo.nFitMkCode=1;
					AutoCorretRegInfo.nChDownTbCnt=1;  //��ѯ��������֤��ͨ���ط�ͳ��
					SetEvent(Event_DOWNTB);
				}
				else
				{
					strTemp="��ϲ�ִֵ��ʧ��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					AutoCorretRegInfo.nFitMkCode=2;
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
			}
		}
		else if((WAIT_OBJECT_0 + 6) == nRet)//Event_DOWNTB �Զ�����������
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_FITMK)
			{
				AutoCorretRegInfo.nStepIndx=STEP_DOWNTB;
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,6);
				strTemp="��ʼ����������";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				AutoCorretRegInfo.nDownTbCode=0;
				::AfxBeginThread(DownLoadCorrectTableThread,lpParam);
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_DOWNTB)
			{
				if (AutoCorretRegInfo.nDownTbCode==1)
				{
					Sleep(500);
					SetEvent(Event_CHECKTB);
				}
				else if (AutoCorretRegInfo.nDownTbCode==2)
				{
					//ʧ��
				}
			}
		}
		else if((WAIT_OBJECT_0 + 7) == nRet)//Event_CHECKTB ��������������Ƿ���ȷ
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_DOWNTB)
			{
				AutoCorretRegInfo.nCheckTbCode=0;
				AutoCorretRegInfo.nStepIndx=STEP_CHECKTB;
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,7);
				::SendMessage(pDlg->GetSafeHwnd(),WM_TAB_CHANGE,0,0);
				strTemp="����������������";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

				pDlg->SendCheckCorrectTb711();    //����712

				AutoCorretRegInfo.nComReSendCnt=0;
				pDlg->StartTimer(TIMERGLADJUST,TIMER_GLADJUST,5000);    //����ͬһ����ʱ�����Ͳ�ѯ������
				//SetEvent(Event_CHECKTB);
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
			{
				if (AutoCorretRegInfo.nCheckTbCode==1)
				{
					strTemp="��������֤��ͨ��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					SetEvent(Event_FINALCK);
				}
				else if (AutoCorretRegInfo.nCheckTbCode==2)
				{
					strTemp="��������֤δͨ��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if (AutoCorretRegInfo.nCheckTbCode==3)
				{
					AutoCorretRegInfo.nCheckTbCode=0;
					AutoCorretRegInfo.nChDownTbCnt++;
					strTemp.Format("��%d�����ؼ���",AutoCorretRegInfo.nChDownTbCnt);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					AutoCorretRegInfo.nStepIndx=STEP_FITMK;
					SetEvent(Event_DOWNTB);
				}
			}
		}
		else if((WAIT_OBJECT_0 + 8) == nRet)//Event_FINALCK ���һ����������Ƿ���
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
			{
				//AutoCorretRegInfo.nStepIndx=STEP_FINALCH;
				AutoCorretRegInfo.nStepIndx=STEP_BEFORECHECKOPEN;
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,8);
				
				strTemp="��֤�����Ƿ���";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				//��ʼһ����֤
				//pDlg->OnOneKeyTestCorrect();
				g_bOpenflag=true;  //��ʼʵʱ�ж��ſ���ʱֹͣ
				strTemp="�Զ��ſ���������450ns";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_OPEN,0,0);
			}
			else if(AutoCorretRegInfo.nStepIndx==STEP_BEFORECHECKOPEN)
			{
				if(AutoCorretRegInfo.nOrigMkChCode==3)
				{
					strTemp="�������ֵ�޷��ﵽ450ns";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if(AutoCorretRegInfo.nOrigMkChCode==4)
				{
					//ʶ��ﵽ450ns��ֹͣ�ſ�
					strTemp="ʶ��ﵽ450ns��ֹͣ�ſ�";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);
				}
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_FINALCH)
			{
				if (AutoCorretRegInfo.nFinalCkCode==1)
				{
					if (g_bCloseflag == true)
					{
						g_bCloseflag = false;
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //ֹͣ�����رչ���
					}
					//������
					//strTemp.Format("���������%d ���Ĺ�%d֡ ���յĹ�%d(%d)(%d)֡ ��ֵ������С֮��Ϊ%d ����������ֵ��СֵΪ%d,%d",g_CheckMKForm.ValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkOffset,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					strTemp.Format("���������%d ���Ĺ�%d֡ ���յĹ�%d(%d)(%d)֡ ��ֵƫ��������СֵΪ%d,%d ����������ֵ��СֵΪ%d,%d",g_CheckMKForm.ValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkMax,g_CheckMKForm.ChMkMin,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,1,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,11);
					strTemp="������һ����֤ͨ��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
				//	Sleep(10000);
					//712��֤ͨ�����ſ�����
					//::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_OPEN,0,0);
					//pDlg->OnOnekeyTestFinish();
					SetEvent(Event_OVEROPEN);
				}
				else if (AutoCorretRegInfo.nFinalCkCode==2)
				{		
					if (g_bCloseflag == true)
					{
						g_bCloseflag = false;
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //ֹͣ�����رչ���
					}
					//������
					//strTemp.Format("���������%d ���Ĺ�%d֡ ���յĹ�%d(%d)(%d)֡ ��ֵ������С֮��Ϊ%d ����������ֵ��СֵΪ%d,%d",g_CheckMKForm.ValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkOffset,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					strTemp.Format("���������%d ���ϸ�֡����%d ���Ĺ�%d֡ ���յĹ�%d(%d)(%d)֡ ��ֵƫ��������СֵΪ%d,%d ����������ֵ��СֵΪ%d,%d",g_CheckMKForm.ValueJudgeCnt,g_CheckMKForm.AveValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkMax,g_CheckMKForm.ChMkMin,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,1,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,10);
					strTemp="������һ����֤δ��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					//712��֤�������ſ�����
					SetEvent(Event_OVEROPEN);
				}
				else if (AutoCorretRegInfo.nFinalCkCode==3)
				{
					pDlg->m_ChartNum=3000;
					strTmp.Format("%d",pDlg->m_ChartNum);
					pDlg->GetDlgItem(IDC_EDIT_ChartNum)->SetWindowTextA(strTmp);
					memset(&g_CheckMKForm,0,sizeof(CheckMKForm));
					pDlg->GetDlgItem(IDC_EDIT_AveError)->GetWindowTextA(strTmp);
					g_CheckMKForm.AveValueError=_ttoi(strTmp);
					pDlg->GetDlgItem(IDC_EDIT_AllSingleError)->GetWindowTextA(strTmp);
					g_CheckMKForm.SingleValueError=_ttoi(strTmp);
					pDlg->GetDlgItem(IDC_EDIT_CheckRangeMINValue)->GetWindowTextA(strTmp);
					g_CheckMKForm.ChMkRegionMin=_ttoi(strTmp);
					//strTemp.Format("��֤��׼Ϊ����ֵ���%d���������%d,�������Χ��Сֵ%d",g_CheckMKForm.AveValueError,g_CheckMKForm.SingleValueError,g_CheckMKForm.ChMkRegionMin);
					strTemp.Format("��֤��׼Ϊ����ֵ����%d����������%d,�������Χ��Сֵ%d",g_CheckMKForm.AveValueError,g_CheckMKForm.SingleValueError,g_CheckMKForm.ChMkRegionMin);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					//g_CheckMKForm.ChMkRegionMin=20;
					g_CheckMKForm.ChMkRegionMax=450;
					g_CheckMKForm.ReferenceValue=g_nReferUpEdge;
				//	g_CheckMKForm.ChMkGetReference=190;
					//g_CheckMKForm.AveValueError=400;
					//g_CheckMKForm.SingleValueError=500;

					g_nRevcNumcCheck=0;
					g_nRevcNumcCheck2=0;
					//pDlg->m_MkCalNum=100000;				
					g_CheckMKForm.ValueJudgeCnt=0;
					g_CheckMKForm.ChMkMin=999999;
					g_CheckMKForm.SingleErrorMIN=999999;
					//712��ʼ�պ���֤
					g_bCloseflag=true;  //��ʼʵʱ�жϱպϺ�ʱֹͣ
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_CLOSE,0,0);
					g_bCloseSpeedChange = true;  //�����жϺ�ʱ���͹ر��ٶ�
					//g_bCountStart=3;   //�ĵ��ر�ָ����Ӧλ��
				}
				else if (AutoCorretRegInfo.nFinalCkCode==4)   //���͹ر��ٶ�
				{
					pDlg->m_InitialSpeed = 50;
					strTmp.Format("%d",pDlg->m_InitialSpeed);
					pDlg->GetDlgItem(IDC_EDIT_InitialSpeed)->SetWindowTextA((LPCTSTR)strTmp);
					pDlg->OnBnClickedButtonSetinispeed();
					Sleep(100);
					pDlg->m_WorkingSpeed = 70;
					strTmp.Format("%d",pDlg->m_WorkingSpeed);
					pDlg->GetDlgItem(IDC_EDIT_WorkingSpeed)->SetWindowTextA((LPCTSTR)strTmp);
					pDlg->OnBnClickedButtonSetworkspeed();
					Sleep(100);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_CLOSE,0,0);    //���·�һ��ر�
				}
				else if (AutoCorretRegInfo.nFinalCkCode==5)    
				{		
					if (g_bCloseflag == true)
					{
						g_bCloseflag = false;
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //ֹͣ�����رչ���
					}
					
					//������
					strTemp="�رճ�ʱ��������֤��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,10);
					strTemp="������һ����֤δ��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
			}
		}
		else if((WAIT_OBJECT_0 + 9) == nRet)//Event_OVEROPEN ����Զ�����ǰ�ſ�����
		{
			//Sleep(10000);
			Sleep(5000);
			pDlg->m_InitialSpeed = 500;
			strTmp.Format("%d",pDlg->m_InitialSpeed);
			pDlg->GetDlgItem(IDC_EDIT_InitialSpeed)->SetWindowTextA((LPCTSTR)strTmp);
			pDlg->OnBnClickedButtonSetinispeed();
			Sleep(1000);
			pDlg->m_WorkingSpeed = 1500;
			strTmp.Format("%d",pDlg->m_WorkingSpeed);
			pDlg->GetDlgItem(IDC_EDIT_WorkingSpeed)->SetWindowTextA((LPCTSTR)strTmp);
			pDlg->OnBnClickedButtonSetworkspeed();
			Sleep(1000);
			pDlg->m_slopDir.SetCurSel(1);
			pDlg->m_slopeSteps = 15000;
			strTmp.Format("%d",pDlg->m_slopeSteps);
			pDlg->GetDlgItem(IDC_EDIT_SlopSteps)->SetWindowTextA((LPCTSTR)strTmp);
			pDlg->OnBnClickedButtonMoveslop();
			SetEvent(Event_STOP);
		}
		else if((WAIT_OBJECT_0 + 10) == nRet)//Event_STOP ֹͣ��������
		{
			switch(AutoCorretRegInfo.nStepIndx)
			{
			case STEP_FINALCH:
				//���Ժͻ滭��ʼ��
				pDlg->m_IntervNum=5;
				pDlg->m_ChartNum=100;
				pDlg->m_MkCalNum=50;
				pDlg->m_prop1.m_ChartReal.GetAxis().GetBottom().SetMinMax(0,pDlg->m_ChartNum*300);
				pDlg->m_prop1.m_ChartReal.GetAxis().GetBottom().SetIncrement(5000);
				pDlg->m_prop1.m_ChartReal.GetAxis().GetLeft().SetAutomatic(TRUE);
				pDlg->m_prop1.m_ChartReal.Series(1).SetActive(TRUE);
				//pDlg->UpdateData(FALSE);
				//pDlg->OnBnClickedButtonDisconnet();
			case STEP_BEFORECHECKOPEN:

			case STEP_CHECKTB:

			case STEP_RESTART:

			case STEP_DOWNTB:

			case STEP_FITMK:

			case STEP_ACQMK:
				bTargetAqcTerm=FALSE;
			case STEP_MAXMK:
				BeginValue=0;
			case STEP_MINMK:
				g_bCountStart=0;
			case STEP_PARAM:
				pDlg->GetDlgItem(IDC_BUTTON_GetParam)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(TRUE);
			case STEP_START:
				bAutoCorrTerm=FALSE;
				pDlg->GetDlgItem(IDC_BUTTON_AutoCorrect)->SetWindowTextA("����һ������");
				if (AutoCorretRegInfo.nFinalCkCode==1)
				{
					strTmp="�Զ������������";
					AfxMessageBox(_T("�Զ���������ɹ�����:[���ͨ��]"),MB_OK);
					//pDlg->OnBnClickedButton2();
					//д�����ݿ�
					if(g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==4)   //711E��711A
					{
						CWinThread *pThread;
						pThread = ::AfxBeginThread(WriteData,lpParam);
					}
				}
				else
				{
					strTmp="�Զ�����������ֹ";
					AfxMessageBox(_T("�Զ�����������ֹ:[���ʧ��]"),MB_OK|MB_ICONSTOP);
				}
				if(m_pMyLog)	//д����־
					m_pMyLog->SetNotify(strTmp.GetBuffer(strTmp.GetLength()));
				m_pMyLog = NULL;
			}
			pDlg->InitAutoCorrectTask();
			
/*			//�ߵ���ֵ�л�
			if(g_ChannelSelect==0)//����ֵ
			{
				////����Ϊ����ֵ
				//CTime time=CTime::GetCurrentTime();
				//char Hour= time.GetHour();
				//char Miu= time.GetMinute();
				//char Sec= time.GetSecond();
				//char zhiling[80]={0xFF,0xAA,0x00,0x4C,0x00,0x00,0x00,Hour,Miu,Sec,0x01,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x02,0x00,0x00};
				//CString strTemp="";
				//
				//memcpy(&zhiling[26],xitongcanshu,50);
				//zhiling[50] = 0x01;
				//newBCC711((char *)zhiling,80);
				//g_nNetSendBuffLen=80;
				//memcpy(g_nNetSendBuff,zhiling,g_nNetSendBuffLen);
				//SendMessage(pDlg->GetSafeHwnd(),WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
				//strTemp="�������ù��ܲ���ָ��";
				//::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				pDlg->m_ChannelSelect.SetCurSel(1);
				g_ChannelSelect=1;
				pDlg->H711_SetCmd65();
				

				//711����
				if(g_SysDevParam.LaserPor== 1)
				{
					//pDlg-> Reset711();
					//pDlg->OnBnClickedButtonDisconnet();//�Ͽ�
					//
					
					DisconnectServer(&(pDlg->m_NetConnection));
					pDlg->m_NetConnection.bConnected=FALSE;
					Sleep(15000);
					if(g_SysDevParam.LaserType==4)
				    {
						Sleep(5000);
					}
					int conntimes=0;
					while(!pDlg->m_NetConnection.bConnected)
					{
						conntimes++;
						
						//pDlg->OnBnClickedButtonConnet();
						if (ConnectServer(&(pDlg->m_NetConnection)))
						{
							bAqcOrgTerm=TRUE;
							pDlg->Get711LmdSinglePoint();
						}
						Sleep(4000);
						if(conntimes>30)
						{
							break;
						}
					}
					//���ӳɹ�
					if(pDlg->m_NetConnection.bConnected)
					{
						if(g_SysDevParam.LaserType==4)//711A��װ�ſ��Ƚ���
						{
						   Sleep(25000);
						}
					  //������������ֵ
						pDlg->OnBnClickedButtonAutocorrect();
					}
				}
			}   */
		}
	}
	return 0L;
}

UINT TargetDataAqcThread(LPVOID lpParam)
{
	CLaseAutoCorrectDlg *pDlg = (CLaseAutoCorrectDlg*)lpParam;
	DWORD nRet =0;
	CString strTemp="";
	while(bTargetAqcTerm)
	{
		if (g_AcqTartetInfo.AqcIndx<g_AcqTartetInfo.SpCnt)
		{
			if (g_AcqTartetInfo.AqcDone[g_AcqTartetInfo.AqcIndx]==0)
			{
				strTemp.Format("��%d������%dns�ɼ�",g_AcqTartetInfo.AqcIndx+1,g_AcqTartetInfo.KeyValue[g_AcqTartetInfo.AqcIndx]);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				g_Pid.e_0=0;
				g_Pid.e_1=0;
				g_Pid.feedback=0;
				g_Pid.Kp=0.2; 
				g_Pid.Ki=0.02;
				g_Pid.Kd=0.01;
				g_Pid.e_s=0;
				g_Pid.target=g_AcqTartetInfo.KeyValue[g_AcqTartetInfo.AqcIndx]*1000;
				g_AcqTartetInfo.DevValue[g_AcqTartetInfo.AqcIndx]=g_AcqTartetInfo.DevValue[g_AcqTartetInfo.AqcIndx]*100;
				g_AcqTartetInfo.AqcDone[g_AcqTartetInfo.AqcIndx]=1;//�����������Բɼ��׶�
				abcd=0;
				g_bCountStart=1;
			}
		}
		else if (g_AcqTartetInfo.AqcIndx==g_AcqTartetInfo.SpCnt)
		{
			AutoCorretRegInfo.nAqcMkCode=1;
			SetEvent(Event_AQCMK);
			bTargetAqcTerm=FALSE;
		}
		Sleep(200);
	}
	return 0L;
}

void CLaseAutoCorrectDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	int nPSeconds = 0;
	CTimeSpan timeSpan = 0;
	CTime currentTime;
	CString strTmp="";
	switch(nIDEvent)
	{
	case TIMER_INPUTDIST:
		if (bAutoCorrTerm==FALSE)
		{
			StopTimer(TIMERINPUTDIST);
			break;
		}
		if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
		{
			AutoCorretRegInfo.nWaitInputDistCnt++;
			if (AutoCorretRegInfo.nWaitInputDistCnt>5)
			{
				StopTimer(TIMERINPUTDIST);
				AutoCorretRegInfo.nParamChCode=3;
				SetEvent(Event_PARAM);
			}
			else
			{
				strTmp="�����������Ϣ��Ȼ����[����ȷ��]��ť";
				::SendMessageA(GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTmp);
			}
		}
		break;
	case TIMER_GETPARAM:
		if (bAutoCorrTerm==FALSE)
		{
			StopTimer(TIMERGETPARMA);
			break;
		}
		if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
		{
			AutoCorretRegInfo.nNetReSendCnt++;
			if (AutoCorretRegInfo.nNetReSendCnt>3)
			{
				StopTimer(TIMERGETPARMA);
				AutoCorretRegInfo.nParamChCode=3;
				SetEvent(Event_PARAM);
			}
			else
			{
				::SendMessageA(this->GetSafeHwnd(),WM_LASER_GETPARAM,0,0);
			}
		}
		break;
	case TIMER_GLADJUST:
		if (bAutoCorrTerm==FALSE)
		{
			g_bMoveFlag=false; //712��ʾֹͣ�˶�
			StopTimer(TIMERGLADJUST);
			break;
		}
		if (AutoCorretRegInfo.nStepIndx==STEP_ACQMK)
		{
			if(g_bMoveFlag==true)
			{
				AutoCorretRegInfo.nComReSendCnt++;
				if (AutoCorretRegInfo.nComReSendCnt>13)//pl 0403 ��>10��Ϊ>13 ����˳�ʼ����ֵ����475�Զ�����������ֹ�����ʧ�� 
				{
					g_bMoveFlag=false;
					StopTimer(TIMERGLADJUST);
					AutoCorretRegInfo.nAqcMkCode=5;
					SetEvent(Event_AQCMK);
				}
				else
				{
					OnBnClickedButtonGetstate();
				}
			}
			
		}
		else if (AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
		{
			AutoCorretRegInfo.nComReSendCnt++;
			if (AutoCorretRegInfo.nComReSendCnt>3)
			{
				StopTimer(TIMERGETPARMA);
				AutoCorretRegInfo.nComReSendCnt=0;
				::SendMessageA(this->GetSafeHwnd(),WM_GL_ALARM,0,0);
				SetEvent(Event_STOP);
			}
			else
			{
				if(g_SysDevParam.LaserType==6)
				{
					SendCheckCorrectTb711();
				}
			}
		}
		break;
	case TIMER_RESTART:
		nPSeconds = 0;
		timeSpan = 0;
		currentTime = CTime::GetCurrentTime();
		timeSpan =  currentTime - TimerRestart;
		nPSeconds = timeSpan.GetTotalSeconds();
		if(nPSeconds > 20)	//����20��
		{
			StopTimer(TIMERRESTART);
			SetEvent(Event_CHECKTB);
		}
		break;
	case TIMER_GETLMD:
		if (g_SysDevParam.LaserPor== 1)
		{
			if (Recv711MKDataCnt>2)
			{
				Get711LmdSinglePoint();
			} 
			else
			{
				Recv711MKDataCnt++;
			}
		} 
		else
		{
			if (Recv710AMKDataCnt>2)
			{
				GetLmdSinglePoint();
				//::PostMessageA(this->GetSafeHwnd(),WM_GET_SPLMD,0,0);
			} 
			else
			{
				Recv710AMKDataCnt++;
			}
		}
		break;
	case TIMER_ORIGMK:
		if (bAutoCorrTerm==FALSE)
		{
			StopTimer(TIMERORIGMK);
			break;
		}
		AutoCorretRegInfo.nComReSendCnt++;
		if (AutoCorretRegInfo.nStepIndx==STEP_PARAM ||(AutoCorretRegInfo.nStepIndx==STEP_MINMK))
		{
			if (AutoCorretRegInfo.nComReSendCnt>5)
			{
				StopTimer(TIMERORIGMK);
				AutoCorretRegInfo.nOrigMkChCode=3;
				SetEvent(Even_ORIGMK);
			}
			else
			{
				//::SendMessageA(this->GetSafeHwnd(),WM_GL_CLOSE,0,0);
				OnBnClickedButtonGetstate();
			}
		}
		else if(AutoCorretRegInfo.nStepIndx==STEP_FINALCH)
		{
			OnBnClickedButtonGetstate();
		}
		break;
	case TIMER_OPENING:
		StopTimer(TIMEROPENING);
		if (bAutoCorrTerm==FALSE)
		{	
			break;
		}
		if (g_nContinuousMoveState==1)
		{
			::SendMessageA(this->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);
			AutoCorretRegInfo.nOrigMkChCode=3;
			if (AutoCorretRegInfo.nStepIndx==STEP_MAXMK)
			{
				SetEvent(Even_ORIGMK);
			}
			else if(AutoCorretRegInfo.nStepIndx==STEP_BEFORECHECKOPEN)
			{
				SetEvent(Event_FINALCK);
			}
		}
		break;
	case TIMER_CLOSING:
		StopTimer(TIMERCLOSING);
		if (bAutoCorrTerm==FALSE)
		{	
			break;
		}
		if (g_nContinuousMoveState==2)
		{
			AutoCorretRegInfo.nFinalCkCode=5;
			SetEvent(Event_FINALCK);
		}
		break;
	case TIMER_712COMSEND:
		StopTimer(TIMER712COMSEND);
		if (bAutoCorrTerm==TRUE)
		{
			if (g_nComComandStat==1)
			{
				::SendMessageA(this->GetSafeHwnd(),WM_GL_OPEN,0,0);
			}
			else if (g_nComComandStat==2)
			{
				::SendMessageA(this->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);
			}
			else if (g_nComComandStat==3)
			{
				::SendMessageA(this->GetSafeHwnd(),WM_GL_CLOSE,0,0);
			}
		}
		break;
	default:
		break;
	}
	CDialogEx::OnTimer(nIDEvent);
}

void CLaseAutoCorrectDlg::StartTimer(UINT &nTimer,UINT nIDEvent,UINT nElapse)
{
	if(nTimer)
	{
		StopTimer(nTimer);
	}
	nTimer = SetTimer(nIDEvent,nElapse,0);
}

void CLaseAutoCorrectDlg::StopTimer(UINT &nTimer)
{
	if(nTimer)
	{
		KillTimer(nTimer);
		nTimer = NULL;
	}
}

void CLaseAutoCorrectDlg::H02_GetCmd54()
{
	char Sendbuff[25] = {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x10,0x73,0x54,0x4e,0x20,0x4c,0x4d,0x44,0x73,0x63,0x61,0x6e,0x64,0x61,0x74,0x61,0x20};
	int SendbuffLen = 24;
	CString strTemp="";
	Add_BCC(Sendbuff,SendbuffLen);
	g_nNetSendBuffLen=SendbuffLen+1;
	memcpy(g_nNetSendBuff,Sendbuff,g_nNetSendBuffLen);
	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="���Ͳ�ѯ�豸����";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::HFA_GetCmd02()
{
	BYTE NetSendBuff[100]={0};
	char NetContentBuff[50]={0};
	int Tmplen=0;

	NetSendBuff[0]=0xFF;
	NetSendBuff[1]=0xAA;

	Tmplen=2;
	NetContentBuff[Tmplen++]=0x02;
	NetContentBuff[Tmplen++]=0x04;

	NetContentBuff[Tmplen++]=0x00;
	NetContentBuff[Tmplen++]=0x00;
	NetContentBuff[Tmplen++]=0x00;

	//У��λ ��һλ
	NetContentBuff[Tmplen++]=0x00;

	//����λ
	NetContentBuff[0]=((Tmplen+1)>>8)&0xFF;
	NetContentBuff[1]=((Tmplen+1))&0xFF;

	NetContentBuff[Tmplen++]=CalcBcc_710A(NetContentBuff,Tmplen-1);

	memcpy(NetSendBuff+2,NetContentBuff,Tmplen);

	Tmplen+=2;

	NetSendBuff[Tmplen++]=0xEE;
	NetSendBuff[Tmplen++]=0xEE;

	g_nNetSendBuffLen=Tmplen;
	memcpy(g_nNetSendBuff,NetSendBuff,g_nNetSendBuffLen);

	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
}

void CLaseAutoCorrectDlg::HFA_SetCmd02()    
{ 
	BYTE SetSysParamBuf[200]={0};
	int nLen=0;
	int j=0;
	char ch[12]={0};
	CString strTemp="";

	UpdateData(TRUE);
	LaserParam.u16APDHvValue=m_ApdPress;
	LaserParam.u16APDTemperValue=m_ApdTemp;
	if (bAutoCorrTerm)
	{
		LaserParam.u16APDHV_OP_Ratio=800;
		m_ApdXs=800;
	}
	else
	{
		LaserParam.u16APDHV_OP_Ratio=m_ApdXs;
	}
	//֡ͷ
	SetSysParamBuf[0] = 0xFF;
	SetSysParamBuf[1] = 0xAA;

	//����
	SetSysParamBuf[2]=0x00;
	SetSysParamBuf[3]=0x76;

	//����
	SetSysParamBuf[4] = 0x02;
	SetSysParamBuf[5] = 0x15;
	//Flash��Ч��־
	SetSysParamBuf[6] = 0x55;
	SetSysParamBuf[7] = 0xAA;

	/*nLen = m_MacAddr.GetLength();
	if (nLen != 12)
	{
		strTemp="MAC����12λ";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		return;
	}*/
	nLen = m_SetMAC.GetLength();
	if (nLen != 12)
	{
		strTemp="MACӦ��Ϊ12λ";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		return;
	}
	for (j=0;j<nLen; j++)
	{
		//ch[j] = ConvertHexChar(m_MacAddr[j]);
		ch[j] = ConvertHexChar(m_SetMAC[j]);
	}
	LaserParam.u16My_Phy_Addr[0] = (ch[0]<<4)+ch[1];
	LaserParam.u16My_Phy_Addr[1] = (ch[2]<<4)+ch[3];
	LaserParam.u16My_Phy_Addr[2] = (ch[4]<<4)+ch[5];
	LaserParam.u16My_Phy_Addr[3] = (ch[6]<<4)+ch[7];
	LaserParam.u16My_Phy_Addr[4] = (ch[8]<<4)+ch[9];
	LaserParam.u16My_Phy_Addr[5] = (ch[10]<<4)+ch[11];

	unsigned char TempForChange[112]={0};
	memcpy(TempForChange,&LaserParam,sizeof(LaserParam));
	for (int i=4;i<112;i=i+2)
	{
		SetSysParamBuf[i+10-4]=TempForChange[i+1];
		SetSysParamBuf[i+1+10-4]=TempForChange[i];
	}

	//memcpy(SetSysParamBuf+10,LaserParam.u16NCdata,sizeof(LaserParam)-4);

	//Add_BCC((char *)SetSysParamBuf,116);
	//У��
	SetSysParamBuf[118]=0x00;
	SetSysParamBuf[119]=CalcBcc_710A((char *)(SetSysParamBuf)+2,116);
	SetSysParamBuf[120]=0xEE;
	SetSysParamBuf[121]=0xEE;
	g_nNetSendBuffLen=122;
	memcpy(g_nNetSendBuff,SetSysParamBuf,g_nNetSendBuffLen);
	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="�������ò���ָ��";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

}

void CLaseAutoCorrectDlg::HFA_ParseCmd02(char *pcBuf, int nBufSize)
{
	int nTmp = 0;
	int i=0;
	CString strTmp="";
	CString strTemp="";
	if (nBufSize != 122)
	{
		strTemp = "��ѯ�����ظ�����:";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		return;
	}
	else
	{
		nTmp=6;
		//char tt[200]={0};
		//memcpy(tt,pcBuf+6,sizeof(LaserParam));
		unsigned char TempForChange[112]={0};
		for (i=0;i<112;i=i+2)
		{
			TempForChange[i]=pcBuf[i+6+1];
			TempForChange[i+1]=pcBuf[i+6];
		}
		memcpy(&LaserParam,TempForChange,sizeof(LaserParam));

		m_ApdPress=LaserParam.u16APDHvValue;
		m_ApdTemp=LaserParam.u16APDTemperValue;
		m_ApdXs=LaserParam.u16APDHV_OP_Ratio;

		m_MacAddr="";

		for (i=0;i<6;i++)
		{
			strTmp.Format("%2.2X", (DWORD)LaserParam.u16My_Phy_Addr[i]);
			m_MacAddr+=strTmp;
		}

		strTemp = "��ѯ�豸�����ɹ�";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(TRUE);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
		{
			StopTimer(TIMERGETPARMA);
			if (LaserParam.u16APDHV_OP_Ratio>880||LaserParam.u16APDHV_OP_Ratio<600)
			{
				AutoCorretRegInfo.nParamChCode=2;
			}
			else
			{
				AutoCorretRegInfo.nParamChCode=1;
			}
			SetEvent(Event_PARAM);
		}
		UpdateData(FALSE);
	}
}

void CLaseAutoCorrectDlg::H711_GetCmd55()
{
	CTime time=CTime::GetCurrentTime();
	char Hour= time.GetHour();
	char Miu= time.GetMinute();
	char Sec= time.GetSecond();
	 char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,Hour,Miu,Sec,0x01,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x03,0x00,0x00};
	 if(g_SysDevParam.LaserType==4)
	 {
		zhiling[13]=0x06;
		zhiling[21]=0x01;
		zhiling[24]=0x01;
		zhiling[25]=0x00;
	 }
	newBCC711(zhiling,34);
	memcpy(g_nNetSendBuff,zhiling,34);
	g_nNetSendBuffLen=34;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	CString strTemp;
	strTemp="���Ͳ�ѯ�豸MAC����";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::H711_GetCmd68()
{

	CTime time=CTime::GetCurrentTime();
	char Hour= time.GetHour();
	char Miu= time.GetMinute();
	char Sec= time.GetSecond();
	char zhiling[36]={0xFF,0xAA,0x00,0x20,0x00,0x00,0x00,Hour,Miu,Sec,0x01,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x00,0x00};
	if(g_SysDevParam.LaserType==4)
	{
		zhiling[13]=0x06;
		zhiling[21]=0x01;
		zhiling[24]=0x01;
		zhiling[25]=0x00;
	}

	newBCC711(zhiling,36);
	memcpy(g_nNetSendBuff,zhiling,36);
	g_nNetSendBuffLen=36;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	CString strTemp="";
	strTemp="���Ͳ�ѯAPD����ָ��";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}


//void CLaseAutoCorrectDlg::H712_GetState()   //712��ȡ��ǰAPD�¶�
//{
//	CTime time=CTime::GetCurrentTime();
//	char Hour= time.GetHour();
//	char Miu= time.GetMinute();
//	char Sec= time.GetSecond();
//	char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,Hour,Miu,Sec,0x01,0x01,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x02,0x00,0x00};
//
//	newBCC711(zhiling,34);
//	memcpy(g_nNetSendBuff,zhiling,34);
//	g_nNetSendBuffLen = 34;
//	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
//
//	CString strTemp="";
//	strTemp="���Ͳ�ѯAPD����ָ��";
//	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
//}

void CLaseAutoCorrectDlg::H712_GetAPD()   //712��ȡAPD
{
	CTime time=CTime::GetCurrentTime();
	char Hour= time.GetHour();
	char Miu= time.GetMinute();
	char Sec= time.GetSecond();
	char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,Hour,Miu,Sec,0x01,0x01,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x04,0x00,0x00};

	newBCC711(zhiling,34);
	memcpy(g_nNetSendBuff,zhiling,34);
	g_nNetSendBuffLen = 34;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);

	CString strTemp="";
	strTemp="���Ͳ�ѯAPD����ָ��";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::H711_GetCmd62()
{
	CTime time=CTime::GetCurrentTime();
	char Hour= time.GetHour();
	char Miu= time.GetMinute();
	char Sec= time.GetSecond();
	 char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,Hour,Miu,Sec,0x01,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x01,0x00,0x00};
	 if(g_SysDevParam.LaserType==4)
	 {
		zhiling[13]=0x06;
		zhiling[21]=0x01;
		zhiling[24]=0x01;
		zhiling[25]=0x00;
	 }
	newBCC711(zhiling,34);
	memcpy(g_nNetSendBuff,zhiling,34);
	g_nNetSendBuffLen=34;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	CString strTemp="";
	strTemp="���Ͳ�ѯϵͳ����ָ��";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::H711_SetCmd55()  
{ 

}

void CLaseAutoCorrectDlg::H711_SetCmd68()
{ 
	/*BYTE SetSysParamBuf[200] = 
	{
		(char)0x02, (char)0x02, (char)0x02, (char)0x02, (char)0x00, (char)0x00, (char)0x00, (char)0x10
		, (char)0x73, (char)0x67, (char)0x4e, (char)0x51, (char)0x4c, (char)0x4d, (char)0x44, (char)0x73
		, (char)0x63, (char)0x61, (char)0x6e, (char)0x64, (char)0x61, (char)0x74, (char)0x61, (char)0x20
	};
	int nLen=0;
	int j=0;
	char ch[12]={0};*/
	CTime time=CTime::GetCurrentTime();
	char Hour= time.GetHour();
	char Miu= time.GetMinute();
	char Sec= time.GetSecond();
	 char zhiling[36]={0xFF,0xAA,0x00,0x20,0x00,0x00,0x00,Hour,Miu,Sec,0x01,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x06,0x00,0x00};
	CString strTemp="";

	UpdateData(TRUE);
	LaserParam711.m_u16APDVSet=m_ApdPress;
	LaserParam711.m_u16APDTemperValue=m_ApdTemp;
	if (bAutoCorrTerm)
	{
		LaserParam711.m_u16APDHV_OP_Ratio=800;
		m_ApdXs=800;
	}
	else
	{
		LaserParam711.m_u16APDHV_OP_Ratio=m_ApdXs;
	}

	zhiling[26]=((LaserParam711.m_u16APDVSet*100)>>8) & 0xff;
	zhiling[27]=(LaserParam711.m_u16APDVSet*100) & 0xff;

	zhiling[28]=((LaserParam711.m_u16APDTemperValue*100)>>8) & 0xff;
	zhiling[29]=(LaserParam711.m_u16APDTemperValue*100) & 0xff;

	zhiling[30]=((LaserParam711.m_u16APDHV_OP_Ratio/10)>>8) & 0xff;
	zhiling[31]=(LaserParam711.m_u16APDHV_OP_Ratio/10) & 0xff;
	if(g_SysDevParam.LaserType==4)
	 {
		zhiling[13]=0x06;
		zhiling[21]=0x01;
		zhiling[24]=0x01;
		zhiling[25]=0x00;
	 }
	newBCC711((char *)zhiling,36);
	g_nNetSendBuffLen=36;
	memcpy(g_nNetSendBuff,zhiling,g_nNetSendBuffLen);
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="��������APD����ָ��";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::H712_SetAPD()
{ 
	CTime time=CTime::GetCurrentTime();
	char Hour= time.GetHour();
	char Miu= time.GetMinute();
	char Sec= time.GetSecond();
	 char zhiling[36]={0xFF,0xAA,0x00,0x20,0x00,0x00,0x00,Hour,Miu,Sec,0x01,0x01,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x05,0x00,0x00};
	CString strTemp="";

	UpdateData(TRUE);
	LaserParam712.m_u16APDVSet=m_ApdPress;
	LaserParam712.m_u16APDTemperValue=m_ApdTemp;
	if (bAutoCorrTerm)
	{
		LaserParam712.m_u16APDHV_OP_Ratio=800;
		m_ApdXs=800;
	}
	else
	{
		LaserParam712.m_u16APDHV_OP_Ratio=m_ApdXs;
	}

	zhiling[26]=((LaserParam712.m_u16APDVSet*100)>>8) & 0xff;
	zhiling[27]=(LaserParam712.m_u16APDVSet*100) & 0xff;

	zhiling[28]=((LaserParam712.m_u16APDTemperValue*100)>>8) & 0xff;
	zhiling[29]=(LaserParam712.m_u16APDTemperValue*100) & 0xff;

	zhiling[30]=((LaserParam712.m_u16APDHV_OP_Ratio/10)>>8) & 0xff;
	zhiling[31]=(LaserParam712.m_u16APDHV_OP_Ratio/10) & 0xff;
	newBCC711((char *)zhiling,36);
	g_nNetSendBuffLen=36;
	memcpy(g_nNetSendBuff,zhiling,g_nNetSendBuffLen);
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="��������APD����ָ��";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::H711_SetCmd65()
{ 
	/*BYTE SetSysParamBuf[200]=
	{
		(char)0x02, (char)0x02, (char)0x02, (char)0x02, (char)0x00, (char)0x00, (char)0x00, (char)0x10
		, (char)0x73, (char)0x65, (char)0x4e, (char)0x51, (char)0x4c, (char)0x4d, (char)0x44, (char)0x73
		, (char)0x63, (char)0x61, (char)0x6e, (char)0x64, (char)0x61, (char)0x74, (char)0x61, (char)0x20
	};
	int nLen=0;
	int j=0;
	char ch[12]={0};*/
	CTime time=CTime::GetCurrentTime();
	char Hour= time.GetHour();
	char Miu= time.GetMinute();
	char Sec= time.GetSecond();
	 char zhiling[80]={0xFF,0xAA,0x00,0x4C,0x00,0x00,0x00,Hour,Miu,Sec,0x01,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x02,0x00,0x00};
	CString strTemp="";

	//UpdateData(TRUE);

	/*SetSysParamBuf[7] = 0x1E;
	SetSysParamBuf[24] = 0x00;*/

	//nLen = m_SetMAC.GetLength();
	//if (nLen != 12)
	//{
	//	strTemp="MACӦ��Ϊ12λ";
	//	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	//	return;
	//}
	//if (bAutoCorrTerm==FALSE)
	//{
	//	UINT MACNum=_ttoi(m_SetMAC);
	//	SetSysParamBuf[28] = (MACNum>>16) & 0xff;
	//	SetSysParamBuf[29] = (MACNum>>16) & 0xff;
	//	SetSysParamBuf[30] = MACNum & 0xff;
	//} 
	//else
	//{
	/*UINT MACNum=_ttoi(m_MacAddr);
	SetSysParamBuf[28] = (MACNum>>16) & 0xff;
	SetSysParamBuf[29] = (MACNum>>8) & 0xff;
	SetSysParamBuf[30] = MACNum & 0xff;*/
	//}
	memcpy(&zhiling[26],xitongcanshu,50);
	if (bAutoCorrTerm==TRUE)    //����һ������
	{
		zhiling[50] = g_ChannelSelect;
	} 
	else
	{
		zhiling[50] = m_ChannelSelect.GetCurSel();
	}
	

	//SetSysParamBuf[32] = (LaserParam711.m_u16ZeroDisc>>8) & 0xff;
	//SetSysParamBuf[33] = LaserParam711.m_u16ZeroDisc & 0xff;

	//SetSysParamBuf[34] = (LaserParam711.m_u16OverallDisDif>>8) & 0xff;
	//SetSysParamBuf[35] = LaserParam711.m_u16OverallDisDif & 0xff;

	//SetSysParamBuf[36] = (LaserParam711.m_u16OverallDisDif_L>>8) & 0xff;
	//SetSysParamBuf[37] = LaserParam711.m_u16OverallDisDif_L & 0xff;
	if(g_SysDevParam.LaserType==4)
	 {
		zhiling[13]=0x06;
		zhiling[21]=0x01;
		zhiling[24]=0x01;
		zhiling[25]=0x00;
	 }
	newBCC711((char *)zhiling,80);
	g_nNetSendBuffLen=80;
	memcpy(g_nNetSendBuff,zhiling,g_nNetSendBuffLen);
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="�������ù��ܲ���ָ��";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	//m_SetMAC="";
	GetDlgItem(IDC_EDIT_MACSET)->SetWindowText(m_SetMAC);
}
void CLaseAutoCorrectDlg::H715_SetHL()
{
	CString strTemp="";
	char zhiling[43]={0xFF,0xAA,0x00,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x27,0xEE,0xEE};
	memcpy(&zhiling[26],xitongcanshu,11);
	if (bAutoCorrTerm==TRUE)    //����һ������
	{
		zhiling[38] = g_ChannelSelect;
	} 
	else
	{
		zhiling[38] = m_ChannelSelect.GetCurSel();
	}
	newBCC711((char *)zhiling,43);
	g_nNetSendBuffLen=43;
	memcpy(g_nNetSendBuff,zhiling,g_nNetSendBuffLen);
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="�������ù��ܲ���ָ��";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::H715_GetAPD()
{
	char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0xEE,0xEE};
	memcpy(g_nNetSendBuff,zhiling,34);
	g_nNetSendBuffLen=34;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	CString strTemp;
	strTemp="���Ͳ�ѯ�豸APD����";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}
void CLaseAutoCorrectDlg::H715_GetHL()//�ߵ���ֵ
{
	char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1D,0xEE,0xEE};
	memcpy(g_nNetSendBuff,zhiling,34);
	g_nNetSendBuffLen=34;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	CString strTemp;
	strTemp="���Ͳ�ѯ�豸ϵͳ����";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}
void CLaseAutoCorrectDlg::H715_GetMAC()
{
	char zhiling[34]={};
	memcpy(g_nNetSendBuff,zhiling,34);
	g_nNetSendBuffLen=34;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	CString strTemp;
	strTemp="���Ͳ�ѯ�豸MAC����";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::H712_GetMAC()
{
	char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0xEE,0xEE};
	memcpy(g_nNetSendBuff,zhiling,34);
	g_nNetSendBuffLen=34;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	CString strTemp;
	strTemp="���Ͳ�ѯ�豸MAC����";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::OnEnChangeEditIntevnum()
{
	// TODO:  ����ÿؼ��� RICHEDIT �ؼ���������
	// ���ʹ�֪ͨ��������д CDialogEx::OnInitDialog()
	// ���������� CRichEditCtrl().SetEventMask()��
	// ͬʱ�� ENM_CHANGE ��־�������㵽�����С�

	// TODO:  �ڴ���ӿؼ�֪ͨ����������
	//UpdateData(TRUE);
	CString strTemp="";
	CString str="";
	GetDlgItem(IDC_EDIT_IntevNum)->GetWindowTextA(str);
	m_IntervNum=_ttoi(str);
	if (m_IntervNum <= 0)
	{ 
		strTemp="�����Ϊ0��ΪĬ��5";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		m_IntervNum = 5;
	}		
	UpdateData(FALSE);
	//GetDlgItem(IDC_EDIT_IntevNum)->SetWindowTextA((LPCTSTR)m_IntervNum);

}

void CLaseAutoCorrectDlg::OnEnChangeEditChartnum()
{
	// TODO:  ����ÿؼ��� RICHEDIT �ؼ���������
	// ���ʹ�֪ͨ��������д CDialogEx::OnInitDialog()
	// ���������� CRichEditCtrl().SetEventMask()��
	// ͬʱ�� ENM_CHANGE ��־�������㵽�����С�

	// TODO:  �ڴ���ӿؼ�֪ͨ����������
	//UpdateData(TRUE);
	CString strTemp="";
	CString str="";
	GetDlgItem(IDC_EDIT_ChartNum)->GetWindowTextA(str);
	m_ChartNum=_ttoi(str);

	if (m_ChartNum <= 0)
	{
		m_ChartNum = 100;
		strTemp="���԰�Ϊ0��ΪĬ��100";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);		
	}
	UpdateData(FALSE);
	//GetDlgItem(IDC_EDIT_ChartNum)->SetWindowTextA((LPCTSTR)m_ChartNum);
	m_prop1.m_ChartReal.GetAxis().GetBottom().SetMinMax(0,m_ChartNum*300);
}

void CLaseAutoCorrectDlg::OnEnChangeEditMkcalnum()
{
	// TODO:  ����ÿؼ��� RICHEDIT �ؼ���������
	// ���ʹ�֪ͨ��������д CDialogEx::OnInitDialog()
	// ���������� CRichEditCtrl().SetEventMask()��
	// ͬʱ�� ENM_CHANGE ��־�������㵽�����С�

	// TODO:  �ڴ���ӿؼ�֪ͨ����������
	//UpdateData(TRUE);
	CString strTemp="";
	CString str="";
	GetDlgItem(IDC_EDIT_MkCalNum)->GetWindowTextA(str);
	m_MkCalNum=_ttoi(str);

	if (m_MkCalNum <= 0)
	{
		m_MkCalNum = 100;
		strTemp="����������ΪĬ��100";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	}
	UpdateData(FALSE);
	//GetDlgItem(IDC_EDIT_MkCalNum)->SetWindowTextA((LPCTSTR)m_MkCalNum);
}

void CLaseAutoCorrectDlg::ResetEvents(void)
{
	//{Event_START,Event_PARAM,Even_ORIGMK,Event_AQCMK,Event_ALIVE,Event_FITMK,Event_DOWNTB,Event_CHECKTB,Event_FINALCK,Event_OVEROPEN,Event_STOP};
	ResetEvent(Event_START);
	ResetEvent(Event_PARAM);
	ResetEvent(Even_ORIGMK);
	ResetEvent(Event_AQCMK);
	ResetEvent(Event_ALIVE);
	ResetEvent(Event_FITMK);
	ResetEvent(Event_DOWNTB);
	ResetEvent(Event_CHECKTB);
	ResetEvent(Event_FINALCK);
	ResetEvent(Event_OVEROPEN);
	ResetEvent(Event_STOP);
}

LRESULT CLaseAutoCorrectDlg::OnLaserGetParam(WPARAM wParam, LPARAM lParam)
{
	if (g_SysDevParam.LaserType==6)    //712
	{
		H712_GetMAC();
		Sleep(100);
		//H712_GetState();
		//Sleep(100);
		H712_GetAPD();
	}
	//StartTimer(TIMERGETPARMA,TIMER_GETPARAM,5000);
	return 0L;
}

LRESULT CLaseAutoCorrectDlg::OnLaserSetParam(WPARAM wParam, LPARAM lParam)
{
	if(g_SysDevParam.LaserType==6)    //712
	{
		H712_SetAPD();//����APD����;
	}
	return 0L;
}

void CLaseAutoCorrectDlg::OnCreateFolder(CString fileName)  //�����ļ���
{
	// TODO: Add your command handler code here
	CString path;
	GetModuleFileName(NULL,path.GetBufferSetLength(MAX_PATH+1),MAX_PATH);
	path.ReleaseBuffer();
	int pos = path.ReverseFind('\\');
	path = path.Left(pos);
	m_SavePath=path + _T("\\") + "�Զ��������";
	if(!PathIsDirectory(m_SavePath))
	{
		BOOL bRet = CreateDirectory(m_SavePath, NULL);//�����ļ���
	}
	m_SavePath=m_SavePath + _T("\\") + fileName;
	if(!PathIsDirectory(m_SavePath))
	{
		BOOL bRet = CreateDirectory(m_SavePath, NULL);//�����ļ���
	}
}

void CLaseAutoCorrectDlg::InitAqcDataTarget(void)
{
	int nTmp=0;
	memset(&g_AcqTartetInfo,0,sizeof(g_AcqTartetInfo));

	g_AcqTartetInfo.SpCnt=43;
	g_AcqTartetInfo.KeyValue[nTmp]=BeginValue/1000;
	g_AcqTartetInfo.DevValue[nTmp]=10;      
	g_AcqTartetInfo.Step[nTmp]=2000;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=440;    //  ns/1000
	g_AcqTartetInfo.DevValue[nTmp]=10;     //  ns/100
	g_AcqTartetInfo.Step[nTmp]=2000;        //����ϵ��
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=420;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=2000;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=400;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=1000;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=380;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=500;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=360;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=500;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=340;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=300;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=320;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=200;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=300;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=150;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=280;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=70;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=260;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=40;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=240;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=220;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=15;
	nTmp++;	
	g_AcqTartetInfo.KeyValue[nTmp]=200;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=10;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=180;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=6;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=160;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=6;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=140;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=5;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=120;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=5;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=100;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=4;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=90;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=4;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=80;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=3;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=70;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=3;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=60;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=3;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=50;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=5;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=40;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=5;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=30;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=5;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=27;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=10;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=24;
	g_AcqTartetInfo.DevValue[nTmp]=10;
	g_AcqTartetInfo.Step[nTmp]=10;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=21;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=10;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=20;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=15;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=19;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=18;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=17;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=16;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=15;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=14;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=13;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=12;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=11;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=10;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=9;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=8;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
	nTmp++;
	g_AcqTartetInfo.KeyValue[nTmp]=7;
	g_AcqTartetInfo.DevValue[nTmp]=5;
	g_AcqTartetInfo.Step[nTmp]=20;
}

LRESULT CLaseAutoCorrectDlg::OnGlClsoe(WPARAM wParam, LPARAM lParam)  //ȷ��ָ����޸�
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	if (Com_Send(a_ucTemp10,6))
	{
		strTemp="���͹������ٹر�ָ��ɹ�";
	}
	else
	{
		strTemp="���͹������ٹر�ָ��ʧ��";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	if (bAutoCorrTerm==TRUE)
	{
		if (g_nComComandStat != 3)    //���ٵ�ʱ�򣬷��͹رգ��������¿�ʼ��ʱ
		{
			g_nComComandStat=3;
			StartTimer(TIMERCLOSING,TIMER_CLOSING,60000);
		}
		StartTimer(TIMER712COMSEND,TIMER_712COMSEND,500);	
	}
	return 0L;
}

afx_msg LRESULT CLaseAutoCorrectDlg::OnGlStopmove(WPARAM wParam, LPARAM lParam)
{
	CString strTemp="";
	if (Com_Send(a_ucTemp12,6))
	{
		strTemp="����ָֹͣ��ɹ�";
	}
	else
	{
		strTemp="����ָֹͣ��ʧ��";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	if (bAutoCorrTerm==TRUE)
	{
		g_nComComandStat=2;
		StartTimer(TIMER712COMSEND,TIMER_712COMSEND,500);
	}
	return 0;
}

LRESULT CLaseAutoCorrectDlg::OnGlOpen(WPARAM wParam, LPARAM lParam)   //ȷ��ָ����޸�
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	if (Com_Send(a_ucTemp11,6))
	{
		strTemp="���͹��������ſ�ָ��ɹ�";
	}
	else
	{
		strTemp="���͹��������ſ�ָ��ʧ��";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	if (bAutoCorrTerm==TRUE)
	{
		g_nComComandStat=1;
		StartTimer(TIMER712COMSEND,TIMER_712COMSEND,500);
		StartTimer(TIMEROPENING,TIMER_OPENING,25000);
	}
	return 0L;
}

LRESULT CLaseAutoCorrectDlg::OnGlAdjust(WPARAM wParam, LPARAM lParam)
{	
	OnBnClickedButtonMoveslop();	
	if (bAutoCorrTerm==TRUE)
	{
		g_bMoveFlag=true; //712��ʾ��ʼ�˶�
		StartTimer(TIMERGLADJUST,TIMER_GLADJUST,500);
	}
	return 0L;
}

LRESULT CLaseAutoCorrectDlg::OnGlAlarm (WPARAM wParam, LPARAM lParam)
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	if (Com_Send(a_ucTemp8,12))
	{
		strTemp="���ͱ���ָ��ɹ�";
	}
	else
	{
		strTemp="���ͱ���ָ��ʧ��";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	return 0L;
}

//����ԭʼ��������
void CLaseAutoCorrectDlg::SaveOrigData(void)
{
	CStdioFile file;
	int i;
	CString  strXY;
	if (file.Open(m_OrigDataPath,CFile::modeReadWrite|CFile::modeCreate))
	{
		for (i=0;i<g_nXYlen;i++)
		{
			strXY.Format("%d      %d\n",g_nXY[i][0],g_nXY[i][1]);			
			file.WriteString(strXY);
		}
		file.Close(); 
	}	
}

void CLaseAutoCorrectDlg::OnCreateFile(CString fileName="NOMAC_ԭʼ����",int nType=0)  //�����ļ�
{
	CFile file;
	CString Fpath="";
	Fpath=m_SavePath+"\\"+fileName;
	if (nType==1)
	{
		m_OrigDataPath=Fpath;
	}
	else if (nType=2)
	{
		m_OperTxtPath=Fpath;
	}
	
	if(file.Open(Fpath,CFile::modeCreate|CFile::modeWrite))//�����ļ�
	{
		file.Close();  //�ر��ļ�
	}
}

int CLaseAutoCorrectDlg::FitPro(CString TxtName="NOMAC_��������")
{
	//��ϲ�ֵ�ɹ�����1  ���򷵻�0
	CFile sfile;
	BOOL pfile;
	int  i=0;
	static int j=0;
	double dx,dy = 0;
	CString inttostr = " ";
	CString FinalTxt="";
	CString strTemp="";
	double pdx[arrayXYsize] = {0}; 
	double pdy[arrayXYsize] = {0};

	m_prop2.m_ChartFit.Series(0).Clear();
	m_prop2.m_ChartFit.Series(1).Clear();
	m_prop2.m_ChartFit.GetAxis().GetBottom().SetMinMax(0,500000);
	
	for (i=0;i<g_nInterpNum;i++)
	{
		pdx[i] = (double) g_datax[i];
		pdy[i] = (double) g_datay[i];
		m_prop2.m_ChartFit.Series(1).AddXY(pdx[i],pdy[i],NULL,RGB(255,0,0));
		m_prop2.m_ChartFit.Series(0).AddXY(pdx[i],pdy[i],NULL,RGB(255,255,0));
	}

	Interp FItInter(pdx,pdy,g_nInterpNum);
	int m_end=500000;   //ʵ�ʴ�450000��ʼ�޵�

	////��������ǰ�������¶���Ϣ
	//inttostr.Format("%d\t\t",g_nAPDStateTemp);
	//FinalTxt = inttostr;
	//j++;

	for (i = 0;i<m_end;i+=32)
	{
		dx = (double)i;
		if (FItInter.GetYByX(dx,dy))
		{
			m_prop2.m_ChartFit.Series(0).AddXY(dx,dy,NULL,RGB(255,255,0));
			inttostr.Format("%d\t\t",(int)(dy+50000));
			FinalTxt += inttostr;
			j++;
			if (j>4)
			{
				FinalTxt += "\r\n";
				j = 0;
			}
		}
		else
		{
			strTemp="��ֵ����Դ����ȷ";
			::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);	
			PostNcDestroy();
			return 0;
		}
	}

	//�����ļ�
	m_FitDataPath=m_SavePath+"\\"+TxtName;
	pfile = sfile.Open(m_FitDataPath,CFile::modeReadWrite|CFile::modeCreate);
	if (pfile == NULL)
	{
		strTemp="�򿪱����ļ�ʧ��";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		return 0;
	}
	sfile.SeekToEnd();
	sfile.Write(FinalTxt,FinalTxt.GetLength());
	sfile.Close();
	//��ͼ
	CString OpenFileName="";
	CString getDir="" ;
	//OpenFileName=m_MacAddr+"���ͼ.jpg";

	if(g_nameadd)
	{
		OpenFileName=m_MacAddr+"���ͼ("+g_cover+").jpg";
	}
	else
	{
		OpenFileName=m_MacAddr+"���ͼ.jpg";
	}
		
	getDir= m_SavePath + "\\"+ OpenFileName;
	m_prop2.m_ChartFit.GetExport().SaveToBitmapFile(getDir);
	return 1;
}

UINT DownLoadCorrectTableThread(LPVOID lpParam)
{
	bccW_packs=0;
	CLaseAutoCorrectDlg* pDlg = (CLaseAutoCorrectDlg*)lpParam;
    const int nFrameLen = 1036;//֡��
	const int nDataLen = 1024; //���ݳ���
	const int nMaxFileData = 512*32;//�ļ�����������
	
	int /*i,j,*/k=0;  
	int nIndex1 = 0;
	int nIndex2 = 0;
//     DWORD nLen;
	DWORD dw ;
	CString hustring;
	int packetlen;//udp���ĳ���
	int cfnum=0;//�����ط�����
	int qbcfnum=0;//��ͷȫ���ط�
	bool PackageFinishFalg=false;
	char psenddata[2000];
	int sendnum =1;
	CString str;
	CString strTemp="";
	bool cfFlag=false;
	DWORD biao;

	//������
	//pDlg->m_FitDataPath="F:\\0727\\��\\Code\\��v7��LaseAutoCorrect-20171106\\Debug\\�Զ��������\\F8-B5-68-90-01-67\\F8-B5-68-90-01-67_����ֵ�������.txt_H";
	//pDlg->m_FitDataPath="E:\\����\\My Task 45.���������״�\\����������λ��\\��v7��������LaseAutoCorrect-20180808\\Debug\\�Զ��������\\3\\001234000004_��������.txt";
	
	if (pDlg->m_FitDataPath == "")
	{
		strTemp="���������ݲ�����";
		::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
		SetEvent(Event_STOP);
		return -1;
	}

	CStdioFile hufile;
	if (hufile.Open(pDlg->m_FitDataPath,CFile::modeRead))
	{	
		while(hufile.ReadString(hustring))
		{
			hustring.TrimLeft();
			hustring.TrimRight();
			if (hustring == "")
			{
				break;
			}
			biao = sscanf_s(hustring,"%d %d %d %d %d",&g_nReadbuf[k],&g_nReadbuf[k+1],&g_nReadbuf[k+2],&g_nReadbuf[k+3],&g_nReadbuf[k+4]);
			if (biao != 0)
			{
				k += biao;
				if (biao!=5)
				{
					break;
				}
			}
	
		}
		hufile.Close();
	}
	if( 0 == k)
	{
		//pDlg->MessageBox("�ļ��������ݣ�",MB_OK);
		strTemp="�����ļ��������ݣ�";
		::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		return 0;
	}
	//UDP
	DWORD dwIP;	
	SOCKADDR_IN addrTo;
	if (g_SysDevParam.LaserType !=3)
	{
		pDlg->m_DestIp.GetAddress(dwIP);
		addrTo.sin_family=AF_INET;
		addrTo.sin_port=htons(pDlg->m_DestPort);
		addrTo.sin_addr.S_un.S_addr=htonl(dwIP);
	}
	
	while (1)
	{
		if (bAutoCorrTerm==FALSE)
		{
			SetEvent(Event_STOP);
			return 0L;
		}
		//������ط��������¶�ȡ���ݣ���Ҫ����У��bccW_packs�������У��
		if (cfFlag == false)
		{
			packetlen = SetPacket712(psenddata,g_nReadbuf,k,sendnum);//�ӵ�1������
		}	
		if (packetlen != 0)  //����
		{	
			newBCC711(psenddata,packetlen);
			pDlg->m_NetConnection.pcSendDataBuf = psenddata;
			pDlg->m_NetConnection.nSendDataBufSize = packetlen;
			str.Format("%d",packetlen);
			SendData(&(pDlg->m_NetConnection));
			
			PackageFinishFalg = false;
		}
		else
		{			
			//break;  //������ ����
			//��д���ݰ�
			CTime time=CTime::GetCurrentTime();
			char Hour= time.GetHour();
			char Miu= time.GetMinute();
			char Sec= time.GetSecond();
			ShaoXie_zhiling[7]=Hour;
			ShaoXie_zhiling[8]=Miu;
			ShaoXie_zhiling[9]=Sec;
			ShaoXie_zhiling[13]=0x07;
			ShaoXie_zhiling[24]=0x02;
			ShaoXie_zhiling[26]=(bccW_packs>>8);
			ShaoXie_zhiling[27]=bccW_packs;
			newBCC711(ShaoXie_zhiling,34);
			pDlg->m_NetConnection.pcSendDataBuf = ShaoXie_zhiling;
			pDlg->m_NetConnection.nSendDataBufSize = 34;
			SendData(&(pDlg->m_NetConnection));

			dw = WaitForSingleObject(g_eventdl,5000); //���¼�  ������ INFINITE 
			ResetEvent(g_eventdl);
			switch(dw)
			{
			case WAIT_OBJECT_0:
				//�ȵ��¼�			
				if (g_flag == TRUE )  //���ͳɹ�
				{
					g_flag = FALSE;
					cfnum = 0;
					//���ؼ�����һ��
					AutoCorretRegInfo.nDownTbCode=1;
					SetEvent(Event_DOWNTB);
					return 0;
				}
				else                      //����ʧ��
				{
					if (qbcfnum>2)
					{
						strTemp="��������дʧ�ܣ����ֶ���ɺ������裡";
						::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						//AutoCorretRegInfo.nDownTbCode=2;
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
						SetEvent(Event_STOP);
						return 0;
					}
					cfnum=0;
					sendnum = 1;
					cfFlag=false;
					bccW_packs = 0;
					qbcfnum++;
					str.Format("%d",qbcfnum);
					strTemp="��д��ָ����Ӧʧ�ܣ���ʼ�������أ�";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					continue;
				}			
				break;
			case WAIT_TIMEOUT:				
			case WAIT_FAILED:
				if (qbcfnum>2)
				{
					strTemp="��������дʧ�ܣ����ֶ���ɺ������裡";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					//AutoCorretRegInfo.nDownTbCode=2;
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
					return 0;
				}
				cfnum=0;
				sendnum = 1;
				qbcfnum++;
				cfFlag=false;
				bccW_packs = 0;
				str.Format("%d",qbcfnum);
				strTemp="��д��ָ����Ӧʧ�ܣ���ʼ�������أ�";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				continue;
			}
		}
		dw = WaitForSingleObject(g_eventdl,5000); //���¼�  ������ INFINITE 
		ResetEvent(g_eventdl);
		
		switch(dw)
		{
			case WAIT_OBJECT_0:
				//�ȵ��¼�			
					if (g_flag == TRUE && g_nNum == sendnum)  //���ͳɹ�
					{
						PackageFinishFalg=true;
						sendnum ++;
						g_flag = FALSE;
						cfnum = 0;
						cfFlag = false;
						//qbcfnum = 0;
					}
					else                      //����ʧ��
					{
						str.Format("%d",g_nNum);
						strTemp="��" + str +"�����ݽ���ʧ�ܣ��ط��˰���";
						::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					}			
				break;
			case WAIT_TIMEOUT:				
				//��ʱδ�յ��ظ�			
				str.Format("%d",sendnum);
				strTemp="��ʱδ�յ���"+str+"�����ջظ����ط��˰���";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				break;
			case WAIT_FAILED:
				str.Format("%d",sendnum);
				strTemp="��"+str+"�����ͳ����ط��˰���";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				break;
	     }	
		 if (PackageFinishFalg == false)
		 {
			if (cfnum > 2)
			{
				if (qbcfnum>2)
				{
					strTemp="����������ʧ�ܣ����ֶ���ɺ������裡";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					//AutoCorretRegInfo.nDownTbCode=2;
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
					return 0;
				}
				cfnum=0;
				sendnum = 1;
				cfFlag=false;
				bccW_packs = 0;
				qbcfnum++;
				str.Format("%d",qbcfnum);
				strTemp="��" + str +"������ʧ�ܣ���ʼ�������أ�";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
			}
			else
			{
				cfnum ++;
				cfFlag = true;
			}
		 }	
		 Sleep(500);
	}

	//strTemp="������дָ��ͳɹ�";
	//::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

	//AutoCorretRegInfo.nDownTbCode=1;
	//SetEvent(Event_DOWNTB);

	return 0L;
}

int SetPacket712(char* sendpacket,int* datapacket,int len,int num)  //Դ���ݵĳ��� ����
{
	bool calcuBCC = TRUE;
	static int currentNum = -1;
	if (num != currentNum)
	{
		currentNum = num;
		calcuBCC = TRUE;
	}
	else
	{
		calcuBCC = FALSE;
	}
	char acTmp[2000];
	int i = 0;
	int j = 512 * (num - 1);//���Ŵ�0��ʼ
	int dlen;
	int plen = (len % 512 == 0) ? (len / 512) : (len / 512 + 1);
	CString tm;
	tm.Format("%d,%d", len, plen);
	TRACE(tm);
	if (j >= len)
	{
		return 0;
	}
	CTime time = CTime::GetCurrentTime();
	char Hour = time.GetHour();
	char Miu = time.GetMinute();
	char Sec = time.GetSecond();

	unsigned char zhiling[26] = { 0xFF, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, Hour, Miu, Sec, 0x01, 0x01, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x0C, 0x02, 0x00 };//����ֵ
	memcpy(acTmp, zhiling, 26);

	acTmp[26] = ((char)plen >> 8);    //�ܰ���
	acTmp[27] = ((char)plen & 0xff);
	acTmp[28] = ((char)num >> 8);//����
	acTmp[29] = ((char)num & 0xff);
	int packlen_cur = 0;
	for (i = 0; i < 512 && (j + i) < len; i++)
	{
		acTmp[33 + i * 2] = (char)(datapacket[j + i] >> 8);
		acTmp[32 + i * 2] = (char)(datapacket[j + i]);
		packlen_cur++;
	}

	j = packlen_cur * 2;
	if (calcuBCC)
	{
		bccW_packs = XorMaiKuan_download(bccW_packs, (short*)(&acTmp[32]), packlen_cur);
	}
	acTmp[30] = (char)(j >> 8);//�������ݳ���
	acTmp[31] = (char)(j & 0xff);
	int lenth = j + 32;
	acTmp[2] = (char)(lenth >> 8);
	acTmp[3] = (char)(lenth & 0xff);
	memcpy(sendpacket, acTmp, (j + 36));
	return (j + 36);
} 

void CLaseAutoCorrectDlg::InitAutoCorrectTask(void)
{

	//��ʼ���ź���
	ResetEvents();
	//��ʼ��������
	g_bCountStart=0;

	g_nReviseNum=0;

	g_bMoveFlag = false;
	g_bOpenflag = false;
	g_bCloseflag = false;
	g_nComComandStat = 0;
	g_nContinuousMoveState = 0;

	memset(g_nXY,0,sizeof(g_nXY));
	memset(g_datax,0,sizeof(g_datax));
	memset(g_datay,0,sizeof(g_datay));
	g_nInterpNum=0;
	g_nXYlen=0;

	g_nAvgStop1=0;
	g_nAvgStop2=0;
	g_nSumStop1=0;
	g_nSumStop2=0;
	g_nRevcNumcCheck1 = 0;

	CString g_cover="1";
	BOOL g_nameadd=FALSE;
	filenames.clear();
	::SendMessage(this->GetSafeHwnd(),WM_IMG_CHANGE,0,0);
	memset(&AutoCorretRegInfo,0,sizeof(AutoCorretRegInfo));
	if (m_pMyLog)
	{
		m_pMyLog->Release();
	}
}

void CLaseAutoCorrectDlg::DispTips(UINT nResID,CString Tips)
{
	CStatic* pWnd = (CStatic*)GetDlgItem(IDC_StepImg); // �õ� Picture Control ���
	CImage image;
	LoadImageFromResource(&image,nResID,_T("PNG"));
	//image.Load(_T("res\\i.png"));
	pWnd->ModifyStyle(0xF,SS_BITMAP|SS_CENTERIMAGE);
	HBITMAP hBmp = image.Detach();
	pWnd->SetBitmap(hBmp);
	//pWnd->SetWindowPos(NULL, 
	//	0, 
	//	0, 
	//	128, 
	//	128, 
	//	SWP_NOACTIVATE | SWP_NOMOVE | SWP_NOZORDER);
	//CRect rtlbl; 
	//GetDlgItem(IDC_STATIC_TIPS)->GetWindowRect(&rtlbl); 
	//ScreenToClient(&rtlbl); //ת���ͻ��˽���
	//InvalidateRect(&rtlbl);//���ˢ�¶Ի��򱳾� 
	//SetDlgItemTextA(IDC_STATIC_TIPS,Tips);
	GetDlgItem(IDC_STATIC_TIPS)->SetWindowTextA(Tips);
}
/////////////////////////////////////////////////////////////////////////
/// ����Դ�ļ��м���ͼƬ
/// @param [in] pImage ͼƬָ��
/// @param [in] nResID ��Դ��
/// @param [in] lpTyp ��Դ����
//////////////////////////////////////////////////////////////////////////
bool CLaseAutoCorrectDlg::LoadImageFromResource(IN CImage* pImage,IN UINT nResID, IN LPCSTR lpTyp)
{
	if ( pImage == NULL) 
		return false;

	pImage->Destroy();

	// ������Դ
	HRSRC hRsrc = ::FindResource(AfxGetResourceHandle(), MAKEINTRESOURCE(nResID), lpTyp);
	if (hRsrc == NULL) return false;

	// ������Դ
	HGLOBAL hImgData = ::LoadResource(AfxGetResourceHandle(), hRsrc);
	if (hImgData == NULL)
	{
		::FreeResource(hImgData);
		return false;
	}

	// �����ڴ��е�ָ����Դ
	LPVOID lpVoid    = ::LockResource(hImgData);

	LPSTREAM pStream = NULL;
	DWORD dwSize    = ::SizeofResource(AfxGetResourceHandle(), hRsrc);
	HGLOBAL hNew    = ::GlobalAlloc(GHND, dwSize);
	LPBYTE lpByte    = (LPBYTE)::GlobalLock(hNew);
	::memcpy(lpByte, lpVoid, dwSize);

	// ����ڴ��е�ָ����Դ
	::GlobalUnlock(hNew);

	// ��ָ���ڴ洴��������
	HRESULT ht = ::CreateStreamOnHGlobal(hNew, TRUE, &pStream);
	if ( ht != S_OK )
	{
		GlobalFree(hNew);
	}
	else
	{
		// ����ͼƬ
		pImage->Load(pStream);

		GlobalFree(hNew);
	}

	// �ͷ���Դ
	::FreeResource(hImgData);

	return true;
}

HBRUSH CLaseAutoCorrectDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	HBRUSH hbr = CDialogEx::OnCtlColor(pDC, pWnd, nCtlColor);

	// TODO:  �ڴ˸��� DC ���κ�����
	switch(pWnd->GetDlgCtrlID()) 
	{ 
	case IDC_STATIC_TIPS: 
		pDC->SetBkMode(TRANSPARENT); 
		pDC->SetTextColor(RGB(0,0,0)); 
		if (pWnd->GetDlgCtrlID() != IDC_STATIC_TIPS)
		{
			return (HBRUSH)GetStockObject(HOLLOW_BRUSH);
		}
		//return (HBRUSH)GetStockObject(HOLLOW_BRUSH); 
	default: 
		break; 
	} 
	// TODO:  ���Ĭ�ϵĲ������軭�ʣ��򷵻���һ������
	return hbr;
}

CString FilePathOper = "";
CFile FileOper;
void CLaseAutoCorrectDlg::RecordVerifyMK(int CurMK)
{
	CString Fpath = m_SavePath + "\\"+m_MacAddr + "��֤����.txt";
	CString strLog;
	strLog.Format("%d", CurMK);
	strLog = strLog + "\r\n";
	if (FilePathOper != Fpath)
	{
		FilePathOper = Fpath;
		FileOper.Open(Fpath, CFile::modeCreate | CFile::modeWrite | CFile::typeBinary);
	}
	else
	{
		FileOper.Open(Fpath, CFile::modeCreate | CFile::modeNoTruncate | CFile::modeWrite | CFile::typeBinary);
	}
	FileOper.SeekToEnd();
	FileOper.Write(strLog.GetBuffer(), strLog.GetLength() * sizeof(TCHAR));//strΪCString����
	FileOper.Close();
}

UINT AqcOrgThreadProc(LPVOID lpParam)
{
	CString strTmp="";
	CLaseAutoCorrectDlg* pDlg = (CLaseAutoCorrectDlg*)lpParam;
	int nAvgMk1=0;
	int nAvgMk2=0;
	int nAvgMk3=0;
	int nClacMk=0;
	int nCalfram=0;
	int nSglThred=0;
	CString strTemp="";
	int SingleValueErrorTemp=0;
	int AVEValueErrorTemp=0;

	int cs=0;
	while (bAqcOrgTerm)
	{
		if(g_nJGProCnt!=g_nJGRecCnt)
		{
			//������������
			if (g_SysDevParam.LaserType==6)//||g_SysDevParam.LaserType==4||g_SysDevParam.LaserType==5)   //710,710B,710C  712
			{
				if (g_bCountStart!=3)//((g_SysDevParam.LaserType==6||g_SysDevParam.LaserType==4) && g_bCountStart!=3)
				{
					//�����һ·���ݵ������ֵ
					nAvgMk1=pDlg->UDP_360DataSum711(g_nJGdata[0][g_nJGProCnt],28);
					//����ڶ�·���ݵ������ֵ
					nAvgMk2=pDlg->UDP_360DataSum711(g_nJGdata[1][g_nJGProCnt],28);
					//�������·���ݵ������ֵ
					//nAvgMk3=pDlg->UDP_360DataSum711(g_nJGdata[2][g_nJGProCnt],8,1);
				}
				else
				{
					//�����һ·���ݵ������ֵ
					nAvgMk1=pDlg->UDP_DataSum711(g_nJGdata[0][g_nJGProCnt],28);
					//����ڶ�·���ݵ������ֵ
					nAvgMk2=pDlg->UDP_DataSum711(g_nJGdata[1][g_nJGProCnt],28);
					//�������·���ݵ������ֵ
					nAvgMk3=pDlg->UDP_DataSum711(g_nJGdata[2][g_nJGProCnt],28);
				}			
			}
			else if(g_SysDevParam.LaserType==1)
			{
				    //�����һ·���ݵ������ֵ
					nAvgMk1=pDlg->UDP_DataSum(g_nJGdata[0][g_nJGProCnt],8,1);
					//����ڶ�·���ݵ������ֵ
					nAvgMk2=pDlg->UDP_DataSum(g_nJGdata[1][g_nJGProCnt],8,1);
					//�������·���ݵ������ֵ
					nAvgMk3=pDlg->UDP_DataSum(g_nJGdata[2][g_nJGProCnt],8,1);
			}
			else if (g_SysDevParam.LaserType==2)    //710A
			{
				//�����һ·���ݵ������ֵ
				nAvgMk1=pDlg->UDP_DataSum(g_nJGdata[0][g_nJGProCnt],9,2);   //������
				//����ڶ�·���ݵ������ֵ
				nAvgMk2=pDlg->UDP_DataSum(g_nJGdata[1][g_nJGProCnt],9,2);   //�½���
				//�������·���ݵ������ֵ
				nAvgMk3=pDlg->UDP_DataSum(g_nJGdata[2][g_nJGProCnt],9,2);   //�������������
			}
				
			//�������
			pDlg->m_MkAvg = nAvgMk2 - nAvgMk1;   //��ʵ����ֵ��������������
			strTmp.Format("%d",pDlg->m_MkAvg);
			pDlg->GetDlgItem(IDC_EDIT_MkAvg)->SetWindowTextA((LPCTSTR)strTmp);
			//�ж��Ƿ�ﵽ450ns
			if (g_bOpenflag == true)
			{
				if ((pDlg->m_MkAvg/1000>MK_ORG0))
				{
					g_bOpenflag = false;
					AutoCorretRegInfo.nOrigMkChCode=4;
					if (AutoCorretRegInfo.nStepIndx==STEP_MAXMK)
					{
						SetEvent(Even_ORIGMK);
					}
					else if(AutoCorretRegInfo.nStepIndx==STEP_BEFORECHECKOPEN)
					{
						SetEvent(Event_FINALCK);
					}
				}	
			}

			if (g_bCountStart==1)//�����������
			{
				nCalfram=25;
				g_nCacuNum2++;
				g_nSumStop1+=nAvgMk1;
				g_nSumStop2+=nAvgMk2;
				if (g_nCacuNum2>=nCalfram)
				{
					nClacMk=(g_nSumStop2-g_nSumStop1)/g_nCacuNum2;
					g_nSumStop1=0;
					g_nSumStop2=0;
					g_nCacuNum2=0;
					g_bCountStart=0;
					//�ж��Ƿ�ﵽ450ns
					if (AutoCorretRegInfo.nStepIndx==STEP_MAXMK)
					{
						if ((nClacMk/1000<MK_ORG0))
						{
							AutoCorretRegInfo.nOrigMkChCode=2;
						}
						else
						{
							if ((nClacMk/1000>MK_ORG1))
							{
								BeginValue=450000;
							}
							else
							{
								BeginValue=nClacMk;
							}
							AutoCorretRegInfo.nOrigMkChCode=1;
						}
						SetEvent(Even_ORIGMK);
					}
					else if (AutoCorretRegInfo.nStepIndx==STEP_ACQMK)
					{
						//PID����
						g_Pid.feedback=nClacMk;
						g_Pid.e_0=g_Pid.target-g_Pid.feedback;
						//�ﵽ���Ҫ�� ���¼
						if ((g_Pid.e_0<(g_AcqTartetInfo.DevValue[g_AcqTartetInfo.AqcIndx]))&&(g_Pid.e_0>(-g_AcqTartetInfo.DevValue[g_AcqTartetInfo.AqcIndx])))
						{
							AutoCorretRegInfo.nAqcMkCode=4;
							SetEvent(Event_AQCMK);
						}
						else
						{
							//Ȼ�������������������ķ���Ͳ���
							strTemp.Format("��ǰ����ֵ%d",nClacMk);
							::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
							AutoCorretRegInfo.nAqcMkCode=2;
							SetEvent(Event_AQCMK);
						}
					}
				}
			}
			else if (g_bCountStart==2)//���м���
			{
				nCalfram=pDlg->m_MkCalNum;
				g_nCacuNum2++;
				g_nSumStop1+=nAvgMk1;
				g_nSumStop2+=nAvgMk2;
				if (g_nCacuNum2>=nCalfram)
				{
					g_bCountStart=0;
					nClacMk=(g_nSumStop2-g_nSumStop1)/g_nCacuNum2;
					if (AutoCorretRegInfo.nStepIndx==STEP_ACQMK)
					{
						if (g_nReviseNum == 0)
						{
							g_nFirstData = g_nSumStop1/g_nCacuNum2;
						}
						//nSglThred = g_nFirstData-g_nSumStop1/g_nCacuNum2;
						nSglThred = g_nReferUpEdge-g_nSumStop1/g_nCacuNum2;

						g_nReviseNum++;

						g_nXY[g_nXYlen][0] = nClacMk;
						g_nXY[g_nXYlen][1] = nSglThred;
						g_datax[g_nInterpNum] = nClacMk;
						g_datay[g_nInterpNum] = nSglThred;
						g_nInterpNum++;
						g_nXYlen++;	
						memset(&g_Pid,0,sizeof(g_Pid));
						g_AcqTartetInfo.AqcIndx=g_AcqTartetInfo.AqcIndx+1;
						g_nSumStop1=0;
						g_nSumStop2=0;
						g_nCacuNum2=0;

						strTemp.Format("��ǰ����ֵ%d,ǰ�ز�ֵ%d",nClacMk,nSglThred);
						::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					}
				}
			}
			else if (g_bCountStart==3)//һ����֤ʱ
			{
				if (AutoCorretRegInfo.nStepIndx==STEP_FINALCH)
				{
					nClacMk=pDlg->m_MkAvg;
					//pDlg->RecordVerifyMK(nClacMk);//��¼����ʹ�õ�������ֵ pl 0402
					if(g_bCloseSpeedChange == true)
					{
						int tempClacMk = nClacMk / 1000;
						if (tempClacMk < 320 && tempClacMk >= 250)//�������С��350ns������ر��ٶ�
						{
							/*g_bCloseSpeedChange = false;*/
							/*AutoCorretRegInfo.nFinalCkCode=4;
							SetEvent(Event_FINALCK);*/
							pDlg->m_InitialSpeed = 100;
							strTmp.Format("%d", pDlg->m_InitialSpeed);
							pDlg->GetDlgItem(IDC_EDIT_InitialSpeed)->SetWindowTextA((LPCTSTR)strTmp);
							pDlg->OnBnClickedButtonSetinispeed();
							Sleep(100);
							pDlg->m_WorkingSpeed = 100;
							strTmp.Format("%d", pDlg->m_WorkingSpeed);
							pDlg->GetDlgItem(IDC_EDIT_WorkingSpeed)->SetWindowTextA((LPCTSTR)strTmp);
							pDlg->OnBnClickedButtonSetworkspeed();
							Sleep(100);
							::SendMessageA(pDlg->GetSafeHwnd(), WM_GL_CLOSE, 0, 0);    //���·�һ��ر�
						}
						if (tempClacMk < 250 && tempClacMk >= 100)//�������С��300ns������ر��ٶ�
						{
							pDlg->m_InitialSpeed = 20;
							strTmp.Format("%d", pDlg->m_InitialSpeed);
							pDlg->GetDlgItem(IDC_EDIT_InitialSpeed)->SetWindowTextA((LPCTSTR)strTmp);
							pDlg->OnBnClickedButtonSetinispeed();
							Sleep(100);
							pDlg->m_WorkingSpeed = 20;
							strTmp.Format("%d", pDlg->m_WorkingSpeed);
							pDlg->GetDlgItem(IDC_EDIT_WorkingSpeed)->SetWindowTextA((LPCTSTR)strTmp);
							pDlg->OnBnClickedButtonSetworkspeed();
							Sleep(100);
							::SendMessageA(pDlg->GetSafeHwnd(), WM_GL_CLOSE, 0, 0);    //���·�һ��ر�
						}
						if (tempClacMk< 100)
						{
							g_bCloseSpeedChange = false;
							pDlg->m_InitialSpeed = 10;
							strTmp.Format("%d", pDlg->m_InitialSpeed);
							pDlg->GetDlgItem(IDC_EDIT_InitialSpeed)->SetWindowTextA((LPCTSTR)strTmp);
							pDlg->OnBnClickedButtonSetinispeed();
							Sleep(100);
							pDlg->m_WorkingSpeed =10;
							strTmp.Format("%d", pDlg->m_WorkingSpeed);
							pDlg->GetDlgItem(IDC_EDIT_WorkingSpeed)->SetWindowTextA((LPCTSTR)strTmp);
							pDlg->OnBnClickedButtonSetworkspeed();
							Sleep(100);
							::SendMessageA(pDlg->GetSafeHwnd(), WM_GL_CLOSE, 0, 0);    //���·�һ��ر�
						}
					}
					if (nClacMk/1000>g_CheckMKForm.ChMkRegionMin)
					{	
						if(g_CheckMKForm.ReferenceValue!=0 )//&& g_CheckMKForm.ValueJudge!=1)
						{
							for (int i=0;i<300;i++)
							{
								SingleValueErrorTemp=g_CheckMKForm.ReferenceValue-g_nCurMKValue[i];//g_nCurMKValue�б�����Ǽ�ʱֵ
								if (SingleValueErrorTemp>g_CheckMKForm.SingleErrorMAX)
								{
									g_CheckMKForm.SingleErrorMAX=SingleValueErrorTemp;
								} 
								if(SingleValueErrorTemp<g_CheckMKForm.SingleErrorMIN)
								{
									g_CheckMKForm.SingleErrorMIN=SingleValueErrorTemp;
								}

								if (g_CheckMKForm.SingleValueError<SingleValueErrorTemp || (-g_CheckMKForm.SingleValueError)>SingleValueErrorTemp)
								{
									g_CheckMKForm.ValueJudgeCnt++;
									//������
									cs=g_nRevcNumcCheck+1;																
									strTemp.Format("����ʱ�����ֵ%d ���ۼ���ֵΪ%d ���ĵ�%d֡��%d��ֵΪ%d ������ֵ֮��Ϊ%d �Ѵ������Ϊ%d",nClacMk,g_CheckMKForm.ReferenceValue,cs,i,g_nCurMKValue[i],SingleValueErrorTemp,g_CheckMKForm.ValueJudgeCnt);
									::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
									if(g_ChannelSelect==0)
									{
										if (g_CheckMKForm.ValueJudgeCnt>5)
										{
											g_CheckMKForm.ValueJudge=1;									
										}	
									}
									else
									{
										if (g_CheckMKForm.ValueJudgeCnt>1)
										{
											g_CheckMKForm.ValueJudge=1;									
										}
									}
								}
							}			
						}
						g_nRevcNumcCheck++;	
						//��Ϊ�òο�ֵȡ����125�������жϣ�ԭ��Ϊֱ���ж���ɢ����250����
						/*if (nAvgMk3>g_CheckMKForm.ChMkMax)
						{
							g_CheckMKForm.ChMkMax=nAvgMk3;
						}
						if (nAvgMk3<g_CheckMKForm.ChMkMin)
						{
							g_CheckMKForm.ChMkMin=nAvgMk3;
						}*/
						AVEValueErrorTemp=g_CheckMKForm.ReferenceValue-nAvgMk3;
						if (AVEValueErrorTemp>g_CheckMKForm.ChMkMax)
						{
							g_CheckMKForm.ChMkMax=AVEValueErrorTemp;
						} 
						if(AVEValueErrorTemp<g_CheckMKForm.ChMkMin)
						{
							g_CheckMKForm.ChMkMin=AVEValueErrorTemp;
						}
						if (AVEValueErrorTemp>g_CheckMKForm.AveValueError || AVEValueErrorTemp<(-g_CheckMKForm.AveValueError))
						{
							g_CheckMKForm.AveValueJudge=1;
							g_CheckMKForm.AveValueJudgeCnt++;
							strTemp.Format("����ʱ�����ֵ%d ���ۼ���ֵΪ%d ���ĵ�%d֡ ������ֵ֮��Ϊ%d �ۼƲ��ϸ�%d֡",nClacMk,g_CheckMKForm.ReferenceValue,cs,AVEValueErrorTemp,g_CheckMKForm.AveValueJudgeCnt);
							::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						}

					}
					else if (nClacMk/1000<=g_CheckMKForm.ChMkRegionMin)
					{
						g_CheckMKForm.ChMkOffset=g_CheckMKForm.ChMkMax-g_CheckMKForm.ChMkMin;
						if (g_CheckMKForm.ValueJudge==1)
						{
							AutoCorretRegInfo.nFinalCkCode=2;
						}
						else
						{
							if (g_CheckMKForm.AveValueJudge == 1)
							{
								AutoCorretRegInfo.nFinalCkCode=2;
								//������
								strTemp="��ֵ��ƫ�����";
								::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
							}
							else
							{
								AutoCorretRegInfo.nFinalCkCode=1;
							}
						}
						g_bCountStart=0;
						SetEvent(Event_FINALCK);
					}
				}
			}
			//����������1
			g_nJGProCnt=g_nJGProCnt+1;
			g_nJGProCnt=g_nJGProCnt%JGMAX_CNT;

		}
		if (g_SysDevParam.NetConnected==FALSE)
		{
			bAqcOrgTerm=FALSE;
		}
		Sleep(5);
	}
	return 0L;
}

LRESULT CLaseAutoCorrectDlg::OnChangeStep(WPARAM wParam, LPARAM lParam)
{
	int nCase=lParam;
	switch(nCase)
	{
	case 0:
		DispTips(IDB_PNG0,"����״̬");
		break;
	case 1:
		DispTips(IDB_PNG1,"���������׶�");
		break;
	case 2:
		DispTips(IDB_PNG2,"�������׶�");
		break;
	case 3:
		DispTips(IDB_PNG3,"��������׶�");
		break;
	case 4:
		DispTips(IDB_PNG4,"ԭʼ���ݲɼ��׶�");
		break;
	case 5:
		DispTips(IDB_PNG5,"��ϲ�ֵ�׶�");
		break;
	case 6:
		DispTips(IDB_PNG6,"����������׶�");
		break;
	case 7:
		DispTips(IDB_PNG7,"��֤���������ؽ׶�");
		break;
	case 8:
		DispTips(IDB_PNG8,"��֤��������׶�");
		break;
	case 9:
		DispTips(IDB_PNG9,"�����������");
		break;
	case 10:
		DispTips(IDB_PNGER,"�Զ��������δͨ��");
		break;
	case 11:
		DispTips(IDB_PNGOK,"�Զ��������ͨ��");
		break;
	default:

		break;
	}
	return 0L;
}

void CLaseAutoCorrectDlg::H02_Restart88(void)//�����������ذ�
{
	char Sendbuff[26] = {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x11,0x73,0x88,0x4e,0x20,0x4c,0x4d,0x44,0x73,0x63,0x61,0x6e,0x64,0x61,0x74,0x61,0x20,0x00};
	int SendbuffLen = 25;
	CString strTemp="";
	Add_BCC(Sendbuff,SendbuffLen);
	g_nNetSendBuffLen=SendbuffLen+1;
	memcpy(g_nNetSendBuff,Sendbuff,g_nNetSendBuffLen);
	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="�����������������ذ�";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::H02_Restart47(void)//����711�������ذ�
{
	char Sendbuff[26] = {(char)0x02, (char)0x02, (char)0x02, (char)0x02, (char)0x00, (char)0x00, (char)0x00, (char)0x11, (char)0x73,(char)0x47, (char)0x4E, 
		(char)0x20, (char)0x4C, (char)0x4D, (char)0x44, (char)0x73, (char)0x63, (char)0x61, (char)0x6E, (char)0x64, (char)0x61, (char)0x74, (char)0x61, (char)0x20, (char)0x00};
	int SendbuffLen = 25;
	CString strTemp="";
	Add_BCC(Sendbuff,SendbuffLen);
	g_nNetSendBuffLen=SendbuffLen+1;
	memcpy(g_nNetSendBuff,Sendbuff,g_nNetSendBuffLen);
	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="�����������������ذ�";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::SendCheckCorrectTb(void)
{
	char SendBuff[25]	= {(char)0x02, (char)0x02, (char)0x02, (char)0x02, (char)0x00, (char)0x00, (char)0x00, (char)0x10, (char)0x73, (char)0x59, (char)0x4e, (char)0x20, (char)0x4c, (char)0x4d, (char)0x44, (char)0x73, (char)0x63, (char)0x61, (char)0x6e, (char)0x64, (char)0x61, (char)0x74, (char)0x61, (char)0x20};
	int SendBuffLen = 24;

	Add_BCC(SendBuff,SendBuffLen);
	SendBuffLen=SendBuffLen+1;

	g_nNetSendBuffLen = SendBuffLen;
	memcpy(g_nNetSendBuff,SendBuff,g_nNetSendBuffLen);
	m_NetConnection.pcSendDataBuf = g_nNetSendBuff;
	m_NetConnection.nSendDataBufSize = g_nNetSendBuffLen;

	if (g_SysDevParam.LaserPor == 1)//TCPЭ��
	{
		if (!SendData(&m_NetConnection))
		{
			MessageBox("�����й��ϣ�����ʧ�ܣ�",MB_OK);
		}
	}
	else if (g_SysDevParam.LaserPor == 0)//UDPЭ��
	{
		DWORD dwIP;
		m_DestIp.GetAddress(dwIP);

		SOCKADDR_IN addrTo;
		addrTo.sin_family=AF_INET;
		addrTo.sin_port=htons(m_DestPort);
		addrTo.sin_addr.S_un.S_addr=htonl(dwIP);             

		UDP_SendData(&m_NetConnection, &addrTo);
	}	
}
//�������������
void CLaseAutoCorrectDlg::SendCheckCorrectTb711(void)
{
	CTime time=CTime::GetCurrentTime();
	char Hour= time.GetHour();
	char Miu= time.GetMinute();
	char Sec= time.GetSecond();
	char MaiKuanSearch_zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,Hour,Miu,Sec,0x01,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x0E,0x00,0x00};
	g_nNetSendBuffLen = 34;
	if(g_SysDevParam.LaserType == 4)
	{
		MaiKuanSearch_zhiling[13]=0x06;
		MaiKuanSearch_zhiling[21]=0x01;
		MaiKuanSearch_zhiling[24]=0x01;
		MaiKuanSearch_zhiling[25]=0x00;
	}
	if(g_SysDevParam.LaserType == 6)
	{
		MaiKuanSearch_zhiling[13]=0x07;
	}
	newBCC711(MaiKuanSearch_zhiling,34);
	memcpy(g_nNetSendBuff,MaiKuanSearch_zhiling,g_nNetSendBuffLen);
	m_NetConnection.pcSendDataBuf = g_nNetSendBuff;
	m_NetConnection.nSendDataBufSize = g_nNetSendBuffLen;

	if (g_SysDevParam.LaserPor == 1)//TCPЭ��
	{
		if (!SendData(&m_NetConnection))
		{
			MessageBox("�����й��ϣ�����ʧ�ܣ�",MB_OK);
		}
	}
	else if (g_SysDevParam.LaserPor == 0)//UDPЭ��
	{
		DWORD dwIP;
		m_DestIp.GetAddress(dwIP);

		SOCKADDR_IN addrTo;
		addrTo.sin_family=AF_INET;
		addrTo.sin_port=htons(m_DestPort);
		addrTo.sin_addr.S_un.S_addr=htonl(dwIP);             

		UDP_SendData(&m_NetConnection, &addrTo);
	}	
}

//�л�ѡ�
LRESULT CLaseAutoCorrectDlg::OnSelfChangeTab(WPARAM wParam, LPARAM lParam)
{
	int nSel=0;
	NMHDR nmhdr;
	nSel=(lParam>2?0:lParam);
	nmhdr.code	=	TCN_SELCHANGE; 
	nmhdr.hwndFrom	= m_Tab.GetSafeHwnd(); 
	nmhdr.idFrom=	m_Tab.GetDlgCtrlID();
	m_Tab.SetCurSel(nSel);
	::SendMessage(m_Tab.GetSafeHwnd(), WM_NOTIFY,MAKELONG(TCN_SELCHANGE,0), (LPARAM)(&nmhdr));
	return 0L;
}

LRESULT CLaseAutoCorrectDlg::OnFitInMainHandle(WPARAM wParam, LPARAM lParam)
{
	CString filename="";
	int nRes=0;
	filename=*((CString *)lParam);
	nRes=FitPro(filename);
	*((int *)wParam)=nRes;
	return 0L;
}

LRESULT CLaseAutoCorrectDlg::OnCutPicture(WPARAM wParam, LPARAM lParam)
{
	CString OpenFileName="";
	CString getDir="" ;

	if(AutoCorretRegInfo.nFinalCkCode==2)
	{
		if(g_nameadd)
		{
			OpenFileName=m_MacAddr+"���ϸ�("+g_cover+").jpg";
		}
		else
		{
			OpenFileName=m_MacAddr+"���ϸ�.jpg";
		}
	}
	else
	{
		if(g_nameadd)
		{
			OpenFileName=m_MacAddr+"�ϸ�("+g_cover+").jpg";
		}else{
			OpenFileName=m_MacAddr+"�ϸ�.jpg";
		}
	}
	
	getDir= m_SavePath + "\\"+ OpenFileName;
	//getDir="C:\\Users\\WANJI\\Desktop\\��V2��LaseAutoCorrect-20161020\\Debug\\�Զ��������\\1.jpg";
	//m_prop1.m_ChartReal.Series(0).SetActive(FALSE);
	m_prop1.m_ChartReal.Series(1).SetActive(FALSE);
	//m_prop1.m_ChartReal.GetAxis().GetLeft().SetMinMax(g_CheckMKForm.ChMkMin-1000,g_CheckMKForm.ChMkMax+1000);
	m_prop1.m_ChartReal.GetAxis().GetLeft().SetMinMax(g_CheckMKForm.ReferenceValue-g_CheckMKForm.ChMkMin-1000,g_CheckMKForm.ReferenceValue-g_CheckMKForm.ChMkMax+1000);
	m_prop1.m_ChartReal.GetAxis().GetLeft().SetIncrement(100);
	m_prop1.m_ChartReal.GetAxis().GetBottom().SetMinMax(0,(g_nRevcNumcCheck1)*300);
	m_prop1.m_ChartReal.GetAxis().GetBottom().SetIncrement((g_nRevcNumcCheck1)*300/8);
	m_prop1.m_ChartReal.GetExport().SaveToBitmapFile(getDir);
	return 0L;
}

void CLaseAutoCorrectDlg::GetLmdSinglePoint()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	BYTE NetSendBuff[100]={0};
	char NetContentBuff[50]={0};
	int Tmplen=0;

	NetSendBuff[0]=0xFF;
	NetSendBuff[1]=0xAA;

	Tmplen=2;
	NetContentBuff[Tmplen++]=0x03;
	NetContentBuff[Tmplen++]=0x01;

	NetContentBuff[Tmplen++]=0x00;
	NetContentBuff[Tmplen++]=0x00;
	NetContentBuff[Tmplen++]=0x01;

	//У��λ ��һλ
	NetContentBuff[Tmplen++]=0x00;

	//����λ
	NetContentBuff[0]=((Tmplen+1)>>8)&0xFF;
	NetContentBuff[1]=((Tmplen+1))&0xFF;

	NetContentBuff[Tmplen++]=CalcBcc_710A(NetContentBuff,Tmplen-1);

	memcpy(NetSendBuff+2,NetContentBuff,Tmplen);

	Tmplen+=2;

	NetSendBuff[Tmplen++]=0xEE;
	NetSendBuff[Tmplen++]=0xEE;
	
	g_nNetSendBuffLen=Tmplen;
	memcpy(g_nNetSendBuff,NetSendBuff,g_nNetSendBuffLen);

	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
}

void CLaseAutoCorrectDlg::Get711LmdSinglePoint()     //����712
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	if(g_SysDevParam.LaserType==4||g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==6)
	{
		CTime time=CTime::GetCurrentTime();
		char Hour= time.GetHour();
		char Miu= time.GetMinute();
		char Sec= time.GetSecond();
		char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,Hour,Miu,Sec,0x01,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x01};
		int SendbuffLen = 34;
		
		g_nNetSendBuffLen=34;
		if(g_SysDevParam.LaserType==4)
		{
			zhiling[13]=0x06;
			zhiling[21]=0x01;
			zhiling[24]=0x01;
			zhiling[25]=0x00;
		}
		else if (g_SysDevParam.LaserType==6)
		{
			zhiling[13]=0x07;
		}
		newBCC711(zhiling,34);
		memcpy(g_nNetSendBuff,zhiling,g_nNetSendBuffLen);
	}
	else if(g_SysDevParam.LaserType==5)
	{
		char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x06,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x1B,0xEE,0xEE};
		g_nNetSendBuffLen=34;
		memcpy(g_nNetSendBuff,zhiling,g_nNetSendBuffLen);
	}
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="���ͻ�ȡ��֡����";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

LRESULT CLaseAutoCorrectDlg::OnGet710ALMD(WPARAM wParam, LPARAM lParam)
{
	GetLmdSinglePoint();
	StartTimer(TIMERGETLMD,TIMER_GETLMD,3000);
	return 0L;
}

LRESULT CLaseAutoCorrectDlg::OnGet711LMD(WPARAM wParam, LPARAM lParam)
{
	Get711LmdSinglePoint();
	StartTimer(TIMERGETLMD,TIMER_GETLMD,3000);
	return 0L;
}

void CLaseAutoCorrectDlg::HFA_Restart0C(void)//�����������ذ�
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	BYTE NetSendBuff[100]={0};
	char NetContentBuff[50]={0};
	int Tmplen=0;

	NetSendBuff[0]=0xFF;
	NetSendBuff[1]=0xAA;

	Tmplen=2;
	NetContentBuff[Tmplen++]=0x02;
	NetContentBuff[Tmplen++]=0x0C;

	NetContentBuff[Tmplen++]=0x00;
	NetContentBuff[Tmplen++]=0x00;
	NetContentBuff[Tmplen++]=0x00;

	//У��λ ��һλ
	NetContentBuff[Tmplen++]=0x00;

	//����λ
	NetContentBuff[0]=((Tmplen+1)>>8)&0xFF;
	NetContentBuff[1]=((Tmplen+1))&0xFF;

	NetContentBuff[Tmplen++]=CalcBcc_710A(NetContentBuff,Tmplen-1);

	memcpy(NetSendBuff+2,NetContentBuff,Tmplen);

	Tmplen+=2;

	NetSendBuff[Tmplen++]=0xEE;
	NetSendBuff[Tmplen++]=0xEE;

	g_nNetSendBuffLen=Tmplen;
	memcpy(g_nNetSendBuff,NetSendBuff,g_nNetSendBuffLen);

	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="�����������������ذ�";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::SendCheckCorrectTb710A(void)
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	BYTE NetSendBuff[100]={0};
	char NetContentBuff[50]={0};
	int Tmplen=0;

	NetSendBuff[0]=0xFF;
	NetSendBuff[1]=0xAA;

	Tmplen=2;
	NetContentBuff[Tmplen++]=0x02;
	NetContentBuff[Tmplen++]=0x0A;

	NetContentBuff[Tmplen++]=0x00;
	NetContentBuff[Tmplen++]=0x00;
	NetContentBuff[Tmplen++]=0x00;

	//У��λ ��һλ
	NetContentBuff[Tmplen++]=0x00;

	//����λ
	NetContentBuff[0]=((Tmplen+1)>>8)&0xFF;
	NetContentBuff[1]=((Tmplen+1))&0xFF;

	NetContentBuff[Tmplen++]=CalcBcc_710A(NetContentBuff,Tmplen-1);

	memcpy(NetSendBuff+2,NetContentBuff,Tmplen);

	Tmplen+=2;

	NetSendBuff[Tmplen++]=0xEE;
	NetSendBuff[Tmplen++]=0xEE;

	g_nNetSendBuffLen=Tmplen;
	memcpy(g_nNetSendBuff,NetSendBuff,g_nNetSendBuffLen);

	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
}

void CLaseAutoCorrectDlg::OnCbnSelchangeComboLasertype()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	int nSel=0;
	nSel=m_LaserType.GetCurSel();
	switch(nSel)
	{
	case 0:
	case 1:
	case 2:
		//m_HostIpAddr.SetAddress(192,168,0,76);
		m_DestIp.SetAddress(192,168,0,2);
		m_DestPort=2110;
		m_PorType.SetCurSel(0);
		GetDlgItem(IDC_COMBO_ChannelSelect)->ShowWindow(FALSE);
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("��ǰMAC��ַ");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("����MAC��ַ");
		GetDlgItem(IDC_EDIT_MACSET)->ShowWindow(TRUE);
		m_SetMAC="001234000";
		GetDlgItem(IDC_EDIT_MACSET)->SetWindowText(m_SetMAC);
		m_AveError=400;
		m_AllSingleError=500;
		m_CheckRangeMINValue=20;
		break;
	case 3:
		//m_HostIpAddr.SetAddress(192,168,0,76);
		m_DestIp.SetAddress(192,168,0,2);
		m_DestPort=2114;
		m_PorType.SetCurSel(0);
		GetDlgItem(IDC_COMBO_ChannelSelect)->ShowWindow(FALSE);
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("��ǰMAC��ַ");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("����MAC��ַ");
		GetDlgItem(IDC_EDIT_MACSET)->ShowWindow(TRUE);
		m_SetMAC="001234000";
		GetDlgItem(IDC_EDIT_MACSET)->SetWindowText(m_SetMAC);
		m_AveError=400;
		m_AllSingleError=500;
		m_CheckRangeMINValue=20;
		break;
	case 4:
		//m_HostIpAddr.SetAddress(192,168,0,76);
		m_DestIp.SetAddress(192,168,0,2);
		m_DestPort=2110;
		m_PorType.SetCurSel(1);
		GetDlgItem(IDC_COMBO_ChannelSelect)->ShowWindow(TRUE);
		m_ChannelSelect.SetCurSel(0);
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("��ǰ�豸��");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("ͨ��ѡ��");
		GetDlgItem(IDC_EDIT_MACSET)->ShowWindow(FALSE);
		m_AveError=250;
		m_AllSingleError=300;
		m_CheckRangeMINValue=15;
		break;
	case 5:
		//m_HostIpAddr.SetAddress(192,168,0,76);
		m_DestIp.SetAddress(192,168,0,2);
		m_DestPort=2110;
		m_PorType.SetCurSel(1);
		GetDlgItem(IDC_COMBO_ChannelSelect)->ShowWindow(TRUE);
		m_ChannelSelect.SetCurSel(0);
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("��ǰ�豸��");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("ͨ��ѡ��");
		GetDlgItem(IDC_EDIT_MACSET)->ShowWindow(FALSE);
		m_AveError=250;
		m_AllSingleError=300;
		m_CheckRangeMINValue=15;
		break;
	case 6:
		//m_HostIpAddr.SetAddress(192,168,0,76);
		m_DestIp.SetAddress(192,168,0,2);
		m_DestPort=2110;
		m_PorType.SetCurSel(1);
		GetDlgItem(IDC_COMBO_ChannelSelect)->ShowWindow(TRUE);
		m_ChannelSelect.SetCurSel(0);
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("��ǰ�豸��");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("ͨ��ѡ��");
		GetDlgItem(IDC_EDIT_MACSET)->ShowWindow(FALSE);
		m_AveError=250;
		m_AllSingleError=300;
		m_CheckRangeMINValue=15;
		break;
	case 7:
		//m_HostIpAddr.SetAddress(192,168,0,76);
		m_DestIp.SetAddress(192,168,0,2);
		m_DestPort=2110;
		m_PorType.SetCurSel(1);
		GetDlgItem(IDC_COMBO_ChannelSelect)->ShowWindow(FALSE);
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("��ǰMAC��ַ");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("����MAC��ַ");
		GetDlgItem(IDC_EDIT_MACSET)->ShowWindow(TRUE);
		m_SetMAC="001234000";
		GetDlgItem(IDC_EDIT_MACSET)->SetWindowText(m_SetMAC);
		m_AveError=125;
		m_AllSingleError=300;
		m_CheckRangeMINValue=7;
		break;
	default:
		break;
	}
	UpdateData(FALSE);
}

void CLaseAutoCorrectDlg::OnBnClickedButton1()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	UpdateData(true);
	CWinThread *pThread;
	g_ChannelSelect = m_ChannelSelect.GetCurSel();
	MK_ORG0=450;
	MK_ORG1=500;

	if (g_SysDevParam.NetConnected&&g_SysDevParam.ComConnected)
	{
		//��ʼ����ر���
		InitAutoCorrectTask();
		//���������߳�
		bAutoCorrTerm=TRUE;
		pThread = ::AfxBeginThread(AutoCheckThreadProc,this);//���������߳�
		if(pThread == NULL)
		{
			bAutoCorrTerm=FALSE;
			return;
		}
		SetEvent(Event_START);
	}
}

UINT AutoCheckThreadProc(LPVOID lpParam)  //��֤������
{
	CLaseAutoCorrectDlg *pDlg = (CLaseAutoCorrectDlg*)lpParam;
	CString strTmp="";
	DWORD nRet =0;
	int nFitRes=0;
	CString Filename="";
	CString strTemp="";
	CWinThread *pThread=NULL;

	while (bAutoCorrTerm)
	{
		nRet = WaitForMultipleObjects(11,Event_List,false,600*1000);	//�ȴ���ʱ

		if(WAIT_TIMEOUT == nRet)	//�ȴ��¼���ʱʱ�䵽
		{
			strTemp="�Զ�������ʱ��ֹ";
			//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,0);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
			SetEvent(Event_STOP);
		}
		else if(WAIT_OBJECT_0 == nRet)	//Event_START �������ź�״̬ �����Զ���������
		{
			AutoCorretRegInfo.nStepIndx=STEP_PARAM;
			::PostMessageA(pDlg->GetSafeHwnd(),WM_LASER_GETPARAM,0,0);
			pDlg->StartTimer(TIMERGETPARMA,TIMER_GETPARAM,5000);
		}
		else if((WAIT_OBJECT_0 + 1) == nRet)	//Event_PARAM���ź� ����г�ʼ�������
		{
			if (AutoCorretRegInfo.nParamChCode==1)
			{
				strTemp="����������ͨ��";
				//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

				SetEvent(Even_ORIGMK);				
			}
			else if (AutoCorretRegInfo.nParamChCode==2)
			{
				strTemp="������鲻��Ҫ��";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
				SetEvent(Event_STOP);
			}
			else if (AutoCorretRegInfo.nParamChCode==3)
			{
				strTemp="�����޷���ȡ��";
				//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
				SetEvent(Event_STOP);
			}
			else if (AutoCorretRegInfo.nParamChCode==4)
			{
				//���������ļ��м��ļ�
				pDlg->OnCreateFolder(pDlg->m_MacAddr);
				CString name="���ͼ";
				pDlg->TraverseDir(CString(pDlg->m_SavePath), filenames);
				for(int i=0;i<filenames.size();i++)
				{
					if(filenames[i].Find(name)>=0)
					{
						g_nameadd=TRUE;
						int t=filenames[i].FindOneOf("(");
						if(t>0)
						{
							int hou=filenames[i].FindOneOf(")");
							CString fileindex= filenames[i].Mid(t+1,hou-t-1);
							int coverindex=atoi(fileindex); 
							coverindex++;
							if(coverindex> atoi(g_cover))
							{
								g_cover.Format("%d",coverindex); 
							}
						}
					}
				}

				if(g_nameadd)
				{
					//pDlg->OnCreateFile(pDlg->m_MacAddr+"_ԭʼ����"+"("+g_cover+").txt",1);
					pDlg->OnCreateFile(pDlg->m_MacAddr+"_���Լ�¼"+"("+g_cover+").txt",2);
				}
				else
				{
					//pDlg->OnCreateFile(pDlg->m_MacAddr+"_ԭʼ����.txt",1);
					pDlg->OnCreateFile(pDlg->m_MacAddr+"_���Լ�¼.txt",2);
				}
			
			
				m_pMyLog = CCDMLogInfo::GetInstance(pDlg->m_OperTxtPath.GetBuffer(pDlg->m_OperTxtPath.GetLength()));//��ȡ��־�ļ�
				strTemp=  "\r\n========" + CTime::GetCurrentTime().Format("%H:%M:%S") + "========\r\n";
				if(m_pMyLog)	//д����־
					m_pMyLog->SetNotify(strTemp.GetBuffer(strTemp.GetLength()));
				strTemp="�������ݱ����ļ�";
				//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				
				AutoCorretRegInfo.nWaitInputDistCnt=0;
				strTmp="�����������Ϣ��Ȼ����[����ȷ��]��ť";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				AfxMessageBox(_T("�����������Ϣ��Ȼ����[����ȷ��]��ť"),MB_OK);
				pDlg->StartTimer(TIMERINPUTDIST,TIMER_INPUTDIST,5000);
			}
		}
		else if ((WAIT_OBJECT_0 + 2) == nRet)//Even_ORIGMK���ź� ��ʼ��ʼ������  712Ϊ���ֵ���450ns
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
			{
				//�ж���������Ƿ�Ϊ450ns
				strTemp="�ж���������Ƿ�Ϊ450ns";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				AutoCorretRegInfo.nStepIndx=STEP_MAXMK;
				g_bCountStart=1;   //��ʼͳ���Ƿ�ﵽ450ns
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_MAXMK)
			{
				if (AutoCorretRegInfo.nOrigMkChCode==2)
				{
					g_bOpenflag=true;  //��ʼʵʱ�ж��ſ���ʱֹͣ
					strTemp="�������ֵδ��450ns���Զ��ſ�����";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_OPEN,0,0);
				}
				else if (AutoCorretRegInfo.nOrigMkChCode==1)
				{
					strTemp="�������ֵok";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					AutoCorretRegInfo.nStepIndx=STEP_CHECKTB;
					SetEvent(Event_FINALCK);
				}
				else if(AutoCorretRegInfo.nOrigMkChCode==3)
				{
					strTemp="�������ֵ�޷��ﵽ450ns";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if(AutoCorretRegInfo.nOrigMkChCode==4)
				{
					//ʶ��ﵽ450ns��ֹͣ�ſ�
					strTemp="ʶ��ﵽ450ns��ֹͣ�ſ�";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);
					//��ʼͳ���Ƿ�ﵽ450ns,����ֹͣ��Ӧ���޸ı�־λ
				}
			}
		}
		else if((WAIT_OBJECT_0 + 8) == nRet)//Event_FINALCK ���һ����������Ƿ���
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
			{
				AutoCorretRegInfo.nStepIndx=STEP_FINALCH;
				strTemp="��ʼ��֤����Ч��";
				AutoCorretRegInfo.nFinalCkCode=3;
				pDlg->m_prop1.m_ChartReal.Series(0).Clear();
				pDlg->m_prop1.m_ChartReal.Series(1).Clear();
				pDlg->m_prop1.m_ChartReal.Series(2).Clear();
				g_nRevcNum3=0;
				SetEvent(Event_FINALCK);
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_FINALCH)
			{
				if (AutoCorretRegInfo.nFinalCkCode==1)
				{
					if (g_bCloseflag == true)
					{
						g_bCloseflag = false;
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //ֹͣ�����رչ���
					}
					//������
					strTemp.Format("���������%d ���Ĺ�%d֡ ���յĹ�%d(%d)(%d)֡ ��ֵƫ��������СֵΪ%d,%d ����������ֵ��СֵΪ%d,%d",g_CheckMKForm.ValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkMax,g_CheckMKForm.ChMkMin,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,11);
					strTemp="������һ����֤ͨ��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
				//	Sleep(10000);
					//712��֤ͨ�����ſ�����
					//::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_OPEN,0,0);
					//pDlg->OnOnekeyTestFinish();
					SetEvent(Event_OVEROPEN);
				}
				else if (AutoCorretRegInfo.nFinalCkCode==2)
				{		
					if (g_bCloseflag == true)
					{
						g_bCloseflag = false;
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //ֹͣ�����رչ���
					}
					//������
					//strTemp.Format("���������%d ���Ĺ�%d֡ ���յĹ�%d(%d)(%d)֡ ��ֵ������С֮��Ϊ%d ����������ֵ��СֵΪ%d,%d",g_CheckMKForm.ValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkOffset,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					strTemp.Format("���������%d ���ϸ�֡����%d ���Ĺ�%d֡ ���յĹ�%d(%d)(%d)֡ ��ֵƫ��������СֵΪ%d,%d ����������ֵ��СֵΪ%d,%d",g_CheckMKForm.ValueJudgeCnt,g_CheckMKForm.AveValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkMax,g_CheckMKForm.ChMkMin,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,10);
					strTemp="������һ����֤δ��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					//712��֤�������ſ�����
					SetEvent(Event_OVEROPEN);
				}
				else if (AutoCorretRegInfo.nFinalCkCode==3)
				{
					pDlg->m_ChartNum=3000;
					strTmp.Format("%d",pDlg->m_ChartNum);
					pDlg->GetDlgItem(IDC_EDIT_ChartNum)->SetWindowTextA(strTmp);
					memset(&g_CheckMKForm,0,sizeof(CheckMKForm));
					pDlg->GetDlgItem(IDC_EDIT_AveError)->GetWindowTextA(strTmp);
					g_CheckMKForm.AveValueError=_ttoi(strTmp);
					pDlg->GetDlgItem(IDC_EDIT_AllSingleError)->GetWindowTextA(strTmp);
					g_CheckMKForm.SingleValueError=_ttoi(strTmp);
					pDlg->GetDlgItem(IDC_EDIT_CheckRangeMINValue)->GetWindowTextA(strTmp);
					g_CheckMKForm.ChMkRegionMin=_ttoi(strTmp);
					strTemp.Format("��֤��׼Ϊ����ֵ����%d����������%d,�������Χ��Сֵ%d",g_CheckMKForm.AveValueError,g_CheckMKForm.SingleValueError,g_CheckMKForm.ChMkRegionMin);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					g_CheckMKForm.ChMkRegionMax=450;
					g_CheckMKForm.ReferenceValue=g_nReferUpEdge;
		

					g_nRevcNumcCheck=0;
					g_nRevcNumcCheck2=0;			
					g_CheckMKForm.ValueJudgeCnt=0;
					g_CheckMKForm.ChMkMin=999999;
					g_CheckMKForm.SingleErrorMIN=999999;
					//712��ʼ�պ���֤
					g_bCloseflag=true;  //��ʼʵʱ�жϱպϺ�ʱֹͣ
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_CLOSE,0,0);
					g_bCloseSpeedChange = true;  //�����жϺ�ʱ���͹ر��ٶ�
				}
				else if (AutoCorretRegInfo.nFinalCkCode==4)   //���͹ر��ٶ�
				{
					pDlg->m_InitialSpeed = 50;
					strTmp.Format("%d",pDlg->m_InitialSpeed);
					pDlg->GetDlgItem(IDC_EDIT_InitialSpeed)->SetWindowTextA((LPCTSTR)strTmp);
					pDlg->OnBnClickedButtonSetinispeed();
					Sleep(100);
					pDlg->m_WorkingSpeed = 70;
					strTmp.Format("%d",pDlg->m_WorkingSpeed);
					pDlg->GetDlgItem(IDC_EDIT_WorkingSpeed)->SetWindowTextA((LPCTSTR)strTmp);
					pDlg->OnBnClickedButtonSetworkspeed();
					Sleep(100);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_CLOSE,0,0);    //���·�һ��ر�
				}
				else if (AutoCorretRegInfo.nFinalCkCode==5)    
				{		
					if (g_bCloseflag == true)
					{
						g_bCloseflag = false;
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //ֹͣ�����رչ���
					}
					
					//������
					strTemp="�رճ�ʱ��������֤��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,10);
					strTemp="������һ����֤δ��";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
			}
		}
		else if((WAIT_OBJECT_0 + 9) == nRet)//Event_OVEROPEN ����Զ�����ǰ�ſ�����
		{
			//Sleep(10000);
			Sleep(3000);
			pDlg->m_InitialSpeed = 500;
			strTmp.Format("%d",pDlg->m_InitialSpeed);
			pDlg->GetDlgItem(IDC_EDIT_InitialSpeed)->SetWindowTextA((LPCTSTR)strTmp);
			pDlg->OnBnClickedButtonSetinispeed();
			Sleep(1000);
			pDlg->m_WorkingSpeed = 1500;
			strTmp.Format("%d",pDlg->m_WorkingSpeed);
			pDlg->GetDlgItem(IDC_EDIT_WorkingSpeed)->SetWindowTextA((LPCTSTR)strTmp);
			pDlg->OnBnClickedButtonSetworkspeed();
			Sleep(1000);
			pDlg->m_slopDir.SetCurSel(1);
			pDlg->m_slopeSteps = 15000;
			strTmp.Format("%d",pDlg->m_slopeSteps);
			pDlg->GetDlgItem(IDC_EDIT_SlopSteps)->SetWindowTextA((LPCTSTR)strTmp);
			pDlg->OnBnClickedButtonMoveslop();
			SetEvent(Event_STOP);
		}
		else if((WAIT_OBJECT_0 + 10) == nRet)//Event_STOP ֹͣ��������
		{
			switch(AutoCorretRegInfo.nStepIndx)
			{
			case STEP_FINALCH:
				//���Ժͻ滭��ʼ��
				pDlg->m_IntervNum=5;
				pDlg->m_ChartNum=100;
				pDlg->m_MkCalNum=50;
				pDlg->m_prop1.m_ChartReal.GetAxis().GetBottom().SetMinMax(0,pDlg->m_ChartNum*300);
				pDlg->m_prop1.m_ChartReal.GetAxis().GetBottom().SetIncrement(5000);
				pDlg->m_prop1.m_ChartReal.GetAxis().GetLeft().SetAutomatic(TRUE);
				pDlg->m_prop1.m_ChartReal.Series(1).SetActive(TRUE);
				//pDlg->UpdateData(FALSE);
				//pDlg->OnBnClickedButtonDisconnet();
			case STEP_BEFORECHECKOPEN:

			case STEP_CHECKTB:

			case STEP_RESTART:

			case STEP_DOWNTB:

			case STEP_FITMK:

			case STEP_ACQMK:
				bTargetAqcTerm=FALSE;
			case STEP_MAXMK:
				BeginValue=0;
			case STEP_MINMK:
				g_bCountStart=0;
			case STEP_PARAM:
				pDlg->GetDlgItem(IDC_BUTTON_GetParam)->EnableWindow(TRUE);
				//pDlg->GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(TRUE);
			case STEP_START:
				bAutoCorrTerm=FALSE;
				pDlg->GetDlgItem(IDC_BUTTON_AutoCorrect)->SetWindowTextA("����һ������");
				if (AutoCorretRegInfo.nFinalCkCode==1)
				{
					strTmp="�Զ������������";
					AfxMessageBox(_T("�Զ���������ɹ�����:[���ͨ��]"),MB_OK);
					//pDlg->OnBnClickedButton2();
					//д�����ݿ�
					if(g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==4)   //711E��711A
					{
						CWinThread *pThread;
						pThread = ::AfxBeginThread(WriteData,lpParam);
					}
				}
				else
				{
					strTmp="�Զ�����������ֹ";
					AfxMessageBox(_T("�Զ�����������ֹ:[���ʧ��]"),MB_OK|MB_ICONSTOP);
				}
				if(m_pMyLog)	//д����־
					m_pMyLog->SetNotify(strTmp.GetBuffer(strTmp.GetLength()));
				m_pMyLog = NULL;
			}
			pDlg->InitAutoCorrectTask();
		}
	}
	return 0L;
}

void CLaseAutoCorrectDlg::OnBnClickedButton3()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	if (Com_Send(a_ucTemp12,6))
	{
		strTemp="����ָֹͣ��ɹ�";
	}
	else
	{
		strTemp="����ָֹͣ��ʧ��";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::OnBnClickedButtonDistconfirm()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	CString str="";	
	
	if (AutoCorretRegInfo.nStepIndx==STEP_PARAM && bAutoCorrTerm)
	{
		GetDlgItem(IDC_EDIT_RealDist)->GetWindowTextA(str);
		m_RealDist=_ttof(str);
		g_nReferUpEdge=(int)(m_RealDist*2*10/3);
		StopTimer(TIMERINPUTDIST);
		AutoCorretRegInfo.nParamChCode=1;
		SetEvent(Event_PARAM);
		strTemp.Format(" ���ۼ�ʱֵΪ %d",g_nReferUpEdge);
		strTemp = "��ʵ������ȷ��" + str + strTemp;
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	}	
}

void CLaseAutoCorrectDlg::OnBnClickedButtonSetinispeed()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	CString str="";
	UINT16 CaclCRC=0;

	GetDlgItem(IDC_EDIT_InitialSpeed)->GetWindowTextA(str);
	m_InitialSpeed=_ttoi(str);
	int speed=m_InitialSpeed; 

	a_ucTemp13[3]=((speed>>24)&0xFF);
	a_ucTemp13[4]=((speed>>16)&0xFF);
	a_ucTemp13[5]=((speed>>8)&0xFF);
	a_ucTemp13[6]=(speed&0xFF);

	CaclCRC=CRC712_Add(a_ucTemp13,7);

	a_ucTemp13[7]=(CaclCRC>>8)&0xFF;
	a_ucTemp13[8]=(CaclCRC)&0xFF;

	if (Com_Send(a_ucTemp13,9))
	{
		strTemp="���ó��ٶ�ָ��ɹ�";
	}
	else
	{
		strTemp="���ó��ٶ�ָ��ʧ��";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}


void CLaseAutoCorrectDlg::OnBnClickedButtonSetworkspeed()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	CString str="";
	UINT16 CaclCRC=0;

	GetDlgItem(IDC_EDIT_WorkingSpeed)->GetWindowTextA(str);
	m_WorkingSpeed=_ttoi(str);
	int speed=m_WorkingSpeed; 

	a_ucTemp14[3]=((speed>>24)&0xFF);
	a_ucTemp14[4]=((speed>>16)&0xFF);
	a_ucTemp14[5]=((speed>>8)&0xFF);
	a_ucTemp14[6]=(speed&0xFF);

	CaclCRC=CRC712_Add(a_ucTemp14,7);

	a_ucTemp14[7]=(CaclCRC>>8)&0xFF;
	a_ucTemp14[8]=(CaclCRC)&0xFF;

	if (Com_Send(a_ucTemp14,9))
	{
		strTemp="���������ٶ�ָ��ɹ�";
	}
	else
	{
		strTemp="���������ٶ�ָ��ʧ��";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}


void CLaseAutoCorrectDlg::OnBnClickedButtonGetstate()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CString strTemp="";
	if (Com_Send(a_ucTemp15,5))
	{
		strTemp="���ͻ�ȡ״ָ̬��ɹ�";
	}
	else
	{
		strTemp="���ͻ�ȡ״ָ̬��ʧ��";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,1,(LPARAM)(LPCTSTR)strTemp);
}

class ADOConn  
{
public:
    _ConnectionPtr m_pConnection;
	_RecordsetPtr m_pRecordset;
public:
	ADOConn();
	virtual ~ADOConn(){};
	bool OnInitADOConn();//��ʼ������
    bool ExecuteSQL(_bstr_t bstrSQL);
};
ADOConn::ADOConn()
{
	CoInitialize(NULL); //��ʼ���ز�����
	HRESULT hr=m_pConnection.CreateInstance(_uuidof(Connection));
	if(FAILED(hr))
	{
	}
}

bool ADOConn::OnInitADOConn()
{
	try
	{	//��COM�ӿ��У�������Ϊ�ַ����� һ��Ҫ��BSTR  
		_bstr_t con_bstr="Driver={sql server};server=192.168.0.103;uid=sa;pwd=wanjikeji_123;database=Laser_Production_Intelligence;";
		m_pConnection->Open(con_bstr,"","",adModeUnknown);// adModeUnknown ȱʡ����ǰ�����Ȩδ����
	}
	catch(_com_error &e) 
	{
		return false;
	}
	return true;
}
bool ADOConn::ExecuteSQL(_bstr_t bstrSQL)
{
	try
	{
		m_pRecordset=m_pConnection->Execute(bstrSQL, NULL, adCmdText);
	}
	catch(_com_error e)
	{
		//AfxMessageBox(_T("���ݿ�д��ʧ��"),MB_OK);
		return false;
	}
	return true;
}

//������
void CLaseAutoCorrectDlg::OnBnClickedButton2()
{
	CWinThread *pThread;
	pThread = ::AfxBeginThread(WriteData,this);

	// TODO: �ڴ���ӿؼ�֪ͨ����������
	/*OnOneKeyTestCorrect();
	OnOnekeyTestFinish();
	::SendMessageA(GetSafeHwnd(),WM_GL_CLOSE,0,0);
	::SendMessageA(GetSafeHwnd(),WM_GL_OPEN,0,0);
	::SendMessageA(GetSafeHwnd(),WM_GL_ALARM,0,0);*/

	//INT_PTR nRes = MessageBox(_T("�������������Ƿ�д�����ݿ⣿"),"������Ϣ��",MB_OKCANCEL|MB_ICONQUESTION);

	//if(nRes == IDCANCEL)
	//{
	//	return;
	//}
	////m_MacAddr="F8B5689000FA";
	////m_SavePath="F:\\0727\\��\\Code\\��v2��LaseAutoCorrect-20171108-���� - �ߵ���ֵ�л� - ����711A - �������ݿ�\\Debug\\F8B5689000FA";
	//
	//CString MuDipath="/\\������-����\\ProductionProcess\\";
	//CString chn;
	//CString chn2;
	//if (g_ChannelSelect==0)  //����ֵ
	//{
	//	chn="H";
	//	chn2="��";
	//}
	//else
	//{
	//	chn="L";
	//	chn2="��";
	//}
	//ADOConn con;  
	//CString strs="select count(*) from DEV_711E_MKXZ where ID like'%";
	//strs.Append(this->m_MacAddr+"%'");
	//strs.Append(" and YSSJ_TXT like '%"+this->m_MacAddr+"_"+chn2+"%'");
	//_bstr_t strSql=strs;
	//con.OnInitADOConn();  
	//con.ExecuteSQL(strSql); 
	//_variant_t vIndex = (long)0; 
	//_variant_t vCount = con.m_pRecordset->GetCollect(vIndex);
	//if(vCount.intVal<=0)//���ݿ�û������
	//{
	//	CString tmp;
	//	CString str="insert into DEV_711E_MKXZ values(";
	//	str.Append("'"+CTime::GetCurrentTime().Format("%Y%m%d%H%M%S")+this->m_MacAddr+"',");
	//	str.Append("'�ɹ�',");
	//	if(!PathIsDirectory(MuDipath+this->m_MacAddr))
	//	{
	//		BOOL bRet = CreateDirectory(MuDipath+this->m_MacAddr, NULL);//�����ļ���
	//		MuDipath=MuDipath+this->m_MacAddr+"\\";
	//	}
	//	else
	//	{
	//		MuDipath=MuDipath+this->m_MacAddr+"\\";
	//	}
	//	if(g_nameadd)
	//	{
	//		str.Append("'"+this->m_MacAddr+"_"+chn2+"��ֵԭʼ����"+"("+g_cover+")."+"txt_"+chn+"',");
	//		str.Append("'"+this->m_MacAddr+"_"+chn2+"��ֵ�������"+"("+g_cover+")."+"txt_"+chn+"',");
	//		str.Append("'"+this->m_MacAddr+"_���Լ�¼"+"("+g_cover+")."+"txt_"+chn+"',");//���Լ�¼
	//		str.Append("'"+this->m_MacAddr+"���ͼ_H("+g_cover+").jpg"+"',");
	//		str.Append("'"+this->m_MacAddr+"�ϸ�_H("+g_cover+").jpg"+"',");

	//		//CString path;

	//		//path = "/\\LZB\\ProductionProcess\\TDC_LOCK2.rbf";
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"��ֵԭʼ����"+"("+g_cover+")."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"��ֵԭʼ����"+"("+g_cover+")."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"��ֵ�������"+"("+g_cover+")."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"��ֵ�������"+"("+g_cover+")."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"��ֵ�������"+"("+g_cover+")."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"��ֵ�������"+"("+g_cover+")."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"���ͼ_"+chn+"("+g_cover+").jpg", MuDipath+this->m_MacAddr+"���ͼ_"+chn+"("+g_cover+").jpg", TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"�ϸ�_"+chn+"("+g_cover+").jpg", MuDipath+this->m_MacAddr+"�ϸ�_"+chn+"("+g_cover+").jpg", TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_���Լ�¼"+"("+g_cover+")."+"txt_"+chn, MuDipath+this->m_MacAddr+"_���Լ�¼"+"("+g_cover+")."+"txt_"+chn, TRUE);
	//	}
	//	else
	//	{
	//		str.Append("'"+this->m_MacAddr+"_"+chn2+"��ֵԭʼ����."+"txt_"+chn+"',");
	//		str.Append("'"+this->m_MacAddr+"_"+chn2+"��ֵ�������."+"txt_"+chn+"',");
	//		str.Append("'"+this->m_MacAddr+"_���Լ�¼."+"txt_"+chn+"',");//���Լ�¼
	//		str.Append("'"+this->m_MacAddr+"���ͼ_H.jpg"+"',");
	//		str.Append("'"+this->m_MacAddr+"�ϸ�_H.jpg"+"',");

	//		//CString path;
	//		//path = "/\\LZB\\ProductionProcess\\TDC_LOCK2.rbf";
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"��ֵԭʼ����."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"��ֵԭʼ����."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"��ֵ�������."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"��ֵ�������."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"��ֵ�������."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"��ֵ�������."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"���ͼ_"+chn+".jpg", MuDipath+this->m_MacAddr+"���ͼ_"+chn+".jpg", TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"�ϸ�_"+chn+".jpg", MuDipath+this->m_MacAddr+"�ϸ�_"+chn+".jpg", TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_���Լ�¼."+"txt_"+chn, MuDipath+this->m_MacAddr+"_���Լ�¼."+"txt_"+chn, TRUE);
	//		//CopyFile("C:\\Users\\wanji\\Desktop\\TDC_LOCK.rbf", "/\\LZB\\ProductionProcess\\TDC_LOCK2.rbf", TRUE);//����������
	//	}
	//	tmp.Format("%d",g_CheckMKForm.ValueJudgeCnt);
	//	str.Append("'"+tmp+"',");//�������
	//	tmp.Format("%d",g_nRevcNumcCheck);
	//	str.Append("'"+tmp+"',");//���֡��
	//	tmp.Format("%d",g_nRevcNumcCheck);
	//	str.Append("'"+tmp+"',");//���֡��
	//	tmp.Format("%d",g_nRevcNumcCheck1);
	//	str.Append("'"+tmp+"(");//����֡��
	//	tmp.Format("%d",g_nRevcNumcCheck2);
	//	str.Append(tmp+")(");
	//	tmp.Format("%d",g_nRevcNumcCheck3);
	//	str.Append(tmp+")'");
	//	tmp.Format("%d",g_CheckMKForm.ChMkOffset);//��ֵ֮��
	//	str.Append("'"+tmp+"',");
	//	tmp.Format("%d",g_CheckMKForm.SingleErrorMAX);//��ֵ����������ֵ
	//	str.Append("'"+tmp+"',");
	//	tmp.Format("%d",g_CheckMKForm.SingleErrorMIN);//��ֵ���������Сֵ
	//	str.Append("'"+tmp+"',");
	//	str.Append("' ',");//������
	//	str.Append("'"+CTime::GetCurrentTime().Format("%Y-%m-%d %H:%M:%S")+"')");//����ʱ��
	//	_bstr_t strSql2=str;
	//	con.ExecuteSQL(strSql2); 
	//}
	//else
	//{
	//	return;
	//	//"Select id,COUNT(*) as gg   from DEV_711E_MKXZ where ID like'%F8B5689000FA%' group by ID"
	//	CString strselect="select * from DEV_711E_MKXZ where ID like'%";
	//	strselect.Append(this->m_MacAddr+"%'");
	//	strselect.Append(" and YSSJ_TXT like '%"+this->m_MacAddr+"_"+chn2+"%'");
	//	_bstr_t strSqlselect=strselect;
	//	con.ExecuteSQL(strSqlselect);
	//	CString res;
	//	while(!con.m_pRecordset->adoEOF)  
	//	{    
	//		//con.m_pRecordset->GetFields()->
	//	   res = con.m_pRecordset->GetFields()->GetItem("ID")->Value;
	//	   CString history= "insert into DEV_CS_PARAMS_HISTORY values(";
	//	   history.Append("'" + res + "',");
	//	   history.Append("'DEV_711E_STEP3_CS',");
	//	   history.Append("'ID',");
	//	   history.Append("'" + res + "',");
	//	   history.Append("' ',");
	//	   history.Append("'"+CTime::GetCurrentTime().Format("%Y-%m-%d %H:%M:%S")+"')");//����ʱ��
	//	   _bstr_t strSqlinsert=history;
	//	   con.ExecuteSQL(strSqlinsert);

	//	   CString Value = con.m_pRecordset->GetFields()->GetItem("ZKBBH")->Value;
	//	   history= "insert into DEV_CS_PARAMS_HISTORY values(";
	//	   history.Append("'" + res + "',");
	//	   history.Append("'DEV_711E_STEP3_CS',");
	//	   history.Append("'ZKBBH',");
	//	   history.Append("'" + Value + "',");
	//	   history.Append("' ',");
	//	   history.Append("'"+CTime::GetCurrentTime().Format("%Y-%m-%d %H:%M:%S")+"')");//����ʱ��
	//	   con.ExecuteSQL(strSqlinsert);
	//	   con.m_pRecordset->MoveNext();
	//	}   

	//}

}
//д�����ݿ�
UINT WriteData(LPVOID lpParam)
{
	CLaseAutoCorrectDlg *pDlg = (CLaseAutoCorrectDlg*)lpParam;
	CString MuDipath="/\\������-����\\ProductionProcess\\";
	CString chn;
	CString chn2;
	if (g_ChannelSelect==0)  //����ֵ
	{
		chn="H";
		chn2="��";
	}
	else
	{
		chn="L";
		chn2="��";
	}
	ADOConn con;  
	bool conn=con.OnInitADOConn();  
	if(!conn)
	{
		CString strconn="���ݿ�����ʧ��";
		::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strconn);
		return 0;
	}

	//CString PROCESSstr="select Dpt_ID from DEV_PROCESS_TEMPLATE where Dpt_DevType='4' and Dpt_Name='��������'";
	CString PROCESSstr="select Dpt_ID from DEV_PROCESS_TEMPLATE where Dpt_DevType='";
	CString type;
	if(g_SysDevParam.LaserType==4)
	{
		type="4";
	}
	else if(g_SysDevParam.LaserType==3)
	{
		type="3";
	}
	PROCESSstr.Append(type);
	PROCESSstr.Append("' and Dpt_Name='��������'");
	_bstr_t PROCESSSql=PROCESSstr;
	bool con1 =con.ExecuteSQL(PROCESSSql); 
	if(!con1)
	{
		return 0;
	}
	 _variant_t vsname;
	CString strguid = (char *)(_bstr_t)con.m_pRecordset->GetCollect("Dpt_ID");  

	//CString PROCESSORDERstr="insert into DEV_PROCESS_ORDER values('";
	char lpPath[MAX_PATH];
	GetCurrentDirectory(MAX_PATH,lpPath);
	//lpPath.Left(lpPath.ReverseFind('\\')+1); 
	(_tcsrchr(lpPath,_T('\\')))[1] = 0;//��ȡ�ϼ�Ŀ¼
	strcat(lpPath,"\\Setup.ini");
	fstream _file;
	_file.open(lpPath,ios::in);
	LPTSTR l_getId = new char[50];
	LPTSTR l_getOrder = new char[50];
	LPTSTR l_getProcessId = new char[50];
	CString ProcessId;
	if (!_file)
	{
		::AfxMessageBox(_T("�����ļ���ʧ�ܡ�"));
	}
	else
	{
		memset(l_getId,0,sizeof(l_getId));
		GetPrivateProfileString("CONFIG","ID","",l_getId,50,lpPath);
		GetPrivateProfileString("CONFIG","Order","",l_getOrder,50,lpPath);
		GetPrivateProfileString("CONFIG","ProcessId","",l_getProcessId,50,lpPath);
		
		ProcessId.Format( _T("%s"), l_getProcessId );
		CString PROCESSORDER="select count(*) from DEV_PROCESS_ORDER where Dpt_ID='"+ProcessId+"'";
		_bstr_t processorderSql=PROCESSORDER;
		con.ExecuteSQL(processorderSql);
		_variant_t vIndex = (long)0; 
		_variant_t vCount = con.m_pRecordset->GetCollect(vIndex);
		int processcount=vCount.intVal;
		if(processcount<=0)//������û�оͲ���
		{
			CString PROCESSORDERstr="insert into DEV_PROCESS_ORDER values(";

			PROCESSORDERstr.Append("'"+ProcessId+"-XZ',");
			PROCESSORDERstr.Append("'-XZ',");
			PROCESSORDERstr.Append("'0',");
			CString Order;
			Order.Format( _T("%s"), l_getOrder);
			PROCESSORDERstr.Append("'"+Order+"',");
			PROCESSORDERstr.Append("'��������',");
			PROCESSORDERstr.Append("'1',");
			PROCESSORDERstr.Append("'"+strguid+"',");
			PROCESSORDERstr.Append("'��ʱ�䡿"+CTime::GetCurrentTime().Format("%Y-%m-%d %H:%M:%S")+"',");
			CString Id;
			Id.Format( _T("%s"), l_getId);
			PROCESSORDERstr.Append("'"+Id+"',");
			PROCESSORDERstr.Append("'"+CTime::GetCurrentTime().Format("%Y-%m-%d %H:%M:%S")+"')");
			_bstr_t PROCESSORDERSql=PROCESSORDERstr;
			con.ExecuteSQL(PROCESSORDERSql); 
		}
	}

	CString PARAMSstr="select  Dpt_ID,Dpt_Sort,Dpt_Name from DEV_PARAMS_TEMPLATE where Dpt_ProcessId='";
	PARAMSstr.Append(strguid);
	PARAMSstr.Append("'");
	_bstr_t PARAMSSql=PARAMSstr;
	bool con2=con.ExecuteSQL(PARAMSSql);
	if(!con2)
	{
		return 0;
	}
	CStringArray m_Sql;
	m_Sql.SetSize(3);
	int j=0;
	CString ShiJian=CTime::GetCurrentTime().Format("%Y%m%d%H%M%S");
	//for(int i=0;i<vCount.intVal;i++)
	for(int i=0;i<6;i++)
	{
		//_variant_t vsIDParm,vsSortParm,vsNameParm;
		CString vsIDParm,vsSortParm,vsNameParm;
		vsIDParm = (char *)(_bstr_t)con.m_pRecordset->GetCollect("Dpt_ID");
		vsSortParm=(char *)(_bstr_t)con.m_pRecordset->GetCollect("Dpt_Sort");
		vsNameParm=(char *)(_bstr_t)con.m_pRecordset->GetCollect("Dpt_Name");
		if(g_ChannelSelect==0)
		{
			if(vsSortParm=="3"||vsSortParm=="4"||vsSortParm=="5")
			{
				con.m_pRecordset->MoveNext();
				continue;
			}
		}
		else if(g_ChannelSelect==1)
		{
			if(vsSortParm=="0"||vsSortParm=="1"||vsSortParm=="2")
			{
				con.m_pRecordset->MoveNext();
				continue;
			}
		}
		//CString parms="select * from DEV_PARAMS_ORDER where Dpt_ProcessId='" +vsIDParm+"' and Dpt_Sort='"+vsSortParm+"'";
		
		char buffer[64] = { 0 };  
		GUID guid;  
		if (CoCreateGuid(&guid))  
		{  

		}  
		/*_snprintf(buffer, sizeof(buffer),  
			"%08X-%04X-%04x-%02X%02X-%02X%02X%02X%02X%02X%02X",  
			guid.Data1, guid.Data2, guid.Data3,  
			guid.Data4[0], guid.Data4[1], guid.Data4[2],  
			guid.Data4[3], guid.Data4[4], guid.Data4[5],  
			guid.Data4[6], guid.Data4[7]); */
		CString guidstr;
		guidstr.Format("%08x-%04x-%04x-%02x%02x-%02x%02x%02x%02x%02x%02x",guid.Data1, guid.Data2, guid.Data3,  
			guid.Data4[0], guid.Data4[1], guid.Data4[2],  
			guid.Data4[3], guid.Data4[4], guid.Data4[5],  
			guid.Data4[6], guid.Data4[7]);
		CString str="insert into DEV_PARAMS_ORDER values(";
		str.Append("'"+guidstr+"',");
		CString m_ProcessId;
		m_ProcessId.Format( _T("%s"), l_getProcessId );
		str.Append("'"+m_ProcessId+"-XZ"+"',");//����ID
		str.Append("'"+vsSortParm+"',");//���
		str.Append("'"+vsNameParm+"',");//����
		int index=vsNameParm.Find("���Լ�¼");
		if(index>=0)//�����Լ�¼֮��
		{
			if(g_ChannelSelect==0)
			{
				str.Append("'"+ShiJian+"_"+pDlg->m_MacAddr+"_"+"���Լ�¼"+".txt_H"+"',");//ֵ
			}
			else
			{
				str.Append("'"+ShiJian+"_"+pDlg->m_MacAddr+"_"+"���Լ�¼"+".txt_L"+"',");//ֵ
			}
		}
		else
		{
			if(g_ChannelSelect==0)
			{
				str.Append("'"+ShiJian+"_"+pDlg->m_MacAddr+"_"+vsNameParm+".txt_H"+"',");//ֵ
			}
			else
			{
			  str.Append("'"+ShiJian+"_"+pDlg->m_MacAddr+"_"+vsNameParm+".txt_L"+"',");//ֵ
			}
		}
		str.Append("'file',");//����
		str.Append("'0',");//dpt_Standard
		str.Append("'0',");//positigve
		str.Append("'0',");//nagative
		str.Append("'1',");
		str.Append("'0',");
		str.Append("' ',");
		str.Append("'"+vsIDParm+"',");//Dpt_ID
		str.Append("' ')");
		m_Sql.SetAt(j,str);
		j++;
		con.m_pRecordset->MoveNext();
	}
	for(int i=0;i<3;i++)
	{
		CString sql=m_Sql[i];
		_bstr_t PARAMSOrderSql=sql;
		con.ExecuteSQL(PARAMSOrderSql);
	}
	if(g_nameadd)
	{

		//path = "/\\LZB\\ProductionProcess\\TDC_LOCK2.rbf";
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_"+chn2+"��ֵԭʼ����"+"("+g_cover+")."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_"+chn2+"��ֵԭʼ����"+"("+g_cover+")."+"txt_"+chn, TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_"+chn2+"��ֵ�������"+"("+g_cover+")."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_"+chn2+"��ֵ�������"+"("+g_cover+")."+"txt_"+chn, TRUE);
		//CopyFile(m_SavePath+ _T("\\")+ShiJian+"_"+pDlg->m_MacAddr+"_"+chn2+"��ֵ�������"+"("+g_cover+")."+"txt_"+chn, MuDipath+pDlg->m_MacAddr+"_"+chn2+"��ֵ�������"+"("+g_cover+")."+"txt_"+chn, TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"���ͼ_"+chn+"("+g_cover+").jpg", MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"���ͼ_"+chn+"("+g_cover+").jpg", TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"�ϸ�_"+chn+"("+g_cover+").jpg", MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"�ϸ�_"+chn+"("+g_cover+").jpg", TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_���Լ�¼"+"("+g_cover+")."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_���Լ�¼"+"("+g_cover+")."+"txt_"+chn, TRUE);
	}
	else
	{
		
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_"+chn2+"��ֵԭʼ����."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_"+chn2+"��ֵԭʼ����."+"txt_"+chn, TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_"+chn2+"��ֵ�������."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_"+chn2+"��ֵ�������."+"txt_"+chn, TRUE);
		//CopyFile(m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_"+chn2+"��ֵ�������."+"txt_"+chn, MuDipath+pDlg->m_MacAddr+"_"+chn2+"��ֵ�������."+"txt_"+chn, TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"���ͼ_"+chn+".jpg", MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"���ͼ_"+chn+".jpg", TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"�ϸ�_"+chn+".jpg", MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"�ϸ�_"+chn+".jpg", TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_���Լ�¼."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_���Լ�¼."+"txt_"+chn, TRUE);

	}
	//�ύ
	CString update="update DEV_PRODUCTION_TO_PROCESS SET Dptp_ProcessId='";
	update.Append(ProcessId+"-XZ' Where  Dptp_MAC=");
	update.Append("'"+pDlg->m_MacAddr+"'and Dptp_ProcessPid=");
	update.Append("'"+strguid+"'");

	//update DEV_PRODUCTION_TO_PROCESS SET Dptp_ProcessId='1111-XZ' Where  Dptp_MAC='F8B568900435' and Dptp_ProcessPid='cc896912-109c-4b96-ad56-51ea6acb8ab8'
	_bstr_t updateSql=update;
	con.ExecuteSQL(updateSql);
	return 0;
}

void CLaseAutoCorrectDlg::TraverseDir(CString& dir, std::vector<CString>& vec)
{
	CFileFind ff;
	//CString dir="F:\\��v2��LaseAutoCorrect-20171108-���� - �ߵ���ֵ�л�\\Debug\\�Զ��������\\F8B5689000FF";
	if (dir.Right(1) != "\\")
		dir += "\\";
	dir += "*.*";
	BOOL ret = ff.FindFile(dir);

	while (ret)
	{
		ret = ff.FindNextFile();
		//if (ret != 0)
		//{
		if (ff.IsDirectory() && !ff.IsDots())
		{
			CString path = ff.GetFilePath();
			TraverseDir(path, vec);
		}
		else if (!ff.IsDirectory() && !ff.IsDots())
		{
			CString name = ff.GetFileName();
			CString path = ff.GetFilePath();
			vec.push_back(name);
		}
		//}
	}
}




void CLaseAutoCorrectDlg::OnEnChangeEditAveerror()
{
	// TODO:  ����ÿؼ��� RICHEDIT �ؼ���������
	// ���ʹ�֪ͨ��������д CDialogEx::OnInitDialog()
	// ���������� CRichEditCtrl().SetEventMask()��
	// ͬʱ�� ENM_CHANGE ��־�������㵽�����С�
	// TODO:  �ڴ���ӿؼ�֪ͨ����������
	/*UpdateData(true);*/
	/*okAvg = m_AveError;*/
}


void CLaseAutoCorrectDlg::OnEnChangeEditAllsingleerror()
{
	// TODO:  ����ÿؼ��� RICHEDIT �ؼ���������
	// ���ʹ�֪ͨ��������д CDialogEx::OnInitDialog()
	// ���������� CRichEditCtrl().SetEventMask()��
	// ͬʱ�� ENM_CHANGE ��־�������㵽�����С�
	/*UpdateData(true);*/
	// TODO:  �ڴ���ӿؼ�֪ͨ����������
}


void CLaseAutoCorrectDlg::OnEnChangeEditCheckrangeminvalue()
{
	// TODO:  ����ÿؼ��� RICHEDIT �ؼ���������
	// ���ʹ�֪ͨ��������д CDialogEx::OnInitDialog()
	// ���������� CRichEditCtrl().SetEventMask()��
	// ͬʱ�� ENM_CHANGE ��־�������㵽�����С�
	/*UpdateData(true);*/
	// TODO:  �ڴ���ӿؼ�֪ͨ����������
}
