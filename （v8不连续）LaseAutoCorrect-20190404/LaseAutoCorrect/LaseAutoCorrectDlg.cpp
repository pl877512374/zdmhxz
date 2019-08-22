
// LaseAutoCorrectDlg.cpp : 实现文件
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
	int ChMkMin;//检测记录的脉宽最大值
	int ChMkMax;//检测记录的脉宽最小值
	int ChMkOffset;//检测记录的脉宽偏差范围
	int ReferenceValue;//参考均值
	int ValueJudgeCnt;//用于计数毛刺点，即超过均值正负300的点
	int ValueJudge;//用验证中点数是否合格的判断
	int AveValueJudgeCnt;//用于计数均值出错的帧数
	int AveValueJudge;//用于验证中均值是否合格的判断
	int ChMkRegionMin;//验证区域脉宽值的最小值
	int ChMkRegionMax;//验证区域脉宽值的最大值
	int ChMkGetReference;//验证参考标准值获取的最小脉宽，即大于此值的脉宽全用来平均得到参考值
	int AveValueError;//平均值验证方式中的最大误差的标准值
	int SingleValueError;//单点验证方式中的最大误差的标准值
	int SingleErrorMAX; //单个点验证结果中差值的最大值
	int SingleErrorMIN; //单个点验证结果中差值的最小值
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
	UINT16	u16FlashProgramFlag;		//Flash数据有效标志位，当该位为FlashDataEnable时，数据有效
	UINT16	u16FlashDataLength;		//Flash中有效数据（半字）的长度，包含Flash数据有效标志位

	UINT16 	u16NCdata[10];			//预留20个字节；

	UINT16	u16Div_AngleNum;		//定义一周被等分为多少等份，720，即0.5度步进
	//电机为0.5度步进，每圈产生720个脉冲。36K
	UINT16	u16Dot_PerAngle;			//定义每多少步扫描一个点.激光驱动频率为12K时，该数为3,36K时为1

	UINT16  u16ZeroStart;				//零点起始位置
	UINT16  u16ZeroEnd;				//零点终止位置
	UINT16	u16ZeroBufLengrh;		//零点有效数据长度

	UINT16	u16ZeroLength;			//求零点时取多少个数做平均
	UINT16	u16ZeroTimerMin;			//寻找零点最小值的下限，低于该值认为不是最小值点。单位pS

	UINT16	u16ScanStart;				//扫描起始位置
	UINT16	u16ScanEnd;				//扫描终止位置
	UINT16	u16ScanBufLength;		//扫描的点数；

	UINT16	u16APDHvValue;		    //APD高压值；
	UINT16	u16APDTemperValue;		//APD温度值；
	UINT16	u16APDHV_OP_Ratio;		//APD高压系数；（/1000）

	UINT16 	u16NCdata1[7];			//预留14个字节；

	//网口传输相关
	UINT16 	u16My_Phy_Addr[6];		//物理地址
	UINT16 	u16My_Sub_Mask[4]; 		//子网掩码
	UINT16 	u16My_IP_Addr[4];		//设备IP地址
	UINT16	u16My_Gateway_IP[4];		//网关地址

	UINT16	u16My_port;				//设备所使用的端口号

	UINT16 	u16Ser_Ip[4];				//服务器IP地址
	UINT16	u16Ser_port;				//服务器所使用的端口号
} SYSParameter;
SYSParameter LaserParam;

typedef struct tagSYSParameter711
{
//系统信息
	UINT16 m_u16EquipNo[2];
	UINT16 m_u16SysMode_SingleOrScanf;
	// APD参数 
	UINT16 m_u16APDVSet;								//击穿测试电压设置to fpga
	UINT16 m_u16APDTemperValue;					//APD温度值
	UINT16 m_u16APDHV_OP_Ratio;					//APD高压系数(/1000)	
	
	UINT16 m_u16ZeroDisc;
	UINT16 m_u16OverallDisDif;//整体偏移距离	
	UINT16 m_u16OverallDisDif_L;

	UINT16 m_u16Channel_Sel;	
}SYSParameter711;
SYSParameter711 LaserParam711;

typedef struct tagSYSParameter712
{
	// APD参数 
	UINT16 m_u16APDVSet;								//击穿测试电压设置to fpga
	UINT16 m_u16APDTemperValue;					//APD温度值
	UINT16 m_u16APDHV_OP_Ratio;					//APD高压系数(/1000)	
}SYSParameter712;
SYSParameter712 LaserParam712;

typedef struct tagAutoCorretInfo
{
	UINT8 nStepIndx;//修正进行步骤索引
	UINT8 nTaskCode;//00 未开启 ;1 开启成功; 2 自动修正结束
	UINT8 nParamChCode;//01参数检查通过  02 参数检查失败 03参数检查超时  
	UINT8 nOrigMkChCode;//01初始脉宽检查通过 02初始脉宽检查未通过  03 初始脉宽检查超时 04 光阑张开完成 05 光阑张开失败 06 光阑张开超时
	UINT8 nAqcMkCode;//01采集脉宽原始数据完成 02采集脉宽原始时需要调整光阑;03 采集原始脉宽数据时调整光阑完成 04 采集原始数据时光阑调整到位可直接统计计算 05 调整光阑超时 
	UINT8 nFitMkCode;//01拟合脉宽成功 02拟合脉宽失败
	UINT8 nDownTbCode;//01修正表下载成功 02修正表下载失败 03修正表下载超时
	UINT8 nCheckTbCode;//01检查修正表参数正确性通过  02检查修正表参数正确性未通过 03 需重新下载修正表
	UINT8 nChDownTbCnt;//检查修正表失败时 重新下载修正表的次数 如果三次下载验证均不过 则终止
	UINT8 nFinalCkCode;//01最终一键验证通过 02最终一键验证未通过 03 下位机准备妥当可以测试了  04 减速设置后继续关闭   05 关闭光阑超时
	UINT8 nNetReSendCnt;//网络重发次数
	UINT8 nComReSendCnt;//串口重发次数
	UINT8 nWaitInputDistCnt; //等待输入距离信息次数
}AutoCorretInfo;

AutoCorretInfo AutoCorretRegInfo;
int BeginValue=0;  //记录获取初始的脉宽值

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
//串口发送指令
unsigned char a_ucTemp1[13] = {0xFF,0xAA,0x00,0x0A,0x09,0x01,0xAA,0x00,0x00,0x00,0x93,0x20,0xEE};              //张开光阑
unsigned char a_ucTemp2[13] = {0xFF,0xAA,0x00,0x0A,0x09,0x02,0xAA,0x00,0x00,0x00,0x7D,0xF2,0xEE};              //闭合光阑
unsigned char a_ucTemp3[15] = {0xFF,0xAA,0x00,0x0C,0x09,0x03,0xAA,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xEE};    //微调光阑
unsigned char a_ucTemp4[12] = {0xFF,0xAA,0x00,0x09,0x09,0x04,0xAA,0x00,0x00,0x50,0x32,0xEE};                   //一键验证
unsigned char a_ucTemp5[12] = {0xFF,0xAA,0x00,0x09,0x09,0x05,0xAA,0x00,0x00,0x26,0x86,0xEE};                   //验证结束
unsigned char a_ucTemp6[13] = {0xFF,0xAA,0x00,0x0A,0x09,0x06,0xAA,0x00,0x00,0x00,0xF4,0xF4,0xEE};              //光阑快速关闭
unsigned char a_ucTemp7[13] = {0xFF,0xAA,0x00,0x0A,0x09,0x07,0xAA,0x00,0x00,0x00,0x5E,0xA5,0xEE};              //光阑快速张开
unsigned char a_ucTemp8[12] = {0xFF,0xAA,0x00,0x09,0x09,0x08,0xAA,0x00,0x00,0x1F,0x00,0xEE};              //报警指令
//712使用
unsigned char a_ucTemp9[9] = {0x01,0x27,0x04,0x00,0x00,0x00,0x00,0x00,0x00};   //微调光阑
unsigned char a_ucTemp10[6] = {0x01,0x28,0x01,0x10,0x00,0x3A};   //连续张开运动
unsigned char a_ucTemp11[6] = {0x01,0x28,0x01,0x20,0x00,0x4A};   //连续闭合运动
unsigned char a_ucTemp12[6] = {0x01,0x29,0x01,0x20,0x00,0x4B};   //急停止运动
unsigned char a_ucTemp13[9] = {0x01,0x23,0x04,0x00,0x00,0x00,0x00,0x00,0x00};   //设置初速度
unsigned char a_ucTemp14[9] = {0x01,0x24,0x04,0x00,0x00,0x00,0x00,0x00,0x00};   //设置运行速度
unsigned char a_ucTemp15[5] = {0x01,0x30,0x00,0x00,0x31};   //查询运行状态

// MFC临界区类对象
CCriticalSection g_clsCriticalSection;
CRITICAL_SECTION g_netcs;

//定义原始数据缓存数组
#define JGMAX_CNT 200
#define JGMAX_LEN 20000
char g_nJGdata[3][JGMAX_CNT][JGMAX_LEN]={0};
int g_nJGRecCnt=0;
int g_nJGProCnt=0;
int g_nMatch=0;

//定义Net解析用的全局缓存
char g_nNetParseBuff[10000]={0};
int  g_nNetParseBuffLen=0;
BYTE g_nComParseBuff[200]={0};
int g_nComParseBuffLen=0;
char g_nNetSendBuff[200]={0};
int g_nNetSendBuffLen=0;
bool b711AFirstClose=true;
//711用
int g_ChannelSelect = 0; //通道选择，分为高阈值和低阈值
bool g_CheckAPD=false;
int jg1=0;
int j2=0;
int j3=0;
char ShaoXie_zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x0D,0x00,0x00};
int shaoxie=34;

//用于判断脉宽最大值是否满足要求
int  MK_ORG0=190;
int  MK_ORG1=300;

//712用
//int g_nAPDStateTemp=-100;
int g_nReferUpEdge = 0;  //根据距离计算的参考计时值
bool g_bMoveFlag = false; //光阑是否运动标志位
bool g_bOpenflag = false;  //张开判断是否到最大
bool g_bCloseflag = false;  //关闭判断是否到最小
int g_nComComandStat = 0;   //串口发送的指令状态，0:无状态；1:张开状态；2:停止状态; 3:关闭状态
int g_nContinuousMoveState = 0;   //连续运动标志位，响应停止指令之前的状态，  0:无状态； 1：张开状态； 2：关闭状态
bool g_bCloseSpeedChange = false;    //判断是否需要改变关闭速度  ture：表示等待改变速度

//程序中线程
UINT WriteData(LPVOID lpParam);
UINT ConnectThreadProc(LPVOID lpParam);
UINT DisConThreadProc(LPVOID lpParam);
void OnNetRecv(LPVOID lpParam, char *pDataBuf, int nDataBufSize);
//UINT ComParseThread(LPVOID lpParam);
BOOL bComParseTerm=FALSE;

UINT AutoCheckThreadProc(LPVOID lpParam);  //测试用
UINT AutoCorrThreadProc(LPVOID lpParam);
BOOL bAutoCorrTerm=FALSE;
UINT TargetDataAqcThread(LPVOID lpParam);
BOOL bTargetAqcTerm=FALSE;

UINT AqcOrgThreadProc(LPVOID lpParam);
BOOL bAqcOrgTerm=FALSE;


int g_nReadbuf[20000] = {0};  //烧写缓存
int g_nReadbuflen = 0; //实际长度
int g_nCurMKValue[800] = {0};//存储当前在处理包的所有脉宽值

void OnComRecData(LPVOID lpParam, BYTE *pDataBuf, int nDataBufSize);
BOOL Com_Send(BYTE *out_buff,int len);

UINT DownLoadCorrectTableThread(LPVOID lpParam);
int SetPacket(char* sendpacket,int* datapacket,int len,int num);  //源数据的长度 包号
int SetPacket711(char index,char* sendpacket,int* datapacket,int len,int num);  //源数据的长度 包号
int SetPacket712(char* sendpacket,int* datapacket,int len,int num);  //源数据的长度 包号
short bccW_packs=0;//711下载完成校验
int SetPacket710A(char* sendpacket,int* datapacket,int len,int num);  //源数据的长度 包号
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

int Recv710AMKDataCnt;//记录接收710A脉宽数据的次数，3次没接到需要重发获取指令
int Recv711MKDataCnt;//记录接收711脉宽数据的次数，3次没接到需要重发获取指令

int abcd=0;

//定义画图相关的变量
int g_nRevcNum1 = 0 ;  //收到的1号包总数 
int g_nRevcNum2 = 0 ;  //收到的2号包总数
int g_nRevcNum3 = 0 ;  //收到的3号包总数
long g_nRevcNumcCheck = 0;
long g_nRevcNumcCheck1 = 0;
long g_nRevcNumcCheck2 = 0;
long g_nRevcNumcCheck3=0;//添加byYSS
int avgdata(0),avgdata1(0),avgdata2(0);  

char g_recv1[2000]={0};
int  g_recv1len=0;
char g_recv2[2000]={0};
int  g_recv2len=0;
char g_recv3[2000]={0};
int  g_recv3len=0;

int g_bCountStart=0;//统计计算采样点的脉宽值标识位

int g_nCacuNum1=0;
int g_nCacuNum2=0;
long  g_nAvgStop1 = 0L; //每n包的上升沿平均值
long g_nAvgStop2 = 0; //每n包的下降沿平均值
long g_nSumStop1 = 0; //每n包的上升沿的和
long g_nSumStop2 = 0; //每n包的下降沿的和
int g_nReviseNum = 0;  //每次修正计算次数
int g_nFirstData = 0;//第一包的上升沿数据
int g_nInterpNum = 0;  //用来拟合的数组大小
CArray<int,int> g_array1; //保存计算脉宽
CArray<int,int> g_array2; //保存计算基准差值
int g_datax[arrayXYsize] = {0};//读取拟合结果存放数组
int g_datay[arrayXYsize] = {0};
int g_nXY[100][2] = {0};  //临时存放数组
int g_nXYlen = 0;     //保存的长度


CEvent Event_START;//开始自动修正
CEvent Event_PARAM;//参数检查
CEvent Even_ORIGMK;//初始脉宽检查;
CEvent Event_AQCMK;//脉宽采集
CEvent Event_ALIVE;//保活防止超时
CEvent Event_FITMK;//脉宽拟合差值
CEvent Event_DOWNTB;//下载修正表
CEvent Event_CHECKTB;//检查修正表
CEvent Event_FINALCK;//最终检查
CEvent Event_OVEROPEN; //完成自动修正前张开光阑
CEvent Event_STOP; //停止自动修正
HANDLE Event_List[11] = {Event_START,Event_PARAM,Even_ORIGMK,Event_AQCMK,Event_ALIVE,Event_FITMK,Event_DOWNTB,Event_CHECKTB,Event_FINALCK,Event_OVEROPEN,Event_STOP};

HANDLE g_eventdl = CreateEvent(NULL,TRUE,FALSE,NULL);//下载事件
int g_nNum = 0;
BOOL g_flag = FALSE;
CCDMLogInfo* m_pMyLog = NULL;
std::vector<CString> filenames;
CString g_cover="1";
BOOL g_nameadd=FALSE;


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
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


// CLaseAutoCorrectDlg 对话框

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


// CLaseAutoCorrectDlg 消息处理程序

BOOL CLaseAutoCorrectDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
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

	// 设置此对话框的图标。当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	LoadGui();
	InitSysParam();
	// TODO: 在此添加额外的初始化代码

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
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

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CLaseAutoCorrectDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CLaseAutoCorrectDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CLaseAutoCorrectDlg::OnBnClickedButtonDisconnet()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	Recv710AMKDataCnt=0;
	Recv711MKDataCnt=0;
	CWinThread *pThread;
	pThread = ::AfxBeginThread(DisConThreadProc,this);//创建连接线程
	if(pThread == NULL)
	{
		return;
	}
}


void CLaseAutoCorrectDlg::OnBnClickedButtonConnet()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	memset(g_nJGdata,0,sizeof(g_nJGdata));
	CWinThread *pThread;
	pThread = ::AfxBeginThread(ConnectThreadProc,this);//创建连接线程
	if(pThread == NULL)
	{
		return;
	}
}

void CLaseAutoCorrectDlg::LoadGui()
{
	CRect rc;
	m_Tab.InsertItem(0,"实时显示");
	m_Tab.InsertItem(1,"拟合插值");

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
	// TODO: 在此添加控件通知处理程序代码
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
//初始化界面参数
void CLaseAutoCorrectDlg::InitSysParam()
{
	//全局变量初始化;
	memset(&g_SysDevParam,0,sizeof(SysDevParam));

	////填充本机IP和端口
	//if(!GetHostAddress(&m_HostIpAddr))
	//{
	//	m_HostIpAddr.SetAddress(192,168,0,76);
	//}

	//初始化目标ip和端口
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

	//712初始化的界面
	GetDlgItem(IDC_COMBO_ChannelSelect)->ShowWindow(FALSE);
	GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("当前MAC地址");
	GetDlgItem(IDC_STATIC_MACSET)->ShowWindow(FALSE);
	GetDlgItem(IDC_EDIT_MACSET)->ShowWindow(FALSE);

	//串口初始化
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


	///读取合格标准配置文件
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
		::AfxMessageBox(_T("配置文件打开失败。"));
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

	//屏显和绘画初始化
	m_IntervNum=5;
	m_ChartNum=100;
	m_MkCalNum=50;
	m_prop1.m_ChartReal.GetAxis().GetBottom().SetMinMax(0,m_ChartNum*300);
	m_prop1.m_ChartReal.GetAxis().GetLeft().SetAutomatic(TRUE);
	UpdateData(FALSE);
	//创建串口接收解析线程
	//CWinThread *pThread;
	//pThread = ::AfxBeginThread(ComParseThread,this);//创建连接线程
	//if(pThread == NULL)
	//{
	//	bComParseTerm=FALSE;
	//	return;
	//}
	//bComParseTerm=TRUE;

	DispTips(IDB_PNG0,"空闲状态");

}

BOOL CLaseAutoCorrectDlg::OnDeviceChange(UINT nEventType,DWORD dwData)
{
	//0x4d36e978L, 0xe325, 0x11ce, 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18
	//DEV_BROADCAST_DEVICEINTERFACE* dbd = (DEV_BROADCAST_DEVICEINTERFACE*) dwData;

	DEV_BROADCAST_HDR* dhr = (DEV_BROADCAST_HDR *)dwData;
	TRACE("--nEventType--:%d\n", nEventType);
	switch (nEventType)
	{
	case DBT_DEVICEREMOVECOMPLETE://移除设备
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
	case DBT_DEVICEARRIVAL://添加设备
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
	if(m_pMyLog)	//写入日志
		m_pMyLog->SetNotify(strLog.GetBuffer(strLog.GetLength()));
	if (b_Show == 0)//为了减少UI的响应，所以有些非主要的内容就只记录到日志而不在UI上显示 pl 0604
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
	pREC->ReplaceSel(strLog);//在RichEditCtrl最后添加字符串

	pREC->PostMessage(WM_VSCROLL, SB_BOTTOM,0);

	if(pREC->GetLineCount()>100)
	{
		pREC->SetSel(0, -1);		
		pREC->ReplaceSel("");
	}
}

//连接线程函数
UINT ConnectThreadProc(LPVOID lpParam)
{
	CString strTemp="";
	CLaseAutoCorrectDlg *pDlg = (CLaseAutoCorrectDlg*)lpParam;
	g_SysDevParam.LaserPor=pDlg->m_PorType.GetCurSel();
	if (!(pDlg->m_NetConnection.bConnected))
	{
		//网络结构体变量赋值
//		pDlg->m_HostIpAddr.GetAddress(pDlg->m_NetConnection.dwServerIP);
		pDlg->m_NetConnection.unServerPort = 3000;
		pDlg->m_NetConnection.lpWnd = lpParam;
		pDlg->m_NetConnection.lpRecvFun = (LPVOID)OnNetRecv;	


		if (g_SysDevParam.LaserPor== 1)   //TCP模式
		{
			strTemp = "正在连接...";
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
				strTemp = "TCP网络连接成功";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				pDlg->GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(FALSE);
				pDlg->GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(TRUE);
				pDlg->GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(FALSE);
				g_SysDevParam.NetConnected=TRUE;
				//初始化图
				pDlg->m_prop1.m_ChartReal.Series(0).Clear();
				pDlg->m_prop1.m_ChartReal.Series(1).Clear();
				pDlg->m_prop1.m_ChartReal.Series(2).Clear();
				Sleep(1000);
				g_nRevcNum3=0;
				g_nJGProCnt=0;
				g_nJGRecCnt =0;
				//创建接收画图线程
				bAqcOrgTerm=TRUE;
				::AfxBeginThread(AqcOrgThreadProc,lpParam);  //创建接收控制线程
				pDlg->Get711LmdSinglePoint();
			}
			else
			{
				strTemp = "TCP网络连接失败";
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
				strTemp = "UDP网络连接成功";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				pDlg->GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(FALSE);
				pDlg->GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(TRUE);
				g_SysDevParam.NetConnected=TRUE;
				if (pDlg->m_LaserType.GetCurSel()<3)     //无711，因为711是tcp模式
				{
					g_SysDevParam.LaserType=1;     //710、710B、710C
				}
				else
				{
					g_SysDevParam.LaserType=2;     //710A
					::PostMessageA(pDlg->GetSafeHwnd(),WM_GET_SPLMD,0,0);
				}
				//初始化图
				pDlg->m_prop1.m_ChartReal.Series(0).Clear();
				pDlg->m_prop1.m_ChartReal.Series(1).Clear();
				pDlg->m_prop1.m_ChartReal.Series(2).Clear();
				g_nRevcNum3=0;
				g_nJGProCnt=0;
				g_nJGRecCnt =0;
				//创建接收画图线程
				bAqcOrgTerm=TRUE;
				::AfxBeginThread(AqcOrgThreadProc,lpParam);//创建连接线程
			}
			else
			{
				strTemp = "UDP网络连接失败";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				pDlg->GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(TRUE);
				pDlg->GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(FALSE);
				g_SysDevParam.NetConnected=FALSE;
			}
		}
	}
	else
	{
		strTemp = "网络已连接请断开重连";
		::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	}
	//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
	//::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	return 0;
}

//断开连接线程函数
UINT DisConThreadProc(LPVOID lpParam)
{
	CString strTemp="";
	CLaseAutoCorrectDlg *pDlg = (CLaseAutoCorrectDlg*)lpParam;
	if (pDlg->m_NetConnection.bConnected)
	{
		if(DisconnectServer(&(pDlg->m_NetConnection)))
		{
			strTemp = "网络连接断开成功";
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
			strTemp = "网络连接断开失败";
			pDlg->GetDlgItem(IDC_BUTTON_ConNet)->EnableWindow(FALSE);
			pDlg->GetDlgItem(IDC_BUTTON_DisConNet)->EnableWindow(TRUE);
		}

		pDlg->m_NetConnection.Init();
	}
	else
	{
		strTemp = "网络连接早已断开";
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
//	if(0 != WSAStartup(MAKEWORD(2, 2), &wsadata))   //初始化
//	{
//		return FALSE;
//	}
//	char szHostName[MAX_PATH + 1];
//	gethostname(szHostName, MAX_PATH);  //得到计算机名
//	hostent *p = gethostbyname(szHostName); //从计算机名得到主机信息
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
//	WSACleanup();               //释放Winsock API
//	return TRUE;
//}

void CLaseAutoCorrectDlg::OnBnClickedButtonOpencom()
{
	// TODO: 在此添加控件通知处理程序代码

	SerPortPar serPortPar;
	int nPort; //串口号
	int nBaud; //波特率
	CString strTemp="";
	int nRes=0;
	
	nPort = m_ComIndex.GetCurSel()+1;
	g_SysDevParam.nPort=nPort;
	m_ComBuand.GetLBText(m_ComBuand.GetCurSel(),strTemp);//波特率char类型转换成int类型
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
		//打开失败
		strTemp="串口打开失败";
		GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(TRUE);
		GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(FALSE);
	}
	else if (0==nRes)
	{
		//串口已打开
		strTemp="串口被占用或不存在";
		GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(TRUE);
		GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(TRUE);
		
	}
	else if (1==nRes)
	{
		//打开成功
		strTemp="串口打开成功";
		GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(FALSE);
		GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(TRUE);
		g_SysDevParam.ComConnected=TRUE;
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}


void CLaseAutoCorrectDlg::OnBnClickedButtonClosecom()
{
	// TODO: 在此添加控件通知处理程序代码
	CString strTemp="";
	int nRes=0;
	//nRes=Com_Close(fd);
	nRes=CloseComm(g_SysDevParam.nPort);
	if (1==nRes)
	{
		//串口已关闭
		strTemp="串口已被关闭";
		GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(TRUE);
		GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(FALSE);
		g_SysDevParam.nBuad=9600;
		g_SysDevParam.nPort=-1;
	}
	else if (0==nRes)
	{
		//串口关闭成功
		strTemp="串口关闭成功";
		GetDlgItem(IDC_BUTTON_OpenCom)->EnableWindow(TRUE);
		GetDlgItem(IDC_BUTTON_CloseCom)->EnableWindow(FALSE);
		g_SysDevParam.ComConnected=FALSE;
	}
	else if (-1==nRes)
	{
		//串口关闭失败
		strTemp="串口已被关闭";
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

//数据处理函数：主线程函数
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
	long int nSum = 0L;//每帧的和
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
			//接收的FF AA格式 包括参数查询和单点测距值
			frame_length= (((byte)g_cNetRecvBuf[nBufID][temp_head+2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][temp_head+3];
			frame_length = frame_length+4;
			if (frame_length <= temp_length)
			{
				if (g_cNetRecvBuf[nBufID][temp_head+4]==0x02 && g_cNetRecvBuf[nBufID][temp_head+5]==0x04)
				{
					//接收到参数查询回复
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
					case (char)0x0A:       //A9BAH表示当前激光已烧写好修正表，并且传输修正表的前20个数据供检测。;
						//WriteLog(&m_RichEdit,"烧写修正表成功。前20个数据如下：");
						strTemp="烧写修正表成功";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						strTemp="前20个数据如下：\n";
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
									AutoCorretRegInfo.nCheckTbCode=3;//需重新下载修正表
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
						//WriteLog(&m_RichEdit,"尚未烧写烧修正表");
						strTemp="尚未烧写烧修正表";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						break;
					case (char)0x0C:
						//WriteLog(&m_RichEdit,"修正表烧写不正确");
						strTemp="修正表烧写不正确";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						break;
					default:
						//WriteLog(&m_RichEdit,"修正表烧写检查指令回复无效");
						strTemp="修正表烧写检查指令回复无效";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						break;
					}
				}
				else if ((g_cNetRecvBuf[nBufID][temp_head+4] == 0x02)&&(g_cNetRecvBuf[nBufID][temp_head+5] == 0x0B))
				{
					switch(g_cNetRecvBuf[nBufID][temp_head+10])
					{
					case (char)0x01:        //成功
						g_nNum = (int)g_cNetRecvBuf[nBufID][temp_head+6];
						g_flag = TRUE;
						SetEvent(g_eventdl);
						strTmp.Format("%d",g_nNum);
						strTemp="第"+strTmp+"包接收成功";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						//WriteLog(&m_RichEdit,"收到回复：第"+strTmp+"包发送成功\r\n");
						break;
					case  (char)0x02:       //失败
						g_nNum = g_cNetRecvBuf[nBufID][temp_head+6];
						g_flag = FALSE;
						SetEvent(g_eventdl);
						strTmp.Format("%d",g_nNum);
						strTemp="第"+strTmp+"包接收失败";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						//WriteLog(&m_RichEdit,"收到回复：第"+strTmp+"包发送失败\r\n");
						break;
					default:
						break;
					}	
				}
				else if ((g_cNetRecvBuf[nBufID][temp_head+4] == 0x02)&&(g_cNetRecvBuf[nBufID][temp_head+5] == 0x0C))
				{
					strTemp = "激光器主控板重启成功";
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
								if (g_nRevcNum3%m_IntervNum == 0)          //每隔m_nInterval个数据包显示一个
								{  

									g_recv1len = frame_length;

									memcpy(g_nJGdata[0][g_nJGRecCnt],&g_cNetRecvBuf[nBufID][temp_head],g_recv1len);

									g_nMatch=1;//收到第一包数据

								}
							}
							break;
						case (char)0x02:
							{
								if (g_nRevcNum3%m_IntervNum == 0)          //每隔5个数据包显示一个
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
								if (g_nRevcNum3%m_IntervNum == 0)          //每隔5个数据包显示一个
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
										//画图

										UDP_DrawWave_710A(g_nJGdata[0][g_nJGRecCnt],1,1);
										UDP_DrawWave_710A(g_nJGdata[1][g_nJGRecCnt],1,2);
										UDP_DrawWave_710A(g_nJGdata[2][g_nJGRecCnt],1,3);
										if (AutoCorretRegInfo.nFinalCkCode==3)
										{
											g_nRevcNumcCheck1++;
										}

										//序号递增
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
			//FF FF开头的 下位机上传的脉宽数据
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
							if (g_nRevcNum3%m_IntervNum == 0)          //每隔m_nInterval个数据包显示一个
							{  
								g_recv1len = frame_length;
								memcpy(g_nJGdata[0][g_nJGRecCnt],&g_cNetRecvBuf[nBufID][temp_head],g_recv1len);				
								g_nMatch=1;//收到第一包数据
							}
						}
						break;
					case (char)0x02:
						{
							j2++;
							if (g_nRevcNum3%m_IntervNum == 0)          //每隔5个数据包显示一个
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
							if (g_nRevcNum3%m_IntervNum == 0)          //每隔5个数据包显示一个
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
									//画图
									UDP_DrawWave(g_nJGdata[0][g_nJGRecCnt],1,1);
									UDP_DrawWave(g_nJGdata[1][g_nJGRecCnt],1,2);
									UDP_DrawWave(g_nJGdata[2][g_nJGRecCnt],1,3);
									if (AutoCorretRegInfo.nFinalCkCode==3)
									{
										g_nRevcNumcCheck1++;
									}

									//序号递增
									g_clsCriticalSection.Lock();
									g_nJGRecCnt=g_nJGRecCnt+1;
									g_nJGRecCnt=g_nJGRecCnt%JGMAX_CNT;
									g_clsCriticalSection.Unlock();
								}
							}
							g_nRevcNum3++;
							if (g_SysDevParam.LaserPor== 1)   //TCP模式，即711用
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

//数据处理函数：主线程函数    711、712
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
	long int nSum = 0L;//每帧的和
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
			case H02_CmdA0://下位机返回
				{
					strTemp = "所发指令出现错误";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				}
				break;
			case H02_CmdA3://下位机返回
				{
					strTemp = "设置设备参数成功";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(FALSE);
				}
				break;
			case H02_Cmd89://下位机返回
				{
					strTemp = "激光器主控板重启成功";
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
					strTemp="参数设置成功";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(FALSE);
				}
				break;
			case 0x02:
				if (g_SysDevParam.LaserType==6)  //获取MAC
				{
					g_nNetParseBuffLen = frame_length;
					memcpy(g_nNetParseBuff,&g_cNetRecvBuf[nBufID],g_nNetParseBuffLen);
					H712_ParseMAC(g_nNetParseBuff, g_nNetParseBuffLen);
				}
				else
				{
					//设置高低阈值通道成功之后
					if((BYTE)g_cNetRecvBuf[nBufID][29]==0x01&&this->m_NetConnection.bConnected)
					{
						strTemp="参数设置成功";
						::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						OnBnClickedButtonDisconnet();
					}
				}
				break;
			case 0x04:
				{
					//712APD查询回复
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
						strTemp="APD参数设置成功";
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
			case 0x0C: //下位机回复发送修正结果发送成功
				switch ((BYTE)g_cNetRecvBuf[nBufID][29])
				{
				case 0x01:        //成功
					g_nNum = (g_cNetRecvBuf[nBufID][26]<<8)+g_cNetRecvBuf[nBufID][27];
					strTmp.Format("%d",g_nNum);
					strTemp="第"+strTmp+"包接收成功";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					g_flag = TRUE;
					SetEvent(g_eventdl);						
					break;
				case 0x02:       //失败
					g_nNum = (g_cNetRecvBuf[nBufID][26]<<8)+g_cNetRecvBuf[nBufID][27];
					strTmp.Format("%d",g_nNum);
					strTemp="第"+strTmp+"包接收失败\r\n";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					g_flag = FALSE;
					SetEvent(g_eventdl);
					break;
				default:
					break;
				}	
				break;
			case (char)0x0D://生产烧写修正表
				if((BYTE)g_cNetRecvBuf[nBufID][29]==0x01)
				{
					strTemp="修正表烧写成功";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					g_flag = TRUE;
					SetEvent(g_eventdl);
				}
				else if((BYTE)g_cNetRecvBuf[nBufID][29]==0x02)
				{
					strTemp="修正表烧写失败";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					g_flag = FALSE;
					SetEvent(g_eventdl);
				}
				break;
			case (char)0x0E://检查修正表烧写情况
				if((BYTE)g_cNetRecvBuf[nBufID][26]==0x01)
				{
					AutoCorretRegInfo.nCheckTbCode=1;
					strTemp="修正表已烧写，前10个数据如下：\n";
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
							strTemp += "，查询验证不一致";
							if (AutoCorretRegInfo.nChDownTbCnt>3)
							{
								AutoCorretRegInfo.nCheckTbCode=2;
							}
							else
							{
								AutoCorretRegInfo.nCheckTbCode=3;//需重新下载修正表
							}
							break;
						}
					}
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				}
				else if((BYTE)g_cNetRecvBuf[nBufID][26]==0x02)
				{
					strTemp="修正表未烧写";
					if (AutoCorretRegInfo.nChDownTbCnt>3)
					{
						AutoCorretRegInfo.nCheckTbCode=2;
					}
					else
					{
						AutoCorretRegInfo.nCheckTbCode=3;//需重新下载修正表
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
				case 0xba:       //A9BAH表示当前激光已烧写好修正表，并且传输修正表的前20个数据供检测。;
					//WriteLog(&m_RichEdit,"烧写修正表成功。前20个数据如下：");

					strTemp="前20个数据如下：\n";
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
								AutoCorretRegInfo.nCheckTbCode=3;//需重新下载修正表
							}
							break;
						}
					}
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					if (g_ChannelSelect==0)    //高阈值
					{
						strTemp="烧写高阈值修正表成功";
					} 
					else
					{
						strTemp="烧写低阈值修正表成功";
					}

					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					if (AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
					{
						SetEvent(Event_CHECKTB);
					}
					break;
				case 0xbb:    
					//WriteLog(&m_RichEdit,"尚未烧写烧修正表");
					strTemp="尚未烧写烧修正表";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					break;
				case 0xbc:
					//WriteLog(&m_RichEdit,"修正表烧写不正确");
					strTemp="修正表烧写不正确";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					break;
				default:
					//WriteLog(&m_RichEdit,"修正表烧写检查指令回复无效");
					strTemp="修正表烧写检查指令回复无效";
					::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					break;
				}
			}
		}
		else if(g_cNetRecvBuf[nBufID][22]==0x01)
		{
			if(g_cNetRecvBuf[nBufID][23]==0x02)//单点连续发数
			{
				if(g_SysDevParam.LaserType== 3 || (g_SysDevParam.LaserType== 6))
				{
					switch (g_cNetRecvBuf[nBufID][24])
					{
					case (char)0x01:
						{   
							g_recv1len=(((byte)g_cNetRecvBuf[nBufID][2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][3]+4;
							jg1++;
							if (g_nRevcNum3%m_IntervNum == 0)          //每隔m_nInterval个数据包显示一个
							{
								memcpy(g_nJGdata[0][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv1len);				
								g_nMatch=1;//收到第一包数据
							}
						}
						break;
					case (char)0x02:
						{
							g_recv2len=(((byte)g_cNetRecvBuf[nBufID][2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][3]+4;
							j2++;
							if (g_nRevcNum3%m_IntervNum == 0)          //每隔5个数据包显示一个
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
							if (g_nRevcNum3%m_IntervNum == 0)          //每隔5个数据包显示一个
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
									//画图
									UDP_DrawWave711(g_nJGdata[0][g_nJGRecCnt],1,1);
									UDP_DrawWave711(g_nJGdata[1][g_nJGRecCnt],1,2);
									UDP_DrawWave711(g_nJGdata[2][g_nJGRecCnt],1,3);
									if (AutoCorretRegInfo.nFinalCkCode==3)
									{
										g_nRevcNumcCheck1++;
									}

									//序号递增
									g_clsCriticalSection.Lock();
									g_nJGRecCnt=g_nJGRecCnt+1;
									g_nJGRecCnt=g_nJGRecCnt%JGMAX_CNT;
									g_clsCriticalSection.Unlock();
								}
							}
							g_nRevcNum3++;
							if (g_SysDevParam.LaserPor== 1)   //TCP模式，即711用
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
							if (g_nRevcNum3%m_IntervNum == 0)          //每隔m_nInterval个数据包显示一个
							{
								memcpy(g_nJGdata[0][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv1len);				
								g_nMatch=1;//收到第一包数据
							}
						}
						break;
					case (char)0x02:
						{
							g_recv2len=(((byte)g_cNetRecvBuf[nBufID][2] & 0xff) << 8) + (byte)g_cNetRecvBuf[nBufID][3]+4;
							j2++;
							if (g_nRevcNum3%m_IntervNum == 0)          //每隔5个数据包显示一个
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
							if (g_nRevcNum3%m_IntervNum == 0)          //每隔5个数据包显示一个
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
									//画图
									UDP_DrawWave711(g_nJGdata[0][g_nJGRecCnt],1,1);
									UDP_DrawWave711(g_nJGdata[1][g_nJGRecCnt],1,2);
									UDP_DrawWave711(g_nJGdata[2][g_nJGRecCnt],1,3);
									if (AutoCorretRegInfo.nFinalCkCode==3)
									{
										g_nRevcNumcCheck1++;
									}

									//序号递增
									g_clsCriticalSection.Lock();
									g_nJGRecCnt=g_nJGRecCnt+1;
									g_nJGRecCnt=g_nJGRecCnt%JGMAX_CNT;
									g_clsCriticalSection.Unlock();
								}
							}
							g_nRevcNum3++;
							if (g_SysDevParam.LaserPor== 1)   //TCP模式，即711用
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
				strTemp="功能参数设置成功";
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
	long int nSum = 0L;//每帧的和
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
				if (g_nRevcNum3%m_IntervNum == 0)          //每隔m_nInterval个数据包显示一个
				{
					memcpy(g_nJGdata[0][g_nJGRecCnt],&g_cNetRecvBuf[nBufID],g_recv1len);				
					g_nMatch=1;//收到第一包数据
				}
			}
			break;
		case (char)0x02:
			{
				g_recv2len=frame_length;
				j2++;
				if (g_nRevcNum3%m_IntervNum == 0)          //每隔5个数据包显示一个
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
				if (g_nRevcNum3%m_IntervNum == 0)          //每隔5个数据包显示一个
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
						//画图
						UDP_DrawWave711(g_nJGdata[0][g_nJGRecCnt],1,1);
						UDP_DrawWave711(g_nJGdata[1][g_nJGRecCnt],1,2);
						UDP_DrawWave711(g_nJGdata[2][g_nJGRecCnt],1,3);
						if (AutoCorretRegInfo.nFinalCkCode==3)
						{
							g_nRevcNumcCheck1++;
						}

						//序号递增
						g_clsCriticalSection.Lock();
						g_nJGRecCnt=g_nJGRecCnt+1;
						g_nJGRecCnt=g_nJGRecCnt%JGMAX_CNT;
						g_clsCriticalSection.Unlock();
					}
				}
				g_nRevcNum3++;
				if (g_SysDevParam.LaserPor== 1)   //TCP模式，即711用
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
* 函数功能：求UDP每帧数据点的和
* 函数参数：pcDataBuf:数据缓冲区，nFirstByte:数据起始字节
* 返 回 值：数据和
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
		nDataNum = (pcDataBuf[7]<<8) + (BYTE)(pcDataBuf[6]);//数据点个数
	}
	else if (nType==2)
	{
		nDataNum = (pcDataBuf[7]<<8) + (pcDataBuf[8]);//数据点个数
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
		nDataNum = (pcDataBuf[26]<<8) + (BYTE)(pcDataBuf[27]);//数据点个数
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
		nDataNum = (pcDataBuf[27]<<8) + (BYTE)(pcDataBuf[28]);//数据点个数
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
		nDataNum = (pcDataBuf[7]<<8) + (BYTE)(pcDataBuf[6]);//数据点个数
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
		nDataNum = (pcDataBuf[26]<<8) + (BYTE)(pcDataBuf[27]);//数据点个数
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
		nDataNum = (pcDataBuf[27]<<8) + (BYTE)(pcDataBuf[28]);//数据点个数
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
		nDataNum = (pcDataBuf[7]<<8) + (BYTE)(pcDataBuf[6]);//数据点个数
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
		strTemp = "查询参数回复有误:";
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

		strTemp = "查询设备参数成功";
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
		strTemp = "查询APD参数回复有误:";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		//WriteLog(&m_RichEdit,m_sLog,(unsigned char *)pcBuf,nBufSize);
		return;
	}
	else
	{
		//if (g_nAPDStateTemp==-100) //若当前APD温度没有查到，则不解析
		//{
		//	strTemp = "APD当前温度未查到";
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

		strTemp = "查询APD参数成功";
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
//		strTemp = "查询参数回复有误:";
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
		strTemp = "查询参数回复有误:";
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
		strTemp = "查询参数回复有误:";
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

		//strTemp = "查询设备参数成功";
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
		strTemp = "查询APD参数回复有误:";
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

		strTemp = "查询APD参数成功";
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
				AutoCorretRegInfo.nParamChCode=4;  //继续其他参数检查
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
		strTemp = "查询功能参数回复有误:";
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

		strTemp = "查询系统参数成功";
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

	strTemp = "查询系统参数成功";
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

	strTemp = "查询APD参数成功";
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
			AutoCorretRegInfo.nParamChCode=4;  //继续其他参数检查
			g_CheckAPD=true;
		}
		SetEvent(Event_PARAM);
	}
	UpdateData(FALSE);
}

void CLaseAutoCorrectDlg::OnBnClickedButtonGetparam()
{
	// TODO: 在此添加控件通知处理程序代码
	::SendMessage(this->GetSafeHwnd(),WM_LASER_GETPARAM,0,0);
	
}

void CLaseAutoCorrectDlg::OnBnClickedButtonSetparam()
{
	// TODO: 在此添加控件通知处理程序代码
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
	//	strTemp="MAC不足12位";
	//	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	//	return;
	//}
	nLen = m_SetMAC.GetLength();
	if (nLen != 12)
	{
		strTemp="MAC应该为12位";
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
	strTemp="发送设置参数指令";
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

LONG CLaseAutoCorrectDlg::OnSendNetData(WPARAM wParam, LPARAM lParam)         //响应属性页中的设置设备参数命令
{
	m_nSendBufDataSize = (int)wParam;	
	memcpy(m_cSendBufData, (char *)lParam, m_nSendBufDataSize);
	
	m_NetConnection.pcSendDataBuf = m_cSendBufData;
	m_NetConnection.nSendDataBufSize = m_nSendBufDataSize;

	if (g_SysDevParam.LaserPor == 1)                       //TCP协议
	{
		SendData(&m_NetConnection);
	}
	else if (g_SysDevParam.LaserPor ==0)                  //UDP协议
	{
		DWORD dwIP;
		m_DestIp.GetAddress(dwIP);
		
		SOCKADDR_IN addrTo;
		addrTo.sin_family=AF_INET;
		addrTo.sin_port=htons(m_DestPort);
		addrTo.sin_addr.S_un.S_addr=htonl(dwIP);             //得到目标IP和端口号
		UDP_SendData(&m_NetConnection,&addrTo);
	}
	
	return 0L;
}

void CLaseAutoCorrectDlg::UDP_DrawWave(char *pcDataBuf, int nBufCount,int nChannel)
{

	int i,j;
	int nDataJi,nDataNum;  //极坐标Y值，有效数据长度
	int k =0 ;

	//点数，1个点占2位
	nDataNum = (pcDataBuf[7]<<8) + (BYTE)pcDataBuf[6]; //万集数据：高位在后，低位在前
	i = 8;


	int nNum = 0;

	for (j=(nBufCount-1)*nDataNum; j<nBufCount*nDataNum; j++)
	{
		nDataJi =0;
		//此处各个字节要和0xFF相与，否则结果为负值
		nDataJi = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
		i+=4;
		switch (nChannel)
		{
		case 1:					
			m_prop1.m_ChartReal.Series(0).Add(nDataJi,"",0); //一路
			break;
		case 2:
			//	m_twodata = nDataJi;			
			m_prop1.m_ChartReal.Series(1).Add(nDataJi,"",0);//二路
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
	int nDataJi,nDataNum;  //极坐标Y值，有效数据长度
	int k =0 ;
	if(g_SysDevParam.LaserType==4)
	{
		//点数，1个点占2位
		nDataNum = (pcDataBuf[27]<<8) + (BYTE)pcDataBuf[28]; //万集数据：高位在后，低位在前
		i = 29;
		int nNum = 0;

		for (j=(nBufCount-1)*nDataNum; j<nBufCount*nDataNum; j++)
		{
			nDataJi =0;
			//此处各个字节要和0xFF相与，否则结果为负值
			nDataJi = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			i+=4;
			switch (nChannel)
			{
			case 1:					
				m_prop1.m_ChartReal.Series(0).Add(nDataJi,"",0); //一路
				break;
			case 2:
				//	m_twodata = nDataJi;			
				m_prop1.m_ChartReal.Series(1).Add(nDataJi,"",0);//二路
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
		//点数，1个点占2位
		nDataNum = (pcDataBuf[26]<<8) + (BYTE)pcDataBuf[27]; //万集数据：高位在后，低位在前
		i = 28;


		int nNum = 0;

		for (j=(nBufCount-1)*nDataNum; j<nBufCount*nDataNum; j++)
		{
			nDataJi =0;
			//此处各个字节要和0xFF相与，否则结果为负值
			nDataJi = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			i+=4;
			switch (nChannel)
			{
			case 1:					
				m_prop1.m_ChartReal.Series(0).Add(nDataJi,"",0); //一路
				break;
			case 2:
				//	m_twodata = nDataJi;			
				m_prop1.m_ChartReal.Series(1).Add(nDataJi,"",0);//二路
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
		//点数，1个点占2位
		nDataNum = (pcDataBuf[7]<<8) + (BYTE)pcDataBuf[6]; //万集数据：高位在后，低位在前
		i = 8;


		int nNum = 0;

		for (j=(nBufCount-1)*nDataNum; j<nBufCount*nDataNum; j++)
		{
			nDataJi =0;
			//此处各个字节要和0xFF相与，否则结果为负值
			nDataJi = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
			i+=4;
			switch (nChannel)
			{
			case 1:					
				m_prop1.m_ChartReal.Series(0).Add(nDataJi,"",0); //一路
				break;
			case 2:
				//	m_twodata = nDataJi;			
				m_prop1.m_ChartReal.Series(1).Add(nDataJi,"",0);//二路
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
	int nDataJi,nDataNum;  //极坐标Y值，有效数据长度
	int k =0 ;

	//有效字节数
	nDataNum = (pcDataBuf[7]<<8) + (BYTE)pcDataBuf[8]; //万集数据：高位在后，低位在前
	i = 9;


	int nNum = 0;
	for (j=(nBufCount-1)*nDataNum; j<nBufCount*nDataNum; j++)
	{
		nDataJi =0;
		//此处各个字节要和0xFF相与，否则结果为负值
		nDataJi = ((pcDataBuf[i+3]&0xFF)<<24) + ((pcDataBuf[i+2]&0xFF)<<16) +((pcDataBuf[i+1]&0xFF)<<8) + (pcDataBuf[i]&0xFF); 
		i+=4;
		switch (nChannel)
		{
		case 1:					
			m_prop1.m_ChartReal.Series(0).Add(nDataJi,"",0); //一路
			break;
		case 2:			
			m_prop1.m_ChartReal.Series(1).Add(nDataJi,"",0);//二路
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
	// TODO: 在此添加控件通知处理程序代码
	CString strTemp="";
	if (Com_Send(a_ucTemp11,6))
	{
		strTemp="发送光阑张开指令成功";
	}
	else
	{
		strTemp="发送光阑张开指令失败";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,1,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::OnBnClickedButtonCloseslop()
{
	// TODO: 在此添加控件通知处理程序代码
	CString strTemp="";
	if (Com_Send(a_ucTemp10,6))
	{
		strTemp="发送光阑闭合指令成功";
	}
	else
	{
		strTemp="发送光阑闭合指令失败";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,1,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::OnBnClickedButtonMoveslop()
{
	// TODO: 在此添加控件通知处理程序代码
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
		strTemp="正向"+str;
	}
	else if (m_slopDir.GetCurSel()==1)
	{
		step=-step;
		strTemp="反向"+str;
	}
	else
	{
		strTemp="正向"+str;
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
		strTemp=strTemp+",发送光阑调整指令成功";
	}
	else
	{
		strTemp=strTemp+",发送光阑调整指令失败";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

//已无用，公司自作的驱动板，对应的协议
/*void CLaseAutoCorrectDlg::OnBnClickedButtonMoveslop()
{
	// TODO: 在此添加控件通知处理程序代码
	CString strTemp="";
	UINT16 CaclCRC=0;
	//UpdateData(TRUE);
	CString str="";
	GetDlgItem(IDC_EDIT_SlopSteps)->GetWindowTextA(str);
	m_slopeSteps=_ttoi(str);

	if (m_slopDir.GetCurSel()==0)
	{
		a_ucTemp3[6]=0xAA;
		strTemp="正向"+str;
	}
	else if (m_slopDir.GetCurSel()==1)
	{
		a_ucTemp3[6]=0xBB;
		strTemp="反向"+str;
	}
	else
	{
		a_ucTemp3[6]=0xAA;
		strTemp="正向"+str;
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
		strTemp=strTemp+",发送光阑调整指令成功";
	}
	else
	{
		strTemp=strTemp+",发送光阑调整指令失败";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}*/

void CLaseAutoCorrectDlg::OnOneKeyTestCorrect()
{
	CString strTemp="";
	if (Com_Send(a_ucTemp4,12))
	{
		strTemp="发送一键验证成功";
	}
	else
	{
		strTemp="发送一键验证失败";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	//WriteLog(&m_RichEdit,strTemp);
}

void CLaseAutoCorrectDlg::OnOnekeyTestFinish()
{
	CString strTemp="";
	if (Com_Send(a_ucTemp5,12))
	{
		strTemp="发送验证结束成功";
	}
	else
	{
		strTemp="发送验证结束失败";
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
	//	//找到帧头
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
	//	//上一帧丢了帧头部分，则会造成该帧的帧尾和下帧的帧头连到一块，去掉前一个ff
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
	//					//完整的帧
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
	//						strTemp="CRC16校验失败";
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
		strTemp="光阑运动中";
		strTempState="运动中";
		::SendMessageA(this->GetSafeHwnd(),WM_MOTOSTATE,0,(LPARAM)(LPCTSTR)strTempState);
		break;
	case 0x02:
		strTemp="光阑运动停止";
		strTempState="停止";
		::SendMessageA(this->GetSafeHwnd(),WM_MOTOSTATE,0,(LPARAM)(LPCTSTR)strTempState);
		if (g_bMoveFlag==true)
		{
			strTemp="光阑微运动成功";
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
		strTemp="抵达左限位";
		strTempState="左限位";
		::SendMessageA(this->GetSafeHwnd(),WM_MOTOSTATE,0,(LPARAM)(LPCTSTR)strTempState);
		if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
		{
			StopTimer(TIMERORIGMK);
			strTemp="光阑快速关闭成功";
			AutoCorretRegInfo.nStepIndx=STEP_MINMK;
			g_bCountStart=1;//进入统计判断最小脉宽
		}		
		break;
	case 0x04:
		strTemp="抵达右限位";
		strTempState="右限位";
		::SendMessageA(this->GetSafeHwnd(),WM_MOTOSTATE,0,(LPARAM)(LPCTSTR)strTempState);
		if (AutoCorretRegInfo.nStepIndx==STEP_MINMK)
		{
			StopTimer(TIMERORIGMK);
			strTemp="光阑快速张开成功";
			AutoCorretRegInfo.nStepIndx=STEP_MAXMK;
			g_bCountStart=1;//进入统计判断最大脉宽
		}
		else if(AutoCorretRegInfo.nStepIndx==STEP_FINALCH)
		{
			StopTimer(TIMERORIGMK);
			strTemp="开始验证修正效果";
			AutoCorretRegInfo.nFinalCkCode=3;
			m_prop1.m_ChartReal.Series(0).Clear();
			m_prop1.m_ChartReal.Series(1).Clear();
			m_prop1.m_ChartReal.Series(2).Clear();
			g_nRevcNum3=0;
			SetEvent(Event_FINALCK);
		}
		break;
	case 0x10:
		strTemp="串口响应成功";
		if (g_nComComandStat==1)
		{
			//StopTimer(TIMEROPENING);
			StopTimer(TIMER712COMSEND);
			g_nComComandStat=0;
			g_nContinuousMoveState = 1;   //表示当前状态为开始张开
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
					g_bCountStart=1;   //开始统计是否达到450ns
				}
				else if(AutoCorretRegInfo.nStepIndx==STEP_BEFORECHECKOPEN)
				{
					AutoCorretRegInfo.nStepIndx=STEP_FINALCH;
					strTemp="开始验证修正效果";
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
			g_nContinuousMoveState = 2;   //表示当前状态为开始闭合
			g_bCountStart=3;    //成功开始关闭后开始验证
		}
		break;
	case 0x20:
		strTemp="串口响应失败";
		break;
	default:
		strTemp="串口收到未知回复";
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
	// TODO: 在此添加控件通知处理程序代码
	//创建串口接收解析线程
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
	else   //包括712
	{
		MK_ORG0=450;
		MK_ORG1=500;
	}
	
	if (BtnCaption=="开启一键修正")
	{
		if (g_SysDevParam.NetConnected&&g_SysDevParam.ComConnected)
		{
			//初始化相关变量
			InitAutoCorrectTask();
			//创建修正线程
			bAutoCorrTerm=TRUE;
			pThread = ::AfxBeginThread(AutoCorrThreadProc,this);//创建连接线程
			if(pThread == NULL)
			{
				bAutoCorrTerm=FALSE;
				return;
			}
			SetEvent(Event_START);
			b711AFirstClose=true;

			//SetEvent(Event_FINALCK);//调试用
			//调试步数用
			//SetEvent(Event_AQCMK);
			//AutoCorretRegInfo.nStepIndx=STEP_MAXMK;
			//BeginValue=450000;

			(GetDlgItem(IDC_BUTTON_AutoCorrect))->SetWindowTextA("停止一键修正");
		}
		else
		{
			AfxMessageBox(_T("请检查网串连接状态"),MB_OK|MB_ICONSTOP);
		}
	}
	else if (BtnCaption=="停止一键修正")
	{
		SetEvent(Event_STOP);
		/*bAutoCorrTerm=FALSE;
		bTargetAqcTerm=FALSE;
		InitAutoCorrectTask();
		(GetDlgItem(IDC_BUTTON_AutoCorrect))->SetWindowTextA("开启一键修正");*/
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
		nRet = WaitForMultipleObjects(11,Event_List,false,600*1000);	//等待超时

		if(WAIT_TIMEOUT == nRet)	//等待事件超时时间到
		{
			strTemp="自动修正超时终止";
			//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,0);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
			SetEvent(Event_STOP);
		}
		else if(WAIT_OBJECT_0 == nRet)	//Event_START 处于有信号状态 开启自动修正任务
		{
			//进入开始状态 获取参数 检查参数项
			strTemp="开启自动修正任务";
			//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
			AutoCorretRegInfo.nStepIndx=STEP_START;//开启修正
			AutoCorretRegInfo.nTaskCode=1;//开启成功
			::PostMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,1);
			Sleep(1000);
			SetEvent(Event_PARAM);
		}
		else if((WAIT_OBJECT_0 + 1) == nRet)	//Event_PARAM有信号 需进行初始参数检查
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_START)
			{ 
				//g_AutoStep=STEP_PARAM;
				strTemp="进入参数检测阶段";
				//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,2);
				AutoCorretRegInfo.nStepIndx=STEP_PARAM;
				//不让操作查询和设置按钮
				pDlg->GetDlgItem(IDC_BUTTON_GetParam)->EnableWindow(FALSE);
				pDlg->GetDlgItem(IDC_BUTTON_SetParam)->EnableWindow(FALSE);
				::PostMessageA(pDlg->GetSafeHwnd(),WM_LASER_GETPARAM,0,0);
				pDlg->StartTimer(TIMERGETPARMA,TIMER_GETPARAM,5000);
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
			{
				if (AutoCorretRegInfo.nParamChCode==1)
				{
					strTemp="检查参数正常通过";
					//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					
					//////////////////////////////////////////////////////////////////////////////////////////////
					SetEvent(Even_ORIGMK);  
				}
				else if (AutoCorretRegInfo.nParamChCode==2)
				{
					strTemp="参数检查不达要求";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if (AutoCorretRegInfo.nParamChCode==3)
				{
					strTemp="参数无法获取！";
					//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if (AutoCorretRegInfo.nParamChCode==4)
				{
					if (g_SysDevParam.LaserType==6)   //712等待填入真实距离
					{
						//创建保存文件夹及文件
						pDlg->OnCreateFolder(pDlg->m_MacAddr);

						CString name="拟合图";
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
							pDlg->OnCreateFile(pDlg->m_MacAddr+"_原始数据"+"("+g_cover+").txt",1);
							pDlg->OnCreateFile(pDlg->m_MacAddr+"_测试记录"+"("+g_cover+").txt",2);
						}
						else
						{
							pDlg->OnCreateFile(pDlg->m_MacAddr+"_原始数据.txt",1);
							pDlg->OnCreateFile(pDlg->m_MacAddr+"_测试记录.txt",2);
						}

						//pDlg->OnCreateFile(pDlg->m_MacAddr+"_测试记录.txt",2);
						m_pMyLog = CCDMLogInfo::GetInstance(pDlg->m_OperTxtPath.GetBuffer(pDlg->m_OperTxtPath.GetLength()));//获取日志文件
						strTemp=  "\r\n========" + CTime::GetCurrentTime().Format("%H:%M:%S") + "========\r\n";
						if(m_pMyLog)	//写入日志
							m_pMyLog->SetNotify(strTemp.GetBuffer(strTemp.GetLength()));
						strTemp="创建数据保存文件";
						//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
						::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

						AutoCorretRegInfo.nWaitInputDistCnt=0;
						strTmp="请输入距离信息，然后点击[距离确认]按钮";
						::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
						AfxMessageBox(_T("请输入距离信息，然后点击[距离确认]按钮"),MB_OK);
						pDlg->StartTimer(TIMERINPUTDIST,TIMER_INPUTDIST,5000);
					}
					else
					{
						//strTemp="继续检查711的其他参数";
						::PostMessageA(pDlg->GetSafeHwnd(),WM_LASER_GETPARAM,0,0);
						pDlg->StartTimer(TIMERGETPARMA,TIMER_GETPARAM,10000);
					}
				}
			}
		}
		else if ((WAIT_OBJECT_0 + 2) == nRet)//Even_ORIGMK有信号 开始初始脉宽检查  712为最大值检查450ns
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
			{
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,3);
				//判断最大脉宽是否为450ns
				strTemp="判断最大脉宽是否为450ns";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				AutoCorretRegInfo.nStepIndx=STEP_MAXMK;
				g_bCountStart=1;   //开始统计是否达到450ns
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_MAXMK)
			{
				if (AutoCorretRegInfo.nOrigMkChCode==2)
				{
					//未到450ns
					//g_bCountStart=0;   //不需计算，只是每包统计直到大于450ns后立即停止张开
					g_bOpenflag=true;  //开始实时判断张开何时停止
					strTemp="脉宽最大值未到450ns，自动张开光阑";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_OPEN,0,0);
				}
				else if (AutoCorretRegInfo.nOrigMkChCode==1)
				{
					strTemp="脉宽最大值ok";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					SetEvent(Event_AQCMK);
				}
				else if(AutoCorretRegInfo.nOrigMkChCode==3)
				{
					strTemp="脉宽最大值无法达到450ns";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if(AutoCorretRegInfo.nOrigMkChCode==4)
				{
					//识别达到450ns，停止张开
					strTemp="识别达到450ns，停止张开";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);
					//开始统计是否达到450ns,放在停止响应后修改标志位
				}
			}
		}
		else if ((WAIT_OBJECT_0 + 3) == nRet) //Event_AQCMK 进行原始数据采集
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_MAXMK)
			{
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,4);
				AutoCorretRegInfo.nStepIndx=STEP_ACQMK;
				//初始化采集目标
				pDlg->InitAqcDataTarget();
				bTargetAqcTerm=TRUE;
				pThread = ::AfxBeginThread(TargetDataAqcThread,lpParam);//创建原始数据采集线程 并自动调整光阑张合
				SetEvent(Event_AQCMK);
				//初始脉宽检测达标后，进入原始数据采集 并自动调整光阑的张合
				strTemp="进入原始数据采集";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);		
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_ACQMK)
			{
				if (AutoCorretRegInfo.nAqcMkCode==1)
				{
					pDlg->SaveOrigData();
					strTemp="保存原始数据成功";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					SetEvent(Event_FITMK);
				}
				else if (AutoCorretRegInfo.nAqcMkCode==2)
				{
					if (g_Pid.e_0>0)
					{
						//误差大于0 则正转调整
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
						//限制最大步数
						if (g_AcqTartetInfo.KeyValue[g_AcqTartetInfo.AqcIndx] <= 120)    //711中在第脉宽时，限制每次调整最大值为50步  (g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==4) && g_ChannelSelect==1 && 
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
						//误差小于0 则反转调整
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
						//限制最大步数
						if (g_AcqTartetInfo.KeyValue[g_AcqTartetInfo.AqcIndx] <= 120)    //711中在第脉宽时，限制每次调整最大值为50步  (g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==4) && g_ChannelSelect==1 && 
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
					//光阑微调整完毕 可以继续进行采集
					g_bCountStart=1;
				}
				else if (AutoCorretRegInfo.nAqcMkCode==4)
				{
					//调整光阑到位 可以直接采集计算了
					g_bCountStart=2;
				}
				else if (AutoCorretRegInfo.nAqcMkCode==5)
				{
					strTemp="光阑调整失效超时";
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
					Filename=pDlg->m_MacAddr+"_修正数据"+"("+g_cover+").txt_MK";
				}
				else
				{
					Filename=pDlg->m_MacAddr+"_修正数据.txt_MK";
				}
				
				::SendMessage(pDlg->GetSafeHwnd(),WM_FIT_MAINTH,(WPARAM)&nFitRes,(LPARAM)&Filename);    //拟合
				
				if (nFitRes)
				{
					strTemp="拟合插值执行完毕";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					AutoCorretRegInfo.nFitMkCode=1;
					AutoCorretRegInfo.nChDownTbCnt=1;  //查询修正表验证不通过重发统计
					SetEvent(Event_DOWNTB);
				}
				else
				{
					strTemp="拟合插值执行失败";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					AutoCorretRegInfo.nFitMkCode=2;
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
			}
		}
		else if((WAIT_OBJECT_0 + 6) == nRet)//Event_DOWNTB 自动下载修正表
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_FITMK)
			{
				AutoCorretRegInfo.nStepIndx=STEP_DOWNTB;
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,6);
				strTemp="开始下载修正表";
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
					//失败
				}
			}
		}
		else if((WAIT_OBJECT_0 + 7) == nRet)//Event_CHECKTB 检查修正表下载是否正确
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_DOWNTB)
			{
				AutoCorretRegInfo.nCheckTbCode=0;
				AutoCorretRegInfo.nStepIndx=STEP_CHECKTB;
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,7);
				::SendMessage(pDlg->GetSafeHwnd(),WM_TAB_CHANGE,0,0);
				strTemp="检查修正表下载情况";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

				pDlg->SendCheckCorrectTb711();    //包括712

				AutoCorretRegInfo.nComReSendCnt=0;
				pDlg->StartTimer(TIMERGLADJUST,TIMER_GLADJUST,5000);    //复用同一个计时，发送查询修正表
				//SetEvent(Event_CHECKTB);
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
			{
				if (AutoCorretRegInfo.nCheckTbCode==1)
				{
					strTemp="修正表验证已通过";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					SetEvent(Event_FINALCK);
				}
				else if (AutoCorretRegInfo.nCheckTbCode==2)
				{
					strTemp="修正表验证未通过";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if (AutoCorretRegInfo.nCheckTbCode==3)
				{
					AutoCorretRegInfo.nCheckTbCode=0;
					AutoCorretRegInfo.nChDownTbCnt++;
					strTemp.Format("第%d次下载检验",AutoCorretRegInfo.nChDownTbCnt);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					AutoCorretRegInfo.nStepIndx=STEP_FITMK;
					SetEvent(Event_DOWNTB);
				}
			}
		}
		else if((WAIT_OBJECT_0 + 8) == nRet)//Event_FINALCK 最后一键检查修正是否达标
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
			{
				//AutoCorretRegInfo.nStepIndx=STEP_FINALCH;
				AutoCorretRegInfo.nStepIndx=STEP_BEFORECHECKOPEN;
				::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,8);
				
				strTemp="验证修正是否达标";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				//开始一键验证
				//pDlg->OnOneKeyTestCorrect();
				g_bOpenflag=true;  //开始实时判断张开何时停止
				strTemp="自动张开光阑都到450ns";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_OPEN,0,0);
			}
			else if(AutoCorretRegInfo.nStepIndx==STEP_BEFORECHECKOPEN)
			{
				if(AutoCorretRegInfo.nOrigMkChCode==3)
				{
					strTemp="脉宽最大值无法达到450ns";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if(AutoCorretRegInfo.nOrigMkChCode==4)
				{
					//识别达到450ns，停止张开
					strTemp="识别达到450ns，停止张开";
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
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //停止继续关闭光阑
					}
					//测试用
					//strTemp.Format("出错点数共%d 检查的共%d帧 接收的共%d(%d)(%d)帧 均值最大和最小之差为%d 单点误差最大值最小值为%d,%d",g_CheckMKForm.ValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkOffset,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					strTemp.Format("出错点数共%d 检查的共%d帧 接收的共%d(%d)(%d)帧 均值偏差的最大最小值为%d,%d 单点误差最大值最小值为%d,%d",g_CheckMKForm.ValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkMax,g_CheckMKForm.ChMkMin,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,1,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,11);
					strTemp="修正后一键验证通过";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
				//	Sleep(10000);
					//712验证通过，张开光阑
					//::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_OPEN,0,0);
					//pDlg->OnOnekeyTestFinish();
					SetEvent(Event_OVEROPEN);
				}
				else if (AutoCorretRegInfo.nFinalCkCode==2)
				{		
					if (g_bCloseflag == true)
					{
						g_bCloseflag = false;
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //停止继续关闭光阑
					}
					//测试用
					//strTemp.Format("出错点数共%d 检查的共%d帧 接收的共%d(%d)(%d)帧 均值最大和最小之差为%d 单点误差最大值最小值为%d,%d",g_CheckMKForm.ValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkOffset,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					strTemp.Format("出错点数共%d 不合格帧数共%d 检查的共%d帧 接收的共%d(%d)(%d)帧 均值偏差的最大最小值为%d,%d 单点误差最大值最小值为%d,%d",g_CheckMKForm.ValueJudgeCnt,g_CheckMKForm.AveValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkMax,g_CheckMKForm.ChMkMin,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,1,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,10);
					strTemp="修正后一键验证未过";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					//712验证结束，张开光阑
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
					//strTemp.Format("验证标准为：均值误差%d，单次误差%d,检测脉宽范围最小值%d",g_CheckMKForm.AveValueError,g_CheckMKForm.SingleValueError,g_CheckMKForm.ChMkRegionMin);
					strTemp.Format("验证标准为：均值误差±%d，单次误差±%d,检测脉宽范围最小值%d",g_CheckMKForm.AveValueError,g_CheckMKForm.SingleValueError,g_CheckMKForm.ChMkRegionMin);
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
					//712开始闭合验证
					g_bCloseflag=true;  //开始实时判断闭合何时停止
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_CLOSE,0,0);
					g_bCloseSpeedChange = true;  //用于判断何时降低关闭速度
					//g_bCountStart=3;   //改到关闭指令响应位置
				}
				else if (AutoCorretRegInfo.nFinalCkCode==4)   //降低关闭速度
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
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_CLOSE,0,0);    //重新发一遍关闭
				}
				else if (AutoCorretRegInfo.nFinalCkCode==5)    
				{		
					if (g_bCloseflag == true)
					{
						g_bCloseflag = false;
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //停止继续关闭光阑
					}
					
					//测试用
					strTemp="关闭超时，重新验证！";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,10);
					strTemp="修正后一键验证未过";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
			}
		}
		else if((WAIT_OBJECT_0 + 9) == nRet)//Event_OVEROPEN 完成自动修正前张开光阑
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
		else if((WAIT_OBJECT_0 + 10) == nRet)//Event_STOP 停止修正任务
		{
			switch(AutoCorretRegInfo.nStepIndx)
			{
			case STEP_FINALCH:
				//屏显和绘画初始化
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
				pDlg->GetDlgItem(IDC_BUTTON_AutoCorrect)->SetWindowTextA("开启一键修正");
				if (AutoCorretRegInfo.nFinalCkCode==1)
				{
					strTmp="自动修正任务完成";
					AfxMessageBox(_T("自动修正任务成功结束:[结果通过]"),MB_OK);
					//pDlg->OnBnClickedButton2();
					//写入数据库
					if(g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==4)   //711E、711A
					{
						CWinThread *pThread;
						pThread = ::AfxBeginThread(WriteData,lpParam);
					}
				}
				else
				{
					strTmp="自动修正任务终止";
					AfxMessageBox(_T("自动修正任务终止:[结果失败]"),MB_OK|MB_ICONSTOP);
				}
				if(m_pMyLog)	//写入日志
					m_pMyLog->SetNotify(strTmp.GetBuffer(strTmp.GetLength()));
				m_pMyLog = NULL;
			}
			pDlg->InitAutoCorrectTask();
			
/*			//高低阈值切换
			if(g_ChannelSelect==0)//高阈值
			{
				////设置为低阈值
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
				//strTemp="发送设置功能参数指令";
				//::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				pDlg->m_ChannelSelect.SetCurSel(1);
				g_ChannelSelect=1;
				pDlg->H711_SetCmd65();
				

				//711重连
				if(g_SysDevParam.LaserPor== 1)
				{
					//pDlg-> Reset711();
					//pDlg->OnBnClickedButtonDisconnet();//断开
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
					//连接成功
					if(pDlg->m_NetConnection.bConnected)
					{
						if(g_SysDevParam.LaserType==4)//711A工装张开比较慢
						{
						   Sleep(25000);
						}
					  //开启修正低阈值
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
				strTemp.Format("第%d组脉宽%dns采集",g_AcqTartetInfo.AqcIndx+1,g_AcqTartetInfo.KeyValue[g_AcqTartetInfo.AqcIndx]);
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
				g_AcqTartetInfo.AqcDone[g_AcqTartetInfo.AqcIndx]=1;//进入该脉宽测试采集阶段
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
	// TODO: 在此添加消息处理程序代码和/或调用默认值
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
				strTmp="请输入距离信息，然后点击[距离确认]按钮";
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
			g_bMoveFlag=false; //712表示停止运动
			StopTimer(TIMERGLADJUST);
			break;
		}
		if (AutoCorretRegInfo.nStepIndx==STEP_ACQMK)
		{
			if(g_bMoveFlag==true)
			{
				AutoCorretRegInfo.nComReSendCnt++;
				if (AutoCorretRegInfo.nComReSendCnt>13)//pl 0403 将>10改为>13 解决了初始脉宽值大于475自动修正任务终止，结果失败 
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
		if(nPSeconds > 20)	//大于20秒
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
	strTemp="发送查询设备参数";
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

	//校验位 第一位
	NetContentBuff[Tmplen++]=0x00;

	//长度位
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
	//帧头
	SetSysParamBuf[0] = 0xFF;
	SetSysParamBuf[1] = 0xAA;

	//长度
	SetSysParamBuf[2]=0x00;
	SetSysParamBuf[3]=0x76;

	//命令
	SetSysParamBuf[4] = 0x02;
	SetSysParamBuf[5] = 0x15;
	//Flash有效标志
	SetSysParamBuf[6] = 0x55;
	SetSysParamBuf[7] = 0xAA;

	/*nLen = m_MacAddr.GetLength();
	if (nLen != 12)
	{
		strTemp="MAC不足12位";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		return;
	}*/
	nLen = m_SetMAC.GetLength();
	if (nLen != 12)
	{
		strTemp="MAC应该为12位";
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
	//校验
	SetSysParamBuf[118]=0x00;
	SetSysParamBuf[119]=CalcBcc_710A((char *)(SetSysParamBuf)+2,116);
	SetSysParamBuf[120]=0xEE;
	SetSysParamBuf[121]=0xEE;
	g_nNetSendBuffLen=122;
	memcpy(g_nNetSendBuff,SetSysParamBuf,g_nNetSendBuffLen);
	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="发送设置参数指令";
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
		strTemp = "查询参数回复有误:";
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

		strTemp = "查询设备参数成功";
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
	strTemp="发送查询设备MAC参数";
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
	strTemp="发送查询APD参数指令";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}


//void CLaseAutoCorrectDlg::H712_GetState()   //712获取当前APD温度
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
//	strTemp="发送查询APD参数指令";
//	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
//}

void CLaseAutoCorrectDlg::H712_GetAPD()   //712获取APD
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
	strTemp="发送查询APD参数指令";
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
	strTemp="发送查询系统参数指令";
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
	strTemp="发送设置APD参数指令";
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
	strTemp="发送设置APD参数指令";
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
	//	strTemp="MAC应该为12位";
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
	if (bAutoCorrTerm==TRUE)    //开启一键修正
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
	strTemp="发送设置功能参数指令";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	//m_SetMAC="";
	GetDlgItem(IDC_EDIT_MACSET)->SetWindowText(m_SetMAC);
}
void CLaseAutoCorrectDlg::H715_SetHL()
{
	CString strTemp="";
	char zhiling[43]={0xFF,0xAA,0x00,0x27,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x27,0xEE,0xEE};
	memcpy(&zhiling[26],xitongcanshu,11);
	if (bAutoCorrTerm==TRUE)    //开启一键修正
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
	strTemp="发送设置功能参数指令";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::H715_GetAPD()
{
	char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0xEE,0xEE};
	memcpy(g_nNetSendBuff,zhiling,34);
	g_nNetSendBuffLen=34;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	CString strTemp;
	strTemp="发送查询设备APD参数";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}
void CLaseAutoCorrectDlg::H715_GetHL()//高低阈值
{
	char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1D,0xEE,0xEE};
	memcpy(g_nNetSendBuff,zhiling,34);
	g_nNetSendBuffLen=34;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	CString strTemp;
	strTemp="发送查询设备系统参数";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}
void CLaseAutoCorrectDlg::H715_GetMAC()
{
	char zhiling[34]={};
	memcpy(g_nNetSendBuff,zhiling,34);
	g_nNetSendBuffLen=34;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	CString strTemp;
	strTemp="发送查询设备MAC参数";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::H712_GetMAC()
{
	char zhiling[34]={0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0xEE,0xEE};
	memcpy(g_nNetSendBuff,zhiling,34);
	g_nNetSendBuffLen=34;
	SendMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	CString strTemp;
	strTemp="发送查询设备MAC参数";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::OnEnChangeEditIntevnum()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	//UpdateData(TRUE);
	CString strTemp="";
	CString str="";
	GetDlgItem(IDC_EDIT_IntevNum)->GetWindowTextA(str);
	m_IntervNum=_ttoi(str);
	if (m_IntervNum <= 0)
	{ 
		strTemp="间隔包为0改为默认5";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		m_IntervNum = 5;
	}		
	UpdateData(FALSE);
	//GetDlgItem(IDC_EDIT_IntevNum)->SetWindowTextA((LPCTSTR)m_IntervNum);

}

void CLaseAutoCorrectDlg::OnEnChangeEditChartnum()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	//UpdateData(TRUE);
	CString strTemp="";
	CString str="";
	GetDlgItem(IDC_EDIT_ChartNum)->GetWindowTextA(str);
	m_ChartNum=_ttoi(str);

	if (m_ChartNum <= 0)
	{
		m_ChartNum = 100;
		strTemp="屏显包为0改为默认100";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);		
	}
	UpdateData(FALSE);
	//GetDlgItem(IDC_EDIT_ChartNum)->SetWindowTextA((LPCTSTR)m_ChartNum);
	m_prop1.m_ChartReal.GetAxis().GetBottom().SetMinMax(0,m_ChartNum*300);
}

void CLaseAutoCorrectDlg::OnEnChangeEditMkcalnum()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。

	// TODO:  在此添加控件通知处理程序代码
	//UpdateData(TRUE);
	CString strTemp="";
	CString str="";
	GetDlgItem(IDC_EDIT_MkCalNum)->GetWindowTextA(str);
	m_MkCalNum=_ttoi(str);

	if (m_MkCalNum <= 0)
	{
		m_MkCalNum = 100;
		strTemp="脉宽计算包改为默认100";
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
		H712_SetAPD();//设置APD参数;
	}
	return 0L;
}

void CLaseAutoCorrectDlg::OnCreateFolder(CString fileName)  //创建文件夹
{
	// TODO: Add your command handler code here
	CString path;
	GetModuleFileName(NULL,path.GetBufferSetLength(MAX_PATH+1),MAX_PATH);
	path.ReleaseBuffer();
	int pos = path.ReverseFind('\\');
	path = path.Left(pos);
	m_SavePath=path + _T("\\") + "自动修正结果";
	if(!PathIsDirectory(m_SavePath))
	{
		BOOL bRet = CreateDirectory(m_SavePath, NULL);//创建文件夹
	}
	m_SavePath=m_SavePath + _T("\\") + fileName;
	if(!PathIsDirectory(m_SavePath))
	{
		BOOL bRet = CreateDirectory(m_SavePath, NULL);//创建文件夹
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
	g_AcqTartetInfo.Step[nTmp]=2000;        //步数系数
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

LRESULT CLaseAutoCorrectDlg::OnGlClsoe(WPARAM wParam, LPARAM lParam)  //确定指令后修改
{
	// TODO: 在此添加控件通知处理程序代码
	CString strTemp="";
	if (Com_Send(a_ucTemp10,6))
	{
		strTemp="发送光阑快速关闭指令成功";
	}
	else
	{
		strTemp="发送光阑快速关闭指令失败";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	if (bAutoCorrTerm==TRUE)
	{
		if (g_nComComandStat != 3)    //调速的时候，发送关闭，无需重新开始计时
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
		strTemp="发送停止指令成功";
	}
	else
	{
		strTemp="发送停止指令失败";
	}
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	if (bAutoCorrTerm==TRUE)
	{
		g_nComComandStat=2;
		StartTimer(TIMER712COMSEND,TIMER_712COMSEND,500);
	}
	return 0;
}

LRESULT CLaseAutoCorrectDlg::OnGlOpen(WPARAM wParam, LPARAM lParam)   //确定指令后修改
{
	// TODO: 在此添加控件通知处理程序代码
	CString strTemp="";
	if (Com_Send(a_ucTemp11,6))
	{
		strTemp="发送光阑快速张开指令成功";
	}
	else
	{
		strTemp="发送光阑快速张开指令失败";
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
		g_bMoveFlag=true; //712表示开始运动
		StartTimer(TIMERGLADJUST,TIMER_GLADJUST,500);
	}
	return 0L;
}

LRESULT CLaseAutoCorrectDlg::OnGlAlarm (WPARAM wParam, LPARAM lParam)
{
	// TODO: 在此添加控件通知处理程序代码
	CString strTemp="";
	if (Com_Send(a_ucTemp8,12))
	{
		strTemp="发送报警指令成功";
	}
	else
	{
		strTemp="发送报警指令失败";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	return 0L;
}

//保存原始采样数据
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

void CLaseAutoCorrectDlg::OnCreateFile(CString fileName="NOMAC_原始数据",int nType=0)  //创建文件
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
	
	if(file.Open(Fpath,CFile::modeCreate|CFile::modeWrite))//建立文件
	{
		file.Close();  //关闭文件
	}
}

int CLaseAutoCorrectDlg::FitPro(CString TxtName="NOMAC_修正数据")
{
	//拟合插值成功返回1  否则返回0
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
	int m_end=500000;   //实际从450000开始修的

	////修正数据前面增加温度信息
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
			strTemp="插值数据源不正确";
			::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);	
			PostNcDestroy();
			return 0;
		}
	}

	//保存文件
	m_FitDataPath=m_SavePath+"\\"+TxtName;
	pfile = sfile.Open(m_FitDataPath,CFile::modeReadWrite|CFile::modeCreate);
	if (pfile == NULL)
	{
		strTemp="打开保存文件失败";
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
		return 0;
	}
	sfile.SeekToEnd();
	sfile.Write(FinalTxt,FinalTxt.GetLength());
	sfile.Close();
	//截图
	CString OpenFileName="";
	CString getDir="" ;
	//OpenFileName=m_MacAddr+"拟合图.jpg";

	if(g_nameadd)
	{
		OpenFileName=m_MacAddr+"拟合图("+g_cover+").jpg";
	}
	else
	{
		OpenFileName=m_MacAddr+"拟合图.jpg";
	}
		
	getDir= m_SavePath + "\\"+ OpenFileName;
	m_prop2.m_ChartFit.GetExport().SaveToBitmapFile(getDir);
	return 1;
}

UINT DownLoadCorrectTableThread(LPVOID lpParam)
{
	bccW_packs=0;
	CLaseAutoCorrectDlg* pDlg = (CLaseAutoCorrectDlg*)lpParam;
    const int nFrameLen = 1036;//帧长
	const int nDataLen = 1024; //数据长度
	const int nMaxFileData = 512*32;//文件数据最大个数
	
	int /*i,j,*/k=0;  
	int nIndex1 = 0;
	int nIndex2 = 0;
//     DWORD nLen;
	DWORD dw ;
	CString hustring;
	int packetlen;//udp包的长度
	int cfnum=0;//单包重发次数
	int qbcfnum=0;//从头全部重发
	bool PackageFinishFalg=false;
	char psenddata[2000];
	int sendnum =1;
	CString str;
	CString strTemp="";
	bool cfFlag=false;
	DWORD biao;

	//测试用
	//pDlg->m_FitDataPath="F:\\0727\\万集\\Code\\（v7）LaseAutoCorrect-20171106\\Debug\\自动修正结果\\F8-B5-68-90-01-67\\F8-B5-68-90-01-67_高阈值拟合数据.txt_H";
	//pDlg->m_FitDataPath="E:\\资料\\My Task 45.导航激光雷达\\脉宽修正上位机\\（v7不连续）LaseAutoCorrect-20180808\\Debug\\自动修正结果\\3\\001234000004_修正数据.txt";
	
	if (pDlg->m_FitDataPath == "")
	{
		strTemp="修正表数据不存在";
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
		//pDlg->MessageBox("文件中无数据！",MB_OK);
		strTemp="修正文件中无数据！";
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
		//如果是重发，则不重新读取数据，主要是总校验bccW_packs不会继续校验
		if (cfFlag == false)
		{
			packetlen = SetPacket712(psenddata,g_nReadbuf,k,sendnum);//从第1包发送
		}	
		if (packetlen != 0)  //发送
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
			//break;  //发送完 结束
			//烧写数据包
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

			dw = WaitForSingleObject(g_eventdl,5000); //等事件  调试用 INFINITE 
			ResetEvent(g_eventdl);
			switch(dw)
			{
			case WAIT_OBJECT_0:
				//等到事件			
				if (g_flag == TRUE )  //发送成功
				{
					g_flag = FALSE;
					cfnum = 0;
					//返回继续下一步
					AutoCorretRegInfo.nDownTbCode=1;
					SetEvent(Event_DOWNTB);
					return 0;
				}
				else                      //发送失败
				{
					if (qbcfnum>2)
					{
						strTemp="修正表烧写失败，请手动完成后续步骤！";
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
					strTemp="烧写包指令响应失败，开始重新下载！";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					continue;
				}			
				break;
			case WAIT_TIMEOUT:				
			case WAIT_FAILED:
				if (qbcfnum>2)
				{
					strTemp="修正表烧写失败，请手动完成后续步骤！";
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
				strTemp="烧写包指令响应失败，开始重新下载！";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				continue;
			}
		}
		dw = WaitForSingleObject(g_eventdl,5000); //等事件  调试用 INFINITE 
		ResetEvent(g_eventdl);
		
		switch(dw)
		{
			case WAIT_OBJECT_0:
				//等到事件			
					if (g_flag == TRUE && g_nNum == sendnum)  //发送成功
					{
						PackageFinishFalg=true;
						sendnum ++;
						g_flag = FALSE;
						cfnum = 0;
						cfFlag = false;
						//qbcfnum = 0;
					}
					else                      //发送失败
					{
						str.Format("%d",g_nNum);
						strTemp="第" + str +"包数据接收失败，重发此包！";
						::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					}			
				break;
			case WAIT_TIMEOUT:				
				//超时未收到回复			
				str.Format("%d",sendnum);
				strTemp="超时未收到第"+str+"包接收回复，重发此包！";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				break;
			case WAIT_FAILED:
				str.Format("%d",sendnum);
				strTemp="第"+str+"包发送出错，重发此包！";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				break;
	     }	
		 if (PackageFinishFalg == false)
		 {
			if (cfnum > 2)
			{
				if (qbcfnum>2)
				{
					strTemp="修正表下载失败，请手动完成后续步骤！";
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
				strTemp="第" + str +"次下载失败，开始重新下载！";
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

	//strTemp="数据烧写指令发送成功";
	//::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

	//AutoCorretRegInfo.nDownTbCode=1;
	//SetEvent(Event_DOWNTB);

	return 0L;
}

int SetPacket712(char* sendpacket,int* datapacket,int len,int num)  //源数据的长度 包号
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
	int j = 512 * (num - 1);//包号从0开始
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

	unsigned char zhiling[26] = { 0xFF, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, Hour, Miu, Sec, 0x01, 0x01, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x0C, 0x02, 0x00 };//低阈值
	memcpy(acTmp, zhiling, 26);

	acTmp[26] = ((char)plen >> 8);    //总包数
	acTmp[27] = ((char)plen & 0xff);
	acTmp[28] = ((char)num >> 8);//包号
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
	acTmp[30] = (char)(j >> 8);//本包数据长度
	acTmp[31] = (char)(j & 0xff);
	int lenth = j + 32;
	acTmp[2] = (char)(lenth >> 8);
	acTmp[3] = (char)(lenth & 0xff);
	memcpy(sendpacket, acTmp, (j + 36));
	return (j + 36);
} 

void CLaseAutoCorrectDlg::InitAutoCorrectTask(void)
{

	//初始化信号量
	ResetEvents();
	//初始化测试量
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
	CStatic* pWnd = (CStatic*)GetDlgItem(IDC_StepImg); // 得到 Picture Control 句柄
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
	//ScreenToClient(&rtlbl); //转到客户端界面
	//InvalidateRect(&rtlbl);//最后刷新对话框背景 
	//SetDlgItemTextA(IDC_STATIC_TIPS,Tips);
	GetDlgItem(IDC_STATIC_TIPS)->SetWindowTextA(Tips);
}
/////////////////////////////////////////////////////////////////////////
/// 从资源文件中加载图片
/// @param [in] pImage 图片指针
/// @param [in] nResID 资源号
/// @param [in] lpTyp 资源类型
//////////////////////////////////////////////////////////////////////////
bool CLaseAutoCorrectDlg::LoadImageFromResource(IN CImage* pImage,IN UINT nResID, IN LPCSTR lpTyp)
{
	if ( pImage == NULL) 
		return false;

	pImage->Destroy();

	// 查找资源
	HRSRC hRsrc = ::FindResource(AfxGetResourceHandle(), MAKEINTRESOURCE(nResID), lpTyp);
	if (hRsrc == NULL) return false;

	// 加载资源
	HGLOBAL hImgData = ::LoadResource(AfxGetResourceHandle(), hRsrc);
	if (hImgData == NULL)
	{
		::FreeResource(hImgData);
		return false;
	}

	// 锁定内存中的指定资源
	LPVOID lpVoid    = ::LockResource(hImgData);

	LPSTREAM pStream = NULL;
	DWORD dwSize    = ::SizeofResource(AfxGetResourceHandle(), hRsrc);
	HGLOBAL hNew    = ::GlobalAlloc(GHND, dwSize);
	LPBYTE lpByte    = (LPBYTE)::GlobalLock(hNew);
	::memcpy(lpByte, lpVoid, dwSize);

	// 解除内存中的指定资源
	::GlobalUnlock(hNew);

	// 从指定内存创建流对象
	HRESULT ht = ::CreateStreamOnHGlobal(hNew, TRUE, &pStream);
	if ( ht != S_OK )
	{
		GlobalFree(hNew);
	}
	else
	{
		// 加载图片
		pImage->Load(pStream);

		GlobalFree(hNew);
	}

	// 释放资源
	::FreeResource(hImgData);

	return true;
}

HBRUSH CLaseAutoCorrectDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor)
{
	HBRUSH hbr = CDialogEx::OnCtlColor(pDC, pWnd, nCtlColor);

	// TODO:  在此更改 DC 的任何特性
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
	// TODO:  如果默认的不是所需画笔，则返回另一个画笔
	return hbr;
}

CString FilePathOper = "";
CFile FileOper;
void CLaseAutoCorrectDlg::RecordVerifyMK(int CurMK)
{
	CString Fpath = m_SavePath + "\\"+m_MacAddr + "验证脉宽.txt";
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
	FileOper.Write(strLog.GetBuffer(), strLog.GetLength() * sizeof(TCHAR));//str为CString类型
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
			//计算脉宽数据
			if (g_SysDevParam.LaserType==6)//||g_SysDevParam.LaserType==4||g_SysDevParam.LaserType==5)   //710,710B,710C  712
			{
				if (g_bCountStart!=3)//((g_SysDevParam.LaserType==6||g_SysDevParam.LaserType==4) && g_bCountStart!=3)
				{
					//计算第一路数据的脉宽均值
					nAvgMk1=pDlg->UDP_360DataSum711(g_nJGdata[0][g_nJGProCnt],28);
					//计算第二路数据的脉宽均值
					nAvgMk2=pDlg->UDP_360DataSum711(g_nJGdata[1][g_nJGProCnt],28);
					//计算第三路数据的脉宽均值
					//nAvgMk3=pDlg->UDP_360DataSum711(g_nJGdata[2][g_nJGProCnt],8,1);
				}
				else
				{
					//计算第一路数据的脉宽均值
					nAvgMk1=pDlg->UDP_DataSum711(g_nJGdata[0][g_nJGProCnt],28);
					//计算第二路数据的脉宽均值
					nAvgMk2=pDlg->UDP_DataSum711(g_nJGdata[1][g_nJGProCnt],28);
					//计算第三路数据的脉宽均值
					nAvgMk3=pDlg->UDP_DataSum711(g_nJGdata[2][g_nJGProCnt],28);
				}			
			}
			else if(g_SysDevParam.LaserType==1)
			{
				    //计算第一路数据的脉宽均值
					nAvgMk1=pDlg->UDP_DataSum(g_nJGdata[0][g_nJGProCnt],8,1);
					//计算第二路数据的脉宽均值
					nAvgMk2=pDlg->UDP_DataSum(g_nJGdata[1][g_nJGProCnt],8,1);
					//计算第三路数据的脉宽均值
					nAvgMk3=pDlg->UDP_DataSum(g_nJGdata[2][g_nJGProCnt],8,1);
			}
			else if (g_SysDevParam.LaserType==2)    //710A
			{
				//计算第一路数据的脉宽均值
				nAvgMk1=pDlg->UDP_DataSum(g_nJGdata[0][g_nJGProCnt],9,2);   //上升沿
				//计算第二路数据的脉宽均值
				nAvgMk2=pDlg->UDP_DataSum(g_nJGdata[1][g_nJGProCnt],9,2);   //下降沿
				//计算第三路数据的脉宽均值
				nAvgMk3=pDlg->UDP_DataSum(g_nJGdata[2][g_nJGProCnt],9,2);   //修正后的上升沿
			}
				
			//成组计算
			pDlg->m_MkAvg = nAvgMk2 - nAvgMk1;   //真实脉宽值，非修正后脉宽
			strTmp.Format("%d",pDlg->m_MkAvg);
			pDlg->GetDlgItem(IDC_EDIT_MkAvg)->SetWindowTextA((LPCTSTR)strTmp);
			//判断是否达到450ns
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

			if (g_bCountStart==1)//进行脉宽调整
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
					//判断是否达到450ns
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
						//PID调节
						g_Pid.feedback=nClacMk;
						g_Pid.e_0=g_Pid.target-g_Pid.feedback;
						//达到误差要求 则记录
						if ((g_Pid.e_0<(g_AcqTartetInfo.DevValue[g_AcqTartetInfo.AqcIndx]))&&(g_Pid.e_0>(-g_AcqTartetInfo.DevValue[g_AcqTartetInfo.AqcIndx])))
						{
							AutoCorretRegInfo.nAqcMkCode=4;
							SetEvent(Event_AQCMK);
						}
						else
						{
							//然后根据误差来决定调整的方向和步数
							strTemp.Format("当前脉宽值%d",nClacMk);
							::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
							AutoCorretRegInfo.nAqcMkCode=2;
							SetEvent(Event_AQCMK);
						}
					}
				}
			}
			else if (g_bCountStart==2)//进行计算
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

						strTemp.Format("当前脉宽值%d,前沿差值%d",nClacMk,nSglThred);
						::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					}
				}
			}
			else if (g_bCountStart==3)//一键验证时
			{
				if (AutoCorretRegInfo.nStepIndx==STEP_FINALCH)
				{
					nClacMk=pDlg->m_MkAvg;
					//pDlg->RecordVerifyMK(nClacMk);//记录测试使用到的脉宽值 pl 0402
					if(g_bCloseSpeedChange == true)
					{
						int tempClacMk = nClacMk / 1000;
						if (tempClacMk < 320 && tempClacMk >= 250)//如果脉宽小于350ns则减慢关闭速度
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
							::SendMessageA(pDlg->GetSafeHwnd(), WM_GL_CLOSE, 0, 0);    //重新发一遍关闭
						}
						if (tempClacMk < 250 && tempClacMk >= 100)//如果脉宽小于300ns则减慢关闭速度
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
							::SendMessageA(pDlg->GetSafeHwnd(), WM_GL_CLOSE, 0, 0);    //重新发一遍关闭
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
							::SendMessageA(pDlg->GetSafeHwnd(), WM_GL_CLOSE, 0, 0);    //重新发一遍关闭
						}
					}
					if (nClacMk/1000>g_CheckMKForm.ChMkRegionMin)
					{	
						if(g_CheckMKForm.ReferenceValue!=0 )//&& g_CheckMKForm.ValueJudge!=1)
						{
							for (int i=0;i<300;i++)
							{
								SingleValueErrorTemp=g_CheckMKForm.ReferenceValue-g_nCurMKValue[i];//g_nCurMKValue中保存的是计时值
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
									//测试用
									cs=g_nRevcNumcCheck+1;																
									strTemp.Format("出错时脉宽均值%d 理论计算值为%d 检查的第%d帧第%d点值为%d 与理论值之差为%d 已错误点数为%d",nClacMk,g_CheckMKForm.ReferenceValue,cs,i,g_nCurMKValue[i],SingleValueErrorTemp,g_CheckMKForm.ValueJudgeCnt);
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
						//改为用参考值取正负125的区间判断，原来为直接判断离散度再250以内
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
							strTemp.Format("出错时脉宽均值%d 理论计算值为%d 检查的第%d帧 与理论值之差为%d 累计不合格%d帧",nClacMk,g_CheckMKForm.ReferenceValue,cs,AVEValueErrorTemp,g_CheckMKForm.AveValueJudgeCnt);
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
								//测试用
								strTemp="均值的偏差过大";
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
			//处理索引加1
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
		DispTips(IDB_PNG0,"空闲状态");
		break;
	case 1:
		DispTips(IDB_PNG1,"任务启动阶段");
		break;
	case 2:
		DispTips(IDB_PNG2,"参数检查阶段");
		break;
	case 3:
		DispTips(IDB_PNG3,"初脉宽检查阶段");
		break;
	case 4:
		DispTips(IDB_PNG4,"原始数据采集阶段");
		break;
	case 5:
		DispTips(IDB_PNG5,"拟合插值阶段");
		break;
	case 6:
		DispTips(IDB_PNG6,"下载修正表阶段");
		break;
	case 7:
		DispTips(IDB_PNG7,"验证修正表下载阶段");
		break;
	case 8:
		DispTips(IDB_PNG8,"验证修正结果阶段");
		break;
	case 9:
		DispTips(IDB_PNG9,"修正任务结束");
		break;
	case 10:
		DispTips(IDB_PNGER,"自动修正结果未通过");
		break;
	case 11:
		DispTips(IDB_PNGOK,"自动修正结果通过");
		break;
	default:

		break;
	}
	return 0L;
}

void CLaseAutoCorrectDlg::H02_Restart88(void)//重启激光主控板
{
	char Sendbuff[26] = {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x11,0x73,0x88,0x4e,0x20,0x4c,0x4d,0x44,0x73,0x63,0x61,0x6e,0x64,0x61,0x74,0x61,0x20,0x00};
	int SendbuffLen = 25;
	CString strTemp="";
	Add_BCC(Sendbuff,SendbuffLen);
	g_nNetSendBuffLen=SendbuffLen+1;
	memcpy(g_nNetSendBuff,Sendbuff,g_nNetSendBuffLen);
	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="发送重启激光器主控板";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::H02_Restart47(void)//重启711激光主控板
{
	char Sendbuff[26] = {(char)0x02, (char)0x02, (char)0x02, (char)0x02, (char)0x00, (char)0x00, (char)0x00, (char)0x11, (char)0x73,(char)0x47, (char)0x4E, 
		(char)0x20, (char)0x4C, (char)0x4D, (char)0x44, (char)0x73, (char)0x63, (char)0x61, (char)0x6E, (char)0x64, (char)0x61, (char)0x74, (char)0x61, (char)0x20, (char)0x00};
	int SendbuffLen = 25;
	CString strTemp="";
	Add_BCC(Sendbuff,SendbuffLen);
	g_nNetSendBuffLen=SendbuffLen+1;
	memcpy(g_nNetSendBuff,Sendbuff,g_nNetSendBuffLen);
	PostMessage(WM_NET_SEND,g_nNetSendBuffLen,(LPARAM)g_nNetSendBuff);
	strTemp="发送重启激光器主控板";
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

	if (g_SysDevParam.LaserPor == 1)//TCP协议
	{
		if (!SendData(&m_NetConnection))
		{
			MessageBox("网络有故障，发送失败！",MB_OK);
		}
	}
	else if (g_SysDevParam.LaserPor == 0)//UDP协议
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
//检查修正表下载
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

	if (g_SysDevParam.LaserPor == 1)//TCP协议
	{
		if (!SendData(&m_NetConnection))
		{
			MessageBox("网络有故障，发送失败！",MB_OK);
		}
	}
	else if (g_SysDevParam.LaserPor == 0)//UDP协议
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

//切换选项卡
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
			OpenFileName=m_MacAddr+"不合格("+g_cover+").jpg";
		}
		else
		{
			OpenFileName=m_MacAddr+"不合格.jpg";
		}
	}
	else
	{
		if(g_nameadd)
		{
			OpenFileName=m_MacAddr+"合格("+g_cover+").jpg";
		}else{
			OpenFileName=m_MacAddr+"合格.jpg";
		}
	}
	
	getDir= m_SavePath + "\\"+ OpenFileName;
	//getDir="C:\\Users\\WANJI\\Desktop\\（V2）LaseAutoCorrect-20161020\\Debug\\自动修正结果\\1.jpg";
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
	// TODO: 在此添加控件通知处理程序代码
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

	//校验位 第一位
	NetContentBuff[Tmplen++]=0x00;

	//长度位
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

void CLaseAutoCorrectDlg::Get711LmdSinglePoint()     //包括712
{
	// TODO: 在此添加控件通知处理程序代码
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
	strTemp="发送获取单帧数据";
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

void CLaseAutoCorrectDlg::HFA_Restart0C(void)//重启激光主控板
{
	// TODO: 在此添加控件通知处理程序代码
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

	//校验位 第一位
	NetContentBuff[Tmplen++]=0x00;

	//长度位
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
	strTemp="发送重启激光器主控板";
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::SendCheckCorrectTb710A(void)
{
	// TODO: 在此添加控件通知处理程序代码
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

	//校验位 第一位
	NetContentBuff[Tmplen++]=0x00;

	//长度位
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
	// TODO: 在此添加控件通知处理程序代码
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
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("当前MAC地址");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("设置MAC地址");
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
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("当前MAC地址");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("设置MAC地址");
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
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("当前设备号");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("通道选择");
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
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("当前设备号");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("通道选择");
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
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("当前设备号");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("通道选择");
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
		GetDlgItem(IDC_STATIC_MACGET)->SetWindowText("当前MAC地址");
		GetDlgItem(IDC_STATIC_MACSET)->SetWindowText("设置MAC地址");
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
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(true);
	CWinThread *pThread;
	g_ChannelSelect = m_ChannelSelect.GetCurSel();
	MK_ORG0=450;
	MK_ORG1=500;

	if (g_SysDevParam.NetConnected&&g_SysDevParam.ComConnected)
	{
		//初始化相关变量
		InitAutoCorrectTask();
		//创建修正线程
		bAutoCorrTerm=TRUE;
		pThread = ::AfxBeginThread(AutoCheckThreadProc,this);//创建连接线程
		if(pThread == NULL)
		{
			bAutoCorrTerm=FALSE;
			return;
		}
		SetEvent(Event_START);
	}
}

UINT AutoCheckThreadProc(LPVOID lpParam)  //验证测试用
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
		nRet = WaitForMultipleObjects(11,Event_List,false,600*1000);	//等待超时

		if(WAIT_TIMEOUT == nRet)	//等待事件超时时间到
		{
			strTemp="自动修正超时终止";
			//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,0);
			::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
			SetEvent(Event_STOP);
		}
		else if(WAIT_OBJECT_0 == nRet)	//Event_START 处于有信号状态 开启自动修正任务
		{
			AutoCorretRegInfo.nStepIndx=STEP_PARAM;
			::PostMessageA(pDlg->GetSafeHwnd(),WM_LASER_GETPARAM,0,0);
			pDlg->StartTimer(TIMERGETPARMA,TIMER_GETPARAM,5000);
		}
		else if((WAIT_OBJECT_0 + 1) == nRet)	//Event_PARAM有信号 需进行初始参数检查
		{
			if (AutoCorretRegInfo.nParamChCode==1)
			{
				strTemp="检查参数正常通过";
				//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

				SetEvent(Even_ORIGMK);				
			}
			else if (AutoCorretRegInfo.nParamChCode==2)
			{
				strTemp="参数检查不达要求";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
				SetEvent(Event_STOP);
			}
			else if (AutoCorretRegInfo.nParamChCode==3)
			{
				strTemp="参数无法获取！";
				//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
				SetEvent(Event_STOP);
			}
			else if (AutoCorretRegInfo.nParamChCode==4)
			{
				//创建保存文件夹及文件
				pDlg->OnCreateFolder(pDlg->m_MacAddr);
				CString name="拟合图";
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
					//pDlg->OnCreateFile(pDlg->m_MacAddr+"_原始数据"+"("+g_cover+").txt",1);
					pDlg->OnCreateFile(pDlg->m_MacAddr+"_测试记录"+"("+g_cover+").txt",2);
				}
				else
				{
					//pDlg->OnCreateFile(pDlg->m_MacAddr+"_原始数据.txt",1);
					pDlg->OnCreateFile(pDlg->m_MacAddr+"_测试记录.txt",2);
				}
			
			
				m_pMyLog = CCDMLogInfo::GetInstance(pDlg->m_OperTxtPath.GetBuffer(pDlg->m_OperTxtPath.GetLength()));//获取日志文件
				strTemp=  "\r\n========" + CTime::GetCurrentTime().Format("%H:%M:%S") + "========\r\n";
				if(m_pMyLog)	//写入日志
					m_pMyLog->SetNotify(strTemp.GetBuffer(strTemp.GetLength()));
				strTemp="创建数据保存文件";
				//pDlg->WriteLog(&(pDlg->m_RichEdit),pDlg->m_sLog);
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				
				AutoCorretRegInfo.nWaitInputDistCnt=0;
				strTmp="请输入距离信息，然后点击[距离确认]按钮";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				AfxMessageBox(_T("请输入距离信息，然后点击[距离确认]按钮"),MB_OK);
				pDlg->StartTimer(TIMERINPUTDIST,TIMER_INPUTDIST,5000);
			}
		}
		else if ((WAIT_OBJECT_0 + 2) == nRet)//Even_ORIGMK有信号 开始初始脉宽检查  712为最大值检查450ns
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_PARAM)
			{
				//判断最大脉宽是否为450ns
				strTemp="判断最大脉宽是否为450ns";
				::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
				AutoCorretRegInfo.nStepIndx=STEP_MAXMK;
				g_bCountStart=1;   //开始统计是否达到450ns
			}
			else if (AutoCorretRegInfo.nStepIndx==STEP_MAXMK)
			{
				if (AutoCorretRegInfo.nOrigMkChCode==2)
				{
					g_bOpenflag=true;  //开始实时判断张开何时停止
					strTemp="脉宽最大值未到450ns，自动张开光阑";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_OPEN,0,0);
				}
				else if (AutoCorretRegInfo.nOrigMkChCode==1)
				{
					strTemp="脉宽最大值ok";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					AutoCorretRegInfo.nStepIndx=STEP_CHECKTB;
					SetEvent(Event_FINALCK);
				}
				else if(AutoCorretRegInfo.nOrigMkChCode==3)
				{
					strTemp="脉宽最大值无法达到450ns";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
				else if(AutoCorretRegInfo.nOrigMkChCode==4)
				{
					//识别达到450ns，停止张开
					strTemp="识别达到450ns，停止张开";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);
					//开始统计是否达到450ns,放在停止响应后修改标志位
				}
			}
		}
		else if((WAIT_OBJECT_0 + 8) == nRet)//Event_FINALCK 最后一键检查修正是否达标
		{
			if (AutoCorretRegInfo.nStepIndx==STEP_CHECKTB)
			{
				AutoCorretRegInfo.nStepIndx=STEP_FINALCH;
				strTemp="开始验证修正效果";
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
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //停止继续关闭光阑
					}
					//测试用
					strTemp.Format("出错点数共%d 检查的共%d帧 接收的共%d(%d)(%d)帧 均值偏差的最大最小值为%d,%d 单点误差最大值最小值为%d,%d",g_CheckMKForm.ValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkMax,g_CheckMKForm.ChMkMin,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,11);
					strTemp="修正后一键验证通过";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
				//	Sleep(10000);
					//712验证通过，张开光阑
					//::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_OPEN,0,0);
					//pDlg->OnOnekeyTestFinish();
					SetEvent(Event_OVEROPEN);
				}
				else if (AutoCorretRegInfo.nFinalCkCode==2)
				{		
					if (g_bCloseflag == true)
					{
						g_bCloseflag = false;
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //停止继续关闭光阑
					}
					//测试用
					//strTemp.Format("出错点数共%d 检查的共%d帧 接收的共%d(%d)(%d)帧 均值最大和最小之差为%d 单点误差最大值最小值为%d,%d",g_CheckMKForm.ValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkOffset,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					strTemp.Format("出错点数共%d 不合格帧数共%d 检查的共%d帧 接收的共%d(%d)(%d)帧 均值偏差的最大最小值为%d,%d 单点误差最大值最小值为%d,%d",g_CheckMKForm.ValueJudgeCnt,g_CheckMKForm.AveValueJudgeCnt,g_nRevcNumcCheck,g_nRevcNumcCheck1,g_nRevcNumcCheck2,g_nRevcNum3,g_CheckMKForm.ChMkMax,g_CheckMKForm.ChMkMin,g_CheckMKForm.SingleErrorMAX,g_CheckMKForm.SingleErrorMIN);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,10);
					strTemp="修正后一键验证未过";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					//712验证结束，张开光阑
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
					strTemp.Format("验证标准为：均值误差±%d，单次误差±%d,检测脉宽范围最小值%d",g_CheckMKForm.AveValueError,g_CheckMKForm.SingleValueError,g_CheckMKForm.ChMkRegionMin);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					g_CheckMKForm.ChMkRegionMax=450;
					g_CheckMKForm.ReferenceValue=g_nReferUpEdge;
		

					g_nRevcNumcCheck=0;
					g_nRevcNumcCheck2=0;			
					g_CheckMKForm.ValueJudgeCnt=0;
					g_CheckMKForm.ChMkMin=999999;
					g_CheckMKForm.SingleErrorMIN=999999;
					//712开始闭合验证
					g_bCloseflag=true;  //开始实时判断闭合何时停止
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_CLOSE,0,0);
					g_bCloseSpeedChange = true;  //用于判断何时降低关闭速度
				}
				else if (AutoCorretRegInfo.nFinalCkCode==4)   //降低关闭速度
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
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_CLOSE,0,0);    //重新发一遍关闭
				}
				else if (AutoCorretRegInfo.nFinalCkCode==5)    
				{		
					if (g_bCloseflag == true)
					{
						g_bCloseflag = false;
						::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_STOPMOVE,0,0);   //停止继续关闭光阑
					}
					
					//测试用
					strTemp="关闭超时，重新验证！";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);

					::SendMessageA(pDlg->GetSafeHwnd(),WM_IMG_CHANGE,0,10);
					strTemp="修正后一键验证未过";
					::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_CUT_PICTURE,0,0);
					::SendMessageA(pDlg->GetSafeHwnd(),WM_GL_ALARM,0,0);
					SetEvent(Event_STOP);
				}
			}
		}
		else if((WAIT_OBJECT_0 + 9) == nRet)//Event_OVEROPEN 完成自动修正前张开光阑
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
		else if((WAIT_OBJECT_0 + 10) == nRet)//Event_STOP 停止修正任务
		{
			switch(AutoCorretRegInfo.nStepIndx)
			{
			case STEP_FINALCH:
				//屏显和绘画初始化
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
				pDlg->GetDlgItem(IDC_BUTTON_AutoCorrect)->SetWindowTextA("开启一键修正");
				if (AutoCorretRegInfo.nFinalCkCode==1)
				{
					strTmp="自动修正任务完成";
					AfxMessageBox(_T("自动修正任务成功结束:[结果通过]"),MB_OK);
					//pDlg->OnBnClickedButton2();
					//写入数据库
					if(g_SysDevParam.LaserType==3||g_SysDevParam.LaserType==4)   //711E、711A
					{
						CWinThread *pThread;
						pThread = ::AfxBeginThread(WriteData,lpParam);
					}
				}
				else
				{
					strTmp="自动修正任务终止";
					AfxMessageBox(_T("自动修正任务终止:[结果失败]"),MB_OK|MB_ICONSTOP);
				}
				if(m_pMyLog)	//写入日志
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
	// TODO: 在此添加控件通知处理程序代码
	CString strTemp="";
	if (Com_Send(a_ucTemp12,6))
	{
		strTemp="发送停止指令成功";
	}
	else
	{
		strTemp="发送停止指令失败";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}

void CLaseAutoCorrectDlg::OnBnClickedButtonDistconfirm()
{
	// TODO: 在此添加控件通知处理程序代码
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
		strTemp.Format(" 理论计时值为 %d",g_nReferUpEdge);
		strTemp = "真实距离已确认" + str + strTemp;
		::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
	}	
}

void CLaseAutoCorrectDlg::OnBnClickedButtonSetinispeed()
{
	// TODO: 在此添加控件通知处理程序代码
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
		strTemp="设置初速度指令成功";
	}
	else
	{
		strTemp="设置初速度指令失败";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}


void CLaseAutoCorrectDlg::OnBnClickedButtonSetworkspeed()
{
	// TODO: 在此添加控件通知处理程序代码
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
		strTemp="设置运行速度指令成功";
	}
	else
	{
		strTemp="设置运行速度指令失败";
	}
	//WriteLog(&m_RichEdit,m_sLog);
	::SendMessageA(this->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strTemp);
}


void CLaseAutoCorrectDlg::OnBnClickedButtonGetstate()
{
	// TODO: 在此添加控件通知处理程序代码
	CString strTemp="";
	if (Com_Send(a_ucTemp15,5))
	{
		strTemp="发送获取状态指令成功";
	}
	else
	{
		strTemp="发送获取状态指令失败";
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
	bool OnInitADOConn();//初始化连接
    bool ExecuteSQL(_bstr_t bstrSQL);
};
ADOConn::ADOConn()
{
	CoInitialize(NULL); //初始化必不可少
	HRESULT hr=m_pConnection.CreateInstance(_uuidof(Connection));
	if(FAILED(hr))
	{
	}
}

bool ADOConn::OnInitADOConn()
{
	try
	{	//在COM接口中，参数若为字符串， 一定要用BSTR  
		_bstr_t con_bstr="Driver={sql server};server=192.168.0.103;uid=sa;pwd=wanjikeji_123;database=Laser_Production_Intelligence;";
		m_pConnection->Open(con_bstr,"","",adModeUnknown);// adModeUnknown 缺省，当前的许可权未设置
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
		//AfxMessageBox(_T("数据库写入失败"),MB_OK);
		return false;
	}
	return true;
}

//测试用
void CLaseAutoCorrectDlg::OnBnClickedButton2()
{
	CWinThread *pThread;
	pThread = ::AfxBeginThread(WriteData,this);

	// TODO: 在此添加控件通知处理程序代码
	/*OnOneKeyTestCorrect();
	OnOnekeyTestFinish();
	::SendMessageA(GetSafeHwnd(),WM_GL_CLOSE,0,0);
	::SendMessageA(GetSafeHwnd(),WM_GL_OPEN,0,0);
	::SendMessageA(GetSafeHwnd(),WM_GL_ALARM,0,0);*/

	//INT_PTR nRes = MessageBox(_T("脉宽修正数据是否写入数据库？"),"生产信息化",MB_OKCANCEL|MB_ICONQUESTION);

	//if(nRes == IDCANCEL)
	//{
	//	return;
	//}
	////m_MacAddr="F8B5689000FA";
	////m_SavePath="F:\\0727\\万集\\Code\\（v2）LaseAutoCorrect-20171108-连续 - 高低阈值切换 - 增加711A - 增加数据库\\Debug\\F8B5689000FA";
	//
	//CString MuDipath="/\\服务器-备份\\ProductionProcess\\";
	//CString chn;
	//CString chn2;
	//if (g_ChannelSelect==0)  //高阈值
	//{
	//	chn="H";
	//	chn2="高";
	//}
	//else
	//{
	//	chn="L";
	//	chn2="低";
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
	//if(vCount.intVal<=0)//数据库没有数据
	//{
	//	CString tmp;
	//	CString str="insert into DEV_711E_MKXZ values(";
	//	str.Append("'"+CTime::GetCurrentTime().Format("%Y%m%d%H%M%S")+this->m_MacAddr+"',");
	//	str.Append("'成功',");
	//	if(!PathIsDirectory(MuDipath+this->m_MacAddr))
	//	{
	//		BOOL bRet = CreateDirectory(MuDipath+this->m_MacAddr, NULL);//创建文件夹
	//		MuDipath=MuDipath+this->m_MacAddr+"\\";
	//	}
	//	else
	//	{
	//		MuDipath=MuDipath+this->m_MacAddr+"\\";
	//	}
	//	if(g_nameadd)
	//	{
	//		str.Append("'"+this->m_MacAddr+"_"+chn2+"阈值原始数据"+"("+g_cover+")."+"txt_"+chn+"',");
	//		str.Append("'"+this->m_MacAddr+"_"+chn2+"阈值拟合数据"+"("+g_cover+")."+"txt_"+chn+"',");
	//		str.Append("'"+this->m_MacAddr+"_测试记录"+"("+g_cover+")."+"txt_"+chn+"',");//测试记录
	//		str.Append("'"+this->m_MacAddr+"拟合图_H("+g_cover+").jpg"+"',");
	//		str.Append("'"+this->m_MacAddr+"合格_H("+g_cover+").jpg"+"',");

	//		//CString path;

	//		//path = "/\\LZB\\ProductionProcess\\TDC_LOCK2.rbf";
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"阈值原始数据"+"("+g_cover+")."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"阈值原始数据"+"("+g_cover+")."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"阈值拟合数据"+"("+g_cover+")."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"阈值拟合数据"+"("+g_cover+")."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"阈值拟合数据"+"("+g_cover+")."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"阈值拟合数据"+"("+g_cover+")."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"拟合图_"+chn+"("+g_cover+").jpg", MuDipath+this->m_MacAddr+"拟合图_"+chn+"("+g_cover+").jpg", TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"合格_"+chn+"("+g_cover+").jpg", MuDipath+this->m_MacAddr+"合格_"+chn+"("+g_cover+").jpg", TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_测试记录"+"("+g_cover+")."+"txt_"+chn, MuDipath+this->m_MacAddr+"_测试记录"+"("+g_cover+")."+"txt_"+chn, TRUE);
	//	}
	//	else
	//	{
	//		str.Append("'"+this->m_MacAddr+"_"+chn2+"阈值原始数据."+"txt_"+chn+"',");
	//		str.Append("'"+this->m_MacAddr+"_"+chn2+"阈值拟合数据."+"txt_"+chn+"',");
	//		str.Append("'"+this->m_MacAddr+"_测试记录."+"txt_"+chn+"',");//测试记录
	//		str.Append("'"+this->m_MacAddr+"拟合图_H.jpg"+"',");
	//		str.Append("'"+this->m_MacAddr+"合格_H.jpg"+"',");

	//		//CString path;
	//		//path = "/\\LZB\\ProductionProcess\\TDC_LOCK2.rbf";
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"阈值原始数据."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"阈值原始数据."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"阈值拟合数据."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"阈值拟合数据."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_"+chn2+"阈值拟合数据."+"txt_"+chn, MuDipath+this->m_MacAddr+"_"+chn2+"阈值拟合数据."+"txt_"+chn, TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"拟合图_"+chn+".jpg", MuDipath+this->m_MacAddr+"拟合图_"+chn+".jpg", TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"合格_"+chn+".jpg", MuDipath+this->m_MacAddr+"合格_"+chn+".jpg", TRUE);
	//		CopyFile(m_SavePath+ _T("\\")+this->m_MacAddr+"_测试记录."+"txt_"+chn, MuDipath+this->m_MacAddr+"_测试记录."+"txt_"+chn, TRUE);
	//		//CopyFile("C:\\Users\\wanji\\Desktop\\TDC_LOCK.rbf", "/\\LZB\\ProductionProcess\\TDC_LOCK2.rbf", TRUE);//局域网拷贝
	//	}
	//	tmp.Format("%d",g_CheckMKForm.ValueJudgeCnt);
	//	str.Append("'"+tmp+"',");//出错点数
	//	tmp.Format("%d",g_nRevcNumcCheck);
	//	str.Append("'"+tmp+"',");//检查帧数
	//	tmp.Format("%d",g_nRevcNumcCheck);
	//	str.Append("'"+tmp+"',");//检查帧数
	//	tmp.Format("%d",g_nRevcNumcCheck1);
	//	str.Append("'"+tmp+"(");//接收帧数
	//	tmp.Format("%d",g_nRevcNumcCheck2);
	//	str.Append(tmp+")(");
	//	tmp.Format("%d",g_nRevcNumcCheck3);
	//	str.Append(tmp+")'");
	//	tmp.Format("%d",g_CheckMKForm.ChMkOffset);//均值之差
	//	str.Append("'"+tmp+"',");
	//	tmp.Format("%d",g_CheckMKForm.SingleErrorMAX);//均值单点误差最大值
	//	str.Append("'"+tmp+"',");
	//	tmp.Format("%d",g_CheckMKForm.SingleErrorMIN);//均值单点误差最小值
	//	str.Append("'"+tmp+"',");
	//	str.Append("' ',");//更新人
	//	str.Append("'"+CTime::GetCurrentTime().Format("%Y-%m-%d %H:%M:%S")+"')");//更新时间
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
	//	   history.Append("'"+CTime::GetCurrentTime().Format("%Y-%m-%d %H:%M:%S")+"')");//更新时间
	//	   _bstr_t strSqlinsert=history;
	//	   con.ExecuteSQL(strSqlinsert);

	//	   CString Value = con.m_pRecordset->GetFields()->GetItem("ZKBBH")->Value;
	//	   history= "insert into DEV_CS_PARAMS_HISTORY values(";
	//	   history.Append("'" + res + "',");
	//	   history.Append("'DEV_711E_STEP3_CS',");
	//	   history.Append("'ZKBBH',");
	//	   history.Append("'" + Value + "',");
	//	   history.Append("' ',");
	//	   history.Append("'"+CTime::GetCurrentTime().Format("%Y-%m-%d %H:%M:%S")+"')");//更新时间
	//	   con.ExecuteSQL(strSqlinsert);
	//	   con.m_pRecordset->MoveNext();
	//	}   

	//}

}
//写入数据库
UINT WriteData(LPVOID lpParam)
{
	CLaseAutoCorrectDlg *pDlg = (CLaseAutoCorrectDlg*)lpParam;
	CString MuDipath="/\\服务器-备份\\ProductionProcess\\";
	CString chn;
	CString chn2;
	if (g_ChannelSelect==0)  //高阈值
	{
		chn="H";
		chn2="高";
	}
	else
	{
		chn="L";
		chn2="低";
	}
	ADOConn con;  
	bool conn=con.OnInitADOConn();  
	if(!conn)
	{
		CString strconn="数据库连接失败";
		::SendMessageA(pDlg->GetSafeHwnd(),WM_RECVDATA,0,(LPARAM)(LPCTSTR)strconn);
		return 0;
	}

	//CString PROCESSstr="select Dpt_ID from DEV_PROCESS_TEMPLATE where Dpt_DevType='4' and Dpt_Name='脉宽修正'";
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
	PROCESSstr.Append("' and Dpt_Name='脉宽修正'");
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
	(_tcsrchr(lpPath,_T('\\')))[1] = 0;//获取上级目录
	strcat(lpPath,"\\Setup.ini");
	fstream _file;
	_file.open(lpPath,ios::in);
	LPTSTR l_getId = new char[50];
	LPTSTR l_getOrder = new char[50];
	LPTSTR l_getProcessId = new char[50];
	CString ProcessId;
	if (!_file)
	{
		::AfxMessageBox(_T("配置文件打开失败。"));
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
		if(processcount<=0)//当表中没有就插入
		{
			CString PROCESSORDERstr="insert into DEV_PROCESS_ORDER values(";

			PROCESSORDERstr.Append("'"+ProcessId+"-XZ',");
			PROCESSORDERstr.Append("'-XZ',");
			PROCESSORDERstr.Append("'0',");
			CString Order;
			Order.Format( _T("%s"), l_getOrder);
			PROCESSORDERstr.Append("'"+Order+"',");
			PROCESSORDERstr.Append("'脉宽修正',");
			PROCESSORDERstr.Append("'1',");
			PROCESSORDERstr.Append("'"+strguid+"',");
			PROCESSORDERstr.Append("'【时间】"+CTime::GetCurrentTime().Format("%Y-%m-%d %H:%M:%S")+"',");
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
		str.Append("'"+m_ProcessId+"-XZ"+"',");//工序ID
		str.Append("'"+vsSortParm+"',");//序号
		str.Append("'"+vsNameParm+"',");//名称
		int index=vsNameParm.Find("测试记录");
		if(index>=0)//除测试记录之外
		{
			if(g_ChannelSelect==0)
			{
				str.Append("'"+ShiJian+"_"+pDlg->m_MacAddr+"_"+"测试记录"+".txt_H"+"',");//值
			}
			else
			{
				str.Append("'"+ShiJian+"_"+pDlg->m_MacAddr+"_"+"测试记录"+".txt_L"+"',");//值
			}
		}
		else
		{
			if(g_ChannelSelect==0)
			{
				str.Append("'"+ShiJian+"_"+pDlg->m_MacAddr+"_"+vsNameParm+".txt_H"+"',");//值
			}
			else
			{
			  str.Append("'"+ShiJian+"_"+pDlg->m_MacAddr+"_"+vsNameParm+".txt_L"+"',");//值
			}
		}
		str.Append("'file',");//名称
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
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_"+chn2+"阈值原始数据"+"("+g_cover+")."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_"+chn2+"阈值原始数据"+"("+g_cover+")."+"txt_"+chn, TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_"+chn2+"阈值拟合数据"+"("+g_cover+")."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_"+chn2+"阈值拟合数据"+"("+g_cover+")."+"txt_"+chn, TRUE);
		//CopyFile(m_SavePath+ _T("\\")+ShiJian+"_"+pDlg->m_MacAddr+"_"+chn2+"阈值拟合数据"+"("+g_cover+")."+"txt_"+chn, MuDipath+pDlg->m_MacAddr+"_"+chn2+"阈值拟合数据"+"("+g_cover+")."+"txt_"+chn, TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"拟合图_"+chn+"("+g_cover+").jpg", MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"拟合图_"+chn+"("+g_cover+").jpg", TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"合格_"+chn+"("+g_cover+").jpg", MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"合格_"+chn+"("+g_cover+").jpg", TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_测试记录"+"("+g_cover+")."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_测试记录"+"("+g_cover+")."+"txt_"+chn, TRUE);
	}
	else
	{
		
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_"+chn2+"阈值原始数据."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_"+chn2+"阈值原始数据."+"txt_"+chn, TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_"+chn2+"阈值拟合数据."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_"+chn2+"阈值拟合数据."+"txt_"+chn, TRUE);
		//CopyFile(m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_"+chn2+"阈值拟合数据."+"txt_"+chn, MuDipath+pDlg->m_MacAddr+"_"+chn2+"阈值拟合数据."+"txt_"+chn, TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"拟合图_"+chn+".jpg", MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"拟合图_"+chn+".jpg", TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"合格_"+chn+".jpg", MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"合格_"+chn+".jpg", TRUE);
		CopyFile(pDlg->m_SavePath+ _T("\\")+pDlg->m_MacAddr+"_测试记录."+"txt_"+chn, MuDipath+ShiJian+"_"+pDlg->m_MacAddr+"_测试记录."+"txt_"+chn, TRUE);

	}
	//提交
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
	//CString dir="F:\\（v2）LaseAutoCorrect-20171108-连续 - 高低阈值切换\\Debug\\自动修正结果\\F8B5689000FF";
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
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。
	// TODO:  在此添加控件通知处理程序代码
	/*UpdateData(true);*/
	/*okAvg = m_AveError;*/
}


void CLaseAutoCorrectDlg::OnEnChangeEditAllsingleerror()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。
	/*UpdateData(true);*/
	// TODO:  在此添加控件通知处理程序代码
}


void CLaseAutoCorrectDlg::OnEnChangeEditCheckrangeminvalue()
{
	// TODO:  如果该控件是 RICHEDIT 控件，它将不
	// 发送此通知，除非重写 CDialogEx::OnInitDialog()
	// 函数并调用 CRichEditCtrl().SetEventMask()，
	// 同时将 ENM_CHANGE 标志“或”运算到掩码中。
	/*UpdateData(true);*/
	// TODO:  在此添加控件通知处理程序代码
}
