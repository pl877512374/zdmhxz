/**********************************************
* 网络连接结构体
* 在结构体的默认构造函数中初始化变量
**********************************************/
#if !defined(__NETWORKUTILS_H)
#define __NETWORKUTILS_H

#define NET_BUF_NUM  100

typedef struct tagNetConnection{
	int nID;               // 客户端Socket编号
	DWORD dwServerIP;      // 服务器IP
	UINT unServerPort;     // 服务器端口
	BOOL bConnected;       //是否连接

	char *pcSendDataBuf;   //发送缓存
	int nSendDataBufSize;  //缓存大小
	LPVOID lpRecvFun;      // 函数指针
	LPVOID lpWnd;          // 窗口指针

	//结构体默认构造函数，初始化变量
	void NET_CONNECTION()	
	{
		Init();
	}
	
	void Init()
	{
		this->dwServerIP = 0;
		this->unServerPort = 0;
		this->bConnected = FALSE;
		this->nID = -1;
		this->pcSendDataBuf = NULL;
		this->nSendDataBufSize = 0;
		this->lpRecvFun = NULL;
		this->lpWnd = NULL;
	}
}NET_CONNECTION;


// 声明函数指针原型
typedef void (*PFCB_NET_RECV)(LPVOID lpParam, char *pDataBuf, int nDataBufSize);

const int MAX_BLOCK = 10000;        // 接收缓存大小
const int MAX_CONNECT_NUM = 10;		// 最大连接数


/**************************************************
* 函数名称：ConnectServer
* 函数功能：连接服务器
* 输入参数：1个参数 
*           参数1:NET_CONNECTION *pNetConnection
* 输出参数：成功返回true，失败返回false 
**************************************************/
BOOL ConnectServer(NET_CONNECTION *pNetConnection);

BOOL UDP_Connect(NET_CONNECTION *pNetConnection);

/**************************************************
* 函数名称：DisconnectServer
* 函数功能：断开连接
* 输入参数：1个参数 
*           参数1:NET_CONNECTION *pNetConnection
* 输出参数：成功返回true，失败返回false 
**************************************************/

BOOL DisconnectServer(NET_CONNECTION *pNetConnection);
/**************************************************
* 函数名称：SendData
* 函数功能：TCP发送数据
* 输入参数：1个参数 
*           参数1:NET_CONNECTION *pNetConnection
* 输出参数：成功返回true，失败返回false 
**************************************************/
BOOL SendData(NET_CONNECTION *pNetConnection);

BOOL UDP_SendData(NET_CONNECTION *pNetConnection, SOCKADDR_IN *pAddrSock);

/**************************************************
* 函数名称：OnNetRecv
* 函数功能：
* 输入参数：
*           LPVOID lpParam：
*           char *pDataBuf：发送缓存
*            nDataBufSize：发送数据长度		
* 输出参数：无
**************************************************/
void OnNetRecv(LPVOID lpParam, char *pDataBuf, int nDataBufSize);

void Add_BCC(char * sendbuf,int sendlen);
BOOL BCC_Check(char* recvbuf,int recvlen);
BOOL BCC_Check2(char* recvbuf,int recvlen);
char BCC_Calc(char * sendbuf,int sendlen);
char CalcBcc_710A(char * sendbuf,int sendlen);
void newBCC711(char * sendbuf,int sendlen);
BOOL newBCC711_Check(char * sendbuf,int sendlen);
short  XorMaiKuan_download( short x, short * sendbuf,int sendlen);
extern int g_nNetRecvInd ;
extern char g_cNetRecvBuf[NET_BUF_NUM][10000];
extern int g_nNetRecvSize[NET_BUF_NUM];

#endif
