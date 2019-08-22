
typedef struct{
	int nPort;
	int nBaud;
	BYTE *pDataBuf;
	int nDataBufSize;
	LPVOID lpRecvFun;
	LPVOID lpSendFun;
	LPVOID lpParam;
	
	//结构体默认构造函数，初始化变量
	void SerPortPar()	
	{
		void Init();
	}
	
	void Init()
	{
		this->nPort = 0;
		this->nBaud = 0;
		this->lpRecvFun = 0;
		this->lpSendFun = 0;
		this->pDataBuf = NULL;
		this->lpParam = FALSE;
	}
}SerPortPar;



const int MAX_PORT_NUM = 20;

typedef void (*PFCALLBACK_RECV)(LPVOID lpParam, BYTE *pDataBuf, int nDataBufSize);
typedef void (*PFCALLBACK_SEND)(LPVOID lpParam, int nStopType);

int OpenComm(SerPortPar *pSerPortPar);

/**************************************************
* 函数名称：CloseComm
* 输入参数：1个参数 
*           参数1:int nPort		串口号
* 输出参数：BOOL 
* 函数功能：
* 创建作者：zyy
* 创建时间：2011-8-9 9:38:25
**************************************************/

int CloseComm(int nPort);
BOOL SendComm(SerPortPar *pSerPortPar);
BOOL StopSendComm(int nPort);


extern unsigned int g_State;
extern unsigned int g_FramLen;
extern unsigned char DataBuf[400];//数据缓存
extern unsigned int DataLen,gcnt;//接收到的数据长度

