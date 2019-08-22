/**********************************************
* �������ӽṹ��
* �ڽṹ���Ĭ�Ϲ��캯���г�ʼ������
**********************************************/
#if !defined(__NETWORKUTILS_H)
#define __NETWORKUTILS_H

#define NET_BUF_NUM  100

typedef struct tagNetConnection{
	int nID;               // �ͻ���Socket���
	DWORD dwServerIP;      // ������IP
	UINT unServerPort;     // �������˿�
	BOOL bConnected;       //�Ƿ�����

	char *pcSendDataBuf;   //���ͻ���
	int nSendDataBufSize;  //�����С
	LPVOID lpRecvFun;      // ����ָ��
	LPVOID lpWnd;          // ����ָ��

	//�ṹ��Ĭ�Ϲ��캯������ʼ������
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


// ��������ָ��ԭ��
typedef void (*PFCB_NET_RECV)(LPVOID lpParam, char *pDataBuf, int nDataBufSize);

const int MAX_BLOCK = 10000;        // ���ջ����С
const int MAX_CONNECT_NUM = 10;		// ���������


/**************************************************
* �������ƣ�ConnectServer
* �������ܣ����ӷ�����
* ���������1������ 
*           ����1:NET_CONNECTION *pNetConnection
* ����������ɹ�����true��ʧ�ܷ���false 
**************************************************/
BOOL ConnectServer(NET_CONNECTION *pNetConnection);

BOOL UDP_Connect(NET_CONNECTION *pNetConnection);

/**************************************************
* �������ƣ�DisconnectServer
* �������ܣ��Ͽ�����
* ���������1������ 
*           ����1:NET_CONNECTION *pNetConnection
* ����������ɹ�����true��ʧ�ܷ���false 
**************************************************/

BOOL DisconnectServer(NET_CONNECTION *pNetConnection);
/**************************************************
* �������ƣ�SendData
* �������ܣ�TCP��������
* ���������1������ 
*           ����1:NET_CONNECTION *pNetConnection
* ����������ɹ�����true��ʧ�ܷ���false 
**************************************************/
BOOL SendData(NET_CONNECTION *pNetConnection);

BOOL UDP_SendData(NET_CONNECTION *pNetConnection, SOCKADDR_IN *pAddrSock);

/**************************************************
* �������ƣ�OnNetRecv
* �������ܣ�
* ���������
*           LPVOID lpParam��
*           char *pDataBuf�����ͻ���
*            nDataBufSize���������ݳ���		
* �����������
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
