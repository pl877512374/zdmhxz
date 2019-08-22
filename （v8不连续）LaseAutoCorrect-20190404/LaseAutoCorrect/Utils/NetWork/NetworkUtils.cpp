#include "stdafx.h"
#include "NetworkUtils.h"

//#include <WINSOCK2.H>

#pragma comment(lib,"ws2_32.lib")
#define TIME_OUT_SECOND 2
//#pragma  comment(lib,"libfitting.lib);


int g_nNetRecvInd = 0;
char g_cNetRecvBuf[NET_BUF_NUM][10000] = {0};
int g_nNetRecvSize[NET_BUF_NUM] = {0};


// 客户端连接结构体
typedef struct tagFullNetConnection{
	NET_CONNECTION netConnection; //客户端网络连接结构体 
	SOCKET sckClient; //客户端套接字	           
	
	//结构体默认构造函数，初始化变量
	void NET_CONNECTION_FULL()	
	{
		Init();
	}
	
	void Init(void)
	{
		this->netConnection.Init();
	}
}NET_CONNECTION_FULL;


NET_CONNECTION_FULL g_aNetConnection[MAX_CONNECT_NUM];//全局变量：网络连接结构体数组变量
extern CRITICAL_SECTION g_netcs;
int g_nConnectNum = 0; //全局变量：实际tcp连接数=nID

typedef struct TagStructNetBuf
{
	char m_acbuf[MAX_BLOCK*10];
	unsigned int m_u32in;
	unsigned int m_u32out;
	unsigned int m_u32size;
}StructNetBuf;
StructNetBuf g_sNetBuf;
int xorflag = 0;
// TCP接收线程处理函数  实时性需要吗
UINT RecvThreadProc(LPVOID lpParam)	
{
	int nID = *((int *)lpParam);
	int nRecvSize = 0;
	char acRecvDataBuf[MAX_BLOCK];
	char acRecvDataBuftmp[MAX_BLOCK];
	int nRecvSizetmp = 0;
	InitializeCriticalSection(&g_netcs);
	PFCB_NET_RECV recvFun = (PFCB_NET_RECV)g_aNetConnection[nID].netConnection.lpRecvFun;
	int l_n32TcpErr = 0;//0正常，1粘帧，2断帧
	memset(&g_sNetBuf,0,sizeof(g_sNetBuf));
	while(g_aNetConnection[nID].netConnection.bConnected)
	{
		EnterCriticalSection(&g_netcs);
		nRecvSize = recv(g_aNetConnection[nID].sckClient,acRecvDataBuf,MAX_BLOCK,0);
		//printf("%d,%d,%d,%d,%d\n",acRecvDataBuf[0],acRecvDataBuf[1],acRecvDataBuf[2],acRecvDataBuf[3],acRecvDataBuf[4]);

		if(nRecvSize < 0)
		{
			continue;
		}
		// recv 函数返回值
		//   <0 出错 
		//   =0 连接关闭 
		//   >0 接收到数据大小，
		//由于每包最大只能有1460字节，72k数据分两个36k包发送
		if((DWORD)nRecvSize > MAX_BLOCK)
		{
			//printf("nRecvSize > MAX_BLOCK");
			continue;
		}
		//-----------------------新解析------------------------------------------
		if ((g_sNetBuf.m_u32in + nRecvSize)>=MAX_BLOCK*10)
		{
			//printf("(g_sNetBuf.m_u32in + nRecvSize)>=MAX_BLOCK*10\n");
			memset(&g_sNetBuf,0,sizeof(g_sNetBuf));//大于最大缓存，原来的数全清0
			continue;
		}
		memcpy(&g_sNetBuf.m_acbuf[g_sNetBuf.m_u32in],acRecvDataBuf,nRecvSize*sizeof(char));
		g_sNetBuf.m_u32in += nRecvSize;
		if (g_sNetBuf.m_u32in >= MAX_BLOCK*10)
		{
			//printf("g_sNetBuf.m_u32in >= MAX_BLOCK\n");
			memset(&g_sNetBuf,0,sizeof(g_sNetBuf));
			//memcpy(&g_sNetBuf.m_acbuf[g_sNetBuf.m_u32in],acRecvDataBuf,nRecvSize*sizeof(char));
			break;
		}
		while(g_sNetBuf.m_u32out < g_sNetBuf.m_u32in)
		{
			if(((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out] == 0xff && 
				(unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+1] == 0xaa)||((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out] == 0xff && 
				(unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+1] == 0xff)  || 
				((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out] == 0x02 &&
				(unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+1] == 0x02 && 
				(unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+2] == 0x02 && 
				(unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+3] == 0x02 ))
			{
				//计算包长度
				unsigned int l_u32reallen = 0;
				if((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out] == 0x02)
				{
					l_u32reallen = ((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+4] << 24) | 
						((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+5] << 16) |
						((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+6] << 8) |
						((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+7] << 0);
					l_u32reallen = l_u32reallen + 9;
				}
				else if((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out] == 0xff)
				{
					if((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+1] == 0xff)//717
					{
						l_u32reallen = ((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+4] << 0) | 
							((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+5] << 8);
					}
					else//711E、711A
					{
						l_u32reallen = ((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+3] << 0) | 
							((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+2] << 8);
					}
				}
				else
					g_sNetBuf.m_u32out++;
				//判断包长度与实际缓存长度之间的关系
				if (l_u32reallen <= (g_sNetBuf.m_u32in - g_sNetBuf.m_u32out+1))
				{
					(*recvFun)(g_aNetConnection[nID].netConnection.lpWnd,&g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out],l_u32reallen);
					if(xorflag == 1)
					{
						//printf("校验 failed!\n");
						int i;
						for( i = 1;i< l_u32reallen;i++)
						{
							if((g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i] == 0x02 && g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i+1] == 0x02 &&g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i+2] == 0x02 &&g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i+3] == 0x02)
								|| (g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i] == 0xff && g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i+1] == 0xff))
							{
								g_sNetBuf.m_u32out += i;
								//memset(&g_sNetBuf,0,sizeof(gti_sNetBuf));
								//printf("！！！连帧 failed!\n");
								xorflag = 0;
								break;
							}

						}
						if(i== l_u32reallen && xorflag == 1)
						{
							g_sNetBuf.m_u32out += l_u32reallen;
						}
						xorflag = 0;
					}
					else
						g_sNetBuf.m_u32out += l_u32reallen;
				}
				else if(l_u32reallen >= MAX_BLOCK)
				{
					//printf("l_u32reallen err %d\n",l_u32reallen);
					memset(&g_sNetBuf,0,sizeof(g_sNetBuf));//大于最大缓存，原来的数全清0
					continue;
				}
				else
				{
					break;
				}
			}
			else
			{
				g_sNetBuf.m_u32out++;
			}

		}
		if ((g_sNetBuf.m_u32in + nRecvSize)>=MAX_BLOCK*10)
		{
			//printf("！！！(g_sNetBuf.m_u32in + nRecvSize)>=MAX_BLOCK*10\n");
			//printf("g_sNetBuf.m_u32in is %d,g_sNetBuf.m_u32out is%d\n",g_sNetBuf.m_u32in,g_sNetBuf.m_u32out);
			memset(&g_sNetBuf,0,sizeof(g_sNetBuf));//大于最大缓存，原来的数全清0
			continue;
		}
		if (g_sNetBuf.m_u32out >= g_sNetBuf.m_u32in)
		{
			memset(&g_sNetBuf,0,sizeof(g_sNetBuf));
		}	
		LeaveCriticalSection(&g_netcs);
	}	
	return 0;
}


// UPD接收线程处理函数
UINT UDP_RecvThreadProc(LPVOID lpParam)	
{
	SOCKADDR_IN addrFrom;        //用于存放发送端的地址
	int nLen = sizeof(SOCKADDR); 
	
	int nID = *((int *)lpParam);
	int nRecvSize = 0;
	char acRecvDataBuf[MAX_BLOCK];
	PFCB_NET_RECV recvFun = (PFCB_NET_RECV)g_aNetConnection[nID].netConnection.lpRecvFun;
	
	while(g_aNetConnection[nID].netConnection.bConnected)
	{
		nRecvSize = recvfrom(g_aNetConnection[nID].sckClient,acRecvDataBuf,MAX_BLOCK,0, (SOCKADDR*)&addrFrom, &nLen);
		if (nRecvSize>0 && recvFun!=NULL)
		{
			(*recvFun)(g_aNetConnection[nID].netConnection.lpWnd,acRecvDataBuf,nRecvSize);
		}
	}	
	return 0;
}

// TCP发送线程处理函数
UINT SendThreadProc(LPVOID lpParam)
{
	int nID = *((int *)lpParam);
	int nSendSize = 0;
	
	while(nSendSize < g_aNetConnection[nID].netConnection.nSendDataBufSize)
	{
			
		nSendSize += send(g_aNetConnection[nID].sckClient,g_aNetConnection[nID].netConnection.pcSendDataBuf+nSendSize,
						  g_aNetConnection[nID].netConnection.nSendDataBufSize-nSendSize,0);
		Sleep(1);//胡孟虎 2013-04-26 解决程序卡
	}
	return 0;
}


// TCP 连接：连接服务器，并创建数据接收线程
// 输入参数：网络连接结构体
BOOL ConnectServer(NET_CONNECTION *pNetConnection)
{
	SOCKET sockClient;
	WORD wVersionRequested;
	WSADATA wsaData;
	int err;
	int len = sizeof(int);
	timeval tm;
	fd_set set;
	unsigned long ul = 1;
	bool ret = false;
	CWinThread *pThread;
	
	wVersionRequested = MAKEWORD(1,1);
	err = WSAStartup(wVersionRequested,&wsaData);
	if (err != 0)
		return FALSE;
	
	if (LOBYTE(wsaData.wVersion) != 1 ||
		HIBYTE(wsaData.wVersion != 1))
	{
		WSACleanup();
		return FALSE;
	}
	// 创建客户端套接字
	sockClient = socket(AF_INET,SOCK_STREAM,0);

	if (sockClient == INVALID_SOCKET)
	{
		return FALSE;
	}
	SOCKADDR_IN addrSrv;
	addrSrv.sin_addr.S_un.S_addr = htonl(pNetConnection->dwServerIP);
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(pNetConnection->unServerPort);
	ioctlsocket(sockClient,FIONBIO,&ul);
	// 向服务器发出连接请求，成功返回0，失败返回-1
	int iConnect = connect(sockClient,(SOCKADDR*)&addrSrv,sizeof(SOCKADDR));
    //若连接成功
	if(iConnect == -1)
	{
		tm.tv_sec = TIME_OUT_SECOND;
		tm.tv_usec = 0;
		FD_ZERO(&set);
		FD_SET(sockClient,&set);
		if (select(sockClient+1,NULL,&set,NULL,&tm) > 0)
		{
			getsockopt(sockClient,SOL_SOCKET,SOCKET_ERROR,(char*)err,&len);
			if (err == 0)
			{
				ret = true;
			} 
			else
			{
				ret = false;
			}
		}
		else
			ret = false;
		
	}
	else
		ret = false;
	ul = 0;
	ioctlsocket(sockClient,FIONBIO,&ul);
	if (!ret)
	{
		return false;
	}
	g_aNetConnection[g_nConnectNum].sckClient = sockClient;
	pNetConnection->nID = g_nConnectNum;
	pNetConnection->bConnected = TRUE;
	g_aNetConnection[g_nConnectNum].netConnection = *pNetConnection;
	g_nConnectNum++;	 
	pThread = ::AfxBeginThread(RecvThreadProc,&g_aNetConnection[pNetConnection->nID].netConnection.nID);
	if (pThread == NULL)
	{
		return FALSE;
	}
	//2013-04-26 胡孟虎 关闭线程
//	CloseHandle(pThread);
	return TRUE;
}

// UDP连接
BOOL UDP_Connect(NET_CONNECTION *pNetConnection)
{
	SOCKET UDP_Socket;
	WORD wVersionRequested;
	WSADATA wsaData;
	int err;
	CWinThread *pThread;
	
	wVersionRequested = MAKEWORD(1,1);
	err = WSAStartup(wVersionRequested,&wsaData);
	if (err != 0)
		return FALSE;
	
	if (LOBYTE(wsaData.wVersion) != 1 ||
		HIBYTE(wsaData.wVersion != 1))
	{
		WSACleanup();
		return FALSE;
	}
	// 创建套接字
	UDP_Socket = socket(AF_INET,SOCK_DGRAM,0);
	SOCKADDR_IN addrSock;
	addrSock.sin_addr.S_un.S_addr = htonl(pNetConnection->dwServerIP);
	addrSock.sin_family = AF_INET;
	addrSock.sin_port = htons(pNetConnection->unServerPort);
	// 绑定套接字
	int nBind = bind(UDP_Socket,(SOCKADDR*)&addrSock,sizeof(SOCKADDR));
	if(SOCKET_ERROR != nBind)
	{
		g_aNetConnection[g_nConnectNum].sckClient = UDP_Socket;
		pNetConnection->nID = g_nConnectNum;
		pNetConnection->bConnected = TRUE;
		g_aNetConnection[g_nConnectNum].netConnection = *pNetConnection;
		g_nConnectNum++;	
		
		pThread = ::AfxBeginThread(UDP_RecvThreadProc,&g_aNetConnection[pNetConnection->nID].netConnection.nID);
		if (pThread == NULL)
		{
			return FALSE;
		}
	//	CloseHandle(pThread);//2013-04-26 胡孟虎 关闭线程
	}
	else
	{
		closesocket(UDP_Socket);
		WSACleanup();	
		return FALSE;
	}
	return TRUE;
}

// 断开连接
BOOL DisconnectServer(NET_CONNECTION *pNetConnection)
{
	int nID;

	nID = pNetConnection->nID;
	if (g_aNetConnection[nID].netConnection.nID == nID &&
		g_aNetConnection[nID].netConnection.bConnected)
	{
		closesocket(g_aNetConnection[nID].sckClient);
		WSACleanup();	
		g_aNetConnection[nID].Init();
		g_nConnectNum--;
	}	
	return TRUE;
}

// TCP发送数据
BOOL SendData(NET_CONNECTION *pNetConnection)
{
	int nID;
	CWinThread *pThread;
	nID = pNetConnection->nID;

	if (g_aNetConnection[nID].netConnection.bConnected)
	{
		//Add_BCC(pNetConnection->pcSendDataBuf,pNetConnection->nSendDataBufSize);
		g_aNetConnection[nID].netConnection.pcSendDataBuf = pNetConnection->pcSendDataBuf;
		g_aNetConnection[nID].netConnection.nSendDataBufSize = pNetConnection->nSendDataBufSize;
		pThread = ::AfxBeginThread(SendThreadProc,&g_aNetConnection[nID].netConnection.nID);
		if (pThread == NULL)
		{
			return FALSE;
		}
//		CloseHandle(pThread);  //关闭发送线程 胡孟虎 2013-04-26
	}
	return TRUE;
}

// UDP发送数据
BOOL UDP_SendData(NET_CONNECTION *pNetConnection, SOCKADDR_IN *pAddrSock)
{
	int nID;
	nID = pNetConnection->nID;
	
	if (g_aNetConnection[nID].netConnection.bConnected)
	{
		g_aNetConnection[nID].netConnection.pcSendDataBuf=pNetConnection->pcSendDataBuf;
		g_aNetConnection[nID].netConnection.nSendDataBufSize=pNetConnection->nSendDataBufSize;
		sendto(g_aNetConnection[nID].sckClient, g_aNetConnection[nID].netConnection.pcSendDataBuf,
			g_aNetConnection[nID].netConnection.nSendDataBufSize,0,(sockaddr *)pAddrSock,sizeof(SOCKADDR));
		return TRUE;
	}
	else
		return FALSE;
//	return 0;
}
//在倒数第二字节加校验
void Add_BCC(char * sendbuf,int sendlen)
{
	int i =0;
	char check=0;
	char* p = sendbuf;
	int len ;
	if (*p == (char)0x02)
	{
		p=p+8;
		len	= sendlen;
		for (i=8;i<len;i++)
		{
			check ^= *p++;
		}
	}
	else if (*p == (char)0xff)
	{
		len = sendlen-2;
		for(i =0;i<len;i++)
		{
			check ^= *p++;
		}
	}
	else
	{
		;
	}
	*p++ = check;
}

char BCC_Calc(char * sendbuf,int sendlen)
{
	int i =0;
	char check=0;
	char* p = sendbuf;
	
	for (i=8;i<sendlen;i++)
	{
		check ^= *p++;
	}
	return check;
}

BOOL BCC_Check(char* recvbuf,int recvlen)
{
	int i =0;
	char check = 0;
	char * p = recvbuf;
	int len ;//末尾两位无需校验
	if (*p == (char)0xff)
	{
		len = recvlen-2;
		for (i = 0 ; i<len;i++)
		{
			check ^= *p++;
		}
	} 
	else if (*p == (char)0x02)
	{
		p = p+8;
		len = recvlen-9;
		for (i = 0; i<len;i++)
		{
			check ^= *p++;
		}
	}
	else
	{
		return FALSE;
	}
	
	if (check == *p)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
BOOL BCC_Check2(char* recvbuf,int recvlen)
{
	int i =0;
	char check = 0;
	char * p = recvbuf;
	int len ;//末尾两位无需校验
	if (*p == (char)0xff)
	{
		len = recvlen-2;
		for (i = 0 ; i<len;i++)
		{
			check ^= *p++;
		}
	} 
	else if (*p == (char)0x02)
	{
		p = p+8;
		len = recvlen-9;
		for (i = 0; i<len+1;i++)//710B 协议长度不对
		{
			check ^= *p++;
		}
	}
	else
	{
		return FALSE;
	}
	
	if (check == *p)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
char CalcBcc_710A(char * sendbuf,int sendlen)
{
	int i =0;
	char check=0;
	for(i =0;i<sendlen;i++)
	{
		check ^= *sendbuf++;
	}
	return check;
}
void newBCC711(char * sendbuf,int sendlen)
{
	int i =0;
	char check=0;
	char* p = sendbuf;
	int len ;
	if (*p == (char)0xff)
	{
		*p++;
		*p++;
		for(i =2;i<sendlen-4;i++)
		{
			check ^= *p++;
		}
		 *p++=0x00;
		*p++ = check;
	    *p++=0xEE;
	    *p++=0xEE;
	}
}

BOOL newBCC711_Check(char * sendbuf,int sendlen)
{
	char check = 0;
	char* p = sendbuf;
	*p++;
	*p++;
	for(int i=2;i<sendlen-4;i++)
	{
		check ^= *p++;
	}
	*p++;
	
	if(check==*p)
	{
	return TRUE;
	}else{
	return FALSE;
	}
}
//711脉宽下载
 short  XorMaiKuan_download( short x, short * sendbuf,int sendlen)
 {
	 short check=0;
	 int i =0;
	  short* p = sendbuf;
	 check ^= x;
	for(i =0;i<sendlen;i++)
	{
		check ^= *p++;
	}
	return check;
 }
