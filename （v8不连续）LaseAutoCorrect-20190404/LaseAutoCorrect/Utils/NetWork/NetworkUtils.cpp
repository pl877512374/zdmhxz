#include "stdafx.h"
#include "NetworkUtils.h"

//#include <WINSOCK2.H>

#pragma comment(lib,"ws2_32.lib")
#define TIME_OUT_SECOND 2
//#pragma  comment(lib,"libfitting.lib);


int g_nNetRecvInd = 0;
char g_cNetRecvBuf[NET_BUF_NUM][10000] = {0};
int g_nNetRecvSize[NET_BUF_NUM] = {0};


// �ͻ������ӽṹ��
typedef struct tagFullNetConnection{
	NET_CONNECTION netConnection; //�ͻ����������ӽṹ�� 
	SOCKET sckClient; //�ͻ����׽���	           
	
	//�ṹ��Ĭ�Ϲ��캯������ʼ������
	void NET_CONNECTION_FULL()	
	{
		Init();
	}
	
	void Init(void)
	{
		this->netConnection.Init();
	}
}NET_CONNECTION_FULL;


NET_CONNECTION_FULL g_aNetConnection[MAX_CONNECT_NUM];//ȫ�ֱ������������ӽṹ���������
extern CRITICAL_SECTION g_netcs;
int g_nConnectNum = 0; //ȫ�ֱ�����ʵ��tcp������=nID

typedef struct TagStructNetBuf
{
	char m_acbuf[MAX_BLOCK*10];
	unsigned int m_u32in;
	unsigned int m_u32out;
	unsigned int m_u32size;
}StructNetBuf;
StructNetBuf g_sNetBuf;
int xorflag = 0;
// TCP�����̴߳�����  ʵʱ����Ҫ��
UINT RecvThreadProc(LPVOID lpParam)	
{
	int nID = *((int *)lpParam);
	int nRecvSize = 0;
	char acRecvDataBuf[MAX_BLOCK];
	char acRecvDataBuftmp[MAX_BLOCK];
	int nRecvSizetmp = 0;
	InitializeCriticalSection(&g_netcs);
	PFCB_NET_RECV recvFun = (PFCB_NET_RECV)g_aNetConnection[nID].netConnection.lpRecvFun;
	int l_n32TcpErr = 0;//0������1ճ֡��2��֡
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
		// recv ��������ֵ
		//   <0 ���� 
		//   =0 ���ӹر� 
		//   >0 ���յ����ݴ�С��
		//����ÿ�����ֻ����1460�ֽڣ�72k���ݷ�����36k������
		if((DWORD)nRecvSize > MAX_BLOCK)
		{
			//printf("nRecvSize > MAX_BLOCK");
			continue;
		}
		//-----------------------�½���------------------------------------------
		if ((g_sNetBuf.m_u32in + nRecvSize)>=MAX_BLOCK*10)
		{
			//printf("(g_sNetBuf.m_u32in + nRecvSize)>=MAX_BLOCK*10\n");
			memset(&g_sNetBuf,0,sizeof(g_sNetBuf));//������󻺴棬ԭ������ȫ��0
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
				//���������
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
					else//711E��711A
					{
						l_u32reallen = ((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+3] << 0) | 
							((unsigned char)g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+2] << 8);
					}
				}
				else
					g_sNetBuf.m_u32out++;
				//�жϰ�������ʵ�ʻ��泤��֮��Ĺ�ϵ
				if (l_u32reallen <= (g_sNetBuf.m_u32in - g_sNetBuf.m_u32out+1))
				{
					(*recvFun)(g_aNetConnection[nID].netConnection.lpWnd,&g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out],l_u32reallen);
					if(xorflag == 1)
					{
						//printf("У�� failed!\n");
						int i;
						for( i = 1;i< l_u32reallen;i++)
						{
							if((g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i] == 0x02 && g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i+1] == 0x02 &&g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i+2] == 0x02 &&g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i+3] == 0x02)
								|| (g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i] == 0xff && g_sNetBuf.m_acbuf[g_sNetBuf.m_u32out+i+1] == 0xff))
							{
								g_sNetBuf.m_u32out += i;
								//memset(&g_sNetBuf,0,sizeof(gti_sNetBuf));
								//printf("��������֡ failed!\n");
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
					memset(&g_sNetBuf,0,sizeof(g_sNetBuf));//������󻺴棬ԭ������ȫ��0
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
			//printf("������(g_sNetBuf.m_u32in + nRecvSize)>=MAX_BLOCK*10\n");
			//printf("g_sNetBuf.m_u32in is %d,g_sNetBuf.m_u32out is%d\n",g_sNetBuf.m_u32in,g_sNetBuf.m_u32out);
			memset(&g_sNetBuf,0,sizeof(g_sNetBuf));//������󻺴棬ԭ������ȫ��0
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


// UPD�����̴߳�����
UINT UDP_RecvThreadProc(LPVOID lpParam)	
{
	SOCKADDR_IN addrFrom;        //���ڴ�ŷ��Ͷ˵ĵ�ַ
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

// TCP�����̴߳�����
UINT SendThreadProc(LPVOID lpParam)
{
	int nID = *((int *)lpParam);
	int nSendSize = 0;
	
	while(nSendSize < g_aNetConnection[nID].netConnection.nSendDataBufSize)
	{
			
		nSendSize += send(g_aNetConnection[nID].sckClient,g_aNetConnection[nID].netConnection.pcSendDataBuf+nSendSize,
						  g_aNetConnection[nID].netConnection.nSendDataBufSize-nSendSize,0);
		Sleep(1);//���ϻ� 2013-04-26 �������
	}
	return 0;
}


// TCP ���ӣ����ӷ����������������ݽ����߳�
// ����������������ӽṹ��
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
	// �����ͻ����׽���
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
	// ������������������󣬳ɹ�����0��ʧ�ܷ���-1
	int iConnect = connect(sockClient,(SOCKADDR*)&addrSrv,sizeof(SOCKADDR));
    //�����ӳɹ�
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
	//2013-04-26 ���ϻ� �ر��߳�
//	CloseHandle(pThread);
	return TRUE;
}

// UDP����
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
	// �����׽���
	UDP_Socket = socket(AF_INET,SOCK_DGRAM,0);
	SOCKADDR_IN addrSock;
	addrSock.sin_addr.S_un.S_addr = htonl(pNetConnection->dwServerIP);
	addrSock.sin_family = AF_INET;
	addrSock.sin_port = htons(pNetConnection->unServerPort);
	// ���׽���
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
	//	CloseHandle(pThread);//2013-04-26 ���ϻ� �ر��߳�
	}
	else
	{
		closesocket(UDP_Socket);
		WSACleanup();	
		return FALSE;
	}
	return TRUE;
}

// �Ͽ�����
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

// TCP��������
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
//		CloseHandle(pThread);  //�رշ����߳� ���ϻ� 2013-04-26
	}
	return TRUE;
}

// UDP��������
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
//�ڵ����ڶ��ֽڼ�У��
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
	int len ;//ĩβ��λ����У��
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
	int len ;//ĩβ��λ����У��
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
		for (i = 0; i<len+1;i++)//710B Э�鳤�Ȳ���
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
//711��������
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
