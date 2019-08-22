#ifndef _PID_H_
#define _PID_H_

typedef struct{
	float limit;		//����޷�
	float target;		//Ŀ�������
	float feedback;		//ʵ�������	
	float Kp;	
	float Ki;	
	float Kd;	
	float e_0;			//��ǰ���
	float e_1;			//��һ�����
	float e_2;			//���ϴ����
	float e_s;          //����ۼ�

}PID;

#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
#define exchange(a, b, tmp) (tmp=a, a=b, b=tmp)
#define myabs(x)			((x<0)? -x:x)

extern float pid_calc(PID *pid);

#endif