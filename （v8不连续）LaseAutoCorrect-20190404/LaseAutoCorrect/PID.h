#ifndef _PID_H_
#define _PID_H_

typedef struct{
	float limit;		//输出限幅
	float target;		//目标输出量
	float feedback;		//实际输出量	
	float Kp;	
	float Ki;	
	float Kd;	
	float e_0;			//当前误差
	float e_1;			//上一次误差
	float e_2;			//上上次误差
	float e_s;          //误差累计

}PID;

#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
#define exchange(a, b, tmp) (tmp=a, a=b, b=tmp)
#define myabs(x)			((x<0)? -x:x)

extern float pid_calc(PID *pid);

#endif