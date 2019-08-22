#include "stdafx.h"
#include "PID.h"

float pid_calc(PID *pid)
{
	float out;
	float ep, ei, ed;

	pid->e_0 = pid->target - pid->feedback;
	ep = pid->e_0  - pid->e_1;
	ei = pid->e_0;
	ed = pid->e_0 - 2*pid->e_1 + pid->e_2;
	out = pid->Kp*ep + pid->Ki*ei + pid->Kd*ed;
	out = range(out, -pid->limit, pid->limit);
	pid->e_2 = pid->e_1;
	pid->e_1 = pid->e_0;
	return out;
}