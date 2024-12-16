
#include "Kalman.h"
#include "CurrentSense.h"
#include "InlineCurrentSense.h"
#include "stdio.h"

// get current magnitude 
//   - absolute  - if no electrical_angle provided 
//   - signed    - if angle provided


float getDCCurrent(float motor_electrical_angle)
{
	PhaseCurrent_s current;
	float sign=1;       // currnet sign - if motor angle not provided the magnitude is always positive
	float i_alpha, i_beta;
	
	// read current phase currents
	current = getPhaseCurrents();
	
	// calculate clarke transform
	if(!current.c)
	{
		// if only two measured currents
		i_alpha = current.a;
		i_beta = _1_SQRT3 * current.a + _2_SQRT3 * current.b;
	}
	else
	{
		// signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
		float mid = (1.f/3) * (current.a + current.b + current.c);
		float a = current.a - mid;
		float b = current.b - mid;
		i_alpha = a;
		i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
	}
	
	// if motor angle provided function returns signed value of the current
	// determine the sign of the current
	// sign(atan2(current.q, current.d)) is the same as c.q > 0 ? 1 : -1  
	if(motor_electrical_angle)sign = (i_beta * _cos(motor_electrical_angle) - i_alpha*_sin(motor_electrical_angle)) > 0 ? 1 : -1;  
	// return current magnitude
	return sign*_sqrt(i_alpha*i_alpha + i_beta*i_beta);
}
/******************************************************************************/
// function used with the foc algorihtm
//   calculating DQ currents from phase currents
//   - function calculating park and clarke transform of the phase currents 
//   - using getPhaseCurrents internally
DQCurrent_s getFOCCurrents(float angle_el)
{
	PhaseCurrent_s current;
	float i_alpha, i_beta;
	float ct,st;
	DQCurrent_s ret;
	
	// read current phase currents
	current = getPhaseCurrents();
//	current.a = Slid_Filter(&Current_a,current.a,10);
//	current.b = Slid_Filter(&Current_b,current.b,10);
//	printf("%.2f,%.2f\n",current.a,current.b);
	// calculate clarke transform
	if(!current.c)
	{
		// if only two measured currents
		i_alpha = -current.b;  
		i_beta = _1_SQRT3 * current.b + _2_SQRT3 * current.a;
	}
	else
	{
		// signal filtering using identity a + b + c = 0. Assumes measurement error is normally distributed.
		float mid = (1.f/3) * (current.a + current.b + current.c);
		float a = current.a - mid;
		float b = current.b - mid;
		i_alpha = a;
		i_beta = _1_SQRT3 * a + _2_SQRT3 * b;
	}
	
	// calculate park transform
	
	ct = _cos(angle_el);
	st = _sin(angle_el);
	ret.d = i_alpha * ct + i_beta * st;
											ret.q = i_beta * ct - i_alpha * st;//-0.21*-0.19-0.16*0.98
	
	ret.q=Kalman_Filter(&kalman_current_Iq_Filter,ret.q);
//	ret.q=Kalman_Filter(&kalman_current_Iq_Filter,ret.q);
//	printf("%.2f\n",ret.q);
	//	printf("%.2f,%.2f,%.2f,%.2f\n",current.a,current.b,ret.q,0.0);
//printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",current.a,current.b,ret.q,0.0,i_alpha,i_beta,ct,st,(angle_el-3.14)/3);

	return ret;
}
/******************************************************************************/
