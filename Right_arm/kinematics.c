#include "kinematics.h"

void trajectory_right_arm (float Ts, float T, float *p0, float *p1, matrix *xd, matrix *xd_dot)
{
   	interpolation (Ts, T, p0[0], p1[0], xd, 0);
	interpolation (Ts, T, p0[1], p1[1], xd, 1);
	interpolation (Ts, T, p0[2], p1[2], xd, 2);

	evaluate_velocity (Ts, T, xd, xd_dot, 0);
	evaluate_velocity (Ts, T, xd, xd_dot, 1);
	evaluate_velocity (Ts, T, xd, xd_dot, 2);
}

void interpolation (float Ts, float T, float x0, float x1, matrix *x, int column)
{
	int i;
	int iterations;
	float t;
	float c, d;

	iterations = T/Ts+1;
	t=0;
	c = 3*(x1-x0)/(T*T);
	d = 2*(x0-x1)/(T*T*T);
	for (i=0;i<iterations;i++) {
        x->vector[identify_element (x, i, column)] = x0 + c*t*t + d*t*t*t;
        t = t+Ts;
	}
}

void evaluate_velocity (float Ts, float T, matrix *x, matrix *v, int column)
{
	int i;
	int iterations;
	int element;
	iterations = T/Ts+1;
	v->vector[identify_element (x, 0, column)]=0;
	for (i=1; i<iterations; i++) {
		element = identify_element (x, i, column);
		v->vector[element] = (x->vector[element]-x->vector[element-3])/Ts;
	}
}

void evaluate_position(float *q, float *p)
{
	p[0] = (-cos(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))+sin(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3);
	p[1] = (-sin(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))-cos(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3);
	p[2] = (-cos(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3+sin(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2));
}

void evaluate_jacobian(float *q, matrix *J)
{
	J->vector[0] = sin(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))+cos(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3;
	J->vector[1] = -cos(q[0])*(-cos(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3+sin(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2));
	J->vector[2] = sin(q[0])*cos(q[1])*(-cos(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3+sin(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))-sin(q[1])*(-sin(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))-cos(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3),
	J->vector[3] = (-sin(q[0])*sin(q[1])*cos(q[2])-cos(q[0])*sin(q[2]))*(-cos(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3+sin(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2)+sin(q[1])*ARM_LINK2)-cos(q[1])*cos(q[2])*(-sin(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))-cos(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3+sin(q[0])*cos(q[1])*ARM_LINK2);
	J->vector[4] = ((-sin(q[0])*sin(q[1])*sin(q[2])+cos(q[0])*cos(q[2]))*sin(q[3])+sin(q[0])*cos(q[1])*cos(q[3]))*(-cos(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3+sin(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2)+sin(q[1])*ARM_LINK2)-(cos(q[1])*sin(q[2])*sin(q[3])+sin(q[1])*cos(q[3]))*(-sin(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))-cos(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3+sin(q[0])*cos(q[1])*ARM_LINK2);
	//J->vector[4] = 0;
	J->vector[5] = -cos(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))+sin(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3;
	J->vector[6] = -sin(q[0])*(-cos(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3+sin(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2));
	J->vector[7] = sin(q[1])*(-cos(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))+sin(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3)-cos(q[0])*cos(q[1])*(-cos(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3+sin(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2));
	J->vector[8] = cos(q[1])*cos(q[2])*(-cos(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))+sin(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3+cos(q[0])*cos(q[1])*ARM_LINK2)-(-cos(q[0])*sin(q[1])*cos(q[2])+sin(q[0])*sin(q[2]))*(-cos(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3+sin(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2)+sin(q[1])*ARM_LINK2);
	J->vector[9] = (cos(q[1])*sin(q[2])*sin(q[3])+sin(q[1])*cos(q[3]))*(-cos(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))+sin(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3+cos(q[0])*cos(q[1])*ARM_LINK2)-((-cos(q[0])*sin(q[1])*sin(q[2])-sin(q[0])*cos(q[2]))*sin(q[3])+cos(q[0])*cos(q[1])*cos(q[3]))*(-cos(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3+sin(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2)+sin(q[1])*ARM_LINK2);
	//J->vector[9] = 0;
	J->vector[10] = 0;
	J->vector[11] = sin(q[0])*(-sin(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))-cos(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3)+cos(q[0])*(-cos(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))+sin(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3);
	J->vector[12] = cos(q[0])*cos(q[1])*(-sin(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))-cos(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3)-sin(q[0])*cos(q[1])*(-cos(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))+sin(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3);
	J->vector[13] = (-cos(q[0])*sin(q[1])*cos(q[2])+sin(q[0])*sin(q[2]))*(-sin(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))-cos(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3+sin(q[0])*cos(q[1])*ARM_LINK2)-(-sin(q[0])*sin(q[1])*cos(q[2])-cos(q[0])*sin(q[2]))*(-cos(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))+sin(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3+cos(q[0])*cos(q[1])*ARM_LINK2);
	J->vector[14] = ((-cos(q[0])*sin(q[1])*sin(q[2])-sin(q[0])*cos(q[2]))*sin(q[3])+cos(q[0])*cos(q[1])*cos(q[3]))*(-sin(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))-cos(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3+sin(q[0])*cos(q[1])*ARM_LINK2)-((-sin(q[0])*sin(q[1])*sin(q[2])+cos(q[0])*cos(q[2]))*sin(q[3])+sin(q[0])*cos(q[1])*cos(q[3]))*(-cos(q[0])*(-sin(q[1])*sin(q[2])*sin(q[3])*ARM_LINK3-cos(q[1])*(-cos(q[3])*ARM_LINK3-ARM_LINK2))+sin(q[0])*cos(q[2])*sin(q[3])*ARM_LINK3+cos(q[0])*cos(q[1])*ARM_LINK2);
	//J->vector[14] = 0;
}

float evaluate_execution_time (float *p0, float *p1, float vm)
{
 	  float diff[3];
	  float T;
	  float norm;
	  vector_subtraction (p0, p1, diff, 3);
	  norm=sqrt(diff[0]*diff[0]+diff[1]*diff[1]+diff[2]*diff[2]);
	  T=norm/vm;
	  printf("\n Time is calculated to be as %f sec. for a distance of %f cm\n",T, norm*100);
	  return T;
}

void evaluate_null_vector(float *q, float *q0)
{
	float k0=50;
	float q_ex[5], q_m[5];
	int j;
    q_ex[0]=(151+91);
    q_ex[1]=(96+1);
	q_ex[2]=(91+91);
	q_ex[3]=(1+115);
	q_ex[4]=(60+60);
	q_m[0]=q_ex[0]/2-91;
	q_m[1]=q_ex[1]/2-1;
	q_m[2]=q_ex[2]/2-91;
	q_m[3]=q_ex[3]/2-115;
	q_m[4]=0;
	for (j=0;j<5;j++) q0[j]=-(PI/180)*(k0/5)*(q[j]-q_m[j])/q_ex[j];
}
