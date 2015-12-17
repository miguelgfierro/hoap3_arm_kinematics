#ifndef __KINEMATICS__
#define __KINEMATICS__

#define ARM_LINK2 0.111
#define ARM_LINK3 0.171
#define PI 3.14159265
#include "matrixes.h"

void trajectory_left_arm (float Ts, float T, float *p0, float *p1, matrix *xd, matrix *xd_dot);
void interpolation (float Ts, float T, float x0, float x1, matrix *x, int row);
void evaluate_velocity (float Ts, float T, matrix *x, matrix *v, int row);
void evaluate_position(float *q, float *p);
void evaluate_jacobian(float *q, matrix *J);
float evaluate_execution_time (float *p0, float *p1, float vm);
void evaluate_null_vector(float *q, float *q0);

#endif
