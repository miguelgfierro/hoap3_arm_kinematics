#ifndef __DATA_ACCESS_H__
#define __DATA_ACCESS_H__
#include "kinematics.h"
#include "matrixes.h"

struct DUNIT;

int send_commands(unsigned short *motor_pos, unsigned char *sensors, DUNIT* unit);
int read_positions(DUNIT *unit, unsigned short *positions);
int readret(DUNIT* unit);
void robot2DH (unsigned short *positions, float *angles);
void DH2robot (float *angles, unsigned short *positions);
int inverse_kinematics (float Ts, float T, matrix *xd, matrix *xd_dot, DUNIT *unit);
int select_desired_position (float *p);
void close_hand (unsigned short *motor, unsigned char *sensor, DUNIT* unit);
void open_hand (unsigned short *motor, unsigned char *sensor, DUNIT* unit);

#define NUM_MOTORS 23
#define NUM_SENSORS 4

#endif // __DATA_ACCESS_H
