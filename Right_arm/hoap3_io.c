#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "MemMan.h"
#include "hoap3_io.h"


int send_commands(unsigned short *motor_pos, unsigned char *sensors, DUNIT* unit)
{
	int i;

	if(!unit) return -1;

	for(i = 0; i < NUM_MOTORS; i++) {
		unit->Ctl.MtrS[i] = motor_pos[i];
	}

	for(i = 0; i < NUM_SENSORS; i++) {
		unit->Ctl.SnsS[i] = sensors[i];
	}

	unit->Step = (unsigned short) 2;
	unit->Header = (unsigned char) 2;
	unit->StepOver = 0;
	unit->ResTime = 0;

	return 0;
}

int read_positions(DUNIT *unit, unsigned short *positions)
{
	int i;
	for(i = 0; i < NUM_MOTORS-2; i++) {
		positions[i]= unit->Ctl.MtrRp[i];
	}
	return 0;
}


int readret(DUNIT* unit)
{
	while(1) {
		if(pSM->VarIF.ResRep) {
			break;
		}
	}
	return 0;
}

void robot2DH (unsigned short *positions, float *angles)
{
 float x;
 x= PI/180/209;
 angles[0]= (short)positions[0]*x;
 angles[1]=-(short)positions[1]*x;
 angles[2]=-(short)positions[2]*x;
 angles[3]=-(short)positions[3]*x;
// angles[4]= (float)((short)positions[6]-60-256)*PI/180;
}

void DH2robot (float *angles, unsigned short *positions)
{
 	 float x=180/PI*209;
 	 //float angle4=angles[4]*180/PI;
 	 positions[0]=(unsigned short) (angles[0]*x);
 	 positions[1]=-(unsigned short) (angles[1]*x);
 	 positions[2]=-(unsigned short) (angles[2]*x);
 	 positions[3]=-(unsigned short) (angles[3]*x);
 	 //if ((angle4>=-60) & (angle4<=60)) positions[6]=(unsigned short)(angle4+60);
}

int inverse_kinematics (float Ts, float T, matrix *xd, matrix *xd_dot, DUNIT *unit)
{
matrix Kp;
matrix J;
matrix J_t,J_1,J_i,J_pi,J_m,J_s,I;
int iterations, i, j;
int ret,pulsa;
float p[3];
float error_p[3];
float q[5], q0[5];
float dq[5];
float v0[3], v1[3];
float v2[5], v3[5];
unsigned short motor[NUM_MOTORS];
unsigned char sensor[NUM_SENSORS];

sensor[0] = 'R';
sensor[1] = 'R';
sensor[2] = 'R';
sensor[3] = 'R';

//matrix Ko, error_o, R;
initialize_matrix (&Kp, 3,3);
initialize_matrix (&J, 3,5);
initialize_matrix (&J_t, 5, 3);
initialize_matrix (&J_1, 3,3);
initialize_matrix (&J_i, 3,3);
initialize_matrix (&J_pi, 5, 3);
initialize_matrix (&J_m, 5, 5);
initialize_matrix (&J_s, 5, 5);
initialize_matrix (&I, 5, 5);
//initialize_matrix (&Ko, 3,3);

create_diagonal_matrix (&Kp, 200);
create_diagonal_matrix (&I, 1);
//create_diagonal_matrix (&Ko, 5);

iterations = T/Ts+1;
q[4]=0;

for (i=0; i<iterations; i++)
	{
 	read_positions(unit, motor);
	robot2DH (&motor[6], q);
	
//	for(j = 0; j < 5; j++) {
//		printf("\n q[%d]:  %f",j+1, q[j]);
//	}

    	//present position
    	evaluate_position(q, p);
//	printf("\nPresent position: (%f,%f,%f)\n",p[0],p[1],p[2]);

    	//evualate position error
    	vector_subtraction (&xd->vector[identify_element(xd, i, 0)], p, error_p, 3);

	//present velocity
	evaluate_jacobian(q, &J);
	matrix_transpose (&J, &J_t);
	matrix_multiply(&J, &J_t, &J_1);
	matrix_inverse (&J_1, &J_i);
  	matrix_multiply(&J_t, &J_i, &J_pi);
	matrix_vector_multiply(&Kp, error_p, v0);
	vector_sum (&xd_dot->vector[identify_element(xd_dot, i, 0)], v0, v1, 3);
	matrix_vector_multiply(&J_pi, v1, v2);

	matrix_multiply(&J_pi, &J, &J_m);
	matrix_diff(&I,&J_m,&J_s);
	evaluate_null_vector(q, q0);
	matrix_vector_multiply(&J_s, q0, v3);
	vector_sum (v2, v3, dq, 5);
	//printf("\nVector nullo:  ");
	//for (j=0;j<5;j++) printf("%f, ",dq[j]);

	for (j=0;j<5;j++) q[j]+=dq[j]*Ts;
	//q[4]+=0.1;

	unit = (DUNIT*)&(pSM->Data.IntDat);
	DH2robot (q, &motor[6]);
	motor[0] = 0;
	motor[1] = 0;
	motor[2] = 2100;
	motor[3] = 4200;
	motor[4] = -2500;
	motor[5] = 0;
	//motor[6] = 18100;
	//motor[7] = -2100;
	//motor[8] = 1000;
	//motor[9] = 6200;
	motor[10] = 0;
	motor[11] = 0;
	motor[12] = -2100;
	motor[13] = -4180;
	motor[14] = 2500;
	motor[15] = 0;
	motor[16] = -18810;
	motor[17] = 2000;
	motor[18] = -1000;
	motor[19] = -6200;
	motor[20] = 2100;
	motor[21] = 60;
	motor[22] = 60;
//	for(j = 0; j < NUM_MOTORS; j++) {
//		printf("\n ID: %d:  %hd",j+1, (short) motor[j]);
//	}
//	scanf("%d",pulsa);

	ret = send_commands(motor, sensor, unit);
	pSM->VarIF.ResRep = 0;
	pSM->VarIF.Mode = IDLE;
	pSM->VarIF.InterruptSend = TRUE;
	//watch return
	//printf("waiting for return...\n");
	readret(unit);

	//printf("\nVector q at time %d:",i);
	}
read_positions(unit, motor);
robot2DH (&motor[6], q);

evaluate_position(q, p);
printf("\nFinal angles:\n");
for (j=0;j<5;j++) printf("q(%d):%f,",j,q[j]);
free(Kp.vector);
free(J.vector);
free(J_1.vector);
free(J_i.vector);
free(J_t.vector);
free(J_pi.vector);
}

int select_desired_position (float *p)
{
 	char choise[5];
 	printf("\nDesired x position: ");
 	scanf("%s",choise);
 	if (strcmp(choise,"q")==0) return -1;
 	if (strcmp(choise,"grasp")==0) return 1;
 	if (strcmp(choise,"leave")==0) return 2;
 	p[0]=atof(choise);
 	printf("\nDesired y position: ");
 	scanf("%s",choise);
 	if (strcmp(choise,"q")==0) return -1;
 	p[1]=atof(choise);
 	printf("\nDesired z position: ");
 	scanf("%s",choise);
 	if (strcmp(choise,"q")==0) return -1;
 	p[2]=atof(choise);
 	return 0;
}

void close_hand (unsigned short *motor, unsigned char *sensor, DUNIT* unit)
{
 	int ret;
	unit = (DUNIT*)&(pSM->Data.IntDat);
	motor[22] = 632;
	ret = send_commands(motor, sensor, unit);
	pSM->VarIF.ResRep = 0;
	pSM->VarIF.Mode = IDLE;
	pSM->VarIF.InterruptSend = TRUE;
	//watch return
	//printf("waiting for return...\n");
	readret(unit);
 }

void open_hand (unsigned short *motor, unsigned char *sensor, DUNIT* unit)
{
 	int ret;
	unit = (DUNIT*)&(pSM->Data.IntDat);
	motor[22] = 572;
	ret = send_commands(motor, sensor, unit);
	pSM->VarIF.ResRep = 0;
	pSM->VarIF.Mode = IDLE;
	pSM->VarIF.InterruptSend = TRUE;
	//watch return
	//printf("waiting for return...\n");
	readret(unit);
 }
