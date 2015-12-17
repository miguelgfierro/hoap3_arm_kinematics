#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <mbuff.h>
#include <MemMan.h>
#define NAME_OF_MEMORY "DataSharedMemory"
SM* pSM;

#include "hoap3_io.h"
#include "kinematics.h"
#include "matrixes.h"


int main(int argc, char *argv[])
{
	int ret;
	int choice;
	int i, j;
	float Ts=0.002;
	float T;
	float vm=0.05; //mean speed
	matrix xd, xd_dot, J;
	float q0[5], q_final[5];
	float p0[3];
	float p1[3];
	
	DUNIT *unit;
	unsigned short motor[NUM_MOTORS];
	unsigned char sensor[NUM_SENSORS];
	
	pSM = (SM*) mbuff_alloc(NAME_OF_MEMORY, sizeof(SM));
	if(pSM == NULL) {
		perror("mbuff_alloc failed");
		return -1;
	}
	
	// Header = 1,;
	// Step = 2,
	//motor[0] = 0;
	motor[1] = 0;
	motor[2] = 2100;
	motor[3] = 4200;
	motor[4] = -2500;
	motor[5] = 0;
	motor[6] = 18100;
	motor[7] = -2100;
	motor[8] = 1000;
	motor[9] = 6200;
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
	
	sensor[0] = 'R';
	sensor[1] = 'R';
	sensor[2] = 'R';
	sensor[3] = 'R';

	unit = (DUNIT*)&(pSM->Data.IntDat);
	ret = send_commands(motor, sensor, unit);
	pSM->VarIF.ResRep = 0;
	pSM->VarIF.Mode = IDLE;
	pSM->VarIF.InterruptSend = TRUE;
	//watch return
	readret(unit);

  	//desired final configuration
  	//q_final[0]=PI*3/5;
  	//q_final[1]=PI/3;
  	//q_final[2]=PI/6;
  	//q_final[3]=-PI/4;
	//q_final[4]=0;
	//evaluate_position(q_final, p1);	
	q0[4]=0;
	open_hand (motor, sensor, unit);
	while (1)
		  {
		  //present position
		  read_positions(unit, motor);
		  robot2DH (&motor[6], q0);
		  evaluate_position(q0, p0);

  		  printf("\nPresent position: (%f,%f,%f)\n",p0[0],p0[1],p0[2]);
		  
		  //select desired position
		  choice=select_desired_position (p1);
		  if (choice==-1) break;
		  if (choice==1) close_hand (motor, sensor, unit);
		  if (choice==2) open_hand (motor, sensor, unit);
		  printf("\nDesired final position: (%f,%f,%f)\n",p1[0],p1[1],p1[2]);
		  
		  //execution time
		  T=evaluate_execution_time (p0, p1, vm);
		  
		  //initializing matrixes
		  initialize_matrix (&xd, T/Ts+1, 3);
		  initialize_matrix (&xd_dot, T/Ts+1, 3);
		  initialize_matrix (&J, 3, 5);

		  //evaluate desired trajectory
		  trajectory_right_arm (Ts, T, p0, p1, &xd, &xd_dot);
		  
		  //inverse kinematics
		  inverse_kinematics (Ts, T, &xd, &xd_dot, unit);
		  }
	
	//close_hand (motor, sensor, unit);  
	free(xd.vector);
	free(xd_dot.vector);
	free(J.vector);
		
	//free memory
	mbuff_free(NAME_OF_MEMORY, (void*)pSM);

	return 0;
}
