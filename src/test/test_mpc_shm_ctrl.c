/**
 * @file test_mpc_shm_ctrl.c
 * @author Enrico	Bini
 * @brief this code was used to test the shared memory
 *  // BUG:delete me 
 */
#include <time.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include "../../include/mpc_interface.h"

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}

int main() {	
	int shm_id;
	shared_data * data;
	struct timespec tic, toc;
	size_t i;
	double time_mpc=0;
	/* 
	 * Try changing the values below to test MPC with different
	 * initial state
	 */
	double state_test[] = {0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
	if (sizeof(state_test)/sizeof(state_test[0]) != MPC_STATE_NUM) {
		PRINT_ERROR("Warning: state dimension mismatch");
	}
	
	shm_id = shmget(MPC_SHM_KEY, 0, 0);
	if (shm_id == -1) {
		PRINT_ERROR("shmget failed");
		return -1;
	}
	data = (shared_data *)shmat(shm_id, NULL, 0);
	
	/* Create a pointer to the real data in the state vector  */
	memcpy(data->state, state_test, sizeof(state_test[0])*MPC_STATE_NUM);

	clock_gettime(CLOCK_MONOTONIC, &tic);
	/* Now asking MPC to compute the optimal input for us... */
	sem_post(data->sems+MPC_SEM_STATE_WRITTEN);
	/* ... and waiting to get a response */
	sem_wait(data->sems+MPC_SEM_INPUT_WRITTEN);
	clock_gettime(CLOCK_MONOTONIC, &toc);

	printf("MPC done\nstate:");
	for (i=0; i<MPC_STATE_NUM; i++) {
		printf("\t%5.2f", data->state[i]);
	}
	printf("\ninput:");
	for (i=0; i<MPC_INPUT_NUM; i++) {
		printf("\t%5.2f", data->input[i]);
	}
	time_mpc  = toc.tv_sec-tic.tv_sec;
	time_mpc += (toc.tv_nsec-tic.tv_nsec)*1e-9;
	printf("\ntime:\t%f\n", time_mpc);

	/* Finally detaching shared memory */
	shmdt(data);

}
