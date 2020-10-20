#include <stdio.h>
#include <errno.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "mpc_interface.h"

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}


int main(int argc, char * argv[]) {
	int mpc_shmid;
	struct shared_data * mpc_data;
	char choice;
	
	/* Getting the MPC data */
	mpc_shmid = shmget(MPC_SHM_KEY, 0, 0);
	if (mpc_shmid == -1) {
		PRINT_ERROR("shmget of MPC failed");
	}
	mpc_data = (struct shared_data *)shmat(mpc_shmid, NULL, 0);

	printf("Currently executing MPC: %s\n",
	       MPC_OFFLOAD_IS_ENABLED(mpc_data) ? "SERVER" : "LOCAL");
	printf("Please enter your choice to change it [S/L]: ");
	scanf("%c", &choice);
	switch (choice) {
	case 's':
	case 'S':
		MPC_OFFLOAD_ENABLE(mpc_data);
		break;
	case 'l':
	case 'L':
		MPC_OFFLOAD_DISABLE(mpc_data);
		break;
	}
	printf("Currently executing MPC: %s\n",
	       MPC_OFFLOAD_IS_ENABLED(mpc_data) ? "SERVER" : "LOCAL");
}


