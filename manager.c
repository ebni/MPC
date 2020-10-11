#define _GNU_SOURCE
#define RM_PERIOD_NSEC 10000000 /* 10 msec */
#define RM_MAX_NOPENDING_TO_ONBOARD 10
#define RM_ENABLE_OFFLOAD
#define RM_LOGFILE "log_rm.csv"

#define RM_LOG_TIMESTAMP_ONBOARD(f,t)	\
	fprintf(f,"%lu.%09lu,0,\n",	\
		t.tv_sec, t.tv_nsec);
#define RM_LOG_TIMESTAMP_OFFLOAD(f,t)	\
	fprintf(f,"%lu.%09lu,1,\n",	\
		t.tv_sec, t.tv_nsec);


/*#define DEBUG_NOMPC */
#define STRLEN_COMMAND 100
/*#define DEBUG_NOPINNING */

#include <signal.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <strings.h>
#include <time.h>
#if 0
#include <sched.h>
#include <semaphore.h>
#include <math.h>
#include <sys/types.h>
#include <sys/wait.h>
#endif

#include "mpc_interface.h"
#include "app_workload.h"

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}


/* GLOBAL VARIABLES (used in handler) */
int app_shmid, mpc_shmid;
struct worker_data * app_data;
struct shared_data * mpc_data;
FILE * logrm;

/*
 * Handler of the Ctrl-C signal (SIGINT).
 */
void term_handler(int signum);


/*
 * argv[1]: ID of the shared memory with pending requests
 */
int main(int argc, char * argv[]) {
	struct sigaction sa;
	struct timespec rm_period, rm_change;
	int reqs_pending;
	int seq_no_pend = -1;
	/*
	 * seq_no_pend == -1: on-board
	 * seq_no_pend >= 0: off-loading and seq_no_pend is
	 *     equal to the number of consecutive samples of
	 *     zero pending requests
	 */

	/* Get the shared memory */
	app_shmid = shmget(APP_SHM_KEY, 0, 0);
	if (app_shmid == -1) {
		PRINT_ERROR("issues in shmget");
		exit(-1);
	}
	app_data = (struct worker_data *)shmat(app_shmid, NULL, 0);
	if (app_data == (void *)-1) {
		PRINT_ERROR("issues in shmat");
		exit(-1);
	}

	/* Init period of resource manager */
	rm_period.tv_sec = 0;
	rm_period.tv_nsec = RM_PERIOD_NSEC;

	/* Init log of RM */ 
	logrm = fopen(RM_LOGFILE,"w");
	if (logrm == NULL) {
		PRINT_ERROR("issues in fopen");
		exit(-1);
	}

	/* Setting up the signal handler for Ctrl-C */
	bzero(&sa, sizeof(sa));
	sa.sa_handler = term_handler;
	sigaction(SIGINT, &sa, NULL);

	/* Getting the MPC data */
	mpc_shmid = shmget(MPC_SHM_KEY, 0, 0);
	if (mpc_shmid == -1) {
		PRINT_ERROR("shmget of MPC failed");
	}
	mpc_data = (struct shared_data *)shmat(mpc_shmid, NULL, 0);
	MPC_OFFLOAD_DISABLE(mpc_data);

	/* Let's go */
	clock_gettime(CLOCK_REALTIME,
		      &rm_change);
	RM_LOG_TIMESTAMP_ONBOARD(logrm,rm_change);
	while (1) {  /* Loop forever. Ctrl-C will terminate */
		nanosleep(&rm_period, NULL);
		sem_getvalue(&app_data->pending,&reqs_pending);
		if (MPC_OFFLOAD_IS_ENABLED(mpc_data)) {
			/* currently off-loading */
			if (reqs_pending) {
				/* off_loading and pending reqs */
				continue;
			}
			if (seq_no_pend>=RM_MAX_NOPENDING_TO_ONBOARD) {
				MPC_OFFLOAD_DISABLE(mpc_data);
				clock_gettime(CLOCK_REALTIME, &rm_change);
				RM_LOG_TIMESTAMP_ONBOARD(logrm,rm_change);
				seq_no_pend = -1;
			} else {
				seq_no_pend++;
			}
		} else {
			/* currently on-board */
			if (reqs_pending) {
				MPC_OFFLOAD_ENABLE(mpc_data);
				clock_gettime(CLOCK_REALTIME, &rm_change);
				RM_LOG_TIMESTAMP_OFFLOAD(logrm,rm_change);
				seq_no_pend = 0;
			}
		}
	}
}

void term_handler(int signum)
{
	printf("Got SIGINT (Ctrl-C). Closing RM and detaching\n");
	fclose(logrm);
	shmdt(mpc_data);
	shmdt(app_data);
	exit(0);
}

