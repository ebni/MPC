#define _GNU_SOURCE
/* 
 * on EB's: no MPC, min period 200msec close to full utilization
 */
#define RM_PERIOD_NSEC 10000000 /* 10 msec */
#define RM_MAX_NOPENDING_TO_ONBOARD 10
#define RM_ENABLE_OFFLOAD
#define LOG_REQS_FILE "log_reqs.csv"
#define LOG_RM_FILE "log_rm.csv"
#define STRLEN_COMMAND 100
/*#define DEBUG_NOPINNING */

#include <stdint.h>
#include <unistd.h>
#include <sched.h>
#include <semaphore.h>
#include <signal.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <strings.h>
#include <time.h>
#include <math.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "app_workload.h"
#include "mpc_interface.h"

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}

typedef struct job_record {
	struct timespec job_release;   /* job release */
	struct timespec job_start;     /* job start */
	struct timespec job_finish;    /* job start */
} job_record;


/* GLOBAL VARIABLES (used in handler) */
int mem_recs_id;
job_record * recs_all;


/*
 * Set prio priority (high number => high priority) and pin the
 * invoking process to CPU cpu_id
 */
void sched_set_prio_affinity(uint32_t prio, int cpu_id);

/*
 * Handler of the Ctrl-C signal (SIGINT).
 */
void term_handler(int signum);

/*
 * Dummy function to simulate the processing of any request
 */
void process_request(void);


/*
 * argv[1]: filename of a CSV file with the following format. On each row:
 *
 *   <integer>,<float>: <integer> is the number of requests separated
 *   by <float> seconds
 */
int main(int argc, char * argv[]) {
	struct sigaction sa;
	worker_data * reqs_info;
	FILE * logfile, *csv_infile;
	size_t i,j,id, job_num=0;
	pid_t child_pid, releaser_pid;
#ifdef APP_USE_RM
	pid_t manager_pid;
#endif
	unsigned long cur_req, req_len=0, *req_howmany = NULL;
	double cur_sep;
	struct timespec * req_period = NULL;

	/* Reading the CSV file from argv[1] */
	csv_infile = fopen(argv[1],"r");
	if (csv_infile == NULL) {
		PRINT_ERROR("unable to open file");
		exit(-1);
	}
	while (fscanf(csv_infile,"%lu,%lf", &cur_req, &cur_sep) == 2) {
		req_len++;
		req_howmany= realloc(req_howmany, req_len*sizeof(*req_howmany));
		req_period = realloc(req_period, req_len*sizeof(*req_period));
		req_howmany[req_len-1] = cur_req;
		req_period[req_len-1].tv_sec = (time_t)floor(cur_sep);
		req_period[req_len-1].tv_nsec = (long)((cur_sep-floor(cur_sep))*1e9);
		job_num += cur_req;
	}
	fclose(csv_infile);

	/* Shared memory for recording the progress of the workload */
	mem_recs_id = shmget(APP_SHM_KEY,
			     sizeof(worker_data)+
			     sizeof(job_record)*job_num,
			     APP_SHM_FLAGS | IPC_CREAT | IPC_EXCL);
	if (mem_recs_id == -1) {
		PRINT_ERROR("Unable to create shared memory");
		exit(EXIT_FAILURE);
	}
	reqs_info = (worker_data *)shmat(mem_recs_id, NULL, 0);
	bzero(reqs_info, sizeof(*reqs_info));
	recs_all  = (job_record *)(reqs_info+1);
	bzero(recs_all, sizeof(*recs_all)*job_num);
	/* no pending requests */
	if (sem_init(&reqs_info->pending, 1, 0) == -1) {
		PRINT_ERROR("Unable to init semaphore for pending requests");
		exit(EXIT_FAILURE);
	}
	/* mutex for next_req available */
	if (sem_init(&reqs_info->mutex_next_req, 1, 1) == -1) {
		PRINT_ERROR("Unable to init semaphore for mutex");
		exit(EXIT_FAILURE);
	}

	/* creating a process group */
	if (setpgid(0, 0) == -1) {
		PRINT_ERROR("problems with setpgid in parent");
		exit(-1);
	}

	/* Creating workers */
	for (i=0; i<APP_WORKER_NUM; i++) {
		child_pid = fork();
		if (child_pid == 0) {
			/* WORKER PROCESSES: START */
			unsigned long id_req;

#ifndef DEBUG_NOPINNING
			/* Pinning workers onto same CPU of MPC */
			sched_set_prio_affinity(
				sched_get_priority_max(SCHED_FIFO)-1,
				MPC_CPU_ID);
#endif
			
			/* get pointers */
#if 0 /* inherited from parent ? */			
			recs_all  = (job_record *)shmat(mem_recs_id,
							       NULL, 0);
			reqs_info = (worker_data *)(recs_all+job_num);
#endif
			while(1) {
				/* Wait for new requests */
				if (sem_wait(&reqs_info->pending) == -1) {
					PRINT_ERROR("error in sem_wait");
					exit(-1);
				}

				/* Get the mutex-protected id of the request */
				sem_wait(&reqs_info->mutex_next_req);
				id_req = reqs_info->next_req;
				reqs_info->next_req++;
				sem_post(&reqs_info->mutex_next_req);

				/* Mark start, process, then mark end */
				clock_gettime(CLOCK_REALTIME,
					      &recs_all[id_req].job_start);
				process_request();
				clock_gettime(CLOCK_REALTIME,
					      &recs_all[id_req].job_finish);
				if (reqs_info->next_req >= job_num) {
					sleep(1);
					exit(0);
				}
			}
			/* WORKER PROCESSES: END */
		} else {
			/* assign child process to parent group */
			if (setpgid(child_pid, getpid()) == -1) {
				PRINT_ERROR("problems with setpgid of child");
				exit(-1);
			}
		}
	}

#ifndef DEBUG_NOPINNING
	/* All next stuff should not interfere with MPC */
	sched_set_prio_affinity(sched_get_priority_max(SCHED_FIFO),
				MPC_CPU_ID-1);
#endif
	releaser_pid =fork();
	if (releaser_pid == 0) {
		/* JOB RELEASER PROCESS: START */
		/* FIXME: set affinity to a different core than MPC */
		
#if 0 /* inherited from parent ? */			
		/* get pointers */
		recs_all  = (job_record *)shmat(mem_recs_id,
						       NULL, 0);
		reqs_info = (worker_data *)(recs_all+job_num);
#endif
		
		for (i=0, id=0; i<req_len; i++) {
			for (j=0; j<req_howmany[i]; j++, id++) {
				nanosleep(req_period+i, NULL);
				clock_gettime(CLOCK_REALTIME,
					      &recs_all[id].job_release);
				sem_post(&reqs_info->pending);
			}
		}
		exit(0);
		/* JOB RELEASER PROCESS: END */
	}

#ifdef APP_USE_RM
	manager_pid = fork();
	if (manager_pid == 0) {
		execl("./manager", "manager", (char *) NULL);
		PRINT_ERROR("execl didn't work");
		exit(-1);
	} else {
		/* Manager belongs to same group ID */
		if (setpgid(manager_pid, getpid()) == -1) {
			PRINT_ERROR("problems with setpgid of child");
			exit(-1);
		}
	}
#endif /* APP_USE_RM */

	/* Setting up the signal handler for Ctrl-C */
	bzero(&sa, sizeof(sa));
	sa.sa_handler = term_handler;
	sigaction(SIGINT, &sa, NULL);

	/* First waiting for the releaser to terminate... */
	waitpid(releaser_pid, NULL, 0);

	/* ... then waiting for the last worker to terminate */
	wait(NULL);
	/* Saving the processing times onto a file */
	logfile = fopen(LOG_REQS_FILE, "w");
	for (i=0; i<job_num; i++) {
		fprintf(logfile, "%lu,%lu.%09lu,%lu.%09lu,%lu.%09lu,\n", i,
			recs_all[i].job_release.tv_sec,
			recs_all[i].job_release.tv_nsec,
			recs_all[i].job_start.tv_sec,
			recs_all[i].job_start.tv_nsec,
			recs_all[i].job_finish.tv_sec,
			recs_all[i].job_finish.tv_nsec);
	}
	fclose(logfile);
	
	killpg(0, SIGINT);
}

void term_handler(int signum)
{
	/* Removing shared memory object */
	shmctl(mem_recs_id, IPC_RMID, NULL);
	printf("Got SIGINT (Ctrl-C). Removed shared memory\n");
	killpg(0, SIGINT);
	exit(0);
}

void process_request()
{
	unsigned long i;
	double var = 2;

	/* Just anything to keep CPU busy hard to optimize by the
	 * compiler */
	for (i=0; i<APP_DUMMY_ITER_NUM; i++) {
		var = 1/var;
	}
}

void sched_set_prio_affinity(uint32_t prio, int cpu_id)
{
	cpu_set_t  mask;

	/* Set CPU affinity */
	CPU_ZERO(&mask);
	CPU_SET(cpu_id, &mask);
	if (sched_setaffinity(0, sizeof(mask), &mask) != 0) {
		PRINT_ERROR("sched_setaffinity");
		exit(-1);
	}

	/* Set priority */
#if SCHED_SETATTR_IN_SCHED_H
	/* EB: TO BE TESTED */
	struct sched_attr attr;
	
	bzero(&attr, sizeof(attr));
	attr.size = sizeof(attr);
	attr.sched_policy = SCHED_FIFO;
	attr.sched_priority = prio;
	if (sched_setattr(0, &attr, 0) != 0) {
		PRINT_ERROR("sched_setattr");
		exit(-1);
	}
#else
	char launched[STRLEN_COMMAND];  /* String with launched command */

	snprintf(launched, STRLEN_COMMAND,
		 "sudo chrt -f -p %d %d", prio, getpid());
	system(launched);
#endif
}
