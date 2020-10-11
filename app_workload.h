#define APP_WORKER_NUM 1
#define APP_DUMMY_ITER_NUM 40000000
/* 
 * on EB's: no MPC, min period 200msec close to full utilization
 */
#define APP_LOG_FILE "log_reqs.csv"
#define APP_SHM_KEY 0xF1D0  /* hard-coded key of shared mem SysV object */
#define APP_SHM_FLAGS 0666  /* we go easy: everybody reads and writes */
/* resource manager is forked by app_workload */
#define APP_USE_RM

#include <semaphore.h>

struct worker_data {
	sem_t pending;
	sem_t mutex_next_req;
	unsigned long next_req;
	/* An array of JOB_NUM struct job_record and then */
};
