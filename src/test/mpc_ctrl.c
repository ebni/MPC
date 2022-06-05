/*
 * mpc_ctrl.c
 *
 * MPC controller. It must be initialized with a JSON model which must
 * be passed as first parameter (argv[1]).
 * Below the invocation arguments:
 *
 *   argv[1], filename of the JSON file describing th problem [MANDATORY]
 *
 *   argv[2],  IP  address  of  the  MPC  server  [OPTIONAL].  If  not
 *   specified,  the  value  of  the macro  MPC_SOLVER_IP  defined  in
 *   mpc_interface.h is assumed
 */

/**
 * @file mpc_ctrl.c
 * @author Enrico	Bini
 * @brief MPC controller. It must be initialized with a JSON model which must
 * be passed as first parameter (argv[1]).
 * @note argv[1], filename of the JSON file describing th problem [MANDATORY]
 * @note argv[2],  IP  address  of  the  MPC  server  [OPTIONAL].  If  not
 *   specified,  the  value  of  the macro  MPC_SOLVER_IP  defined  in
 *   mpc_interface.h is assumed
 */

#define _GNU_SOURCE
#include "../../include/mpc_interface.h"
#define STRLEN_COMMAND 100

//#define PRINT_LOG
#define MPC_STATUS_X0_ONLY
/*
 * Below are some #define which trigger something:
 *
 * PRINT_LOG, print log information at every iteration
 *
 * PRINT_PROBLEM, print the problem formulation in txt files
 *
 * DEBUG_SIMPLEX, turn on all Simplex messages for debugging
 *
 * MPC_STATUS_X0_ONLY, touch only x0, not basis stuff
 */
/*
#define PRINT_LOG
#define PRINT_PROBLEM
#define DEBUG_SIMPLEX
#define MPC_STATUS_X0_ONLY
*/

#include <sys/ipc.h>
#include <sys/shm.h>
#include <signal.h>
#include <sched.h>
#include <errno.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <limits.h> 
#include <stdio.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "../../include/mpc.h"

//!stuff used for tracing
#include <sys/stat.h>
#include <fcntl.h>
#define MARKER_FILE "/sys/kernel/tracing/trace_marker"
#define BUF_LEN 300


/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}

/* GLOBAL VARIABLES (used in handler) */
int shm_id;


/*
 * Set prio priority (high number => high priority) and pin the
 * invoking process to CPU cpu_id
 */

/**
 * @brief Set prio priority (high number => high priority) and pin the
 * invoking process to CPU cpu_id
 * 
 * @param prio   uint32_t the priority of the process 
 * @param cpu_id int id of the process
 */
void sched_set_prio_affinity(uint32_t prio, int cpu_id);

/*
 * Initializing the model with JSON file
 */

/**
 * @brief  Initializing the model with JSON filr 
 * 
 * @param mpc mpc_glpk* contain the representation of the MPC problem
 * @param in  json_object* json object containing initialization data read from file
 * @return int 0 if no error 
 */
int model_mpc_startup(mpc_glpk * mpc, json_object * in);

/*
 * Signal handler. This process will terminate only on Ctrl-C. It will
 * also terminate on other standard terminating signals. Upon process
 * termination, the shared memory is removed.
 */

/**
 * @brief Signal handler. This process will terminate only on Ctrl-C. It will
 * also terminate on other standard terminating signals. Upon process
 * termination, the shared memory is removed.
 * 
 * @param signum int  
 */
void term_handler(int signum);

/*
 * Handling  the segmentation  fault  signal (SIGSEGV).  By setting  a
 * breakpoint within  the signal handler,  it is then possible  to see
 * what is the line of code which generated the error.
 */

/**
 * @brief Handling  the segmentation  fault  signal (SIGSEGV).  By setting  a
 * breakpoint within  the signal handler,  it is then possible  to see
 * what is the line of code which generated the error.
 * 
 * @param signum int 
 */
void seg_fault_handler(int signum);
void print_mark(const char *msg);



int main(int argc, char * argv[]) {
	shared_data * data;
	double * shared_state;
	double * shared_input;
	int model_fd;
	char * buffer;
	ssize_t size;
	size_t i;

	json_object *model_json;
	json_tokener* tok;

	struct sigaction sa;

	mpc_status * mpc_st;
	mpc_glpk my_mpc;
	int sockfd;
	struct sockaddr_in servaddr;
	#ifdef PRINT_PROBLEM
		char s_sol[100] = SOL_FILENAME;
		char tmp[100];
	#endif
	#ifdef PRINT_LOG
		char * log_rec;
		size_t offset_rec;
		struct timespec after_wait, before_post;
	#define LOG_REC_SIZE (2*(30)+                   \
				15*data->state_num+	\
				15*data->input_num)	 
	#endif /* PRINT_LOG */
	

	if (argc <= 1) {
		PRINT_ERROR("Too few arguments. At least 1 needed: <JSON model>");
		return -1;
	}
	  
	/* Reading the JSON file with the problem model */
	if ((model_fd = open(argv[1], O_RDONLY)) == -1) {
		PRINT_ERROR("Missing/wrong JSON file");
		return -1;
	}
	
	/* Getting the size of the file */
	size = lseek(model_fd, 0, SEEK_END);
	lseek(model_fd, 0, SEEK_SET);

	/* Allocate the buffer and store data */
	buffer = malloc((size_t)size);
	size = read(model_fd, buffer, (size_t)size);
	close(model_fd);
	tok = json_tokener_new();
	model_json = json_tokener_parse_ex(tok, buffer, (int)size);
	free(buffer);

	/* Setting up the signal handler for termination */

	bzero(&sa, sizeof(sa));
	sa.sa_handler = term_handler;
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGPIPE, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGSEGV, &sa, NULL);

	/* Pinning MPC to a fixed CPU */
	sched_set_prio_affinity(sched_get_priority_max(SCHED_FIFO),
				MPC_CPU_ID);

	/* Initializing the model */
	#ifdef TRACING
	print_mark("CLIENT: @model_mpc_startup# - start");
	#endif
	model_mpc_startup(&my_mpc, model_json);
	#ifdef TRACING
	print_mark("CLIENT: @model_mpc_startup# - end");
	#endif
 	/* 
	 * Shared memory is used to read state from and write input to
	 * the  plant. Allocating  enough  space for  both the  struct
	 * shared_data and the two arrays for state/input.
	 */
	shm_id = shmget(MPC_SHM_KEY,
			sizeof(*data)+
			sizeof(*shared_state)*my_mpc.model->n+
			sizeof(*shared_input)*my_mpc.model->m,
			MPC_SHM_FLAGS | IPC_CREAT | IPC_EXCL);
	if (shm_id == -1) {
		PRINT_ERROR("Unable to create shared memory. Maybe key in use (try ipcs)");
		exit(EXIT_FAILURE);
	}
	data = (shared_data *)shmat(shm_id, NULL, 0);
	shared_state = (double*)(data+1); /* starts just after *data */
	shared_input = shared_state+my_mpc.model->n;
	bzero(data, sizeof(*data)+
		sizeof(*shared_state)*my_mpc.model->n+
	    sizeof(*shared_input)*my_mpc.model->m);
	data->state_num = my_mpc.model->n;
	data->input_num = my_mpc.model->m;
	MPC_OFFLOAD_ENABLE(data);
	
	/* Resetting all semaphores */
	for (i=0; i<MPC_SEM_NUM; i++) {
		if (sem_init(data->sems+i,1,0) < 0) {
			PRINT_ERROR("issue in sem_init");
			exit(EXIT_FAILURE);
		}
	}
	
	#ifdef PRINT_PROBLEM
		glp_print_sol(my_mpc.op, "000glpk_sol.txt");
	#endif

	/* Allocating struct of solver status after problem defined */
	#ifdef TRACING
	print_mark("CLIENT: @mpc_status_alloc# - start");
	#endif
	mpc_st = mpc_status_alloc(&my_mpc);
	#ifdef TRACING
	print_mark("CLIENT: @mpc_status_alloc# - end");
	#endif
	#ifdef PRINT_PROBLEM
		/* Save initial status */
		mpc_status_save(&my_mpc, mpc_st);
		fprintf(stdout, "Initial status\n");
		mpc_status_fprintf(stdout, &my_mpc, mpc_st);
		glp_write_lp(my_mpc.op, NULL, "initial_mpc.txt");
		glp_print_sol(my_mpc.op, "initial_sol.txt");
	#endif

	/* Setting up the socket to server */
	bzero(&servaddr, sizeof(servaddr)); 
	if (argc >= 3) {
		/* using command-line arg as IP address */
		servaddr.sin_addr.s_addr = inet_addr(argv[2]);
		/*
		if(arg[3]==1){
			TODO: MPC_PREDICTIVE_MODE_ENABLED(data);
		}
		
		*/
	} else {
		/* using the default IP address */
		servaddr.sin_addr.s_addr = inet_addr(MPC_SOLVER_IP);
	}
	if (servaddr.sin_addr.s_addr == INADDR_NONE) {
		PRINT_ERROR("invalid IP address");
		exit(-1);
	}
	servaddr.sin_port = htons(MPC_SOLVER_PORT);
	servaddr.sin_family = AF_INET;

	/* create and connect UPD socket */
	sockfd = socket(AF_INET, SOCK_DGRAM, 0); 
	if(connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
		PRINT_ERROR("client: error in connect");

	#ifdef PRINT_LOG
		log_rec = malloc(LOG_REC_SIZE); //BUG: replace with calloc
		bzero(log_rec, LOG_REC_SIZE);
	#endif
        printf("MPC Ctrl start...\n");
	/* 
	 * Cycling forever to get the state of the plant. Ctrl-C will
	 * terminate
	 */
	while (1) {
		/* Blocked until the system wrote the state in shared_state */
		sem_wait(data->sems+MPC_SEM_STATE_WRITTEN);
		clock_gettime(CLOCK_REALTIME, &after_wait);

		/* Store the lastest solver status in mpc_st */
		#ifndef MPC_STATUS_X0_ONLY
			mpc_status_save(&my_mpc, mpc_st);
			*mpc_st->steps_bdg = INT_MAX;  /* max iterations */
			*mpc_st->time_bdg = INT_MAX;   /* max seconds */
			/* Setting the status of cur solution */
			*mpc_st->prim_stat = GLP_INFEAS;
			*mpc_st->dual_stat = GLP_FEAS;
		#endif /* MPC_STATUS_X0_ONLY */
		memcpy(mpc_st->state, shared_state,
		       sizeof(*shared_state)*data->state_num);
		if (data->flags & MPC_OFFLOAD) {
			/* MPC offloaded to server */
			data->stats_int[MPC_STATS_INT_OFFLOAD] = 1;
			
			/* Sending/receiving status to/from server */
			#ifdef TRACING
			print_mark("CLIENT: @send# - start");
			#endif
			send(sockfd, mpc_st->block, mpc_st->size, 0);
			#ifdef TRACING
			print_mark("CLIENT: @send# - end");
			#endif
			#ifdef TRACING
			print_mark("CLIENT: @recv# - start");
			#endif
			recv(sockfd, mpc_st->block, mpc_st->size, 0);
			#ifdef TRACING
			print_mark("CLIENT: @recv# - end");
			#endif

			/*  
			 * After recv, the optimal input found by the
			 * server is saved in mpc_st->input
			 */

			//printf("MPC_CTRL: VALORE U: %lf\n", data->u);
			#ifdef PRINT_PROBLEM
				sprintf(tmp, "%02luA", k);
				strcat(tmp, s_sol);
				glp_print_sol(my_mpc.op, tmp);
			#endif
		} else {
			/* MPC runs locally */
			data->stats_int[MPC_STATS_INT_OFFLOAD] = 0;
			#ifdef PRINT_PROBLEM
				sprintf(tmp, "%02luB", k);
				strcat(tmp, s_sol);
				glp_print_sol(my_mpc.op, tmp);
			#endif
			#ifndef MPC_STATUS_X0_ONLY
				mpc_status_resume(&my_mpc, mpc_st);
			#else
				/* update initial state */
				mpc_status_set_x0(&my_mpc, mpc_st);
			#endif /* MPC_STATUS_X0_ONLY */
			#ifdef TRACING
			char msg[BUF_LEN+40];
			char status_buf[BUF_LEN];
			bzero(msg, BUF_LEN+40);
			bzero(status_buf, BUF_LEN);			
			strcat(msg, "CLIENT: @glp_simplex# - start {");
			mpc_get_status(status_buf, &my_mpc, mpc_st);
			strcat(msg, status_buf);
			strcat(msg, "}");
			print_mark(msg);
			#endif
			glp_simplex(my_mpc.op, my_mpc.param);
			#ifdef TRACING
			bzero(msg, BUF_LEN+40);
			bzero(status_buf, BUF_LEN);
			strcat(msg, "CLIENT: @glp_simplex# - end {");
			mpc_get_status(status_buf, &my_mpc, mpc_st);
			strcat(msg, status_buf);
			strcat(msg, "}");
			print_mark(msg);
			#endif
			mpc_status_save(&my_mpc, mpc_st);
		}
		clock_gettime(CLOCK_REALTIME, &before_post);
		data->stats_dbl[MPC_STATS_DBL_TIME] =
			(double)(before_post.tv_sec-after_wait.tv_sec);
		data->stats_dbl[MPC_STATS_DBL_TIME] +=
			((double)(before_post.tv_nsec-after_wait.tv_nsec))*1e-9;

		#ifdef PRINT_PROBLEM
			sprintf(tmp, "%02luC", k);
			strcat(tmp, s_sol);
			glp_print_sol(my_mpc.op, tmp);
			mpc_status_save(&my_mpc, mpc_st);
			mpc_status_fprintf(stdout, &my_mpc, mpc_st);
		#endif

		/* Write solution and stats to shared mem and let the plant know */
		memcpy(shared_input, mpc_st->input,
		       sizeof(*shared_state)*data->input_num);
		/* FIXME: add writing stats */
		sem_post(data->sems+MPC_SEM_INPUT_WRITTEN);
		#ifdef PRINT_LOG
			offset_rec = 0;
			offset_rec += snprintf(log_rec+offset_rec,
						(LOG_REC_SIZE)-offset_rec,
						"%ld.%09ld,%ld.%09ld,",
						after_wait.tv_sec, after_wait.tv_nsec,
						before_post.tv_sec, before_post.tv_nsec);
			for (i=0; i<data->state_num ; i++) {
				offset_rec += snprintf(log_rec+offset_rec,
							(LOG_REC_SIZE)-offset_rec,
							"%6.3f,", shared_state[i]);
			}
			for (i=0; i<data->input_num ; i++) {
				offset_rec += snprintf(log_rec+offset_rec,
							(LOG_REC_SIZE)-offset_rec,
							"%6.3f,", shared_input[i]);
			}
			printf("%s\n", log_rec);
		#endif /* PRINT_LOG */
	}
}

	
int model_mpc_startup(mpc_glpk * mpc, json_object * in)
{
	/* Cleanup the MPC struct */
	bzero(mpc,sizeof(*mpc));
	
	/* Init the solver control parameters */
	mpc->param = malloc(sizeof(*(mpc->param)));
	glp_init_smcp(mpc->param);
	#ifdef DEBUG_SIMPLEX
		mpc->param->msg_lev = GLP_MSG_DBG; /* all messages */
	#else
		mpc->param->msg_lev = GLP_MSG_OFF; /* no message */
	#endif
	mpc->param->meth    = GLP_DUAL;    /* dual simplex */
	mpc->param->it_lim  = INT_MAX;     /* max num of iterations */

	/* Initialize the plant */
	mpc->model = malloc(sizeof(*(mpc->model)));
	dyn_init_discrete(mpc->model, in);

	/* Setting up a GLPK problem instance */
	mpc->op = glp_create_prob();
	glp_set_prob_name(mpc->op, "Model Predictive Control");

	/* Setting up variables and bounds of control inputs */
	mpc_input_addvar(mpc, in);
	mpc_input_set_bnds(mpc, in);

	/* Setting up constraints: bounding input variation */
	/*	mpc_input_set_delta(mpc, in); */

	/* Add a variable for each norm of states X(1), ..., X(H)*/
	mpc_state_norm_addvar(mpc, in);
	
	/* Setting bounds to the states X(1), ..., X(H)*/
	mpc_state_set_bnds(mpc, in);
	
	/* Set a minimization cost for the MPC */
	mpc_goal_set(mpc, in);

	mpc_warmup(mpc);

	return 0;
}

void term_handler(int signum)
{
	/* Removing shared memory object */
	shmctl(shm_id, IPC_RMID, NULL);
	switch (signum) {
	case SIGINT:
		printf("Got SIGINT (Ctrl-C). Removed IPC object with key %X\n",
		       MPC_SHM_KEY);
		exit(0);
	case SIGHUP:
	case SIGPIPE:
	case SIGTERM:
	case SIGSEGV:
		fprintf(stderr,
			"Got unexpected terminating signal %d. Still removing IPC object with key %X\n",
			signum,
			MPC_SHM_KEY);
		exit(-1);
	}
}

void sched_set_prio_affinity(uint32_t prio, int cpu_id)
{
	#ifdef ENABLE_CPU_PIN
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
	#endif
}

void print_mark(const char *msg)
{
	char s[BUF_LEN];
	int marker_fd;
	
	marker_fd = open(MARKER_FILE, O_WRONLY);
	if (marker_fd == -1) {
		fprintf(stderr, "Error\n");
	}
	
		snprintf(s,BUF_LEN,"%s\n",msg);
		write(marker_fd, s, BUF_LEN);
	
	if(close(marker_fd)==-1)
		exit(EXIT_FAILURE);
}