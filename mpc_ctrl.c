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
#include "mpc_interface.h"

#define PRINT_LOG
/*
 * Below are some #define which trigger something:
 *
 * PRINT_LOG, print log information every time 
 *
 * PRINT_PROBLEM, print the problem formulation in txt files
 *
 * DEBUG_SIMPLEX, turn on all Simplex messages for debugging
 *
 * PRINT_MAT, print matrices (dont remember really how much stuff is
 * printed)
 */
/*
#define PRINT_LOG
#define PRINT_PROBLEM
#define DEBUG_SIMPLEX
#define PRINT_MAT
*/

#include <sys/ipc.h>
#include <sys/shm.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>
#include <strings.h>
#include <stdlib.h>
#include <unistd.h>
#include <limits.h> 
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "mpc.h"

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}

/* GLOBAL VARIABLES (used in handler) */
int shm_id;


/*
 * Initializing the model with JSON file
 */
int model_mpc_startup(mpc_glpk * mpc, struct json_object * in);

/*
 * Signal handler (Ctrl-C). This process will terminate only on Ctrl-C
 */
void term_handler(int signum);

/*
 * Handling  the segmentation  fault  signal (SIGSEGV).  By setting  a
 * breakpoint within  the signal handler,  it is then possible  to see
 * what is the line of code which generated the error.
 */
void seg_fault_handler(int signum);


int main(int argc, char * argv[]) {
	struct shared_data * data;
	double * shared_state;
	double * shared_input;
	int model_fd;
	char * buffer;
	ssize_t size;
	size_t i;

	struct json_object *model_json;
	struct json_tokener * tok;

	struct sigaction sa;

	mpc_status * mpc_st;
	mpc_glpk my_mpc;
	int sockfd;
	struct sockaddr_in servaddr;
#ifdef PRINT_PROBLEM
	char s_sol[100] = SOL_FILENAME;
	char tmp[100];
#endif


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

	/* Initializing the model */
	model_mpc_startup(&my_mpc, model_json);

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
	data = (struct shared_data *)shmat(shm_id, NULL, 0);
	shared_state = (double*)(data+1); /* starts just after *data */
	shared_input = shared_state+my_mpc.model->n;
	bzero(data, sizeof(*data)+
	      sizeof(*shared_state)*my_mpc.model->n+
	      sizeof(*shared_input)*my_mpc.model->m);
	data->state_num = my_mpc.model->n;
	data->input_num = my_mpc.model->m;
	
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
	mpc_st = mpc_status_alloc(&my_mpc);
	  
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

	/* 
	 * Cycling forever to get the state of the plant. Ctrl-C will
	 * terminate
	 */
	while (1) {
		sem_wait(data->sems+MPC_SEM_STATE_WRITTEN);
		/* Now the system wrote the state in shared_state */

#ifdef PRINT_LOG
		/* Printing received message */
		printf("%s: Got state from plant. First state: %f\n",
		       __FILE__, shared_state[0]);
#endif /* PRINT_LOG */

		/* Store the lastest solver status in mpc_st */
		mpc_status_save(&my_mpc, mpc_st);
		memcpy(mpc_st->state, shared_state,
		       sizeof(*shared_state)*data->state_num);
		*mpc_st->steps_bdg = INT_MAX;  /* max iterations */
		*mpc_st->time_bdg = INT_MAX;   /* max seconds */
		/* Setting the status of cur solution */
		*mpc_st->prim_stat = GLP_INFEAS;
		*mpc_st->dual_stat = GLP_FEAS;
		if (data->flags & MPC_OFFLOAD) {
			/* MPC offloaded to server */
			
			/* Sending/receiving status to/from server */
			send(sockfd, mpc_st->block, mpc_st->size, 0);
			recv(sockfd, mpc_st->block, mpc_st->size, 0);
			/* 
			 * After recv, the optimal input found by the
			 * server is saved in mpc_st->input
			 */
#ifdef PRINT_PROBLEM
			sprintf(tmp, "%02luA", k);
			strcat(tmp, s_sol);
			glp_print_sol(my_mpc.op, tmp);
#endif
		} else {
			/* MPC runs locally */
			mpc_status_resume(&my_mpc, mpc_st);
#ifdef PRINT_PROBLEM
			sprintf(tmp, "%02luB", k);
			strcat(tmp, s_sol);
			glp_print_sol(my_mpc.op, tmp);
#endif
			glp_simplex(my_mpc.op, my_mpc.param);
			mpc_status_save(&my_mpc, mpc_st);
		}
		
#ifdef PRINT_PROBLEM
		sprintf(tmp, "%02luC", k);
		strcat(tmp, s_sol);
		glp_print_sol(my_mpc.op, tmp);
		mpc_status_save(&my_mpc, mpc_st);
		mpc_status_fprintf(stdout, &my_mpc, mpc_st);
#endif

		/* Write solution to shared mem and let the plant know */
		memcpy(shared_input, mpc_st->input,
		       sizeof(*shared_state)*data->input_num);
		sem_post(data->sems+MPC_SEM_INPUT_WRITTEN);
	}
}


	
int model_mpc_startup(mpc_glpk * mpc, struct json_object * in)
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
#if 1
	mpc->param->it_lim  = INT_MAX;     /* max num of iterations */
#else
	mpc->param->it_lim  = 2;     /* max num of iterations */
#endif

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
