/**
 * @file mpc_server.c
 * @author Enrico	Bini
 * @brief MPC server solves the problem formulated by the JSON
 * model <JSON model> . The server is listening behind the ports set by
 * the PORT_* #define
 * 
 * @note This is code should be invoked as: ./mpc_server <JSON model>
 */
#define _GNU_SOURCE
#include <stdio.h> 
#include <strings.h> 
#include <sys/types.h> 
#include <arpa/inet.h> 
#include <sys/socket.h> 
#include <netinet/in.h>
#include <string.h>
#include <errno.h>
#include <sched.h>
#include <byteswap.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <limits.h>
#include <json-c/json.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_sf_exp.h>
#include <glpk.h>
#include <signal.h>
#include <stdlib.h>
#include "../../include/dyn.h"
#include "../../include/mpc.h"
#include "../../include/mpc_interface.h"

#define PORT_MATLAB 6004
/*#define PORT_SOLVER 6001*/
#define STRLEN_COMMAND 100

#define USE_DUAL
#define CLIENT_SOLVER
//#define PRINT_LOG

#include <sys/stat.h>
#include <fcntl.h>


#define MARKER_FILE "/sys/kernel/tracing/trace_marker"
#define BUF_LEN 400

void print_mark(const char *msg);

void print_mark(const char *msg)
{
	char s[BUF_LEN];
	int marker_fd;
	
	marker_fd = open(MARKER_FILE, O_WRONLY);
	if (marker_fd == -1) {
		fprintf(stderr, "Error opening trace_marker file\n");
		return;
	}
	snprintf(s,BUF_LEN,"%s\n",msg);
	write(marker_fd, s, BUF_LEN);
	if(close(marker_fd)==-1)
		exit(EXIT_FAILURE);
}

/*
 * Below are some #define which trigger something:
 *
 * INIT_X0_JSON, expect a "state_init" in JSON and use it as state
 * initialization. Otherwise using state zero to initialize the problem
 *
 * CLIENT_SOLVER, expect UDP messages from a solver of the format as
 * in the struct mpc_status defined in mpc.h
 *
 * CLIENT_MATLAB, expect UPD messages from Matlab: receiving the plant
 * state x and sending the MPC optimal input u. That's is
 *
 * PRINT_LOG, print log information every time 
 *
 * USE_DUAL, use dual Simplex method (should be always set)
 *
 * PRINT_PROBLEM, print the problem formulation in txt files
 *
 * DEBUG_SIMPLEX, turn on all Simplex messages for debugging
 *
 * HAVE_OBSTACLE, allow problem formulation with obstacles (WARNING:
 * not working!!! Consider fixing or removing the obstacle stuff)
 *
 * PRINT_MAT, print matrices (dont remember really how much stuff is
 * printed)
 */
/*
#define INIT_X0_JSON
#define CLIENT_SOLVER
#define CLIENT_MATLAB
#define PRINT_LOG
#define USE_DUAL
#define PRINT_PROBLEM
#define DEBUG_SIMPLEX
#define HAVE_OBSTACLE
#define PRINT_MAT
*/

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}

#define DONTCARE 0 /* any constant to be ignored */


/*
 * Set prio priority (high number => high priority) and pin the
 * invoking process to CPU cpu_id
 */

/**
 * @brief Set priority and pin the invoking process to a CPU
 * 
 * @param prio 	 New priority of the process
 * @param cpu_id Id of the cpu to which the invoking process is to be pinned on
 * @note priority high number => high priority
 */
void sched_set_prio_affinity(uint32_t prio, int cpu_id);

/*
 * MPC controller
 *
 * Computes  the  control law  u  by  MPC from  the  state  x and  the
 * parameters  param. The  two  vectors  x and  u  must be  previously
 * allocated.
 */

/**
 * @brief MPC controller: computes the control law u by MPC from the state x 
 * and the param parameters.
 * 
 * @param x 	const gsl_vector*	State vector
 * @param u 	gsl_vector*			Input vector (control law)
 * @param param void* 				Parameters to be passed to control law	//FIXME what is control law?
 * @note x and u vectors must be allocated previously
 */
void ctrl_by_mpc(const gsl_vector * x, gsl_vector * u, void *param);

/*
 * Initializing the model with JSON file
 */

/**
 * @brief Initialize the model with JSON file
 * 
 * @param  mpc mpc_glpk*			The representation of the MPC problem
 * @param  in  json_object*	JSON object used to contain input read from file
 * @return int 						0 if it terminates correctly
 */
int model_mpc_startup(mpc_glpk * mpc, json_object * in);
void term_handler(int signum);

void term_handler(int signum)
{
	/* Removing shared memory object */
	switch (signum) {
	case SIGINT:
		exit(0);
	case SIGHUP:
	case SIGPIPE:
	case SIGTERM:
	case SIGSEGV:
		exit(-1);
	}
}

//FIXME: add documentation
json_object* init_model_json(const char *json_file);

json_object* init_model_json(const char *json_file)
{
	json_object *model_json;
	json_tokener *tok;
	ssize_t size;
	int model_fd;
	char * buffer;

	if (json_file == NULL || (model_fd = open(json_file, O_RDONLY)) == -1) {
		PRINT_ERROR("Missing/wrong file");
		exit(1);
	}
	size = lseek(model_fd, 0, SEEK_END);
	lseek(model_fd, 0, SEEK_SET);
	buffer = malloc((size_t)size);
	size = read(model_fd, buffer, (size_t)size);
	close(model_fd);
	tok = json_tokener_new();
	model_json = json_tokener_parse_ex(tok, buffer, (int)size);
	free(buffer);
	return model_json;
}


/*
 * This is code should be invoked as:
 *
 * ./mpc_server <JSON model>
 *
 * to have the  MPC server solving the problem formulated  by the JSON
 * model <JSON model>. The server is listening behind the ports set by
 * the PORT_* #define
 */

int main(int argc, char *argv[]) 
{
	mpc_glpk my_mpc;
	struct sigaction sa;
	#ifdef CLIENT_MATLAB
	gsl_vector *x, *u;	
	size_t i;
	#endif
	#ifdef CLIENT_SOLVER
	mpc_status * mpc_st;
	#endif
	#ifdef TEST_PARTIAL_OPTIMIZATION
	struct timespec tic, toc;
	double time;
	#endif
	int num;
	//int model_fd;
	//char * buffer;
	//ssize_t size;
	size_t k, size_in, size_out;
	uint16_t port;
	uint64_t * buf_in, *buf_out; /* as many bytes as double */
	json_object *model_json;
	//json_tokener * tok;
	int listenfd;
	socklen_t len;
	struct sockaddr_in servaddr, cliaddr;
	
	if (argc <= 1) {
		PRINT_ERROR("Too few arguments. 1 needed: <JSON model>");
		return -1;
	}

	/* Setting up the signal handler for termination */
	bzero(&sa, sizeof(sa));
	sa.sa_handler = term_handler;
	sigaction(SIGHUP, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGPIPE, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGSEGV, &sa, NULL);
	
	model_json = init_model_json(argv[1]);
	sleep(3);
	/* Opening JSON model of the plant */
	/*
	if ((model_fd = open(argv[1], O_RDONLY)) == -1) {
		PRINT_ERROR("Missing/wrong file");
		return -1;
	}*/
	/* Getting the size of the file */
	/*size = lseek(model_fd, 0, SEEK_END);
	lseek(model_fd, 0, SEEK_SET);
	*/
	/* Allocate the buffer and store data */
	/*buffer = malloc((size_t)size);
	size = read(model_fd, buffer, (size_t)size);
	close(model_fd);
	tok = json_tokener_new();
	model_json = json_tokener_parse_ex(tok, buffer, (int)size);
	free(buffer);
	*/
	
	//TODO: place to check how use U
	
	/* Initializing the model */
	print_mark("SERVER: @model_mpc_startup# - start");
	model_mpc_startup(&my_mpc, model_json);
	print_mark("SERVER: @model_mpc_startup# - end");
	/* Opening socket and all server stuff */
	#ifdef CLIENT_MATLAB
	port = PORT_MATLAB;
	#else
	port = MPC_SOLVER_PORT;
	#endif
	listenfd = socket(AF_INET, SOCK_DGRAM, 0);         
	bzero(&servaddr, sizeof(servaddr)); 
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY); 
	servaddr.sin_port = htons(port); 
	servaddr.sin_family = AF_INET;  
	if (bind(listenfd, (struct sockaddr*) &servaddr, sizeof(servaddr)) == -1)
		PRINT_ERROR("issue in bind");
	printf("MPC server up and running: listening behind port %d\n", port);

	/* Pin server to a CPU different than client */
	//printf("PRI_MAX: %d CPU_AFFINITY: %d\n", sched_get_priority_max(SCHED_FIFO), sched_getaffinity(0, sizeof(mask), &mask));
	sched_set_prio_affinity(sched_get_priority_max(SCHED_FIFO), (MPC_CPU_ID-1)%2);
	/*
	CPU_ZERO(&my_mask);
	CPU_SET(0, &my_mask);
	sched_setaffinity(0, sizeof(my_mask), &my_mask); */
	
	/* Pre-allocating vectors */
	#ifdef CLIENT_MATLAB
		x = gsl_vector_calloc(my_mpc.model->n);
		u = gsl_vector_calloc(my_mpc.model->m);
		buf_in   = (unsigned long *)x->data;
		buf_out  = (unsigned long *)u->data;
		size_in  = sizeof(*buf_in)*x->size;
		size_out = sizeof(*buf_out)*u->size;
	#endif
	#ifdef CLIENT_SOLVER
		#ifdef TRACING
		print_mark("SERVER: @mpc_status_alloc# - start");
		#endif
		mpc_st = mpc_status_alloc(&my_mpc);
		#ifdef TRACING
		print_mark("SERVER: @mpc_status_alloc# - end");
		#endif
		buf_in = buf_out = mpc_st->block;
		size_in = size_out = mpc_st->size;
	#endif
	/* Server cycle: Listening forever  */
	for (k=0; /* never stop */; k++) {
		len = sizeof(cliaddr);
		
		#ifdef TEST_PARTIAL_OPTIMIZATION
		num = (int)
		#endif

		#ifdef TRACING
		printf("k:%ld\n", k);
		print_mark("SERVER: @recvfrom# - start");		
		#endif

		recvfrom(listenfd, buf_in, size_in, 
			    0, (struct sockaddr*)&cliaddr, &len);
		#ifdef TRACING
		print_mark("SERVER: @recvfrom# - end");
		#endif
		
		#ifdef CLIENT_MATLAB
			/* 
			* Receiving  the   state  x  via  an   UDP  datagram.
			* REMEMBER: both buf_in AND x->data point to the same
			* memory area. Hence, receiving  on buf_in will write
			* to x
			*/
			/* Swap bytes to be meaningful */
			for (i=0; i < x->size; i++) {
				buf_in[i] = bswap_64(buf_in[i]);
			}
			#ifdef PRINT_LOG
				printf("k=%06lu\n\trecv %d bytes.  State x:",
					k, num);
				gsl_vector_pretty(stdout, x, "%7.3f");
			#endif /* PRINT_LOG */

			/* Getting steps and time of simplex */
			num = (long)glp_get_it_cnt(my_mpc.op);
			clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
			ctrl_by_mpc(x, u, &my_mpc);
			clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
			num = glp_get_it_cnt(my_mpc.op) - num;
			time =  (double)(toc.tv_sec-tic.tv_sec);
			time += (double)(toc.tv_nsec-tic.tv_nsec)*1e-9;
			#ifdef PRINT_LOG
					printf("\t\t\tInput u:");
					gsl_vector_pretty(stdout, u, "%7.3f");
					printf("\tsteps: %5d, time: %e\n", num, time);
			#endif /* PRINT_LOG */

			/* Preparing the buffer and sending */
			for (i=0; i < u->size; i++) {
				buf_out[i] = bswap_64(buf_out[i]);
			}
		#endif /* CLIENT_MATLAB */
		#ifdef CLIENT_SOLVER
			#ifdef PRINT_LOG
			printf("MESSAGE: %ld\n", k);
			fprintf(stdout, "status received\n");
			mpc_status_fprintf(stdout, &my_mpc, mpc_st);
			#endif /* PRINT_LOG */
			
			/* 
			* Condition to launch MPC server: current solution is
			* not optimal and we have budgets
			*/
			#ifdef TEST_PARTIAL_OPTIMIZATION /* OLD CODE: TO BE FIXED */
			if ((*mpc_st->prim_stat != GLP_FEAS ||
				*mpc_st->dual_stat != GLP_FEAS) &&
				*mpc_st->steps_bdg > 0 && *mpc_st->time_bdg > 0) {
				/* Resuming the status of the solver just received */
				mpc_status_resume(&my_mpc, mpc_st);
			
				/* Solve it by Simplex and measure time/steps */
				num = glp_get_it_cnt(my_mpc.op);
				clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
				glp_simplex(my_mpc.op, my_mpc.param);
				clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
				num = glp_get_it_cnt(my_mpc.op) - num;
				time =  (double)(toc.tv_sec-tic.tv_sec);
				time += (double)(toc.tv_nsec-tic.tv_nsec)*1e-9;

				/* Storing used budget */
				*mpc_st->steps_bdg = num;
				*mpc_st->time_bdg  = time;

				/* Saving the updated solver status: ready to send */
				mpc_status_save(&my_mpc, mpc_st);
			} else {
				/* Do nothing. Just set to 0 the used budgets */
				*mpc_st->steps_bdg = 0;
				*mpc_st->time_bdg  = 0;
			}
			#else
			/* update initial state */
			#ifdef TRACING
			print_mark("SERVER: @mpc_status_set_x0# - start");		
			#endif
			
			mpc_status_set_x0(&my_mpc, mpc_st);
			
			#ifdef TRACING			
			print_mark("SERVER: @mpc_status_set_x0# - end");
			char msg[BUF_LEN+1000];
			char status_buf[BUF_LEN];
			num = glp_get_it_cnt(my_mpc.op);
			print_mark("SERVER: @glp_simplex# - start");
			#endif			

			glp_simplex(my_mpc.op, my_mpc.param);

			#ifdef TRACING
			num = glp_get_it_cnt(my_mpc.op) - num;
			bzero(status_buf, BUF_LEN);
			mpc_get_status(status_buf, &my_mpc, mpc_st);
			snprintf(msg, BUF_LEN+1000, "SERVER: @glp_simplex# - end {%s\tITERATION COUNT:\t%d}", status_buf, num);
			print_mark(msg);			
			#endif		
			
			mpc_status_save(&my_mpc, mpc_st);
			
			#endif  /* TEST_PARTIAL_OPTIMIZATION */
			
			#ifdef PRINT_LOG
			printf("MESSAGE: %ld\n", k);
			fprintf(stdout, "status after optimization\n");
			mpc_status_fprintf(stdout, &my_mpc, mpc_st);
			#endif /* PRINT_LOG */
		#endif /* CLIENT_SOLVER */
		
		#ifdef TRACING
		print_mark("SERVER: @sendto# - start");		
		#endif
		
		sendto(listenfd, buf_out, size_out, 0, 
		    (struct sockaddr*)&cliaddr, sizeof(cliaddr));
		
		#ifdef TRACING
		print_mark("SERVER: @sendto# - end");
		#endif
	}

	/* Free all */
	free(my_mpc.model);
	gsl_vector_free(my_mpc.w);
	gsl_vector_free(my_mpc.x0);
	glp_delete_prob(my_mpc.op);
	free(my_mpc.param);

	return 0;
}

/*
 * Computes  the  control law  u  by  MPC from  the  state  x and  the
 * parameters  param. The  two  vectors  x and  u  must be  previously
 * allocated.
 */

void ctrl_by_mpc(const gsl_vector * x, gsl_vector * u, void *param)
{
	mpc_glpk *my_mpc;
	size_t i;

	my_mpc = (mpc_glpk *)param;

	/* Update the initial state  */
	gsl_vector_memcpy (my_mpc->x0, x);
	mpc_update_x0(my_mpc);

	/* Solve it by Simplex method */
	glp_simplex(my_mpc->op, my_mpc->param);

	/* Getting the solution. FIXME: need a more efficient way */
	for (i = 0; i < my_mpc->model->m; i++) {
		gsl_vector_set(u, i, glp_get_col_prim(my_mpc->op,
			my_mpc->v_U+(int)i));
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
		mpc->param->out_frq = 1;           /* output every iteration */
	#else
		mpc->param->msg_lev = GLP_MSG_OFF; /* no message */
	#endif
	mpc->param->meth    = GLP_DUAL;    /* dual simplex */

	#if 1
		mpc->param->it_lim  = INT_MAX;     /* max num of iterations */
	#else
		mpc->param->it_lim  = 15;     /* max num of iterations */
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

	/* Warm the solver up with initial state equal to zero */
	mpc_warmup(mpc);
	
	/* 
	 * Setting the max delta constraint, assuming an initial zero
	 * input 
	 */
	/*
	u_old = gsl_vector_calloc(mpc->model->m);
	gsl_vector_set_zero(u_old);
	mpc_input_set_delta0(mpc, u_old);
	gsl_vector_free(u_old);
	*/

	return 0;
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
		printf("PID: %d\n", getpid());
		#endif
	#endif
}
