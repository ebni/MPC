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
#include <json-c/json.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_sf_exp.h>
#include <glpk.h>
#include "dyn.h"
#include "mpc.h"

#define PORT_MATLAB 6004
#define PORT_SOLVER 6001

#define USE_DUAL
#define CLIENT_SOLVER
#define PRINT_LOG

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
 * MPC controller
 *
 * Computes  the  control law  u  by  MPC from  the  state  x and  the
 * parameters  param. The  two  vectors  x and  u  must be  previously
 * allocated.
 */
void ctrl_by_mpc(const gsl_vector * x, gsl_vector * u, void *param);

/*
 * Initializing the model with JSON file
 */
int model_mpc_startup(mpc_glpk * mpc, struct json_object * in);

/*
 * This is code should be invoked as:
 *
 * ./mpc_server <JSON model>
 *
 * to have the  MPC server solving the problem formulated  by the JSON
 * model <JSON model>. The server is listening behind the ports set by
 * the PORT_* #define
 */
int main(int argc, char *argv[]) {
	mpc_glpk uav_mpc;
#ifdef CLIENT_MATLAB
	gsl_vector *x, *u;	
	size_t i;
#endif
#ifdef CLIENT_SOLVER
	mpc_status * cur_st;
#endif

	int model_fd;
	char * buffer;
	ssize_t size;
	size_t k, size_in, size_out;
	uint16_t port;
	uint64_t * buf_in, *buf_out; /* as many bytes as double */

	struct json_object *model_json;
	struct json_tokener * tok;

	int listenfd;
	socklen_t len;
	struct sockaddr_in servaddr, cliaddr;
	cpu_set_t my_mask;
	struct timespec tic, toc;
	double time;
	int num;
	
	if (argc <= 1) {
		PRINT_ERROR("Too few arguments. 1 needed: <JSON model>");
		return -1;
	}

	/* Opening JSON model of the plant */
	if ((model_fd = open(argv[1], O_RDONLY)) == -1) {
		PRINT_ERROR("Missing/wrong file");
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

	/* Initializing the model */
	model_mpc_startup(&uav_mpc, model_json);

	/* Opening socket and all server stuff */
#ifdef CLIENT_MATLAB
	port = PORT_MATLAB;
#else
	port = PORT_SOLVER;	
#endif
	listenfd = socket(AF_INET, SOCK_DGRAM, 0);         
	bzero(&servaddr, sizeof(servaddr)); 
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY); 
	servaddr.sin_port = htons(port); 
	servaddr.sin_family = AF_INET;  
	if (bind(listenfd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1)
		PRINT_ERROR("issue in bind");
	printf("MPC server up and running: listening behind port %d\n", port);

	/* Pin the server to CPU 0 */
	CPU_ZERO(&my_mask);
	CPU_SET(0, &my_mask);
	sched_setaffinity(0, sizeof(my_mask), &my_mask);
	
	/* Pre-allocating vectors */
#ifdef CLIENT_MATLAB
	x = gsl_vector_calloc(uav_mpc.model->n);
	u = gsl_vector_calloc(uav_mpc.model->m);
	buf_in   = (unsigned long *)x->data;
	buf_out  = (unsigned long *)u->data;
	size_in  = sizeof(*buf_in)*x->size;
	size_out = sizeof(*buf_out)*u->size;
#endif
#ifdef CLIENT_SOLVER
	cur_st = mpc_status_alloc(&uav_mpc);
	buf_in = buf_out = cur_st->block;
	size_in = size_out = cur_st->size;
#endif
	/* Server cycle: Listening forever  */
	for (k=0; /* never stop */; k++) {
		len = sizeof(cliaddr);
		num = (int)recvfrom(listenfd, buf_in, size_in, 
				    0, (struct sockaddr*)&cliaddr, &len);
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
		num = (long)glp_get_it_cnt(uav_mpc.op);
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
		ctrl_by_mpc(x, u, &uav_mpc);
		clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
		num = glp_get_it_cnt(uav_mpc.op) - num;
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
		mpc_status_fprintf(stdout, &uav_mpc, cur_st);
#endif /* PRINT_LOG */
		/* 
		 * Condition to launch MPC server: current solution is
		 * not optimal and we have budgets
		 */
		if ((*cur_st->prim_stat != GLP_FEAS ||
		     *cur_st->dual_stat != GLP_FEAS) &&
		    *cur_st->steps_bdg > 0 && *cur_st->time_bdg > 0) {
			/* Resuming the status of the solver just received */
			mpc_status_resume(&uav_mpc, cur_st);
		
			/* Solve it by Simplex and measure time/steps */
			num = glp_get_it_cnt(uav_mpc.op);
			clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
			glp_simplex(uav_mpc.op, uav_mpc.param);
			clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
			num = glp_get_it_cnt(uav_mpc.op) - num;
			time =  (double)(toc.tv_sec-tic.tv_sec);
			time += (double)(toc.tv_nsec-tic.tv_nsec)*1e-9;

			/* Storing used budget */
			*cur_st->steps_bdg = num;
			*cur_st->time_bdg  = time;

			/* Saving the updated solver status: ready to send */
			mpc_status_save(&uav_mpc, cur_st);
		} else {
			/* Do nothing. Just set to 0 the used budgets */
			*cur_st->steps_bdg = 0;
			*cur_st->time_bdg  = 0;
		}
#ifdef PRINT_LOG
		printf("MESSAGE: %ld\n", k);
		fprintf(stdout, "status after optimization\n");
		mpc_status_fprintf(stdout, &uav_mpc, cur_st);
#endif /* PRINT_LOG */
#endif /* CLIENT_SOLVER */
		sendto(listenfd, buf_out, size_out, 0, 
		       (struct sockaddr*)&cliaddr, sizeof(cliaddr));
	}

	/* Free all */
	free(uav_mpc.model);
	gsl_vector_free(uav_mpc.w);
	gsl_vector_free(uav_mpc.x0);
	glp_delete_prob(uav_mpc.op);
	free(uav_mpc.param);

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
		gsl_vector_set(u, i,
			       glp_get_col_prim(my_mpc->op,
						my_mpc->v_U+(int)i));
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
