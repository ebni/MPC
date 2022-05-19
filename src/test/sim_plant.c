/*
 * sim_plant.c
 *
 * Code used to simulate a plant which starts from some given initial
 * state and then it is controlled by MPC, possibly using the MPC
 * server.
 */
#define _GNU_SOURCE
#include <stdio.h> 
#include <strings.h> 
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
/*
#include <arpa/inet.h> 
#include <sys/socket.h> 
#include <netinet/in.h>
*/
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
#include "../../include/dyn.h"
#include "../../include/mpc.h"
#include "../../include/mpc_interface.h"

#define INIT_X0_JSON

/*
 * Below are some #define which trigger something:
 *
 * INIT_X0_JSON, expect a "state_init" in JSON and use it as state
 * initialization. Otherwise using state zero to initialize the problem
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
#define INIT_X0_JSON
#define PRINT_LOG
#define USE_DUAL
#define PRINT_PROBLEM
#define DEBUG_SIMPLEX
#define PRINT_MAT
*/

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}

/*
 * MPC controller
 */
void ctrl_by_mpc(size_t k, dyn_trace * t, void *param);

/*
 * Initializing the model with JSON file
 */
int model_mpc_startup(mpc_glpk * mpc, json_object * in);

/*
 * GLOBAL VARIABLES
 *
 * Global variables  are necessary  since they are  used both  in main
 * file and the controller function ctrl_by_mpc(...)
 */
mpc_status * mpc_st;
int sockfd;
gsl_vector *x_k;
shared_data * data;
double * shared_state;
double * shared_input;

/*
 * This is code should be invoked as:
 *
 * ./sim_plant <JSON model> <number of steps>
 *
 * to simulate  a plant described by  the <JSON model> for  <number of
 * steps>.
 */
int main(int argc, char *argv[]) {
	mpc_glpk uav_mpc;
	dyn_trace * uav_trace;

	int model_fd, shm_id;
	char * buffer;
	ssize_t size;
	size_t steps;

	json_object *model_json;
	json_tokener* tok;

	if (argc <= 2) {
		PRINT_ERROR("Too few arguments. 2 needed: <JSON model> <number of steps>");
		return -1;
	}

	/* Getting number of steps to be simulated */
	steps = (size_t)strtol(argv[2], NULL, 10);
	if (errno) {
		PRINT_ERROR("Wrong number of steps");
		return -1;
	}
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
#ifdef PRINT_PROBLEM
	glp_print_sol(uav_mpc.op, "000glpk_sol.txt");
#endif

	/* Allocating struct of solver status after problem defined */
	mpc_st = mpc_status_alloc(&uav_mpc);
	x_k = gsl_vector_calloc(uav_mpc.model->n);
	  
	/* Save initial status */
	mpc_status_save(&uav_mpc, mpc_st);
	fprintf(stdout, "Initial status\n");
	mpc_status_fprintf(stdout, &uav_mpc, mpc_st);
 	glp_write_lp(uav_mpc.op, NULL, "initial_mpc.txt");
	glp_print_sol(uav_mpc.op, "initial_sol.txt");

	/* Allocating for a trace */ 
	uav_trace = dyn_trace_alloc(uav_mpc.model->n, uav_mpc.model->m, steps);

	/* Getting the shared memory area */
	shm_id = shmget(MPC_SHM_KEY, 0, 0);
	if (shm_id == -1) {
		PRINT_ERROR("shmget failed");
	}
	/* Setting up pointers to state/input arrays */
	data = (shared_data *)shmat(shm_id, NULL, 0);
	shared_state = (double*)(data+1); /* starts just after *data */
	shared_input = shared_state+data->state_num;

	/* Computing the system dynamics */
	dyn_plant_dynamics(uav_mpc.model, uav_mpc.x0, uav_trace,
			   ctrl_by_mpc, &uav_mpc);

	/* Output */
	printf("\nSTATE EVOLUTION\n");
	gsl_matrix_pretty(stdout, uav_trace->x, "%7.3f");
	printf("\nINPUT APPLIED\n");
	gsl_matrix_pretty(stdout, uav_trace->u, "%7.3f");
	printf("\nTIME NEEDED\n");
	gsl_vector_pretty(stdout, uav_trace->time, "%e");
	printf("\n");
	/* Free all */
	free(uav_mpc.model);
	gsl_vector_free(uav_mpc.w);
	gsl_vector_free(uav_mpc.x0);
	glp_delete_prob(uav_mpc.op);
	free(uav_mpc.param);
	dyn_trace_free(uav_trace);
	gsl_vector_free(x_k);

	return 0;
}

void ctrl_by_mpc(size_t k, dyn_trace * t, void *param)
{
	size_t i;
	struct timespec before_post, after_wait;
	double cur_time;

	/* Get the current state from the k-th column of t->x */
	gsl_matrix_get_col(x_k, t->x, k);

	/* Invoke MPC by writing state and then reading input */
	memcpy(shared_state, x_k->data,
	       sizeof(*shared_state)*data->state_num);
	clock_gettime(CLOCK_REALTIME, &before_post);
	sem_post(data->sems+MPC_SEM_STATE_WRITTEN);
	sem_wait(data->sems+MPC_SEM_INPUT_WRITTEN);
	clock_gettime(CLOCK_REALTIME, &after_wait);
	for (i = 0; i < t->m; i++) {
		gsl_matrix_set(t->u, i, k, shared_input[i]);
	}
	
	cur_time =  (double)(after_wait.tv_sec-before_post.tv_sec);
	cur_time += 1e-9*(double)(after_wait.tv_nsec-before_post.tv_nsec);
	gsl_vector_set(t->time, k, cur_time);
}

int model_mpc_startup(mpc_glpk * mpc, json_object * in)
{
#ifdef INIT_X0_JSON
	size_t i;
	json_object *tmp_elem, *elem;
#endif

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
 
#ifdef INIT_X0_JSON
	/* 
	 * Get the initial state from JSON, store it in the MPC
	 * struct, and update the problem accordingly
	 */
	if (!json_object_object_get_ex(in, "state_init", &tmp_elem)) {
		PRINT_ERROR("missing state_init in JSON");
		return -1;
	}
	if ((size_t)json_object_array_length(tmp_elem) != mpc->model->n) {
		PRINT_ERROR("wrong size of state_init in JSON");
		return -1;
	}
	mpc->x0 = gsl_vector_calloc(mpc->model->n);
	for (i=0; i < mpc->model->n; i++) {
		elem = json_object_array_get_idx(tmp_elem, (int)i);
		errno = 0;
		mpc->x0->data[i] = json_object_get_double(elem);
		if (errno) {
			fprintf(stderr, "%i\n", (int)i);
			PRINT_ERROR("issues in converting element in state_init");
			return -1;
		}
	}
	mpc_update_x0(mpc);
#else
	mpc_warmup(mpc);
#endif

	
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
