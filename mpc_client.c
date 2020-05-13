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
#define INIT_X0_JSON

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
 *
 * USE_SERVER, offload to a server
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
#define USE_SERVER
*/

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}

#define DONTCARE 0 /* any constant to be ignored */

/*
 * MPC controller
 */
void ctrl_by_mpc(size_t k, dyn_trace * t, void *param);

/*
 * Initializing the model with JSON file
 */
int model_mpc_startup(mpc_glpk * mpc, struct json_object * in);

/*
 * GLOBAL VARIABLES
 *
 * Global variables  are necessary  since they are  used both  in main
 * file and the controller function ctrl_by_mpc(...)
 */
mpc_status * mpc_st;
int sockfd;
gsl_vector *x_k;

/*
 * This is code should be invoked as:
 *
 * ./mpc_client <JSON model> <number of steps> <server IP>
 *
 * to have the  MPC client solving the problem formulated  by the JSON
 * model <JSON  model> off loading to  the server at <server  IP>. The
 * port is  set by  a #define.  The simulation is  run for  <number of
 * steps> iterations.
 */
int main(int argc, char *argv[]) {
	mpc_glpk uav_mpc;
#ifdef USE_SERVER
	struct sockaddr_in servaddr;
#endif

#ifdef HAVE_OBSTACLE
	/* TO BE FIXED: ADDED TO JSON FILE?? */
	double center[] = OBSTACLE_CENTER;
	double radius[] = OBSTACLE_RADIUS;
#endif
	dyn_trace * uav_trace;

	int model_fd;
	char * buffer;
	ssize_t size;
	size_t i, steps;

	struct json_object *model_json;
	struct json_tokener * tok;

#ifdef USE_SERVER
	if (argc <= 3) {
		PRINT_ERROR("Too few arguments. 3 needed: <JSON model> <number of steps> <server IP>");
		return -1;
	}
#else
	if (argc <= 2) {
		PRINT_ERROR("Too few arguments. 2 needed: <JSON model> <number of steps>");
		return -1;
	}
#endif /* USE_SERVER */

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
#if 0	
	enum json_tokener_error jerr;

	if ((jerr = json_tokener_get_error(tok)) != json_tokener_continue) {
		fprintf(stderr, "error string = %s\n",
			json_tokener_error_desc(jerr));
		PRINT_ERROR("error parsing JSON");
		return -1;
	}
#endif
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

#ifdef USE_SERVER
	/* Setting up the client: server to connect to */
	bzero(&servaddr, sizeof(servaddr)); 
	servaddr.sin_addr.s_addr = inet_addr(argv[3]);
	servaddr.sin_port = htons(PORT_SOLVER);
	servaddr.sin_family = AF_INET;
      
	/* create and connect UPD socket */
	sockfd = socket(AF_INET, SOCK_DGRAM, 0); 
	if(connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
		PRINT_ERROR("client: error in connect");
#endif
	
	/* Allocating for a trace */ 
	uav_trace = dyn_trace_alloc(uav_mpc.model->n, uav_mpc.model->m, steps);

	/* Computing the system dynamics */
	dyn_plant_dynamics(uav_mpc.model, uav_mpc.x0, uav_trace,
			   ctrl_by_mpc, &uav_mpc);

	/* Output */
	printf("\nSTATE EVOLUTION\n");
	gsl_matrix_pretty(stdout, uav_trace->x, "%7.3f");
	printf("\nINPUT APPLIED\n");
	gsl_matrix_pretty(stdout, uav_trace->u, "%7.3f");
	printf("\nSTEPS NEEDED\n");
	gsl_vector_int_pretty(stdout, uav_trace->steps, "%5ld");
	printf("\nTIME NEEDED\n");
	gsl_vector_pretty(stdout, uav_trace->time, "%e");
	printf("\nOPTIMALITY (prim,dual): undefined=%d, feasible=%d, infeasible=%d, empty=%d\n", GLP_UNDEF, GLP_FEAS, GLP_INFEAS, GLP_NOFEAS);
	for (i=0; i<steps; i++) {
		printf("\t(%d,%d)", uav_trace->opt[i].prim, uav_trace->opt[i].dual);
	}
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
	mpc_glpk *my_mpc;
	size_t i;
#ifdef USE_SERVER
	int steps_serv;
	double time_serv;
#endif
#ifdef PRINT_PROBLEM
	char s_sol[100] = SOL_FILENAME;
	char tmp[100];
#endif

	my_mpc = (mpc_glpk *)param;

	/* Get the current state from the k-th column of t->x */
	gsl_matrix_get_col(x_k, t->x, k);

	/* Prepare the status to be sent */
	mpc_status_save(my_mpc, mpc_st);
	memcpy(mpc_st->state, x_k->data, sizeof(*x_k->data)*my_mpc->model->n);
#ifdef USE_SERVER
	*mpc_st->steps_bdg = 50;   /* number of max iterations */
	*mpc_st->time_bdg = 1000; /* max seconds (large value) */
	*mpc_st->prim_stat = GLP_INFEAS;  /* changing x0 makes primal undef */
	*mpc_st->dual_stat = GLP_FEAS;    /* should always be dual feasible */

	/* Sending/receiving status to/from server */
	send(sockfd, mpc_st->block, mpc_st->size, 0);
	recv(sockfd, mpc_st->block, mpc_st->size, 0);

	/* Storing server used budgets */
 	steps_serv = *mpc_st->steps_bdg;
	time_serv  = *mpc_st->time_bdg; 
	gsl_vector_int_set(t->steps, k, steps_serv);
	gsl_vector_set(t->time, k, time_serv);

#ifdef PRINT_PROBLEM
	sprintf(tmp, "%02luA", k);
	strcat(tmp, s_sol);
	glp_print_sol(my_mpc->op, tmp);
#endif
	/* Printing the status after the local computations */
	fprintf(stdout, "\nServer steps: %d\n", steps_serv);
	fprintf(stdout, "Server time: %f\n", time_serv);
#endif /* USE SERVER */
	/* giving ourself "infinite" time/iterations */
	*mpc_st->steps_bdg = INT_MAX;
	*mpc_st->time_bdg  = INT_MAX;
	mpc_status_resume(my_mpc, mpc_st);
#ifdef PRINT_PROBLEM
	sprintf(tmp, "%02luB", k);
	strcat(tmp, s_sol);
	glp_print_sol(my_mpc->op, tmp);
#endif
	glp_factorize(my_mpc->op); /* INVESTIGATE: DONT KNOW IF USEFUL */
	glp_simplex(my_mpc->op, my_mpc->param);
	
#ifdef PRINT_PROBLEM
	sprintf(tmp, "%02luC", k);
	strcat(tmp, s_sol);
	glp_print_sol(my_mpc->op, tmp);
#endif	

	mpc_status_save(my_mpc, mpc_st);
	mpc_status_fprintf(stdout, my_mpc, mpc_st);

	/*
	 * There is  a more efficient way  to make the steps  below by
	 * something using
	 *
	 *   mpc_status_save(my_mpc, mpc_st);
	 */

	
	/* Getting the solution */
	for (i = 0; i < t->m; i++) {
		gsl_matrix_set(t->u, i, k,
			       glp_get_col_prim(my_mpc->op,
						my_mpc->v_U+(int)i));
	}

	/* Store optimality of solution */
	t->opt[k].prim = glp_get_prim_stat(my_mpc->op);
	t->opt[k].dual = glp_get_dual_stat(my_mpc->op);

}

int model_mpc_startup(mpc_glpk * mpc, struct json_object * in)
{
#ifdef HAVE_OBSTACLE
	/* TO BE FIXED: ADDED TO JSON FILE?? */
	double center[] = OBSTACLE_CENTER;
	double radius[] = OBSTACLE_RADIUS;
#endif
#ifdef INIT_X0_JSON
	size_t i;
	struct json_object *tmp_elem, *elem;
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
#ifdef USE_DUAL
	mpc->param->meth    = GLP_DUAL;    /* dual simplex */
#endif

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
 
#ifdef HAVE_OBSTACLE
	/* Add the obstacle */
	mpc_state_obstacle_add(mpc, center, radius);
#ifdef PRINT_PROBLEM
	/* DEBUG ONLY: Writing the GLPK formulation in CPLEX form */
	glp_write_lp(mpc->op, NULL, "problem_obstacle.txt");
#endif

#endif

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
