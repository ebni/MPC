/**
 * @file test_sys.c
 * @author Enrico	Bini
 * @brief this code is used for test the sync between matlab and c part of MPC 
  // BUG:delete me
 */
#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_sf_exp.h>
#include <glpk.h>
#include "../../include/dyn.h"
#include "../../include/mpc.h"
 
#include <json-c/json.h>

#define USE_DUAL
#define PRINT_PROBLEM
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

/* Filename with the problem formulation */
#ifdef PRINT_PROBLEM
#define PROB_FILENAME "glpk_mpc.txt"
#define SOL_FILENAME "glpk_sol.txt"
#endif

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
int model_mpc_startup(mpc_glpk * mpc, json_object * in);

/*
 * GLOBAL VARIABLES
 */
mpc_status * mpc_st;

/*
 * This is code should be invoked as:
 *
 * ./test_sys <filename-of-JSON-model>
 *
 * to have the JSON model <filename-of-JSON-model> loaded
 */
int main(int argc, char *argv[]) {
	mpc_glpk uav_mpc;

#ifdef HAVE_OBSTACLE
	/* TO BE FIXED: ADDED TO JSON FILE?? */
	double center[] = OBSTACLE_CENTER;
	double radius[] = OBSTACLE_RADIUS;
#endif
	dyn_trace * uav_trace;

	int model_fd;
	char * buffer, tmp_str[1000];
	ssize_t size;
	size_t i, steps;
	FILE * matfile;

	json_object *model_json;
	json_tokener* tok;

	
	if (argc <= 3) {
		PRINT_ERROR("Too few arguments. 3 needed: <JSON model> <outfile prefix> <number of steps>");
		return -1;
	}

	/* Getting number of steps to be simulated */
	steps = (size_t)strtol(argv[3], NULL, 10);
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

	/* Allocating struct of solver status after problem defined */
	mpc_st = mpc_status_alloc(&uav_mpc);
	  
	/* Save initial status */
	mpc_status_save(&uav_mpc, mpc_st);
	
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
	gsl_vector_long_pretty(stdout, uav_trace->steps, "%5ld");
	printf("\nTIME NEEDED\n");
	gsl_vector_pretty(stdout, uav_trace->time, "%e");
	printf("\nOPTIMALITY (prim,dual): undefined=%d, feasible=%d, infeasible=%d, empty=%d\n", GLP_UNDEF, GLP_FEAS, GLP_INFEAS, GLP_NOFEAS);
	for (i=0; i<steps; i++) {
		printf("\t(%d,%d)", uav_trace->opt[i].prim, uav_trace->opt[i].dual);
	}

	printf("\n\nALSO STORED THE FOLLOWING FILES:\n");
	strncpy(tmp_str, argv[2], sizeof(tmp_str));
	strncat(tmp_str, "_X", sizeof(tmp_str));
	matfile = fopen(tmp_str, "w");
	gsl_matrix_pretty(matfile, uav_trace->x, "%21.18e");
	printf("\t%s\n", tmp_str);
	fclose(matfile);
	strncpy(tmp_str, argv[2], sizeof(tmp_str));
	strncat(tmp_str, "_U", sizeof(tmp_str));
	matfile = fopen(tmp_str, "w");
	gsl_matrix_pretty(matfile, uav_trace->u, "%21.18e");
	printf("\t%s\n", tmp_str);
	fclose(matfile);
	strncpy(tmp_str, argv[2], sizeof(tmp_str));
	strncat(tmp_str, "_steps", sizeof(tmp_str));
	matfile = fopen(tmp_str, "w");
	gsl_vector_long_pretty(matfile, uav_trace->steps, "%8ld");
	printf("\t%s\n", tmp_str);
	fclose(matfile);
	strncpy(tmp_str, argv[2], sizeof(tmp_str));
	strncat(tmp_str, "_time", sizeof(tmp_str));
	matfile = fopen(tmp_str, "w");
	gsl_vector_pretty(matfile, uav_trace->time, "%21.18e");
	printf("\t%s\n", tmp_str);
	fclose(matfile);

	/* Free all */
	free(uav_mpc.model);
	gsl_vector_free(uav_mpc.w);
	gsl_vector_free(uav_mpc.x0);
	glp_delete_prob(uav_mpc.op);
	free(uav_mpc.param);
	dyn_trace_free(uav_trace);

	return 0;
}

void ctrl_by_mpc(size_t k, dyn_trace * t, void *param)
{
	gsl_vector *x_k;
	mpc_glpk *my_mpc;
	size_t i;
#ifdef PRINT_PROBLEM
	char s_file[100] = PROB_FILENAME;
	char s_sol[100] = SOL_FILENAME;
	char tmp[100];
#endif

	my_mpc = (mpc_glpk *)param;
	x_k = gsl_vector_calloc(t->n);

	/* Get the current state from the k-th column of t->x */
	gsl_matrix_get_col(x_k, t->x, k);
	/* Update the initial state accordingly */
	gsl_vector_memcpy (my_mpc->x0, x_k);
	mpc_update_x0(my_mpc);

#ifdef PRINT_PROBLEM
	/* DEBUG ONLY: Writing the GLPK formulation in CPLEX form */
	sprintf(tmp, "%02lu", k);
	strcat(tmp, s_file);
	glp_write_lp(my_mpc->op, NULL, tmp);
#endif
	/* resuming old basis for testing purpose */
	mpc_status_resume(my_mpc, mpc_st);
	
#ifdef HAVE_OBSTACLE
	glp_simplex(my_mpc->op, my_mpc->param);
	glp_intopt(my_mpc->op, NULL);
#else
	/* Solve it by Simplex method */
	glp_simplex(my_mpc->op, my_mpc->param);
#endif

	/* Getting the solution */
	for (i = 0; i < t->m; i++) {
		gsl_matrix_set(t->u, i, k,
			       glp_get_col_prim(my_mpc->op,
						my_mpc->v_U+(int)i));
	}

	/* Store optimality of solution */
	t->opt[k].prim = glp_get_prim_stat(my_mpc->op);
	t->opt[k].dual = glp_get_dual_stat(my_mpc->op);

#ifdef PRINT_PROBLEM
	/* DEBUG ONLY: get the U0 from the solution. For the moment
	   just printing */
	glp_print_prob(my_mpc->op);
	sprintf(tmp, "%02lu", k);
	strcat(tmp, s_sol);
	glp_print_sol(my_mpc->op, tmp);
#endif	
	gsl_vector_free(x_k);


}

int model_mpc_startup(mpc_glpk * mpc, json_object * in)
{
#ifdef HAVE_OBSTACLE
	/* TO BE FIXED: ADDED TO JSON FILE?? */
	double center[] = OBSTACLE_CENTER;
	double radius[] = OBSTACLE_RADIUS;
#endif
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
	mpc->param->msg_lev = GLP_MSG_ALL; /* all messages */
#else
	mpc->param->msg_lev = GLP_MSG_OFF; /* no message */
#endif
#ifdef USE_DUAL
	mpc->param->meth    = GLP_DUAL;    /* dual simplex */
#endif

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
