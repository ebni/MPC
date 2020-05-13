#ifndef _MPC_H_
#define _MPC_H_
#include <glpk.h>
#include "dyn.h"

/*
 * Formulating a  Model-Predictive Control  (MPC) problem as  a Linear
 * Programming  (LP) problem.  Then  solving  it via  the  GNU LP  Kit
 * (GLPK).
 */
typedef struct {
	dyn_plant *model; /* the plant */
	gsl_vector *x0;   /* initial state */
	gsl_vector *x_lo; /* state lower bounds */
	gsl_vector *x_up; /* state upper bounds */
	gsl_vector *w;    /* weight to the (final) state */
	size_t h_ctrl;    /* length of control horizon */
	glp_prob *op;     /* the optimization problem */
	glp_smcp *param;  /* param of the solver: max_iter, dual/primal */
	gsl_vector * max_rate;/* array of max input rates (if <0 no max rate) */
	int v_U;          /* index of the 1st input variable */
	int v_Ninf_X;     /* index of the 1st state norm-infty vars */
	int v_absU;       /* index of the 1st variable of abs(input) */
	int v_B;          /* index of binary vars (to model obstacles) */
	int id_deltaU;    /* index of the 1st constraint on max delta U */
	int id_norm;      /* index of the 1st constraint on state norm */
	int id_absU;      /* index of the 1st constraint of abs(input) */
	int id_state_bnds;/* index of the 1st constraint on state bounds */
	int id_obstacle;  /* index of the 1st constraint of the obstacle */
} mpc_glpk;

/*
 * State of  the solver which can  be saved and restored  properly. In
 * the case of  MPC being solved by  GLPK, the state of  the solver is
 * modelled by the basic/non-basic/else status  of rows and columns of
 * the LP problem corresponding to the MPC.
 */
typedef struct {
	double * state;        /* Initial state x0 */
	double * input;        /* Input found */
	uint32_t * row_stat;   /* Basic/non-basic status of rows */
	uint32_t * col_stat;   /* Basic/non-basic status of columns */
	int * steps_bdg;      /* steps budget. recv: avail. sent: consumed */
	double * time_bdg;     /* time budget (sec). recv: avail. sent: cons */
	int * prim_stat;       /* primal status of the basis */
	int * dual_stat;       /* dual status of the basis */
	size_t size;           /* Size of allocated block */
	void * block; /* data is store contiguously to facilitate */
} mpc_status;


/*
 * Adding  the variables  for  the  control input  to  the MPC  problem
 * pointed  by  mpc. A successful invocation needs:
 * - mpc->model be initialized
 * - the JSON object in have the following fields:
 *     "len_ctrl", number of steps in which a new input is applied
 *
 * Each input variable  is a vector of size  mpc->model->m. The numper
 * of  input  vectors are  "len_ctrl"+1:  the  first "len_ctrl"  input
 * vectors (U(0),  U(1), ..., U("len_ctrl"-1)) are  held constant over
 * the corresponding sampling interval.  The last input labelled U(XX)
 * is  held  constant   until  the  end  of   the  prediction  horizon
 * mpc->model->H.
 */
void mpc_input_addvar(mpc_glpk * mpc, struct json_object * in);

/*
 * Set the bound on the input variables. A successful invocation needs:
 * - GLPK control variables of mpc->op be initialized
 * - the JSON object in have the following fields:
 *     "input_bounds", array of lower/upper bound of input
 */
void mpc_input_set_bnds(mpc_glpk * mpc, struct json_object * in);

/*
 * Add constraints on maximum admissible rate of inputs. A successful
 * invocation needs:
 * - GLPK control control variables of mpc->op be initialized
 * - the JSON object in have the following fields:
 *     "input_rate_max", array of max input rates be initiaized
 */
void mpc_input_set_delta(mpc_glpk * mpc, struct json_object * in);

/*
 * Add  constraints  on  maximum/minimum  first  input  based  on  max
 * admissible  rate of  inputs and  last applied  input. A  successful
 * invocation needs:
 * - GLPK control control variables of mpc->op be initialized
 * - the JSON object in have the following fields:
 *     "input_rate_max", array of max input rates be initiaized
 * - the parameter u0 be set with last input
 */
void mpc_input_set_delta0(mpc_glpk * mpc, const gsl_vector * u0);

/*
 * Add (mpc->model->H)  variables corresponding  to the  infty-norm of
 * the state from  X(1) to X(H). A successful invocation needs:
 * - GLPK control variables of mpc->op be initialized
 * - the JSON object in have the following fields:
 *     "state_weight", array of weight of each element of the state to
 *       compute the state infty-norm
 * After a successful  invocation mpc->v_Ninf_X is equal to  the index of
 * the first variable of this type.
 */
void mpc_state_norm_addvar(mpc_glpk * mpc, struct json_object * in);

void mpc_input_norm_addvar(mpc_glpk * mpc);

/*
 * Set the bound on the state variables. A successful invocation needs:
 * - GLPK state norm constraints initialized  (mpc->v_Ninf_X non zero)
 * - the JSON object in have the following fields:
 *     "state_bounds", array of lower/upper bound of the state
 */
void mpc_state_set_bnds(mpc_glpk * mpc, struct json_object * in);

/*
 * Set the goal of the MPC optimization. A successful invocation needs:
 * - the JSON object in have the following fields:
 *     "cost_model", an object describing the model of cost to be
 *       used.  Such an object must have the string filed "type"
 *       describing the type of cost model.
 * Depending on the cost model (which are selected by "type", other
 * initializations may be needed. Currently, the following "type" of
 * "cost_model" are implemented:
 *   "min_steps_to_zero", tries to reach a zero norm state (norm being
 *     defined as  weighted infty-norm  by the  weights "state_weight"
 *     set in  mpc_state_norm_addvar(...)) in  the smallest  number of
 *     steps. This is achieved giving exponentially incresing costs to
 *     state norms  over time. The  field "coef"  is the base  of such
 *     exponential.
 */
void mpc_goal_set(mpc_glpk * mpc, struct json_object * in);


/*
 * Warm up the  MPC with initial zero  state and solve it  so that the
 * initial basis are initialized, etc.
 */
void mpc_warmup(mpc_glpk * mpc);

/*
 * Update the initial state of the plant and the goal of the
 * optimization accordingly. The initial state must be previously
 * stored in mpc->x0.
 */
void mpc_update_x0(mpc_glpk * mpc);

/*
 * Model the presence of an obstacle by adding BINARY (not continuous)
 * variables. The obstable is modeled by  an array center and an array
 * size.   Both arrays  are  expected to  be  mpc->model->n long.  The
 * obstacle is modeled  as a box centered at center.  The edges of the
 * box  are 2*size  long. If  size[i] is  equal to  zero, then  no box
 * contraint along the i-th dimension.
 *
 * FUTURE EXTENSION (not too complicated): model a moving obstable,
 * possibly with varying size.
 */
void mpc_state_obstacle_add(mpc_glpk * mpc, double *center, double *size);


/*
 * Allocate and return the struct for storing/re-storing the status of
 * the MPC  problem passed as  parameter.  In  case GLPK is  used, the
 * solver  state  is  the  row/column basic/non-basic  status  of  the
 * corresponding LP problem
 */
mpc_status * mpc_status_alloc(const mpc_glpk * mpc);

void mpc_status_free(mpc_status * sol_st);

/*
 * Store the status of the solver in the corresponding struct. In case
 * GLPK is  used, the solver  state is the  row/column basic/non-basic
 * status of the corresponding LP problem
 */
void mpc_status_save(const mpc_glpk * mpc, mpc_status * sol_st);

/*
 * Get the status  of the solver from the parameter  sol_st and update
 * the optimization  problem accordingly.  In case  GLPK is  used, the
 * solver  state  is  the  row/column basic/non-basic  status  of  the
 * corresponding LP problem
 */
void mpc_status_resume(mpc_glpk * mpc, const mpc_status * sol_st);

/*
 * Print the solver status (mostly for debugging purpose)
 */
void mpc_status_fprintf(FILE *f,
			const mpc_glpk * mpc, const mpc_status * sol_st);
#endif  /* _MPC_H_ */
