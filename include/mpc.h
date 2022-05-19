/**
 * @file mpc.h 
 * @author Enrico	Bini
 * @brief  Formulating a  Model-Predictive Control  (MPC) problem as  a Linear
 * Programming  (LP) problem.  Then  solving  it via  the  GNU LP  Kit
 * (GLPK).
 */

#ifndef _MPC_H_
#define _MPC_H_
#include <glpk.h>
#include "dyn.h"


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
 * Status of the  solver which can be saved and  restored properly. In
 * the case of  MPC being solved by  GLPK, the state of  the solver is
 * modelled by the basic/non-basic/else status  of rows and columns of
 * the LP problem corresponding to the MPC.
 *
 * This data  structure has a  block of data,  which will be  send and
 * received,  and several  pointers to  this block  of data,  used for
 * convenience.
 */
typedef struct {
	double * state;       /* Initial state x0 */
	double * input;       /* Input found */
	uint32_t * row_stat;  /* Basic/non-basic status of rows */
	uint32_t * col_stat;  /* Basic/non-basic status of columns */
	int * steps_bdg;      /* steps budget. recv: avail. sent: consumed */
	double * time_bdg;    /* time budget (sec). recv: avail. sent: cons */
	int * prim_stat;      /* primal status of the basis */
	int * dual_stat;      /* dual status of the basis */
	size_t size;          /* Size of allocated block */
	void * block;         /* all data which is then sent if needed  */
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

/**
 * @brief Adding  the variables  for  the  control input  to  the MPC  problem
 * pointed  by  mpc.
 * 
 * @param mpc mpc_glpk* contain the representation of the MPC problem
 * @param in  json_object* json object containing initialization data read from file
 * @note mpc model must be initialized 
 * @note The json object must have the field "len_ctrl",  
 * 		 number of steps in which a new input is applied initialized
 */
void mpc_input_addvar(mpc_glpk * mpc, json_object * in);

/*
 * Set the bound on the input variables. A successful invocation needs:
 * - GLPK control variables of mpc->op be initialized
 * - the JSON object in have the following fields:
 *     "input_bounds", array of lower/upper bound of input
 */

/**
 * @brief Set the bound on the input variables.
 * 
 * @param mpc mpc_glpk* contain the representation of the MPC problem
 * @param in  json_object* json object containing initialization data read from file
 * @note GLPK control variables of mpc->op be initialized
 * @note The JSON object in must have the "input_bounds" field,
 * 	     array of lower/upper bound of input, initialized
 */
void mpc_input_set_bnds(mpc_glpk * mpc, json_object * in);

/*
 * Add constraints on maximum admissible rate of inputs. A successful
 * invocation needs:
 * - GLPK control control variables of mpc->op be initialized
 * - the JSON object in have the following fields:
 *     "input_rate_max", array of max input rates be initiaized
 */

/**
 * @brief Add constraints on maximum admissible rate of inputs.
 * 
 * @param mpc mpc_glpk* contain the representation of the MPC problem
 * @param in  json_object* json object containing initialization data read from file
 * @note GLPK control variables of mpc->op be initialized
 * @note The JSON object in must have the "input_rate_max" field,
 * 	     array of max input rates, initialized 
 */
void mpc_input_set_delta(mpc_glpk * mpc, json_object * in);

/*
 * Add  constraints  on  maximum/minimum  first  input  based  on  max
 * admissible  rate of  inputs and  last applied  input. A  successful
 * invocation needs:
 * - GLPK control control variables of mpc->op be initialized
 * - the JSON object in have the following fields:
 *     "input_rate_max", array of max input rates be initiaized
 * - the parameter u0 be set with last input
 */

/**
 * @brief Add  constraints  on  maximum/minimum  first  input  based  on  max
 * admissible  rate of  inputs and  last applied  input.
 * 
 * @param mpc mpc_glpk* 			The representation of the MPC problem 
 * @param u0  const gsl_vector * 	last applied input
 * @note GLPK control control variables of mpc->op be initialized
 * @note The JSON object in have the field "input_rate_max", 
 * 		 array of max input rates, initiaized
 * @note The parameter u0 must be set with last input
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

/**
 * @brief Add (mpc->model->H) variables corresponding to the infty-norm of
 * the state from X(1) to X(H).
 * 
 * @param mpc mpc_glpk* 			The representation of the MPC problem
 * @param in  json_object* 	json object containing initialization data read from file
 * @note  GLPK control variables of mpc->op must be initialized
 * @note  The JSON object in have the "state_weight" field, 
 * 		  array of weight of each element of the state to
 *        compute the state infty-norm, initialized
 */
void mpc_state_norm_addvar(mpc_glpk * mpc, json_object * in);

/**
 * @brief //TODO finish write brief
 * 
 * @param mpc mpc_glpk*	The representation of the MPC problem
 */
void mpc_input_norm_addvar(mpc_glpk * mpc);

/*
 * Set the bound on the state variables. A successful invocation needs:
 * - GLPK state norm constraints initialized  (mpc->v_Ninf_X non zero)
 * - the JSON object in have the following fields:
 *     "state_bounds", array of lower/upper bound of the state
 */

/**
 * @brief Set the bound on the state variables.
 * 
 * @param mpc mpc_glpk*				The representation of the MPC problem
 * @param in  json_object* 	JSON object containing initialization data read from file
 */
void mpc_state_set_bnds(mpc_glpk * mpc, json_object * in);

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

/**
 * @brief Set the goal of the MPC optimization.
 * 
 * @param mpc mpc_glpk*				The representation of the MPC problem
 * @param in  json_object* 	JSON object containing initialization data read from file
 * @note  The JSON object in have the "cost_model" field,
 * 		  an object describing the model of cost to be
 *        used, initialized. Such an object must have the string filed "type"
 *        describing the type of cost model.
 * @note  Depending on the cost model (which are selected by "type", other
 * 		  initializations may be needed. 
 * @note  Currently, the following "type" of "cost_model" are implemented:
 *   	  "min_steps_to_zero", tries to reach a zero norm state (norm being
 *     	  defined as  weighted infty-norm  by the  weights "state_weight"
 *     	  set in  mpc_state_norm_addvar(...)) in  the smallest  number of
 *     	  steps. This is achieved giving exponentially incresing costs to
 *     	  state norms  over time. The  field "coef"  is the base  of such
 *     	  exponential.
 */
void mpc_goal_set(mpc_glpk * mpc, json_object * in);


/*
 * Warm up the  MPC with initial zero  state and solve it  so that the
 * initial basis are initialized, etc.
 */

/**
 * @brief  Warm up the  MPC with initial zero state and solve it so that the
 * initial basis are initialized, etc.
 * 
 * @param mpc mpc_glpk*	The representation of the MPC problem
 */
void mpc_warmup(mpc_glpk * mpc);

/*
 * Update the initial state of the plant and the goal of the
 * optimization accordingly. The initial state must be previously
 * stored in mpc->x0.
 */

/**
 * @brief Update the initial state of the plant and the goal of the
 * optimization accordingly.
 * 
 * @param mpc mpc_glpk*	The representation of the MPC problem
 * @note  The initial state must be previously stored in mpc->x0.
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

/**
 * @brief Model the presence of an obstacle by adding BINARY (not continuous)
 * variables.
 * 
 * @param mpc 	 mpc_glpk*	The representation of the MPC problem
 * @param center double*	Position of the obstacle to avoid
 * @param size 	 double*	Size of the obstacle to avoid
 */
void mpc_state_obstacle_add(mpc_glpk * mpc, double *center, double *size);


/*
 * Allocate and return the struct for storing/re-storing the status of
 * the MPC  problem passed as  parameter.  In  case GLPK is  used, the
 * solver  state  is  the  row/column basic/non-basic  status  of  the
 * corresponding LP problem
 */

/**
 * @brief Allocate the struct for storing/re-storing the status of
 * the MPC  problem passed as  parameter.
 * 
 * @param  mpc mpc_glpk*	The representation of the MPC problem
 * @return 	   mpc_status*	The struct containing the status of the mpc problem 
 */
mpc_status * mpc_status_alloc(const mpc_glpk * mpc);

/**
 * @brief free mpc_status
 * 
 * @param sol_st mpc_status* Solution state
 */
void mpc_status_free(mpc_status * sol_st);

/*
 * Set the  initial state x0  from sol_st->state to  the corresponding
 * field in mpc
 */

/**
 * @brief Set the  initial state x0  from sol_st->state to  the corresponding
 * field in mpc
 * 
 * @param mpc    mpc_glpk*			The representation of the MPC problem
 * @param sol_st const mpc_status*	Solution state
 */
void mpc_status_set_x0(mpc_glpk * mpc, const mpc_status * sol_st);

/*
 * Store the status of the solver in the corresponding struct. In case
 * GLPK is  used, the solver  state is the  row/column basic/non-basic
 * status of the corresponding LP problem
 */

/**
 * @brief Store the status of the solver in the corresponding struct.
 * 
 * @param mpc    const mpc_glpk*	The representation of the MPC problem
 * @param sol_st mpc_status*		Solution state
 * @note  In case GLPK is used, the solver state is the row/column basic/non-basic
 * 		  status of the corresponding LP problem
 */
void mpc_status_save(const mpc_glpk * mpc, mpc_status * sol_st);

/*
 * Get the status  of the solver from the parameter  sol_st and update
 * the optimization  problem accordingly.  In case  GLPK is  used, the
 * solver  state  is  the  row/column basic/non-basic  status  of  the
 * corresponding LP problem
 */

/**
 * @brief Get the status of the solver from the parameter sol_st and update
 * the optimization problem accordingly.
 * 
 * @param mpc    mpc_glpk*			The representation of the MPC problem
 * @param sol_st const mpc_status*	Solution state
 * @note In case GLPK is used, the solver state is the row/column 
 * 		 basic/non-basic status of the corresponding LP problem
 */
void mpc_status_resume(mpc_glpk * mpc, const mpc_status * sol_st);

/*
 * Print the solver status (mostly for debugging purpose)
 */

/**
 * @brief Print the solver status (mostly for debugging purpose)
 * 
 * @param f 	 FILE*				File stream where to print
 * @param mpc    const mpc_glpk*	The representation of the MPC problem
 * @param sol_st const mpc_status*	Solution state
 */
void mpc_status_fprintf(FILE *f,
			const mpc_glpk * mpc, const mpc_status * sol_st);
#endif  /* _MPC_H_ */
