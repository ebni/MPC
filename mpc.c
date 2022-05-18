/**
 * @file mpc.c
 * @author Enrico	Bini
 * @brief implementation of the library mpc.h
 * @copyright Copyright (c) 2022
 * 
 */
#define HAVE_INLINE
#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_sf_exp.h>
#include <glpk.h>
#include "dyn.h"
#include "mpc.h"

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}

#define DOUBLE_SMALL 1e-10 /* may be needed to be a small number 1e-10 */
#define DONTCARE 0 /* any constant to be ignored */
#define BIG_M 1e4 /* only needed for obstacles */

/* macros for lower/upper bounds */
#define HAS_NONE  0x00
#define HAS_LOWER 0x01
#define HAS_UPPER 0x02

//aux functions

/**
 * @brief function aux for mcp_input_set_bnds that parse input from json obj.
 * 
 * @param bnds 
 * @param bnds_lo 
 * @param bnds_up 
 * @param mpc 
 * @param bnds_has 
 */
void check_input_json(json_object * bnds, double* bnds_lo, double* bnds_up,
					mpc_glpk* mpc, char* bnds_has );


/**
 * @brief auxiliary function for mpc_input_set_delta() for parsing json data
 * 
 * @param mpc 
 * @param vec_rates 
 */
void parse_json_input_set_delta(mpc_glpk* mpc, json_object *vec_rates);

/**
 * @brief support funtion that store normal state weights
 * 
 * @param mpc   describing mpc problem structure
 * @param vec_w vector of weights
 */
void mcp_state_normal_store_state_weights(mpc_glpk * mpc, json_object * vec_w);

/**
 * @brief minimizes an overall  cost in which: the
 *        state is  weighted as  described in  "min_steps_to_zero" (hence
 *        "coef"  needs  to   b  defined),  the  input   is  weighted  by
 *        "input_weight"
 * 
 * @param mpc describing mpc problem structure
 * @param in input
 */
void min_state_input_norms(mpc_glpk* mpc, json_object * in);

/**
 * @brief tries to reach a zero norm state (norm being
 *     defined as  weighted infty-norm  by the  weights "state_weight"
 *     set in  mpc_state_norm_addvar(...)) in  the smallest  number of
 *     steps. This is achieved giving exponentially incresing costs to
 *     state norms  over time. The  field "coef"  is the base  of such
 *     exponential
 * 
 * @param mpc describing mpc problem structure
 * @param in input
 */
void min_steps_to_zero(mpc_glpk* mpc, json_object * in);
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
void mpc_input_addvar(mpc_glpk * mpc, json_object * in)
{
	char s[200];
	size_t i, j;
	int id;
	json_object *tmp;
	
	/* Get the length of control horizon */
	if (!json_object_object_get_ex(in, "len_ctrl", &tmp)) {
		PRINT_ERROR("missing len_ctrl in JSON");
		return;
	}
	mpc->h_ctrl = (size_t)json_object_get_int(tmp);
	/*mpc->h_ctrl = p; */
	for (i=0; i < mpc->h_ctrl+1; i++) {
		for (j=0; j < mpc->model->m; j++) {
		  id = (size_t)glp_add_cols(mpc->op, 1);
			if (!(i|j)) { /* i==0 && j==0 */
				mpc->v_U = id;
			}
			/* Giving a name to variables */
			if (i < mpc->h_ctrl) {
				sprintf(s,"U%i[%02i]",(int)j,(int)i);
			} else {
				sprintf(s,"U%i[XX]",(int)j);
			}
			glp_set_col_name(mpc->op, (int)id, s);
		}
	}
}

/**
 * @brief add a normalized var to mpc
 * 
 * @param mpc mpc_glpk*	The representation of the MPC problem
 */
void mpc_input_norm_addvar(mpc_glpk * mpc)
{
	char s[200];
	size_t i, j;
	int v_cur, c_cur;
	int ind[3] = {DONTCARE, DONTCARE, DONTCARE};
	double val[3] = {DONTCARE, 1.0, 1.0};
	
	/* absolute values of inputs already set */
	if (mpc->v_absU > 0)
		return;

	/* looping over all input vars */
	for (i=0; i < mpc->h_ctrl+1; i++) {
		/* looping over the components */
		for (j=0; j < mpc->model->m; j++) {
			/* one variable, two constraints */
			v_cur = glp_add_cols(mpc->op, 1);
			glp_set_col_bnds(mpc->op, v_cur, GLP_FR, DONTCARE, DONTCARE);
			c_cur = glp_add_rows(mpc->op, 2);
			if (i==0 && j==0) {
				mpc->v_absU = v_cur;
				mpc->id_absU = c_cur;
			}
			/* Giving a name to variables */
			if (i < mpc->h_ctrl)
				sprintf(s,"|U%i(%02i)|",(int)j,(int)i);
			else
				sprintf(s,"|U%i(XX)|",(int)j);
			
			glp_set_col_name(mpc->op, (int)v_cur, s);
			/* Setting U_j(i) <= |U_j(i)|*/
			sprintf(s,"|U%i(%02d)|_UP", (int)j, (int)i);
			glp_set_row_name(mpc->op, c_cur, s);
			ind[1] = mpc->v_U+(int)(i*(mpc->model->m)+j);
			ind[2] = v_cur;
			/* val[1] = 1.0; */ /* same as initialization */
			val[2] = -1.0;
			glp_set_mat_row(mpc->op, c_cur, 2, ind, val);
			glp_set_row_bnds(mpc->op, c_cur, GLP_UP, DONTCARE, 0);
			/* Setting U_j(i) >= -|U_j(i)|*/
			sprintf(s,"|U%i(%02d)|_LO", (int)j, (int)i);
			glp_set_row_name(mpc->op, c_cur+1, s);
			val[2] = 1.0;
			glp_set_mat_row(mpc->op, c_cur+1, 2, ind, val);
			glp_set_row_bnds(mpc->op, c_cur+1, GLP_LO, 0, DONTCARE);
		}
	}
}



void check_input_json(json_object * bnds, double* bnds_lo, double* bnds_up, mpc_glpk* mpc, char* bnds_has )
{
	size_t i;
	json_object *bnds1,*elem;
	
		
	for (i=0; i < mpc->model->m; i++) {
		if ((bnds1 = json_object_array_get_idx(bnds, (int)i)) == NULL) {
			fprintf(stderr, "Erroneous index %i\n", (int)i);
			PRINT_ERROR("Error in getting an input bound in JSON");
			return;
		}
		/* getting lower bound */
		elem = json_object_array_get_idx(bnds1, 0);
		bnds_lo[i] = json_object_get_double(elem);
		
		bnds_has[i]|=(isfinite(bnds_lo[i]))? HAS_LOWER:  HAS_NONE;
		
		elem = json_object_array_get_idx(bnds1, 1);
		bnds_up[i] = json_object_get_double(elem);
		if (isfinite(bnds_up[i]))
			bnds_has[i] |= HAS_UPPER;
		else
			bnds_has[i] |= HAS_NONE;
	}

}


/*
 * Set the bound on the input variables. A successful invocation needs:
 * - GLPK variables of mpc->op be initialized
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
void mpc_input_set_bnds(mpc_glpk * mpc, json_object * in)
{
	size_t i,j;
	int id;
	json_object * bnds;
	char *bnds_has;
	double *bnds_lo, *bnds_up;
	
	/* Get the input bounds */
	if (!json_object_object_get_ex(in, "input_bounds", &bnds)) {
		PRINT_ERROR("missing input_bounds in JSON");
		return;
	}
	if ((size_t)json_object_array_length(bnds) != mpc->model->m) {
		PRINT_ERROR("wrong size of input_bounds in JSON");
		return;
	}
	bnds_has = calloc(mpc->model->m, sizeof(*bnds_has));
	bnds_lo  = calloc(mpc->model->m, sizeof(*bnds_lo));
	bnds_up  = calloc(mpc->model->m, sizeof(*bnds_up));

	check_input_json(bnds, bnds_lo,  bnds_up, mpc, bnds_has);
	
	/* Setting the bounds in the GLPK problem */
	id = mpc->v_U;
	for (i=0; i < mpc->h_ctrl+1; i++) {
		for (j=0; j < mpc->model->m; j++, id++) {
			switch (bnds_has[j]) {
			case (HAS_NONE):
				glp_set_col_bnds(mpc->op, id, GLP_FR,DONTCARE,DONTCARE);
				break;
			case (HAS_LOWER):
				glp_set_col_bnds(mpc->op, id, GLP_LO,bnds_lo[j],DONTCARE);
				break;
			case (HAS_UPPER):
				glp_set_col_bnds(mpc->op, id, GLP_UP,DONTCARE,bnds_up[j]);
				break;
			case (HAS_LOWER | HAS_UPPER): glp_set_col_bnds(mpc->op, id,
						 GLP_DB,bnds_lo[j],bnds_up[j]);
				break;
			}
		}
	}
	free(bnds_has);
	free(bnds_lo);
	free(bnds_up);
}


void parse_json_input_set_delta(mpc_glpk* mpc, json_object *vec_rates)
{
	size_t i;
	json_object *elem;
		
	for (i=0; i < mpc->model->m; i++) {
		if ((elem = json_object_array_get_idx(vec_rates, (int)i)) == NULL) {
			fprintf(stderr, "Erroneous index %i\n", (int)i);
			PRINT_ERROR("Error in getting an input max rate in JSON");
			return;
		}
		gsl_vector_set(mpc->max_rate, i, json_object_get_double(elem));
		if (!isfinite(gsl_vector_get(mpc->max_rate, i)))
			/* means no max rate is set */
			gsl_vector_set(mpc->max_rate, i, -1);
	}
}

/*
 * Add constraints on maximum admissible rate of inputs. A successful
 * invocation needs:
 * - GLPK variables of mpc->op be initialized
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
//TODO:
void mpc_input_set_delta(mpc_glpk * mpc, json_object * in)
{
	char s[200];
	double * val, *bnd;
	size_t i, j;
	int id, *ind;
	json_object * vec_rates; //,*elem;
	
	/* Get the input bounds */
	if (!json_object_object_get_ex(in, "input_rate_max", &vec_rates)) {
		PRINT_ERROR("missing input_rate_max in JSON");
		return;
	}
	if ((size_t)json_object_array_length(vec_rates) != mpc->model->m) {
		PRINT_ERROR("wrong size of input_rate_max in JSON");
		return;
	}
	mpc->max_rate = gsl_vector_calloc(mpc->model->m);
	parse_json_input_set_delta(mpc, vec_rates);
	/* Parsing input_bounds from JSON file*/
	//TODO:Parsing input_bounds from JSON obj
	/* for (i=0; i < mpc->model->m; i++) {
		if ((elem = json_object_array_get_idx(vec_rates, (int)i)) == NULL) {
			fprintf(stderr, "Erroneous index %i\n", (int)i);
			PRINT_ERROR("Error in getting an input max rate in JSON");
			return;
		}
		gsl_vector_set(mpc->max_rate, i, json_object_get_double(elem));
		if (!isfinite(gsl_vector_get(mpc->max_rate, i)))
			// means no max rate is set
			gsl_vector_set(mpc->max_rate, i, -1);
	} */

	/* extra variable to have more compact code */
	bnd = mpc->max_rate->data; 
	
	ind = malloc(3*sizeof(*ind));
	val = malloc(3*sizeof(*val));
	for (i=0; i < mpc->h_ctrl; i++) {
		for (j=0; j < mpc->model->m; j++) {
			if (bnd[j] < 0) {
				/* No max rate */
				continue;
			}
			id = glp_add_rows(mpc->op, 1);
			if (!(i|j)) { /* i==0 && j==0 */
				mpc->id_deltaU = id;
			}
			sprintf(s,"U%i[%02i]_(rate)",(int)j,(int)i);
			glp_set_row_name(mpc->op, id, s);
			ind[1] = (int)(i*mpc->model->m+j+1);
			ind[2] = (int)((i+1)*mpc->model->m+j+1);
			val[1] = -1;
			val[2] = 1;
			glp_set_mat_row(mpc->op, id, 2, ind, val);
	#ifdef USE_SAMPLED
			glp_set_row_bnds(mpc->op, id, GLP_DB,
					 -bnd[j]*mpc->model->tau, bnd[j]*mpc->model->tau);
	#else
			glp_set_row_bnds(mpc->op, id, GLP_DB, -bnd[j], bnd[j]);
	#endif
		}
	}
	free(ind);
	free(val);
}

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
void mpc_input_set_delta0(mpc_glpk * mpc, const gsl_vector * u0)
{
	gsl_vector *lo, *up;
	int id;
	double bnd, col_lo, col_up;
	size_t i, j;

	lo = gsl_vector_calloc(mpc->model->m);
	gsl_vector_memcpy(lo, u0);
	up = gsl_vector_calloc(mpc->model->m);
	gsl_vector_memcpy(up, u0);

	/* Get and possibly set new bounds in the GLPK problem */
	id = mpc->v_U;
	for (i=0; i < mpc->h_ctrl+1; i++) {
		/* Widen the bounds as steps move forward */
		gsl_vector_sub(lo, mpc->max_rate);
		gsl_vector_add(up, mpc->max_rate);
		for (j=0; j < mpc->model->m; j++, id++) {
			if (gsl_vector_get(mpc->max_rate, j) < 0) {
				/* no max rate, just skip */
				continue;
			}
			if (gsl_vector_get(mpc->max_rate, j) == 0) {
				/* no admitted change, keep var constant */
				bnd = gsl_vector_get(u0, j);
				glp_set_col_bnds(mpc->op, id, GLP_FX, bnd, bnd);
				continue;
			}
			/* mpc->max_rate[j] > 0 */
			col_lo = glp_get_col_lb(mpc->op, id);
			col_lo = GSL_MAX(col_lo, gsl_vector_get(lo,j));
			col_up = glp_get_col_ub(mpc->op, id);
			col_up = GSL_MIN(col_up, gsl_vector_get(up,j));
			glp_set_col_bnds(mpc->op, id, GLP_DB, col_lo, col_up);
		}
	}

	
}

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

void mcp_state_normal_store_state_weights(mpc_glpk * mpc, json_object * vec_w)
{
	size_t i;
	json_object *elem;
	mpc->w = gsl_vector_calloc(mpc->model->n);
	for (i=0; i < mpc->model->n; i++) {
		elem = json_object_array_get_idx(vec_w, (int)i);
		errno = 0;
		mpc->w->data[i] = json_object_get_double(elem);
		if (errno) {
			fprintf(stderr, "Error at index %i\n", (int)i);
			PRINT_ERROR("issues in converting element of state weight");
			return;
		}
	}
}

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
void mpc_state_norm_addvar(mpc_glpk * mpc, json_object * in)
{
	size_t i, j, k, n, m, H, p;
	int *ind, id;
	double *val_up; /*, *val_lo; */
	gsl_vector *gsl_val;
	gsl_matrix **L; /* linear op from U(0)...U(P) to X(i) */
	gsl_matrix *tmp;
	char s[200];
	json_object * vec_weight;
	
	/* Get the weight of each state component */
	if (!json_object_object_get_ex(in, "state_weight", &vec_weight)) {
		PRINT_ERROR("missing state_weight in JSON");
		return;
	}
	if ((size_t)json_object_array_length(vec_weight) != mpc->model->n) {
		PRINT_ERROR("wrong size of state_weight in JSON");
		return;
	}

	/* Allocate/store state weights */
	mpc->w = gsl_vector_calloc(mpc->model->n);
	mcp_state_normal_store_state_weights(mpc, vec_weight);
	

	/* Just to make code more compact/readable */
	n = mpc->model->n;
	m = mpc->model->m;
	H = mpc->model->H;
	p = mpc->h_ctrl;

	/* Allocate and initialize the linear operator from U to X(i) */
	L = malloc((p+1)*sizeof(*L));
	for (i=0; i <= p+1; i++) {
		L[i] = gsl_matrix_calloc(n, m);
	}

	/* Indices and value arrays to store the coefficients */
	ind = calloc((2+m*(mpc->h_ctrl+1)), sizeof(*ind));
	val_up = calloc((2+m*(mpc->h_ctrl+1)), sizeof(*val_up));
	/*	val_lo = calloc((2+m*(mpc->h_ctrl+1)), sizeof(*val_lo)); */
	gsl_val = gsl_vector_calloc(m);

	/* Looping over all state variables from X(1) to X(H) */
	for (i=1; i<=H; i++) {
		/* 
		 * Add variable |X(i)|_inf.
		 * Also, preparing the linear operator from U to X(i)
		 */
		if (i==1) {
			mpc->v_Ninf_X = id = glp_add_cols(mpc->op, 1);
			gsl_matrix_memcpy(L[0], mpc->model->ABd[0]);
		} else {
			id = glp_add_cols(mpc->op, 1);
			if (i <= p+1) {
				tmp = L[i-1];
				for (j=i-1; j>=1; j--)
					L[j] = L[j-1];
				L[0] = tmp;
				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
					       1,mpc->model->Ad[0],L[1],0,L[0]);
			} else {
				tmp = L[p];
				for (j=p; j>=1; j--)
					L[j] = L[j-1];
				gsl_matrix_add(L[p], tmp);
				L[0] = tmp;
				gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,
					       1,mpc->model->Ad[0],L[1],0,L[0]);
			}
		}
		sprintf(s,"|X(%02d)|_inf", (int)i);
		glp_set_col_name(mpc->op, id, s);
		/* |X(i)|_inf should always be >= 0. Not enforcing it
		 * explicitly for debugging */
		glp_set_col_bnds(mpc->op, id, GLP_FR, DONTCARE, DONTCARE);

		/* index of |X(i)|_inf */
		ind[1] = id;
		
		/* indices of U */
		if (i <= p+1) {
			for (j=0; j<m*i; j++) {
				ind[2+j] = mpc->v_U+(int)j;
			}
		} else {
			/* ind already properly filled */
		}

		/* Loop over components of X(i) */
		for (k=0; k<n; k++) {
			/* Loop over control inputs */
			for (j=0; j < i && j <= p; j++) {
				gsl_matrix_get_row(gsl_val, L[j], k);
				/* set coefs of U for upper bound */
				bcopy(gsl_val->data, val_up+m*j+2,
				      m*sizeof(*val_up));
				/* set coefs of U for lower bound  */
				/*				gsl_vector_scale(gsl_val, -1);
				bcopy(gsl_val->data, val_lo+m*j+2,
				m*sizeof(*val_lo)); */
			} /* j: loop over input U */
			/* coefficient of |X(i)|_inf */
			if (gsl_vector_get(mpc->w,k) > 0) {
				val_up[1] = /*val_lo[1] = */
					-1.0/gsl_vector_get(mpc->w,k);
			} else {
				/* UNUSED: any placeholder */
				val_up[1] = /*val_lo[1] = */0.12345;
			}

			/* Setting up upper bound on X_k(i) */
			if (i==1 && k==0) {
				mpc->id_norm = id =
					glp_add_rows(mpc->op, 1);
			} else {
				id = glp_add_rows(mpc->op, 1);
			}
			sprintf(s,"X%i(%02d) LE norm", (int)k, (int)i);
			glp_set_row_name(mpc->op, id, s);
			glp_set_mat_row(mpc->op, id, (int)(m*j+1),
					ind, val_up);
			
			/* setting up lower bound on X_k(i) */
			id = glp_add_rows(mpc->op, 1);
			sprintf(s,"X%i(%02d) GE norm", (int)k, (int)i);
			glp_set_row_name(mpc->op, id, s);
			val_up[1] = -val_up[1];
			glp_set_mat_row(mpc->op, id, (int)(m*j+1),
					ind, val_up);
		} /* k: loop over components of X(i) */
	} /* i: loop over X(i) */

	/* 
	 * The RHS of inequalities are set by invoking mpc_update_x0
	 * in the main or whenever is needed
	 */
#if 0
	/* Printing the full problem for debugging */
	glp_write_lp(mpc->op, NULL, "test.txt");
	glp_print_prob(mpc->op);
#endif

	/* Free memory */
	for (i=0; i <= p+1; i++) {
		/*
		 * EB: messing up 
		 gsl_matrix_free(L[i]);
		*/
	}
	/*	free(L);
	free(ind);
	free(val_up); */
/*	free(val_lo); */
}

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
void mpc_state_set_bnds(mpc_glpk * mpc, json_object * in)
{	
	
	size_t i,j,k,num_vars;
	json_object * bnds, *bnds1, *elem;
	char s[100];
	int * ind, id_norm, id, len, first;
	double * val;

	
	if (mpc->id_norm <= 0) {
		PRINT_ERROR("state norm variables not created, but needed here");
		return;
	}
	/* Get the input bounds */
	if (!json_object_object_get_ex(in, "state_bounds", &bnds)) {
		PRINT_ERROR("missing state_bounds in JSON");
		return;
	}
	if ((size_t)json_object_array_length(bnds) != mpc->model->n) {
		PRINT_ERROR("wrong size of state_bounds in JSON");
		return;
	}

	/* Allocate state upper bounds */
	mpc->x_lo = gsl_vector_calloc(mpc->model->n);
	mpc->x_up = gsl_vector_calloc(mpc->model->n);
	
	/* Parsing input_bounds from JSON file*/
	for (i=0; i < mpc->model->n; i++) {
		if ((bnds1 = json_object_array_get_idx(bnds, (int)i)) == NULL) {
			fprintf(stderr, "Erroneous index %i\n", (int)i);
			PRINT_ERROR("Error in getting an state bound in JSON");
			return;
		}
		/* get lower bound from JSON and set in MPC struct */
		elem = json_object_array_get_idx(bnds1, 0);
		gsl_vector_set(mpc->x_lo, i, json_object_get_double(elem));
		/* get upper bound from JSON and set in MPC struct */
		elem = json_object_array_get_idx(bnds1, 1);
		gsl_vector_set(mpc->x_up, i, json_object_get_double(elem));
	}
	
	/* Setting the bounds in the GLPK problem */
	num_vars = mpc->model->m*mpc->h_ctrl+1;
	/* Allocating for num_vars+1 because GLPK counts indices in array from 1 */
	ind = calloc(num_vars+1, sizeof(int));
	val = calloc(num_vars+1, sizeof(double));

	/* Getting coefficients from the norm constraints */
	id_norm = mpc->id_norm;
	first = 1;
	/* Looping over all state variables from X(1) to X(H) */
	for (i=1; i <= mpc->model->H; i++) {
		/* Loop over components of X(i) */
		for (k=0; k < mpc->model->n; k++) {
			/* Get the coefs of the corresponding norm constraint */
			len = glp_get_mat_row(mpc->op, id_norm, ind, val);
			id_norm += 2;
			/* searching for id of v_Ninf_X+i in ind */
			for (j=1; j<=(size_t)len; j++)
				if (ind[j] == mpc->v_Ninf_X+(int)i-1)
					break;

			/* removing the Z index and coef */
			if (j == (size_t)len) {
				/* Z var is last: do nothing */
			} else {
				memmove(ind+j, ind+j+1, sizeof(*ind)*((size_t)len-j));
				memmove(val+j, val+j+1, sizeof(*val)*((size_t)len-j));
			}
			len--;

			/* Setting constraint bounds: name, coefficients */
			id = glp_add_rows(mpc->op, 1);
			if (first) {
				mpc->id_state_bnds = id;
				first = 0;
			}
			sprintf(s,"X%i(%02d)_B", (int)k, (int)i);
			glp_set_row_name(mpc->op, id, s);
			glp_set_mat_row(mpc->op, id, len, ind, val);
			
			/* RHS NOT set here. Will be set by mpc_update_x0 */
		} /* k: loop over components of X(i) */
	} /* i: loop over X(i) */

	free(ind);
	free(val);
}

/*
 * Update the initial state of the plant.
 */

/**
 * @brief Update the initial state of the plant and the goal of the
 * optimization accordingly.
 * 
 * @param mpc mpc_glpk*	The representation of the MPC problem
 * @note  The initial state must be previously stored in mpc->x0.
 */
void mpc_update_x0(mpc_glpk * mpc) {
	gsl_vector *x_k;
	int id_normZ, id_Xbnds;
	size_t i, n, k, H;
	double lo, up, x_ik;

	n = mpc->model->n;
	H = mpc->model->H;
	id_normZ = mpc->id_norm;
	id_Xbnds = mpc->id_state_bnds;
	x_k = gsl_vector_calloc(n);

	/* Looping over all state variables from X(1) to X(H) */
	for (k=1; k<=H; k++) {
		/* Computing free evolution of X(k): Ad^k*x_0 */
		gsl_blas_dgemv(CblasNoTrans, 1, mpc->model->Ad[k-1],
			       mpc->x0, 0, x_k);
		/* Loop over components of X(k) */
		for (i=0; i < n; i++) {
			/* i-th component of X(k) */
			x_ik = gsl_vector_get (x_k, i);
			
			/* Updating RHS of state norm constraints */
			if (gsl_vector_get(mpc->w,i) > 0) {
				glp_set_row_bnds(mpc->op, id_normZ++, GLP_UP, DONTCARE, -x_ik);
				glp_set_row_bnds(mpc->op, id_normZ++, GLP_LO, -x_ik, DONTCARE);
			} else {
				/* no weight to this component of the state */
				#if 1 /* setting a very large upper bound */
				glp_set_row_bnds(mpc->op, id_normZ++, GLP_UP, DONTCARE, 1e10);
				glp_set_row_bnds(mpc->op, id_normZ++, GLP_UP, DONTCARE, 1e10);
				#else /* setting infinity as upper bound */
				glp_set_row_bnds(mpc->op, id++, GLP_FR, DONTCARE, DONTCARE);
				glp_set_row_bnds(mpc->op, id++,	GLP_FR, DONTCARE, DONTCARE);
				#endif
			}

			/* Updating RHS of state bound constraints */
			//TODO:write rhs update state bound constraints
			lo = gsl_vector_get(mpc->x_lo, i);
			up = gsl_vector_get(mpc->x_up, i);
			if (isfinite(lo) && isfinite(up))
				glp_set_row_bnds(mpc->op, id_Xbnds++, GLP_DB,
						lo - x_ik, up - x_ik);
			else if (isfinite(lo))
				glp_set_row_bnds(mpc->op, id_Xbnds++, GLP_LO,
						lo - x_ik, DONTCARE);
			else if (isfinite(up))
				glp_set_row_bnds(mpc->op, id_Xbnds++, GLP_UP,
						DONTCARE, up - x_ik);
			else /* no bounds */
				glp_set_row_bnds(mpc->op, id_Xbnds++, GLP_FR,
						DONTCARE, DONTCARE);
		}
	}
	gsl_vector_free(x_k);
}

/*
 * Set the goal of the MPC optimization. A successful invocation needs:
 * - the JSON object in have the following fields:
 *     "cost_model", an object describing the model of cost to be
 *       used.  Such an object must have the string filed "type"
 *       describing the type of cost model.
 * Depending on  the cost model  (which are selected by  "type", other
 * initializations may  be needed. Currently, the  following "type" of
 * "cost_model" are implemented:
 *   "min_steps_to_zero", tries to reach a zero norm state (norm being
 *     defined as  weighted infty-norm  by the  weights "state_weight"
 *     set in  mpc_state_norm_addvar(...)) in  the smallest  number of
 *     steps. This is achieved giving exponentially incresing costs to
 *     state norms  over time. The  field "coef"  is the base  of such
 *     exponential.
 *   "min_state_input_norms", minimizes an overall  cost in which: the
 *     state is  weighted as  described in  "min_steps_to_zero" (hence
 *     "coef"  needs  to   b  defined),  the  input   is  weighted  by
 *     "input_weight"
 */



void min_state_input_norms(mpc_glpk* mpc, json_object * in)
{
	size_t j;
	double coef, cur;
	json_object * cost, *tmp;

	if (!json_object_object_get_ex(in, "cost_model", &cost)) {
		PRINT_ERROR("missing cost_model in JSON");
		return;
	}

	/* Getting the type of the cost model */
	if (!json_object_object_get_ex(cost, "type", &tmp)) {
		PRINT_ERROR("missing type in cost_model in JSON");
		return;
	}

		if (mpc->v_Ninf_X <= 0)
			mpc_state_norm_addvar(mpc, in);
		
		
		if (!json_object_object_get_ex(cost, "coef", &tmp)) {
			PRINT_ERROR("missing coef in cost_model in JSON");
			return;
		}
		coef = json_object_get_double(tmp);

		/* Setting objective function */
		glp_set_obj_name(mpc->op, "Min weighted L_infty norm of states X(1) to X(H)");
		glp_set_obj_dir(mpc->op, GLP_MIN);
		cur = 1;
		for (j = 0; j < mpc->model->H; j++) {
			glp_set_obj_coef(mpc->op, mpc->v_Ninf_X + (int)j, cur);
			cur *= coef;
		}
}

void min_steps_to_zero(mpc_glpk* mpc, json_object * in)
{

	size_t j, i;
	double coef, cur;
	json_object  *vec_w, *elem, *cost, *tmp;
	
	 
	
	 //Get the cost model of the MPC 
	 if (!json_object_object_get_ex(in, "cost_model", &cost)) {
		PRINT_ERROR("missing cost_model in JSON");
		return;
	}

	/* Getting the type of the cost model*/ 
	if (!json_object_object_get_ex(cost, "type", &tmp)) {
		PRINT_ERROR("missing type in cost_model in JSON");
		return;
	}


	if (mpc->v_absU <= 0)
		mpc_input_norm_addvar(mpc);

	/* Cost of the state as in "min_steps_to_zero" */
	if (!json_object_object_get_ex(cost, "coef", &tmp)) {
		PRINT_ERROR("missing coef in cost_model in JSON");
		return;
	}
	coef = json_object_get_double(tmp);

	/* Setting objective function */
	glp_set_obj_name(mpc->op, "Min weighted L_infty norm of states X(1) to X(H) and L_1 norm of inputs");
	glp_set_obj_dir(mpc->op, GLP_MIN);
	cur = 1;
	for (j = 0; j < mpc->model->H; j++) {
		glp_set_obj_coef(mpc->op, mpc->v_Ninf_X+(int)j, cur);
		cur *= coef; /* increasing cost for future states */
	}

	/* Get the weight of each input */
	if (!json_object_object_get_ex(cost, "input_weight", &vec_w)) {
		PRINT_ERROR("missing input_weight in JSON");
		return;
	}
	if ((size_t)json_object_array_length(vec_w) != mpc->model->m) {
		PRINT_ERROR("wrong size of input_weight in JSON");
		return;
	}
	
	/* looping over the components */
	for (j=0; j < mpc->model->m; j++) {
		elem = json_object_array_get_idx(vec_w, (int)j);
		errno = 0;
		cur = json_object_get_double(elem);
		if (errno) {
			fprintf(stderr, "Error at index %i\n", (int)j);
			PRINT_ERROR("issues in converting element of state weight");
			return;
		}
		/* looping over all input vars */
		for (i=0; i < mpc->h_ctrl+1; i++) {
			glp_set_obj_coef(mpc->op, mpc->v_absU + (int)((mpc->model->m) * i + j), cur);
		}
	}
}

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
void mpc_goal_set(mpc_glpk * mpc, json_object * in)
{   

	 json_object * cost,*tmp;
	 const char * type_str;
	
	 //Get the cost model of the MPC 
	 if (!json_object_object_get_ex(in, "cost_model", &cost)) {
		PRINT_ERROR("missing cost_model in JSON");
		return;
	}

	/* Getting the type of the cost model*/ 
	if (!json_object_object_get_ex(cost, "type", &tmp)) {
		PRINT_ERROR("missing type in cost_model in JSON");
		return;
	} 

	/* Getting the string and then customizing the code accordingly */
	type_str = json_object_get_string(tmp);

	/* Cost is "min_steps_to_zero" */
	if (strcmp(type_str, "min_steps_to_zero") == 0) {
		min_state_input_norms(mpc,in);
		return;
	}
	
	/* Cost is "min_state_input_norms" */
	if (strcmp(type_str, "min_state_input_norms") == 0) {		
	    min_steps_to_zero(mpc, in);
		return;
	}
	
	fprintf(stderr, "cost type \"%s\"\n", type_str);
	PRINT_ERROR("cost type in JSON not implemented");
	return;
}

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
void mpc_warmup(mpc_glpk * mpc)
{
	mpc->x0 = gsl_vector_calloc(mpc->model->n);
	mpc_update_x0(mpc);
	glp_simplex(mpc->op, mpc->param);
}


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
void mpc_state_obstacle_add(mpc_glpk * mpc, double *center, double *size)
{
	size_t i, j, k, constr_num, num_vars=0;
	char s[100];
	int * ind, id_norm, v_B, id, len;
	double * val, rhs;

	/* add two constraints (R, L) for any non-zero size */
	for (i=0, constr_num = 0; i < mpc->model->n; i++) {
		if (fabs(size[i]) > DOUBLE_SMALL)
			constr_num += 2;
	}
	if (constr_num == 0) { /* No need of constraints */
		mpc->id_obstacle = 0;
		mpc->v_B = 0;
		return;
	}

	/* 
	 * num_vars is the max number of variables with non-zero
	 * coefficient in any constraint
	 */
	num_vars = GSL_MAX(num_vars, mpc->model->m * mpc->h_ctrl + 1);
	num_vars = GSL_MAX(num_vars, constr_num);
	/* Allocating for num_vars+1 because GLPK counts indices in array from 1 */
	ind = calloc(num_vars + 1, sizeof(int));
	val = calloc(num_vars + 1, sizeof(double));

	/*
	 * Getting coefficients from the norm constraints and set the
	 * constraints to be OR_ed
	 */
	id_norm = mpc->id_norm;
	/* Looping over all state variables from X(1) to X(H) */
	for (i = 1; i <= mpc->model->H; i++) {
		
	  	/* Creating constr_num binary variables and constraint */
		if (i == 1) {
			/* first time */
			mpc->v_B = v_B = glp_add_cols(mpc->op, (int)constr_num);
			mpc->id_obstacle = glp_add_rows(mpc->op, 1);
		} else {
			glp_add_cols(mpc->op, (int)constr_num);
			glp_add_rows(mpc->op, 1);
		}

		/* 
		 * At least one binary variable must be zero <=> at least one
		 * constraint must be true
		 */
		sprintf(s,"Ob(%02d)_one_true", (int)i);
		glp_set_row_name(mpc->op, mpc->id_obstacle+(int)((i-1)*(constr_num+1)), s);
		for (k = 0; k < constr_num; k++) {
			ind[k + 1] = mpc->v_B + (int)(constr_num * (i - 1) + k);
			val[k + 1] = 1.0;
		}
		glp_set_mat_row(mpc->op, mpc->id_obstacle+(int)((i-1)*(constr_num+1)),
				(int)constr_num, ind, val);
		glp_set_row_bnds(mpc->op, mpc->id_obstacle+(int)((i-1)*(constr_num+1)),
				GLP_UP, DONTCARE, (double)constr_num - 1 + DOUBLE_SMALL);
			
		/* Loop over components of X(i) */
		for (k = 0; k < mpc->model->n; k++) {
			if (fabs(size[k]) <= DOUBLE_SMALL) { /* no obstacle along this dimension */
				id_norm += 2; /* skip next 2 */
				continue;
			}
			
			/* Get the coefs of the corresponding norm constraint */
			len = glp_get_mat_row(mpc->op, id_norm, ind, val);
			rhs = glp_get_row_ub(mpc->op, id_norm);
			id_norm += 2;
			/* searching for id of v_Ninf_X+i in ind */
			for (j=1; j<=(size_t)len; j++)
				if (ind[j] == mpc->v_Ninf_X+(int)i-1)
					break;

			/* Setting constraint of "upper" boundary: X_k(i) >= center[k]+size[k] */
			id = glp_add_rows(mpc->op, 1);
			sprintf(s,"Ob(%02d) X%i_UP", (int)i, (int)k);
			glp_set_row_name(mpc->op, id, s);

			/* Setting coef and name of "upper" binary var */
			glp_set_col_kind(mpc->op, v_B, GLP_BV);
			sprintf(s,"B%i_UP(%02d)", (int)k, (int)i);
			glp_set_col_name(mpc->op, v_B, s);

			/* setting the BIG_M coefficient to bin var */
			ind[j] = v_B++;
			val[j] = BIG_M;
			rhs += center[k]+size[k];
			glp_set_mat_row(mpc->op, id, len, ind, val);
			glp_set_row_bnds(mpc->op, id, GLP_LO, rhs, DONTCARE);

			/* Setting "lower" boundary: X_k(i) <= center[k]-size[k]  */
			id = glp_add_rows(mpc->op, 1);
			sprintf(s,"Ob(%02d) X%i_LO", (int)i, (int)k);
			glp_set_row_name(mpc->op, id, s);

			/* Setting coef and name of "lower" binary var */
			glp_set_col_kind(mpc->op, v_B, GLP_BV);
			sprintf(s,"B%i_LO(%02d)", (int)k, (int)i);
			glp_set_col_name(mpc->op, v_B, s);

			/* setting the -BIG_M coefficient to bin var */
			ind[j] = v_B++;
			val[j] = -BIG_M;
			rhs -= 2*size[k];
			glp_set_mat_row(mpc->op, id, len, ind, val);
			glp_set_row_bnds(mpc->op, id, GLP_UP, DONTCARE, rhs);
		} /* k: loop over components of X(i) */
	} /* i: loop over X(i) */

	free(ind);
	free(val);
}


#if 0
/* 
 * 20200326, EB: below old code for minimizing the norm of the last
 * state. Not using it anymore because it leads to instability. It may
 * be useful one day. Maybe. Or maybe not.
 */

/*
 * Set the  goal of  the optimization as  minimizing some  norm (whose
 * weights are set in w) of the final state, assuming the plant starts
 * at the initial state x_0.
 */
void mpc_goal_min_final(mpc_glpk * mpc) {
	gsl_matrix * gsl_Mul_Up;
	size_t i, j, n, m, H;
	char s[200];
	int *ind, id_up, id_lo;
	double *val, *val_up, *val_lo;
	gsl_vector *gsl_val;
	
	/* Just to make code more compact/readable */
	n = mpc->model->n;
	m = mpc->model->m;
	H = mpc->model->H;

	/* 
	 * Since the last input is constant, we need to sum several
	 * Ad^k*Bd in mpc->model->ABd[]
	 */
	gsl_Mul_Up = gsl_matrix_calloc(n, m);
	for (i=0; i < H-mpc->h_ctrl; i++) {
		gsl_matrix_add(gsl_Mul_Up, mpc->model->ABd[i]);
	}
#ifdef PRINT_MAT
	printf("\nMul_Up\n");
	gsl_pretty_fprintf(stdout, gsl_Mul_Up, "%9.5f");
#endif

	/* 
	 * Adding the constraint of L-\infty weighted norm of x_H to
	 * be leq than an additional variable z. Later, we set the
	 * constraint as minimizing z
	 */
	/* Add variable z. Goal will be se to minimize z */
	mpc->v_Ninf_X = glp_add_cols(mpc->op, 1);
	sprintf(s,"Z");
	glp_set_col_name(mpc->op, mpc->v_Ninf_X, s);
	/* Z should always be >= 0. Not enforcing it explicitly for debugging */
	glp_set_col_bnds(mpc->op, mpc->v_Ninf_X, GLP_FR, DONTCARE, DONTCARE);
	/* Indices and value arrays to store the coefficients */
	ind = malloc((2+m*(mpc->h_ctrl+1))*sizeof(*ind));
	val_up = malloc((2+m*(mpc->h_ctrl+1))*sizeof(*val));
	val_lo = malloc((2+m*(mpc->h_ctrl+1))*sizeof(*val));
	gsl_val = gsl_vector_calloc(m);
	/* index and coefficient of Z */
	ind[m*(mpc->h_ctrl+1)+1] = mpc->v_Ninf_X;
	val_up[m*(mpc->h_ctrl+1)+1] = -1;
	val_lo[m*(mpc->h_ctrl+1)+1] = -1;
	/* indices of U */
	for (j=1; j<=m*(mpc->h_ctrl+1); j++) {
		ind[j] = (int)j;
	}
	/* Loop over components of X_H */
	for (i=0; i<n; i++) {
		if (gsl_vector_get(mpc->w,i) == 0) {
			/* no weight to this component of final state */
			continue;
		}
		/* Loop over control inputs */
		for (j=0; j <= mpc->h_ctrl; j++) {
			if (j < mpc->h_ctrl) {
				gsl_matrix_get_row(gsl_val, mpc->model->ABd[H-1-j], i);
			} else {
				/* special weight for the last constant value Up */
				gsl_matrix_get_row(gsl_val, gsl_Mul_Up, i);
			}
			gsl_vector_scale(gsl_val, gsl_vector_get(mpc->w,i));
			/* set coefs of U for upper bound */
			bcopy(gsl_val->data, val_up+m*j+1, m*sizeof(*val_up));
			/* set coefs of U for lowere bound  */
			gsl_vector_scale(gsl_val, -1);
			bcopy(gsl_val->data, val_lo+m*j+1, m*sizeof(*val_lo));
		}
		/* setting up upper bound on X_H */
		id_up = glp_add_rows(mpc->op, 1);
		if (!i) { /* i==0 */
			mpc->id_bound_final = id_up;
		}
		sprintf(s,"X%i(H)_UP", (int)i);
		glp_set_row_name(mpc->op, id_up, s);
		glp_set_mat_row(mpc->op, id_up, (int)(m*(mpc->h_ctrl+1)+1), ind, val_up);
		/* setting up lower bound on X_H */
		id_lo = glp_add_rows(mpc->op, 1);
		sprintf(s,"X%i(H)_LO", (int)i);
		glp_set_row_name(mpc->op, id_lo, s);
		/* set RHS of inequality */
		glp_set_mat_row(mpc->op, id_lo, (int)(m*(mpc->h_ctrl+1)+1), ind, val_lo);
	}
	free(ind);
	free(val_up);
	free(val_lo);
	/* set RHS of inequality */
	mpc_update_x0(mpc);
		
	/* Setting objective function */
	glp_set_obj_name(mpc->op, "Min L_infty norm of final state");
	glp_set_obj_dir(mpc->op, GLP_MIN);
	glp_set_obj_coef(mpc->op, mpc->v_Ninf_X, 1);
}

#endif

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
mpc_status * mpc_status_alloc(const mpc_glpk * mpc)
{
	mpc_status * tmp;
	int rows, cols;
	size_t n, m;
	
	tmp = malloc(sizeof(*tmp));
	n = mpc->model->n;
	m = mpc->model->m;
	rows = glp_get_num_rows(mpc->op);
	cols = glp_get_num_cols(mpc->op);
	tmp->size = n * sizeof(tmp->state[0]) + m * sizeof(tmp->input[0])
		/* "+1" because indices in GLPK arrays starts from 1 */
		+ ((size_t)rows + 1) * sizeof(tmp->row_stat[0])
		+ ((size_t)cols + 1) * sizeof(tmp->col_stat[0])
		+ sizeof(tmp->steps_bdg[0]) + sizeof(tmp->time_bdg[0])
		+ sizeof(tmp->prim_stat) + sizeof(tmp->dual_stat);
	tmp->block = malloc(tmp->size);
	bzero(tmp->block, tmp->size);
	tmp->state = (double *)tmp->block;
	tmp->input = (double *)(tmp->state+n);
	tmp->time_bdg = (double *)(tmp->input+m);
	tmp->steps_bdg = (int *)(tmp->time_bdg+1);
	tmp->prim_stat = (int *)(tmp->steps_bdg+1);
	tmp->dual_stat = (int *)(tmp->prim_stat+1);
	tmp->row_stat = (uint32_t *)(tmp->dual_stat+1);
	tmp->col_stat = (uint32_t *)(tmp->row_stat+rows+1);

	return tmp;
}

/**
 * @brief free mpc_status
 * 
 * @param sol_st mpc_status* Solution state
 */
void mpc_status_free(mpc_status * sol_st)
{
	free(sol_st->block);
	free(sol_st);
}

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
void mpc_status_set_x0(mpc_glpk * mpc, const mpc_status * sol_st)
{
	/* update initial state */
	memcpy(mpc->x0->data, sol_st->state, sizeof(*sol_st->state)*mpc->model->n);
	mpc_update_x0(mpc);
}

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
void mpc_status_resume(mpc_glpk * mpc, const mpc_status * sol_st)
{
	int i;

	/* Setting the steps/time budgets */
	mpc->param->it_lim  = *sol_st->steps_bdg;
	mpc->param->tm_lim  = (int)(*sol_st->time_bdg*1e-3); /* sec to msec */

	/* update initial state */
	mpc_status_set_x0(mpc, sol_st);
	/*
	memcpy(mpc->x0->data, sol_st->state,
	       sizeof(*sol_st->state)*mpc->model->n);
	mpc_update_x0(mpc);
	*/

	/* Storing basic/non-basic status of rows */
	for (i = 1; i <= glp_get_num_rows(mpc->op); i++) {
		glp_set_row_stat(mpc->op, i, (int)sol_st->row_stat[i]);
	}

	/* Storing basic/non-basic status of cols */
	for (i = 1; i <= glp_get_num_cols(mpc->op); i++) {
		glp_set_col_stat(mpc->op, i, (int)sol_st->col_stat[i]);
	}
}

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
void mpc_status_save(const mpc_glpk * mpc, mpc_status * sol_st)
{
	int i;

	/* Getting the solution. FIXME: need an efficient way */
	for (i = 0; i < (int)mpc->model->m; i++) {
		sol_st->input[i] = glp_get_col_prim(mpc->op, mpc->v_U+i);
	}
	
	/* Storing the optimality of the solution */
	*sol_st->prim_stat = glp_get_prim_stat(mpc->op);
	*sol_st->dual_stat = glp_get_dual_stat(mpc->op);
	
	/* Saving basic/non-basic status of rows */
	for (i = 1; i <= glp_get_num_rows(mpc->op); i++) {
		sol_st->row_stat[i] = (uint32_t)glp_get_row_stat(mpc->op, i);
	}

	/* Saving basic/non-basic status of cols */
	for (i = 1; i <= glp_get_num_cols(mpc->op); i++) {
		sol_st->col_stat[i] = (uint32_t)glp_get_col_stat(mpc->op, i);
	}
}

/**
 * @brief Print the solver status (mostly for debugging purpose)
 * 
 * @param f 	 FILE*				File stream where to print
 * @param mpc    const mpc_glpk*	The representation of the MPC problem
 * @param sol_st const mpc_status*	Solution state
 */
void mpc_status_fprintf(FILE *f,
			const mpc_glpk * mpc, const mpc_status * sol_st)
{
	size_t i;
	fprintf(f, "State\n");
	for (i = 0; i < mpc->model->n; i++)
		fprintf(f, "%f\t", sol_st->state[i]);

	fprintf(f, "\nInput\n");
	for (i = 0; i < mpc->model->m; i++)
		fprintf(f, "%f\t", sol_st->input[i]);
	#if 0  /* Let's omit the basic status for a while */
		fprintf(f, "\nRow status\n");
		for (i = 1; i <= (size_t)glp_get_num_rows(mpc->op); i++) {
			fprintf(f, "%d\t", sol_st->row_stat[i]);
		}
		fprintf(f, "\nColumns status\n");
		for (i = 1; i <= (size_t)glp_get_num_cols(mpc->op); i++) {
			fprintf(f, "%d\t", sol_st->col_stat[i]);
		}
	#endif
	
	fprintf(f, "\nSteps: %d\n", *sol_st->steps_bdg);
	fprintf(f, "Time: %f\n", *sol_st->time_bdg);
	fprintf(f, "Primal status: %d\n", *sol_st->prim_stat);
	fprintf(f, "Dual status: %d\n\n", *sol_st->dual_stat);
}
