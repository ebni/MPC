/**
 * @file dyn.h
 * @brief this library define dyn functions //FIXME
 * 
 */


#ifndef _DYN_H_
#define _DYN_H_
#include <json-c/json.h>
#include <gsl/gsl_matrix.h>

/*
 * Data structure for the plant
 */

typedef struct {
	gsl_matrix *A;   /* continuous time A */
	char has_eig; /* 1 if A if factorized by A = VDV^{-1}*/
	gsl_matrix *A_eigV; /* unused if has_eig == 0 */
	double *A_eigD;     /* array with eigenvalues of A */
	gsl_matrix *B; /* continuous time B */
	size_t n; /* number of states */
	size_t m; /* number of inputs */
	double tau; /* sampling time */
	size_t H; /* horizon (in number of intervals) of dynamics */
	/* 
	 * Ad[0] is the discretized dynamics: Ad=exp(A*tau).  Storing
	 * also powers of Ad: Ad[0]=Ad, Ad[1]=Ad^2, ..., Ad[H-1]=Ad^H
	 */
	gsl_matrix **Ad;
	/* 
	 * ABd[0] is the discretized B:
	 *   Bd = \int_0^tau exp(A*(tau-x)) dx*B.  
	 *
	 * Storing (powers of Ad) * Bd: ABd[0]=Bd, ABd[1]=Ad*Bd,
	 * ABd[2]=Ad*Ad*Bd, ..., ABd[H-1]=Ad^{H-1}*Bd
	 */
	gsl_matrix **ABd;
	
} dyn_plant;

/*
 * Data structure for the a state evolution in presence of given
 * inputs
 */
typedef struct {
	size_t n; /* number of states */
	size_t m; /* number of inputs */
	size_t H; /* horizon (in number of intervals) of dynamics */
	gsl_matrix *x;   /* n times H+1 array of states (from x_0 to x_H */
	gsl_matrix *u;   /* m times H array of inputs (from u_0 to u_{H-1} */
	gsl_vector *time;      /* H-long vector. time[i] seconds for u[i] */
} dyn_trace;

/*
 * Initialize the continuous-time linear time-invariant (LTI) plant p,
 * typically modeled as
 *
 *   \dot{A} = Ax + Bu
 *
 * by taking the following parameters as input:
 *
 * - n, size of the plant state x
 * - m, size of the plant input u
 * - D, n sized vector with eigenvalues of A
 * - V, n*n sized vector with the n eigen vectors of A
 * - B, m*n sized vector of B
 *
 * Notice that the matrix A is  not passed as parameter. Rather, it is
 * computed by:
 *
 * A = V*D*V^(-1)
 *
 * and then stored in p.
 */

/**
 * @brief Initialize the continuous-time linear time-invariant (LTI) plant p typically modeled as \dot{A} = Ax + Bu
 * 
 * @param p dyn_plant*    plant 
 * @param n const size_t  size of the plant state x
 * @param m const size_t  size of the plant imput u
 * @param D const double* n sized vector with eigenvalues of A
 * @param V const double* n*n sized vector with the n eigen vectors of A
 * @param B const double* m*n sized vector of B
 */
void dyn_init_witheig(dyn_plant * p, const size_t n, const size_t m, const double *D, const double *V, const double *B);

/*
 * Discretize the dynamics taking the following inputs:
 * - tau, sample interval
 * - H, horizon (number of intervals) of the explicit discretization
 *
 * Upon a successful  invocation, proper powers of matrices  Ad and Bd
 * are stored in p->Ad and p->ABd
 */

/**
 * @brief Discretize the dynamics taking the following inputs.Upon a successful  invocation, 
 * proper powers of matrices  Ad and Bd are stored in p->Ad and p->ABd
 * 
 * @param p   dyn_plant*  plant
 * @param tau double      sample interval (sampling)
 * @param H   size_t      horizon (number of intervals) of the explicit discretization
 */

void dyn_discretize(dyn_plant * p, double tau, size_t H);

/*
 * Initialize a discrete-time system by reading from the JSON
 * struct. Expected field in JSON file are: 
 *   "num_states", number of states of the system
 *   "num_inputs", number of inputs of the system
 *   "horizon", length (in discrete steps) of the time horizon
 *   "Ad", matrix A of the discrete-time dynamics
 *   "Bd", matrix B of the discrete-time dynamics
 * The continuous part is set to null.
 */

/**
 * @brief Initialize a discrete-time system by reading from the JSON struct.
 * 
 * @param p  dyn_plant*  plant
 * @param in struct json_object* a json object containing info read from file for initialization 
 * @note the JSON structure is supposed to have the following fields initialized:
 *       "num_states", number of states of the system
 *       "num_inputs", number of inputs of the system
 *       "horizon", length (in discrete steps) of the time horizon
 *       "Ad", matrix A of the discrete-time dynamics
 *       "Bd", matrix B of the discrete-time dynamics
 *        and the continuous part is set to null
 */
void dyn_init_discrete(dyn_plant * p, struct json_object * in);


/*
 * TO BE DEPRECATED SOON in favour of
 *
 * void dyn_plant_dynamics(...);
 * 
 * Computing the state evolution from the  initial state x_0 and for a
 * given input u. The dynamics is  computed for p->H steps. The states
 * from x_1 up  to x_H are stored  in a p->n rows  \times p->H columns
 * matrix x_full.  The  input is specified by the p->m  rows matrix u,
 * which specified u_0,  u_1, ...  If the input u  is NULL, a constant
 * zero input is  assumed. If the number of inputs  is less than p->H,
 * the  last  input is  assumed  to  be  held constantly.  The  inputs
 * exceeding p->H are discarded.
 */


/** 
 * @brief Computing the state evolution from the  initial state x_0 and for a given input u.
 * The dynamics is  computed for p->H steps.
 * 
 * @param p      const dyn_plant * plant 
 * @param x_0    const dyn_plant * initial state
 * @param u      const gsl_matrix * input
 * @param x_full gsl_matrix *  //TODO
 * @note to be deprecated soon
 */
void dyn_state_dynamics(const dyn_plant * p, const gsl_vector * x_0, const gsl_matrix * u, gsl_matrix * x_full);

/*
 * De-allocating all used memory
 */

/**
 * @brief De-allocating all used memory p
 * 
 * @param p dyn_plant * plant
 */
void dyn_free(dyn_plant * p);

/*
 * Allocate,  initialize and  return the  trace data  structure for  a
 * plant with  state of  size n, input  of size m,  and length  of the
 * trace equal to H. Remember, there  are going to be H+1 states, from
 * x_0 to x_H.
 */

/**
 * @brief Allocate,  initialize and  return the  trace data  structure for  a
 * plant with  state of  size n, input  of size m,  and length  of the
 * trace equal to H. Remember, there  are going to be H+1 states, from
 * x_0 to x_H
 *
 * @param n           const size_t n   size of states of dyn
 * @param m           const size_t m   size of inputs
 * @param H           const size_t     size of Horizion
 * @return dyn_trace* return the  trace data  structure for  a
 * 					  plant with  state of  size n, input  of size m,  and length  of the
 *                    trace equal to H 
 */
dyn_trace * dyn_trace_alloc(const size_t n, const size_t m, const size_t H);

void dyn_trace_free(dyn_trace * t);


/*
 * Compute  the dynamics  of the  plant p,  starting from  the initial
 * state x_0, and subject to the control law specified by the function
 * ctrl_law. If ctrl_law == NULL, then no control law is assumed (same
 * as constant zero input). The trace is then stored in t. The control
 * law  also gets  some parameters  which are  passed via  the generic
 * pointer param, which needs to be properly casted inside the control
 * law.
 *
 * The control law is assumed to be defined as
 *
 * void crtl_law(size_t k, dyn_trace * t, void *param);
 *
 * and stores the k-th input (with k = 0,...,t->H-1) as k-th column of
 * t->u.
 */

/**
 * @brief  Compute  the dynamics  of the  plant p,  starting from  the initial
 * state x_0, and subject to the control law specified by the function
 * ctrl_law. 
 * 
 * @param p        const dyn_plant * plant
 * @param x_0      const gsl_vector * initial state 
 * @param t        dyn_trace * trace
 * @param ctrl_law void (*ctrl_law)(size_t, dyn_trace *, void *) is a function pointer and also gets  some parameters  which are  passed via  the generic
 *                 pointer param, which needs to be properly casted inside the control law 
 * @param param    void * are the parameters for the  ctrl_law funtion
 * @note   If ctrl_law == NULL, then no control law is assumed (same
 * as constant zero input),
 * 
 */
void dyn_plant_dynamics(const dyn_plant * p,
			const gsl_vector * x_0,
			dyn_trace * t,
			void (*ctrl_law)(size_t, dyn_trace *, void *),
			void * param);


/*
 * Guys, the GSL function to print  matrices is just does not even put
 * numbers in  rows.... This is a  basic exercise I give  to students.
 * Getting a better printing format in just a few lines.
 */

/**
 * @brief print m in a human readable format
 * 
 * @param f   FILE *
 * @param m   const gsl_matrix*
 * @param fmt const char *
 */
void gsl_matrix_pretty(FILE *f, const gsl_matrix *m, const char *fmt);


/**
 * @brief print m in a human readable format
 * 
 * @param f   FILE *
 * @param m   const gsl_vector*
 * @param fmt const char *
 */
void gsl_vector_pretty(FILE *f, const gsl_vector *m, const char *fmt);



/**
 * @brief print f in a human readable format
 * 
 * @param f FILE*
 * @param m const gsl_vector_int *
 * @param fmt const char*
 */
void gsl_vector_int_pretty(FILE *f, const gsl_vector_int *m, const char *fmt);

#endif /* _DYN_H_ */
