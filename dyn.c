/**
 * @file dyn.c
 * @author Enrico	Bini
 * @brief impementation of dyn.h
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_sf_exp.h>
#include <json-c/json.h>
#include "dyn.h"
#include "mpc.h"

/* Put this macro where debugging is needed */
#define PRINT_ERROR(x) {fprintf(stderr, "%s:%d errno=%i, %s\n",	\
				__FILE__, __LINE__, errno, (x));}

/* Uncomment PRINT_MAT to print the matrices */
/*#define PRINT_MAT */
#define NO_FREE

void dyn_init_witheig(dyn_plant * p, const size_t n, const size_t m, const double *D, const double *V, const double *B) {
	size_t i;
	
	p->has_eig = 1; /* init with eigenvectors */
	/* Storing sizes */
	p->n = n;
	p->m = m;

	/* Copying the eigenvalues from D to p->eigD*/
	p->A_eigD = malloc(p->n*sizeof(*(p->A_eigD)));
	bcopy(D, p->A_eigD, p->n*sizeof(*(p->A_eigD)));
		
	/* Eigenvectors of A */
	p->A_eigV = gsl_matrix_calloc(p->n, p->n);
	bcopy(V, p->A_eigV->data, p->n*p->n*sizeof(*V));

	/* Init continuous-time A */
	p->A = gsl_matrix_calloc(p->n, p->n);
	/* computing A from D and V */
	for (i = 0; i<p->n; i++) {
		gsl_matrix_set(p->A, i, i, D[i]);
	}
	/* TODO: the code below works only if V is upper
	 * triangular with ones along the diagonal!!! */
	/* Left- and right-multiply by the eigenvectors to get A */
	gsl_blas_dtrmm(CblasLeft, CblasUpper, CblasNoTrans, CblasUnit, 1,
		       p->A_eigV, p->A);
	gsl_blas_dtrsm(CblasRight, CblasUpper, CblasNoTrans, CblasUnit, 1,
		       p->A_eigV, p->A);

	/* Init+storing continuous-time B */
	p->B  = gsl_matrix_calloc(p->n, p->m);
	bcopy(B, p->B->data, p->n*p->m*sizeof(*B));
}


void dyn_discretize(dyn_plant * p, double tau, size_t H) {
	double aux;
	gsl_matrix *tmp_M;
	size_t i,j;
	
	if (p->has_eig == 0) {
		fprintf(stderr, "Discretization not implemented if eigenvectors aren't specified\n");
		exit(1);
	}

	p->tau = tau;   /* sampling interval */
	p->H = H;   /* num of intervals over which dynamics is urolled */

	/* 
         * Discretizing A by tau: Ad = e^(A*tau) 
	 * Also storing powers of Ad in p->Ad[j]
         */
	p->Ad = malloc(p->H*sizeof(*(p->Ad)));
	for(j = 0; j < p->H; j++) {
		p->Ad[j] = gsl_matrix_calloc(p->n, p->n);
		for (i = 0; i < p->n; i++) {
			aux = j==0 ? gsl_sf_exp(p->tau*p->A_eigD[i]) :
				gsl_matrix_get(p->Ad[0], i, i)*gsl_matrix_get(p->Ad[j-1], i, i);
			gsl_matrix_set(p->Ad[j], i, i, aux);
		}
        }
	/* Left- and right-multiply by the eigenvectors to get Ad */
	for(j = 0; j < p->H; j++) {
		gsl_blas_dtrmm(CblasLeft, CblasUpper, CblasNoTrans, CblasUnit, 1,
			       p->A_eigV, p->Ad[j]);
		gsl_blas_dtrsm(CblasRight, CblasUpper, CblasNoTrans, CblasUnit, 1,
			       p->A_eigV, p->Ad[j]);
#ifdef PRINT_MAT
		printf("\nAd[%i]\n", (int)j);
		gsl_pretty_fprintf(stdout, p->Ad[j], "%20.15f");
#endif
	}
	
	/* Discretizing B by tau */
	/* 
	 * TODO: the  Bd computed below  differs by a factor  of ~1e-4
	 * from  the  matrix computed  by  Matlab.  It remains  to  be
	 * investigated why this happens. Probably the "critical" case
	 * is when  the eigenvalue  is close to  zero as  this induces
	 * numerical error (EB:  I may tend to think the  GSL are more
	 * reliable than Matlab)
	 */

	/* 
	 * ABd[0] is the discretized B:
	 *   Bd = \int_0^tau exp(A*(tau-x)) dx*B.  
	 *
	 * Storing (powers of Ad) * Bd: ABd[0]=Bd, ABd[1]=Ad*Bd,
	 * ABd[2]=Ad*Ad*Bd,..., ABd[N-1]=Ad^{N-1}*Bd
	 */
	p->ABd = malloc(p->H*sizeof(*(p->ABd)));
	p->ABd[0] = gsl_matrix_calloc(p->n, p->m);
	tmp_M = gsl_matrix_calloc(p->n, p->n);
	for (i = 0; i < p->n; i++) {
		if (fabs(p->A_eigD[i]) < 1e-6) {
			/* 
			 * special case: if eigenvalue is close to
			 * zero then integrating the constant 1 over
			 * the interval [0,tau]
			 */
			aux = p->tau;
		} else {
			/* value of the integral is (exp(...)-1)/lambda */
			aux = gsl_sf_expm1(p->tau*p->A_eigD[i])/p->A_eigD[i];
		}
		gsl_matrix_set(tmp_M, i, i, aux);
	}
	/* Left- and right-multiply by the eigenvectors ... */
	gsl_blas_dtrmm(CblasLeft, CblasUpper, CblasNoTrans, CblasUnit, 1,
		       p->A_eigV, tmp_M);
	gsl_blas_dtrsm(CblasRight, CblasUpper, CblasNoTrans, CblasUnit, 1,
		       p->A_eigV, tmp_M);
	/* ... and then multiply by B to get Bd */
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, tmp_M, p->B, 1, p->ABd[0]);
	gsl_matrix_free(tmp_M);
#ifdef PRINT_MAT
	printf("\nBd\n");
	gsl_pretty_fprintf(stdout, p->ABd[0], "%20.15f");
#endif

	/* Computing an array of the Ad^{...}*Bd */
	for (i=1; i < p->H; i++) {
		p->ABd[i] = gsl_matrix_calloc(p->n, p->m);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, p->Ad[i-1], p->ABd[0], 0, p->ABd[i]);
#ifdef PRINT_MAT
		printf("\nAd^%i*Bd\n", (int)i);
		gsl_pretty_fprintf(stdout, p->ABd[i], "%20.15f");
#endif
	}
}

/*
 * Functions to allocate/compute/store the powers of Ad and ABd. It
 * assumes that needed data is properly stored in p. Not exported in
 * the API
 */
static void dyn_init_power_AB(dyn_plant * p)
{
	size_t i;

	for(i = 1; i < p->H; i++) {
		/* Powers of Ad */
 		p->Ad[i] = gsl_matrix_calloc(p->n, p->n);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, p->Ad[i-1], p->Ad[0], 0, p->Ad[i]);

		/* Powers of Ad times Bd */
 		p->ABd[i] = gsl_matrix_calloc(p->n, p->m);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1, p->Ad[i-1], p->ABd[0], 0, p->ABd[i]);
#ifdef PRINT_MAT
		printf("\nAd[%i]\n", (int)i);
		gsl_pretty_fprintf(stdout, p->Ad[i], "%20.15f");
		printf("\nAd^%i*Bd\n", (int)i);
		gsl_pretty_fprintf(stdout, p->ABd[i], "%20.15f");
#endif
	}
}

/*
 * Initialize a discrete-time system by reading from the JSON
 * struct. The continuous part is set to null
 */
void dyn_init_discrete(dyn_plant * p, json_object * in)
{
	size_t i;
	json_object *tmp, *elem;

	/* Ignore eigensystems */
	p->has_eig = 0; 
	p->A_eigD = NULL;
	p->A_eigV = NULL;

	/* Ignore continuous-time matrices */
	p->A = NULL;
	p->B = NULL;
	p->tau = 0.f/0.f;   /* sampling interval should be ignored  */

	/* Get the number of system states */
	if (!json_object_object_get_ex(in, "state_num", &tmp)) {
		PRINT_ERROR("missing state_num in JSON");
		return;
	}
	p->n = (size_t)json_object_get_int(tmp);

	/* Get the number of system inputs */
	if (!json_object_object_get_ex(in, "input_num", &tmp)) {
		PRINT_ERROR("missing input_num in JSON");
		return;
	}
	p->m = (size_t)json_object_get_int(tmp);

	/* Get the horizon (used also in MPC) */
	if (!json_object_object_get_ex(in, "len_horizon", &tmp)) {
		PRINT_ERROR("missing len_horizon in JSON");
		return;
	}
	p->H = (size_t)json_object_get_int(tmp);

	/* Get the discrete matrix Ad */
	if (!json_object_object_get_ex(in, "state_Ad", &tmp)) {
		PRINT_ERROR("missing state_Ad in JSON");
		return;
	}
	if ((size_t)json_object_array_length(tmp) != (p->n)*(p->n)) {
		PRINT_ERROR("wrong size of state_Ad in JSON");
		return;
	}
	p->Ad = malloc(p->H*sizeof(*(p->Ad)));
	p->Ad[0] = gsl_matrix_calloc(p->n, p->n);
	for (i=0; i<(p->n)*(p->n); i++) {
		elem = json_object_array_get_idx(tmp, (int)i);
		errno = 0;
		p->Ad[0]->data[i] = json_object_get_double(elem);
		if (errno) {
			fprintf(stderr, "%i\n", (int)i);
			PRINT_ERROR("issues in converting element in Ad");
			return;
		}
	}

	/* Get the discrete matrix Bd */
	if (!json_object_object_get_ex(in, "input_Bd", &tmp)) {
		PRINT_ERROR("missing input_Bd in JSON");
		return;
	}
	if ((size_t)json_object_array_length(tmp) != (p->m)*(p->n)) {
		PRINT_ERROR("wrong size of Bd in JSON");
		return;
	}
	p->ABd = malloc(p->H*sizeof(*(p->ABd)));
	p->ABd[0] = gsl_matrix_calloc(p->n, p->m);
	for (i=0; i<(p->n)*(p->m); i++) {
		elem = json_object_array_get_idx(tmp, (int)i);
		errno = 0;
		p->ABd[0]->data[i] = json_object_get_double(elem);
		if (errno) {
			fprintf(stderr, "%i\n", (int)i);
			PRINT_ERROR("issues in converting element in Bd");
			return;
		}
	}

	/* Storing powers of Ad and ABd */
	dyn_init_power_AB(p);
}


void dyn_state_dynamics(const dyn_plant * p, const gsl_vector * x_0, const gsl_matrix * u, gsl_matrix * x_full) {
	size_t i, id_u=0;
	gsl_vector *x_cur, *x_next, *x_aux, *u_cur;

	x_cur = gsl_vector_calloc(p->n);
	x_next = gsl_vector_calloc(p->n);
	u_cur = gsl_vector_calloc(p->m);	
	gsl_vector_memcpy(x_cur, x_0);
	for(i = 0; i < p->H; gsl_matrix_set_col(x_full, i, x_cur), i++) {
		gsl_blas_dgemv(CblasNoTrans, 1, p->Ad[0], x_cur , 0, x_next); //!< used to do product between a matrix and a vector
		x_aux = x_next;
		x_next = x_cur;
		x_cur = x_aux;
		if (u == NULL) {
			continue;
		}
		id_u = i < u->size2 ? i : u->size2-1;
		gsl_matrix_get_col(u_cur, u, id_u);
		gsl_blas_dgemv(CblasNoTrans, 1, p->ABd[0], u_cur , 1, x_cur);
	}
	gsl_vector_free(x_cur);
	gsl_vector_free(x_next);
	gsl_vector_free(u_cur);
}

void dyn_free(dyn_plant * p) {
	size_t i;

	gsl_matrix_free(p->A);
	gsl_matrix_free(p->A_eigV);
	free(p->A_eigD);
	gsl_matrix_free(p->B);
	for (i=0; i < p->H; i++) {
		gsl_matrix_free(p->Ad[i]);
		gsl_matrix_free(p->ABd[i]);
	}
	free(p->Ad);
	free(p->ABd);
}

void gsl_matrix_pretty(FILE *f, const gsl_matrix *m, const char *fmt) {
	size_t i, j;
	double * val;
	char s[100] = " ";

	strncat(s, fmt, sizeof(s));
	val = m->data;
	for (i=0; i < m->size1; i++) {
		for (j=0; j < m->size2; val++, j++) {
			fprintf(f, s, *val);
		}
		fprintf(f, "\n");
	}
}

void gsl_vector_pretty(FILE *f, const gsl_vector *m, const char *fmt) {
	size_t j;
	double * val;
	char s[100] = " ";

	strncat(s, fmt, sizeof(s));
	val = m->data;
	for (j=0; j < m->size; val++, j++) {
		fprintf(f, s, *val);
	}
	fprintf(f, "\n");
}

void gsl_vector_int_pretty(FILE *f, const gsl_vector_int *m, const char *fmt) {
	size_t j;
	int * val;
	char s[100] = " ";

	strncat(s, fmt, sizeof(s));
	val = m->data;
	for (j=0; j < m->size; val++, j++) {
		fprintf(f, s, *val);
	}
	fprintf(f, "\n");
}

dyn_trace * dyn_trace_alloc(const size_t n, const size_t m, const size_t H)
{
	dyn_trace * t;

	t = calloc(1,sizeof(*t));
	t->x = gsl_matrix_calloc(n, H+1);
	t->u = gsl_matrix_calloc(m, H);
	t->time = gsl_vector_calloc(H);
	t->n = n;
	t->m = m;
	t->H = H;
	return t;
}

void dyn_trace_free(dyn_trace * t)
{
	gsl_matrix_free(t->x);
	gsl_matrix_free(t->u);
	gsl_vector_free(t->time);
#ifdef NO_FREE

#else
	gsl_vector_long_free(t->steps);
#endif
	free(t);
}


void dyn_plant_dynamics(const dyn_plant * p, const gsl_vector * x_0, dyn_trace * t,
						void (*ctrl_law)(size_t, dyn_trace *, void *), void * param)
{
	size_t i;
	gsl_vector *x_next, *x_cur, *u_cur;
	struct timespec tic, toc;
	double time;

	x_next = gsl_vector_calloc(p->n);
	x_cur  = gsl_vector_calloc(p->n);
	u_cur  = gsl_vector_calloc(p->m);
	gsl_matrix_set_col(t->x, 0, x_0);

	for (i = 0; i < t->H; i++) {
		if (ctrl_law == NULL) {
			/* No control law specified. Assuming zero input */
			gsl_vector_set_zero(u_cur);
			gsl_matrix_set_col(t->u, i, u_cur);
			gsl_vector_set_zero(x_next);

			/* No control law, no time */
			if (t->time != NULL)
				gsl_vector_set(t->time, i, 0.0);
		} else {
			/* Getting time of simplex */
			clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
			/* Control law writes the input on i-th col of t->u */
			ctrl_law(i, t, param);
			clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
			if (t->time != NULL) {
				time =  (double)(toc.tv_sec-tic.tv_sec);
				time += (double)(toc.tv_nsec-tic.tv_nsec)*1e-9;
				*gsl_vector_ptr(t->time, i) += time;
			}
			
			gsl_matrix_get_col(u_cur, t->u, i);
			gsl_blas_dgemv(CblasNoTrans,
				       1, p->ABd[0], u_cur, 0, x_next);
		}
		gsl_matrix_get_col(x_cur, t->x, i);
		gsl_blas_dgemv(CblasNoTrans,
			       1, p->Ad[0], x_cur , 1, x_next);
		gsl_matrix_set_col(t->x, i+1, x_next);
	}
	gsl_vector_free(x_cur);
	gsl_vector_free(x_next);
	gsl_vector_free(u_cur);
}
