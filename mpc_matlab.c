/*
 * The calling syntax is:
 *
 *	input = mpc_matlab(state);
 *	[input, time] = mpc_matlab(state);
 *
 * To compile, invoke just
 *
 *    make matlab
 */
#include <sys/types.h>
#include <sys/shm.h>
#include <time.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include "mpc_interface.h"
#include "mex.h"

#define STRING_ERROR(x) "%s:%d errno=%i, %s\n",	\
		__FILE__, __LINE__, errno, (x)

/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{
	size_t state_num, input_num, n, m;
	double *state_rd, *input_wr, time;
	int shm_id, offloaded;
	struct shared_data * data;
	char error_string[1024];
	double * shared_state;
	double * shared_input;
	
	/* check for proper number of arguments */
	if(nrhs!=1) {
		mexErrMsgIdAndTxt("MyToolbox:mpc_matlab:nrhs",
				  "The state is a necessary argument.");
	}
	if(nlhs < 1) {
		mexErrMsgIdAndTxt("MyToolbox:mpc_matlab:nlhs",
				  "At least one output required.");
	}
	
	/* Make sure the state argument is type double */
	if(!mxIsDouble(prhs[0]) || mxIsComplex(prhs[0])) {
		mexErrMsgIdAndTxt("MyToolbox:mpc_matlab:notDouble",
				  "State must be type double.");
	}
    
	/* Check that at least one among rows/cols must be 1 */
	n = mxGetN(prhs[0]);
	m = mxGetM(prhs[0]);
	if(n!=1 && m!=1) {
		snprintf(error_string, sizeof(error_string),
			 "wrong input size: getN=%i, getM=%i",
			 (int)n, (int)m);
		mexErrMsgIdAndTxt("MyToolbox:mpc_matlab:notVector",
				  error_string);
	}

	/* 
	 * Now we have all data. We can open the shared memory. 
	 * Must be created earlier (by mpc_ctrl.c)
	 */
	shm_id = shmget(MPC_SHM_KEY, 0, 0);
	if (shm_id == -1) {
		mexErrMsgIdAndTxt("MyToolbox:mpc_matlab:shmget",
				  "Unable to open shared memory");
	}
	data = (struct shared_data *)shmat(shm_id, NULL, 0);
	shared_state = (double*)(data+1); /* starts just after *data */
	shared_input = shared_state+data->state_num;
	
	/* State must have the same dimension as in mpc_interface.h */
	if(n == 1) {
		state_num = m;
	} else {
		/* m must be 1 */
		state_num = n;
	}
	if (state_num != data->state_num) {
		mexErrMsgIdAndTxt("MyToolbox:mpc_matlab:wrongSize",
				  "Wrong size of the state.");
		shmdt(data);
	}

	/* Create a pointer to the real data in the state vector  */
#if MX_HAS_INTERLEAVED_COMPLEX
	state_rd = mxGetDoubles(prhs[0]);
#else
	state_rd = mxGetPr(prhs[0]);
#endif
	memcpy(shared_state, state_rd, sizeof(state_rd[0])*state_num);

	/* Now asking MPC to compute the optimal input for us... */
	sem_post(data->sems+MPC_SEM_STATE_WRITTEN);
	/* ... and waiting to get a response */
	sem_wait(data->sems+MPC_SEM_INPUT_WRITTEN);

	/* Create a Matlab vector to pass the MPC input */
	plhs[0] = mxCreateDoubleMatrix(1, (mwSize)data->input_num, mxREAL);
	
	/* Get a pointer to the real data of the Matlab input vector  */
#if MX_HAS_INTERLEAVED_COMPLEX
	input_wr = mxGetDoubles(plhs[0]);
#else
	input_wr = mxGetPr(plhs[0]);
#endif
	memcpy(input_wr, shared_input, sizeof(input_wr[0])*data->input_num);
	if(nlhs >= 2) {
		time = data->stats_dbl[MPC_STATS_DBL_TIME];
		plhs[1] = mxCreateDoubleScalar(time);
	}
	if(nlhs >= 3) {
		offloaded = data->stats_int[MPC_STATS_INT_OFFLOAD];
		plhs[2] = mxCreateDoubleScalar(offloaded);
	}
	
	/* Finally detaching shared memory */
	shmdt(data);
}
