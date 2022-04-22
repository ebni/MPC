/**
 * @file mpc_interface.h
 * @author Enrico	Bini
 * @brief   The interaction  between the MPC  controller and the  plant happens
 * via shared memory. It is  expected that the interaction follows the
 * steps:
 *
 * 1. the  plant writes  the state  in the state  field of  the shared
 * struct
 *
 * 2.  the  plant notifies  a  fresh  state  by sem_post(...)  on  the
 * semaphore MPC_SEM_STATE_WRITTEN
 *
 * 3.  the plant  then waits  for the  input by  sem_wait(...) on  the
 * semaphore  MPC_SEM_INPUT_WRITTEN. Also  sem_timedwait may  be used,
 * with care.
 *
 * 4. the input is available in the struct to be fed to the plant
 *
 * The shared memory is created and  removed by the MPC controller. In
 * case, the  default shared  memory key 0xC1A0  is already  used (for
 * example, because  another MPC controller  is up and  running), then
 * macro MPC_SHM_KEY must also be set  to another unused key agreed by
 * the MPC controller and the plant.
 * 
 */
#ifndef _MPC_INTERFACE_H_
#define _MPC_INTERFACE_H_

#include <semaphore.h>
#include <stdint.h>

#define MPC_SHM_KEY 0xC1A0  /* hard-coded key of shared mem SysV object */
#define MPC_SHM_FLAGS 0666  /* we go easy: everybody reads and writes */

#define MPC_CPU_ID 1        /* CPU where processes sharing memory reside */


/* Semapohore stuff */
#define MPC_SEM_NUM            3 /*2*/  /* how many semaphores used */
#define MPC_SEM_STATE_WRITTEN  0   /* +1: plant; -1 MPC controller */
#define MPC_SEM_INPUT_WRITTEN  1   /* +1: MPC controller; -1 plant */

/* Configuration flags */
#define MPC_OFFLOAD 0x01    /* if set, off-load MPC computation */

/*  if set, predictive mode is active and u0 
	vector is passed as parameter and 
	is used to predict u1 */
#define MPC_PREDICTIVE_MODE 0x02 

/* Statistics */
#define MPC_STATS_DBL_LEN  1   /* how many double statistics */
#define MPC_STATS_INT_LEN  1   /* how many int statistics */

/* Statistics IDs */
#define MPC_STATS_DBL_TIME 0      /* time taken for MPC computation */
#define MPC_STATS_INT_OFFLOAD 0   /* 1: offloaded, 0: local */
 
/*
 * MPC server configuration parameters. The server IP may be
 * overwritten by the command-line arguments of mpc_cltr.
 */
#define MPC_SOLVER_IP   "127.0.0.1"  /* default IP is localhost */
#define MPC_SOLVER_PORT 6001         /* default port is something random */

struct shared_data {
	sem_t  sems[MPC_SEM_NUM];    /* semaphores to regulate communication */
	size_t state_num;            /* number of states */
	size_t input_num;            /* number of inputs */
	double u;						 /*test value 42*/
#if MPC_STATS_INT_LEN
	int stats_int[MPC_STATS_INT_LEN];
#endif
#if MPC_STATS_DBL_LEN
	double stats_dbl[MPC_STATS_DBL_LEN];
#endif
	uint32_t flags;
	/*
	 * The shared memory then continues with two arrays of double
	 * whose size is dynamic:
	 *
	 *   double state[state_num]
	 *     used by the application to communicate the state to MPC
	 *
	 *   double input[input_num]
	 *     used by MPC to communicate the input to the application
	 */
};

/*
 * The  following  two  macros  rispectively enable  and  disable  the
 * offloading to the MPC server.  The  macro parameter is the "var" is
 * a pointer  to a struct share_data  which should be declared  by the
 * user.
 */
#define MPC_OFFLOAD_ENABLE(var)       var->flags |= MPC_OFFLOAD;
#define MPC_OFFLOAD_DISABLE(var)      var->flags &= ~((uint32_t)MPC_OFFLOAD);
#define MPC_OFFLOAD_IS_ENABLED(var)   (var->flags & MPC_OFFLOAD)
/*
 * The following three macros rispectively enable, disable, and check, 
 * the predictive mode. The  macro parameter is the "var" is
 * a pointer to a struct share_data  which should be declared  by the
 * user. 
 */
#define MPC_PREDICTIVE_MODE_ENABLE(var) var->flags |= MPC_PREDICTIVE_MODE;
#define MPC_PREDICTIVE_MODE_DISABLE(var) var->flags &= ~((uint32_t)MPC_PREDICTIVE_MODE);
#define MPC_PREDICTIVE_MODE_IS_ENABLED(var) (var->flags & MPC_PREDICTIVE_MODE)

#endif /* _MPC_INTERFACE_H_ */
