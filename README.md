# C-based MPC controller
This library offers several programs to run Model-Predictive Control (MPC). All programs are written in C for higher efficiency. 

#### Requirements
* [GNU Scientific Library (GSL)](https://www.gnu.org/software/gsl/), which you can get by
```sudo apt-get install libgsl-dev```
Such a library is used to perform martix/vector operations
* [GNU Linear Programming Kit (GLPK)](https://www.gnu.org/software/glpk/), which you can get by
```sudo apt-get install libglpk-dev```
In future developments, the GLPK library may be replaced C++ [COIN-OR](https://github.com/coin-or) libraries, as they seem more efficient and more actively developed.

#### Compilation
If all needed libraries are installed, then all compilation should properly happen with
```make all```

#### Content
  * `mpc_ctrl.c` is the MPC controller in charge of reading the state from and writing the input to a shared memory area. This program may make all the computations or off-load part/all of it to a server
  * `mpc_server.c` launches a server which listen for client wishing to solve an instance of an MPC problem
  * `mpc_interface.h` is a C header file which includes the declarations needed to use the MPC controller (such as the shared memory). Such file **must be included** by the application wishing to use the MPC controller (ROS, Matlab or else)
  * `trace_proc.c` is a used to trace the scheduling events of some processes. In the MPC context is used to monitor the schedule of MPC execution, although its usage is not strictly bound to MPC.

#### Interface to the MPC controller
The communication between any application and the MPC controller happens through a shared memory area. Such a memory area is the concatenation of the following data structures:

* A `struct shared_data` declared in `mpc_interface.h`. The struct is declared as follows
```
struct shared_data {
	sem_t  sems[MPC_SEM_NUM];    /* semaphores to regulate communication */
	size_t state_num;            /* number of states */
	size_t input_num;            /* number of inputs */
};
```
* An array of `state_num` double floating-point variables containing the plant state. This array should be written by the application and is read by MPC.
* An array of `input_num` double floating-point variables containing the plant input. The MPC writes here the optimal solution found, which can then be read by the application.



## MPC controller (`mpc_shm_ctrl`)

This is the executable running the local MPC controller. Such a controller interacts with the system to be controlled through a shared memory. The `struct shared_data` declared in `mpc_interface.h`.


2. Compile the MPC controller using the shared memory and used by Matlab, by:
  ```
  make mpc_shm_ctrl
  ```

## Local MPC execution

1. Make sure that USE_SERVER *is not* defined in mpc_ctrl.c. Then
  ```
  make
  ```

## Compilation of a small tracing tool

1. From the home directory of the project, launch
  ```
  make trace_proc
  ```

## Running the MPC Server

1. From project root, run:
```
./launch_MPC_server
```

2. In a new terminal, form project root run:
```
./mpc/mpc_shm_ctrl ./mpc/uav12_iris.json
```
