#include <sys/types.h>
#include <sys/shm.h>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <semaphore.h>
#include <errno.h>
#include "../include/mpc_interface.h"
#include <fcntl.h>
#include <unistd.h>
#include "mex.h"

#define MARKER_FILE "/sys/kernel/tracing/trace_marker"
#define BUF_LEN 400
#define STRING_ERROR(x) "%s:%d errno=%i, %s\n",	\
		__FILE__, __LINE__, errno, (x)

void print_mark(const char *msg);

void print_mark(const char *msg)
{
	char s[BUF_LEN];
	int marker_fd;
	
	marker_fd = open(MARKER_FILE, O_WRONLY);
	if (marker_fd == -1) {
		fprintf(stderr, "Error opening trace_marker file\n");
		return;
	}
	snprintf(s,BUF_LEN,"%s\n",msg);
	write(marker_fd, s, BUF_LEN);
	if(close(marker_fd)==-1)
		exit(EXIT_FAILURE);
}

/**
 * @brief this function is the gateway between matlab and the mpc library
 * 
 * @param nlhs size of plhs
 * @param plhs data to send to matlab, corresponds to array of left-side output arguments 
 * @param nrhs size of prhs
 * @param prhs data received from matlab, corresponds to array of right-side output arguments
 */
void mexFunction( int nlhs, mxArray *plhs[],int nrhs, const mxArray *prhs[]){
            
                char input[128];
                char msg[256];
                bzero(msg,256);
                strcat(msg,"MATLAB:\t");
                mxGetString(prhs[0],input,sizeof(input)/sizeof(char));
                input[(sizeof(input)/sizeof(char)-1)] = '\0';
                puts("test mpc_msg XXXD");
                printf("messaggio da mathlab=%s\n",input);
                strcat(msg,input);
                print_mark(msg);
                  }