# Working flags on the Pelican
#CFLAGS = -I/home/asctec/usrlocal/include -pedantic -Wall -Wno-sign-conversion -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -fshort-enums -fno-common -Dinline= -O0 -g
#LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk -lpthread -L/home/asctec/usrlocal/lib -Wl,-rpath -Wl,/home/asctec/usrlocal/lib

# Working flages elsewhere
CFLAGS = -pedantic -Werror -Wall -Wno-sign-conversion -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -fshort-enums -fno-common -Dinline= -O0 -g -pg
LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk -lpthread -pg 
T =
ifndef $(T)
	T = t
endif

GPROF_FLAGS = 

.PHONY: all
all: mpc_server mpc_ctrl mpc_conf matlab

mpc_server: mpc_server.o mpc.o dyn.o
	gcc mpc_server.o mpc.o dyn.o $(LDFLAGS) -o mpc_server

mpc_ctrl: mpc_ctrl.o mpc.o dyn.o
	gcc mpc_ctrl.o mpc.o dyn.o $(LDFLAGS) -o mpc_ctrl

mpc_ctrl.o: mpc_ctrl.c
	gcc -c mpc_ctrl.c $(CFLAGS) -o mpc_ctrl.o

mpc.o: mpc.c mpc.h Makefile
	gcc -c mpc.c $(CFLAGS) -o mpc.o

dyn.o: dyn.c dyn.h Makefile
	gcc -c dyn.c $(CFLAGS) -o dyn.o

mpc_matlab.mexa64: mpc_matlab.c
	mex -O -v mpc_matlab.c

.PHONY: matlab
matlab: mpc_matlab.mexa64

.PHONY: clean
clean:
	rm -rf *.o *~ mpc mpc_server mpc_client

.PHONY: run_server
run_server:
	gnome-terminal --tab -- bash -c "cd server; sudo ../mpc_server ../test.json; exec bash -i"

.PHONY: run_ctrl
run_ctrl:
	gnome-terminal --tab -- bash -c " cd client; sudo ../mpc_ctrl ../test.json; exec bash -i"

.PHONY: run_matlab
run_matlab:
	gnome-terminal --tab -- bash -c "cd ../matlab_sim; matlab -softwareopengl; exec bash -i"

.PHONY: run_matlab_alt
run_matlab_alt:
	gnome-terminal --tab -- bash -c "cd ../matlab_sim; matlab -softwareopengl -nosplash -nodesktop -r "launch"; exec bash -i"

.PHONY: run_matlab_alt_autoclose
run_matlab_alt_autoclose:
	gnome-terminal --tab -- bash -i -c "cd ../matlab_sim; matlab -softwareopengl -nosplash -nodesktop -r "launch"; xdotool key --clearmodifiers Ctrl+Shift+W key --clearmodifiers KP_Enter"
	
.PHONY: run
run: run_server run_ctrl run_matlab

.PHONY: run_c
run_c: run_server run_ctrl


gprof_server:
	gprof  $(GPROF_FLAGS) mpc_server server/gmon.out > testCambiamentiParametriJson/$(T)/prof_server.txt

gprof_ctrl:
	gprof $(GPROF_FLAGS)  mpc_ctrl client/gmon.out > testCambiamentiParametriJson/$(T)/prof_ctrl.txt

.PHONY: all_gprof
all_gprof: gprof_server gprof_ctrl 

