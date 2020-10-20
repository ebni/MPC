# Working flags on the Pelican
#CFLAGS = -I/home/asctec/usrlocal/include -pedantic -Wall -Wno-sign-conversion -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -fshort-enums -fno-common -Dinline= -O0 -g
#LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk -lpthread -L/home/asctec/usrlocal/lib -Wl,-rpath -Wl,/home/asctec/usrlocal/lib

# Working flages elsewhere
CFLAGS = -pedantic -Werror -Wall -Wno-sign-conversion -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -fshort-enums -fno-common -Dinline= -O0 -g
LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk -lpthread


.PHONY: clean

manager: manager.o
	gcc manager.o $(LDFLAGS) -o manager

manager.o: manager.c mpc_interface.h app_workload.h
	gcc -c manager.c $(CFLAGS) -o manager.o

app_workload: app_workload.o
	gcc app_workload.o $(LDFLAGS) -o app_workload

app_workload.o: app_workload.c app_workload.h
	gcc -c app_workload.c $(CFLAGS) -o app_workload.o

mpc_server: mpc_server.o mpc.o dyn.o
	gcc mpc_server.o mpc.o dyn.o $(LDFLAGS) -o mpc_server

sim_plant: sim_plant.o mpc.o dyn.o
	gcc sim_plant.o mpc.o dyn.o $(LDFLAGS) -o sim_plant

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

matlab: mpc_matlab.mexa64

all: mpc_server mpc_ctrl sim_plant app_workload matlab manager mpc_conf

clean:
	rm -rf *.o *~ mpc mpc_server mpc_client

