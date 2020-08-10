# Suggested by GSL manual
CFLAGS = -pedantic -Werror -Wall -Wno-sign-conversion -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -fshort-enums -fno-common -Dinline= -O2

LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk -lpthread

.PHONY: clean

mpc_server: mpc_server.o mpc.o dyn.o
	gcc mpc_server.o mpc.o dyn.o $(LDFLAGS) -o mpc_server

mpc_client: mpc_client.o mpc.o dyn.o
	gcc mpc_client.o mpc.o dyn.o $(LDFLAGS) -o mpc_client

mpc_ctrl: mpc_ctrl.o mpc.o dyn.o
	gcc mpc_ctrl.o mpc.o dyn.o $(LDFLAGS) -o mpc_ctrl

mpc_ctrl.o: mpc_ctrl.c
	gcc -c mpc_ctrl.c $(CFLAGS) -o mpc_ctrl.o

mpc.o: mpc.c mpc.h Makefile
	gcc -c mpc.c $(CFLAGS) -o mpc.o

dyn.o: dyn.c dyn.h Makefile
	gcc -c dyn.c $(CFLAGS) -o dyn.o

all: mpc_server mpc_client mpc_ctrl

clean:
	rm -rf *.o *~ mpc mpc_server mpc_client

