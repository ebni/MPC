# Suggested by GSL manual
CFLAGS = -pedantic -Werror -Wall -W -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -fshort-enums -fno-common -Dinline= -O2

LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk

mpc.o: mpc.c mpc.h Makefile
	gcc -c mpc.c $(CFLAGS) -o mpc.o

test_mpc.o: test_mpc.c Makefile uav.h
	gcc -c test_mpc.c $(CFLAGS) -o test_mpc.o

test_mpc: test_mpc.o mpc.o dyn.o Makefile
	gcc test_mpc.o mpc.o dyn.o $(LDFLAGS) -o test_mpc

dyn.o: dyn.c dyn.h Makefile
	gcc -c dyn.c $(CFLAGS) -o dyn.o

clean:
	rm -rf *.o *~ mpc mpc_server mpc_client

mpc_server: mpc_server.o mpc.o dyn.o
	gcc mpc_server.o mpc.o dyn.o -L$(LIBDIR) $(LDFLAGS) -o mpc_server

mpc_client: mpc_client.o mpc.o dyn.o
	gcc mpc_client.o mpc.o dyn.o -L$(LIBDIR) $(LDFLAGS) -o mpc_client
