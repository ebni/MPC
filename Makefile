# Suggested by GSL manual
#CFLAGS = -ansi -pedantic -Werror -Wall -W -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -fshort-enums -fno-common -Dinline= -g -O0
CFLAGS = -pedantic -Werror -Wall -W -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -fshort-enums -fno-common -Dinline= -g -O0  

GLPKVERSION = glpk-4.65
GLPKROOTDIR = $(PWD)/usr
GLPK_DIFFEXCLUDE = --exclude=config.h --exclude=config.log  --exclude=config.status --exclude=.deps --exclude=glpsol* --exclude=*Makefile --exclude=*~ --exclude=*.la --exclude=*.lo --exclude=*.o --exclude=*libtool --exclude=*libs --exclude=*stamp-h1

INCLUDEDIR = $(GLPKROOTDIR)/include


LIBDIR = $(GLPKROOTDIR)/lib
#CFLAGS=-g -O0
LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk -Wl,-rpath -Wl,$(LIBDIR)
#LDFLAGS = -lgsl -lgslcblas -lglpk -Wl,--verbose

#CFLAGS += $(shell pkg-config --cflags json-c)
#LDFLAGS += $(shell pkg-config --libs json-c)

glpk-config:
	wget https://ftp.gnu.org/gnu/glpk/$(GLPKVERSION).tar.gz  #download
	tar -xzvf $(GLPKVERSION).tar.gz  # unpack
	cd $(GLPKVERSION) ; \
	./configure --prefix=$(GLPKROOTDIR); \
	cd ..

glpk-patch: #glpk_patch
	diff -ubr $(GLPK_DIFFEXCLUDE) $(GLPKVERSION).orig $(GLPKVERSION)_eb > glpk_patch
	ls

glpk-install: #glpk-config
	patch -p0 < glpk_patch ;\
	cd $(GLPKVERSION) ; \
	make install; \
	cd ..

glpk-clean:
	rm -rf $(GLPKVERSION)*

default: mpc

mpc.o: mpc.c mpc.h Makefile
	gcc -c mpc.c -I$(INCLUDEDIR) $(CFLAGS) -o mpc.o

test_mpc.o: test_mpc.c Makefile uav.h
	gcc -c test_mpc.c -I$(INCLUDEDIR) $(CFLAGS) -o test_mpc.o

test_mpc: test_mpc.o mpc.o dyn.o Makefile
	gcc test_mpc.o mpc.o dyn.o -L$(LIBDIR) $(LDFLAGS) -o test_mpc

dyn.o: dyn.c dyn.h Makefile
	gcc -c dyn.c -I$(INCLUDEDIR) $(CFLAGS) -o dyn.o

test_dyn: dyn.o uav.h test_dyn.c Makefile
	gcc dyn.o test_dyn.c -I$(INCLUDEDIR) $(CFLAGS) -L$(LIBDIR) $(LDFLAGS) -o test_dyn

clean:
	rm -rf *.o *~ mpc test_dyn a.out

test_sys.o: test_sys.c Makefile sys.h
	gcc -c test_sys.c -I$(INCLUDEDIR) $(CFLAGS) -o test_sys.o

test_sys: test_sys.o mpc.o dyn.o Makefile
	gcc test_sys.o mpc.o dyn.o -L$(LIBDIR) $(LDFLAGS) -o test_sys

mpc_server: mpc_server.o mpc.o dyn.o
	gcc mpc_server.o mpc.o dyn.o -L$(LIBDIR) $(LDFLAGS) -o mpc_server

mpc_client: mpc_client.o mpc.o dyn.o
	gcc mpc_client.o mpc.o dyn.o -L$(LIBDIR) $(LDFLAGS) -o mpc_client

mpc_ctrl: mpc_ctrl.o mpc.o dyn.o
	gcc mpc_ctrl.o mpc.o dyn.o -L$(LIBDIR) $(LDFLAGS) -o mpc_ctrl

run: test_sys
	./test_sys json/sys_state_input.json var 10
#run: mpc_server
#	./mpc_server json/sys_state_input.json 6004
