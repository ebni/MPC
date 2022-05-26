# Working flags on the Pelican
#CFLAGS = -I/home/asctec/usrlocal/include -pedantic -Wall -Wno-sign-conversion -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -fshort-enums -fno-common -Dinline= -O0 -g
#LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk -lpthread -L/home/asctec/usrlocal/lib -Wl,-rpath -Wl,/home/asctec/usrlocal/lib

# Working flages elsewhere

CFLAGS = -pedantic -Werror -Wall -Wno-sign-conversion -Wmissing-prototypes -Wstrict-prototypes -Wconversion -Wshadow -Wpointer-arith -Wcast-qual -Wcast-align -Wwrite-strings -Wnested-externs -fshort-enums -fno-common -Dinline= -O0 -g -pg
LDFLAGS = -lm -ljson-c -lrt -lgsl -lgslcblas -lglpk -lpthread -pg 
T =
BROWSER =
ifndef $(T)
	T = t
endif

GPROF_FLAGS = 
I =
ifndef $(I)
	I = 1 
endif

ifndef $(BROWSER)
	BROWSER = xdg-open
endif
.PHONY: all
all: mpc_server mpc_ctrl matlab
#directories
OUT = out
SRC = src
INCLUDE = include
DOC = doc
OBJ = $(OUT)/obj
TEST = $(SRC)/test
INPUT = $(TEST)/input
REPORT = $(DOC)/report
TRACE = /sys/kernel/tracing/
#end directories

#command parts
TERMINAL = gnome-terminal --tab -- bash -i -c
RECORD_CMD = sudo trace-cmd record -e sched_switch -F 
REPORT_CMD = trace-cmd report -t | grep
PYTHON = python3 $(REPORT)/filtro3.py
#end command parts

#files
JSON = $(INPUT)/test.json
SERVER = $(OUT)/mpc_server.out
CTRL = $(OUT)/mpc_ctrl.out
INDEX = $(DOC)/html/index.html
#end files
mpc_server: mpc_server.o mpc.o dyn.o
	gcc $(OBJ)/mpc_server.o $(OBJ)/mpc.o $(OBJ)/dyn.o $(LDFLAGS) -o $(SERVER)

mpc_ctrl: mpc_ctrl.o mpc.o dyn.o
	gcc $(OBJ)/mpc_ctrl.o $(OBJ)/mpc.o $(OBJ)/dyn.o $(LDFLAGS) -o $(CTRL)

mpc_server.o: $(TEST)/mpc_server.c
	gcc -c $(TEST)/mpc_server.c $(CFLAGS) -o $(OBJ)/mpc_server.o

mpc_ctrl.o: $(TEST)/mpc_ctrl.c
	gcc -c $(TEST)/mpc_ctrl.c $(CFLAGS) -o $(OBJ)/mpc_ctrl.o

mpc.o: $(SRC)/mpc.c $(INCLUDE)/mpc.h Makefile
	gcc -c $(SRC)/mpc.c $(CFLAGS) -o $(OBJ)/mpc.o

dyn.o: $(SRC)/dyn.c $(INCLUDE)/dyn.h Makefile
	gcc -c $(SRC)/dyn.c $(CFLAGS) -o $(OBJ)/dyn.o

mpc_matlab.mexa64: $(SRC)/mpc_matlab.c
	mex -O -v $(SRC)/mpc_matlab.c;
	mv mpc_matlab.mexa64 $(OUT)/mpc_matlab.mexa64

.PHONY: matlab
matlab: mpc_matlab.mexa64

.PHONY: clean
clean:
	rm -v $(OBJ)/*.o $(OUT)/*.out $(OUT)/*.mexa64 

.PHONY: run_server
run_server: 
	$(TERMINAL) "sudo $(SERVER) $(JSON); exec bash -i" 

.PHONY: run_ctrl
run_ctrl:
	$(TERMINAL) "sudo $(CTRL) $(JSON); exec bash -i"

.PHONY: run_matlab
run_matlab:
	$(TERMINAL) "cd ../matlab_sim; matlab -softwareopengl; exec bash -i"

.PHONY: run_matlab_alt
run_matlab_alt:
	$(TERMINAL) "cd ../matlab_sim; matlab -softwareopengl -nosplash -nodesktop -r "launch"; exec bash -i"

.PHONY: run_matlab_alt_autoclose
run_matlab_alt_autoclose:
	gnome-terminal --tab --bash -i -c "cd ../matlab_sim; matlab -softwareopengl -nosplash -nodesktop -r "launch"; xdotool key --clearmodifiers Ctrl+Shift+W key --clearmodifiers KP_Enter"
	
.PHONY: run
run: run_server run_ctrl run_matlab

.PHONY: run_c
run_c: run_server run_ctrl


# gprof_server:
# 	gprof  $(GPROF_FLAGS) mpc_server server/gmon.out > testCambiamentiParametriJson/$(T)/prof_server.txt
# # mv server/gmon.out server/gmon.out.$(I) 

# gprof_ctrl:
# 	gprof $(GPROF_FLAGS)  mpc_ctrl client/gmon.out > testCambiamentiParametriJson/$(T)/prof_ctrl.txt
# # mv client/gmon.out client/gmon.out.$(I) 

# .PHONY: all_gprof
# all_gprof: gprof_server gprof_ctrl 

# .PHONY: gprof_sum_server
# gprof_sum_server:
# 	gprof -s mpc_server server/gmon.out.*;
# 	mv gmon.sum server/gmon.sum;
# 	gprof mpc_server server/gmon.sum > server/prof_server_sum.txt

# gprof_sum_client:
# 	gprof -s mpc_ctrl client/gmon.out.*;
# 	mv gmon.sum client/gmon.sum;
# 	gprof mpc_ctrl client/gmon.sum > client/prof_ctrl_sum.txt

# gprof_sum: gprof_sum_client gprof_sum_server

record_server:
	$(RECORD_CMD) ./$(SERVER) $(JSON)

record_client:
	$(RECORD_CMD) ./$(CTRL) $(JSON)

report_server:
	$(REPORT_CMD) SERVER > $(REPORT)/trace_server.txt;
	$(PYTHON) $(REPORT)/trace_server.txt $(REPORT)/trace_server_sum.txt

report_client:
	$(REPORT_CMD) CLIENT > $(REPORT)/trace_client.txt;
	$(PYTHON) $(REPORT)/trace_client.txt $(REPORT)/trace_client_sum.txt

report: report_server report_client

run_server_trace:
	gnome-terminal --tab -- bash -c "make record_server; make report_server; exec bash -i";
	gnome-terminal --tab -- bash -c "sudo $(CTRL) $(JSON); exec bash -i";
	gnome-terminal --tab -- bash -c "cd ../matlab_sim; matlab -softwareopengl; exec bash -i"

run_client_trace:
	gnome-terminal --tab -- bash -c "sudo $(SERVER) $(JSON); exec bash -i";
	gnome-terminal --tab -- bash -c "make record_client; make report_client; exec bash -i";
	gnome-terminal --tab -- bash -c "cd ../matlab_sim; matlab -softwareopengl; exec bash -i"

#requires doxygen and doxygen-gui packages to be installed
doxy_gui:
	doxywizard

doxy:
	cd $(DOC);
	doxygen Doxygen

read_doc:
	$(BROWSER) $(INDEX)

trace:
	./trace.sh