#bin/sh

#this file launches the simulation an trace mpc_server.out and mpc_ctrl.out execution, 
#and extrapolates execution data, to work it needs: matlab, xdotool, 
#and gnome-terminal correctlyinstalled, and the executable files must exist 

#launch server in a different window (with root privileges) and close the window when it terminates; 
#get server pid after it starts and save it
sudo gnome-terminal --tab -- sh -i -c "sudo ./out/mpc_server.out src/test/input/test.json; echo \"server ended\"; xdotool key --clearmodifiers Ctrl+Shift+W key --clearmodifiers KP_Enter" && echo "server launched" && SERVER_PID=$(pidof ./out/mpc_server.out src/test/input/test.json) && echo "with PID: $SERVER_PID"

#launch ctrl in a different window (with root privileges) and close the window when it terminates; 
#get ctrl pid after it starts and save it
sudo gnome-terminal --tab -- sh -i -c "sudo ./out/mpc_ctrl.out src/test/input/test.json; echo \"ctrl ended\"; xdotool key --clearmodifiers Ctrl+Shift+W key --clearmodifiers KP_Enter" && echo "ctrl launched" && CTRL_PID=$(pidof ./out/mpc_ctrl.out src/test/input/test.json) && echo "with PID: $CTRL_PID"

#start tracing server and ctrl by pid in a new window (with root privileges), 
#when they end: makes report, extract trace of server and ctrl, 
#then extrapolate total functions' execution data for server and ctrl,
#then removes the generated tracing files, and closes the window
sudo gnome-terminal --tab -- sh -i -c "echo \"starting tracing\" && sudo trace-cmd record -e sched_switch -P $SERVER_PID,$CTRL_PID && echo \"tracing ended\" && echo \"making reports\" && trace-cmd report > doc/report/trace_sim.txt && echo \"extracting server trace\" && cat doc/report/trace_sim.txt | grep SERVER > doc/report/trace_server.txt && echo \"extracting ctrl trace\" && cat doc/report/trace_sim.txt | grep CLIENT > doc/report/trace_ctrl.txt && echo \"extrapolating server trace\" && python3 doc/report/filtro3.py doc/report/trace_server.txt doc/report/trace_server_sum.txt && echo \"extrapolating ctrl trace\" && python3 doc/report/filtro3.py doc/report/trace_ctrl.txt doc/report/trace_ctrl_sum.txt; sudo rm -v -f *.dat* *.out ; xdotool key --clearmodifiers Ctrl+Shift+W key --clearmodifiers KP_Enter"

#launch matlab from matlab_sim dir in terminal in a different window (with root privileges) 
#running runsim.m then quit after simulation is done and interrupts 
#server and ctrl by sigint signal, then close window  
sudo gnome-terminal --tab -- sh -i -c "cd ../matlab_sim; matlab -softwareopengl -nosplash -nodesktop -r \"runsim ; quit\"; sudo kill -s SIGINT $SERVER_PID; sudo kill -s SIGINT $CTRL_PID; xdotool key --clearmodifiers Ctrl+Shift+W key --clearmodifiers KP_Enter" 
