%%%%%%%%%%%%%
% IMPORTANT %
%%%%%%%%%%%%%
% Before launching this script, the MPC server must be up and running.
% Below the sufficient steps
%
% (1) Download the latest source code (only the first time) by
%        git pull
%
% (2) Compile the latest source code (only if some update was downloaded at
%      step (1)) by
%        make mpc_server
%
% (3) Launch the MPC server (always) by
%        ./mpc_server json/sys_state_input.json
%     the server is listening at the default port 6004. To change this
%     value, the C code of the mpc_server needs to be modified
%
% In the invocation above:
%   - the first parameter is the JSON model used by MPC. Of course, the
%   closer is such a model to the reality, the more accurate are going to
%   be the results
%   - the second parameter is the port behind which the MPC server will be
%   listening. It must be the same port as the one used below to send the
%   state

server_IP   = '127.0.0.1';   % this should be the IP of the MPC server
server_port = 6004;          % DO NOT CHANGE (port where the MPC server is listening)
sizeof_double = 8;
num_iter = 10;
x = [1;1;1];

A = [-1, 1, 0;
	1, 1, 0;
	0, 1, 2];
B = [1, 0;
	0, 1;
	0, 0];

[n,m] = size(B);

% opening an UDP socket to the server
socket_id = udp(server_IP, server_port);
fopen(socket_id);

for i=1:num_iter
	fwrite(socket_id,x,'double');  % send state to the MPC server
	u = fread(socket_id,m*sizeof_double,'double'); % get the imput from MPC server
	x = A*x+B*u;	% plant dynamics
end
fclose(socket_id);
