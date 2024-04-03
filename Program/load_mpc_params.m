function mpc_params = load_mpc_params(veh_params, sim_params)

%% veh params
mass = veh_params.mass;
Iz = veh_params.Iz;
Calfa_front = veh_params.Calfa_front;
Calfa_rear = veh_params.Calfa_rear;
lf = veh_params.lf;
lr = veh_params.lr;
Vx = veh_params.Vx;

%% simulation params
Ts = sim_params.Ts;

%% mpc matrix
% continuous matrix
mpc_matrix = set_mpc_matrix(veh_params, Vx);
Ac = mpc_matrix.Ac;
Bc = mpc_matrix.Bc;
Cc = mpc_matrix.Cc;
% Ac = [0 1 0 0;
%       0 -(2*Calfa_front + 2*Calfa_rear)/(mass*Vx) (2*Calfa_front + 2*Calfa_rear)/(mass) -(2*Calfa_front*lf + 2*Calfa_rear*lr)/(mass*Vx);
%       0 0 0 1;
%       0 -(2*Calfa_front*lf - 2*Calfa_rear*lr)/(Iz*Vx) (2*Calfa_front*lf - 2*Calfa_rear*lr)/(Iz) -(2*Calfa_front*lf^2 + 2*Calfa_rear*lr^2)/(Iz*Vx)];
% 
% Bc = [0;
%       2*Calfa_front/mass;
%       0;
%       2*Calfa_front*lf/Iz];
%    
% Cc = [0;
%      -(2*Calfa_front*lf - 2*Calfa_rear*lr)/(mass*Vx)-Vx;
%      0;
%      -(2*Calfa_front*lf^2 + 2*Calfa_rear*lr^2)/(Iz*Vx)];
%Cc = zeros(4,1);

%% penalty matrix
Q = diag([1, 0, 1, 0, 0, 1]);
R = diag([1, 1]);

%% 
I = eye(size(Ac,1));

%% save mpc_params
mpc_params.Ac = Ac;
mpc_params.Bc = Bc;
mpc_params.Cc = Cc;
mpc_params.I  = I;
mpc_params.Ts = Ts;
mpc_params.Q  = Q;
mpc_params.R  = R;
mpc_params.N  = 10; 
mpc_params.state_size = 6;
mpc_params.control_size = 2;

end