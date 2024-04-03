function true_mpc_params = load_true_mpc_params(veh_params, sim_params)
%% veh params
mass = veh_params.mass*0.5;
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
Ac = [0 1 0 0;
      0 -(2*Calfa_front + 2*Calfa_rear)/(mass*Vx) (2*Calfa_front + 2*Calfa_rear)/(mass) (-2*Calfa_front*lf + 2*Calfa_rear*lr)/(mass*Vx);
      0 0 0 1;
      0 -(2*Calfa_front*lf - 2*Calfa_rear*lr)/(Iz*Vx) (2*Calfa_front*lf - 2*Calfa_rear*lr)/(Iz) -(2*Calfa_front*lf^2 + 2*Calfa_rear*lr^2)/(Iz*Vx)];

Bc = [0;
      2*Calfa_front/mass;
      0;
      2*Calfa_front*lf/Iz];
   
Cc = [0;
     -(2*Calfa_front*lf - 2*Calfa_rear*lr)/(mass*Vx)-Vx;
     0;
     -(2*Calfa_front*lf^2 + 2*Calfa_rear*lr^2)/(Iz*Vx)];
 
[Ad, Bd, Cd] = matrix_discretization(Ac, Bc, Cc, Ts);
%Cc = zeros(4,1);

%% penalty matrix
Q = diag([1, 0, 1, 0, 0, 1]);
R = 1;

%% 
I = eye(size(Ac,1));

%% save mpc_params
true_mpc_params.Ac = Ac;
true_mpc_params.Bc = Bc;
true_mpc_params.Cc = Cc;
true_mpc_params.Ad = Ad;
true_mpc_params.Bd = Bd;
true_mpc_params.Cd = Cd;
true_mpc_params.I  = I;
true_mpc_params.Ts = Ts;
true_mpc_params.Q  = Q;
true_mpc_params.R  = R;
true_mpc_params.N  = 10; 

end