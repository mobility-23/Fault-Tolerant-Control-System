function mpc_matrix = set_mpc_matrix(veh_params, Vx)

mass = veh_params.mass;
Iz = veh_params.Iz;
Calfa_front = veh_params.Calfa_front;
Calfa_rear = veh_params.Calfa_rear;
lf = veh_params.lf;
lr = veh_params.lr;
Vx
Ac = [0 1 0 0 0 0;
      0 -(2*Calfa_front + 2*Calfa_rear)/(mass*Vx) (2*Calfa_front + 2*Calfa_rear)/(mass) (-2*Calfa_front*lf + 2*Calfa_rear*lr)/(mass*Vx) 0 0;
      0 0 0 1 0 0;
      0 -(2*Calfa_front*lf - 2*Calfa_rear*lr)/(Iz*Vx) (2*Calfa_front*lf - 2*Calfa_rear*lr)/(Iz) -(2*Calfa_front*lf^2 + 2*Calfa_rear*lr^2)/(Iz*Vx) 0 0;
      0 0 0 0 0 1;
      0 0 0 0 0 0];

Bc = [0 0;
      2*Calfa_front/mass 0;
      0 0;
      2*Calfa_front*lf/Iz 0;
      0 0;
      0 1];
   
Cc = [0;
     -(2*Calfa_front*lf - 2*Calfa_rear*lr)/(mass*Vx) - Vx;
     0;
     -(2*Calfa_front*lf^2 + 2*Calfa_rear*lr^2)/(Iz*Vx)
     0;
     0];
Ac
Bc
Cc

mpc_matrix.Ac = Ac;
mpc_matrix.Bc = Bc;
mpc_matrix.Cc = Cc;
 
end