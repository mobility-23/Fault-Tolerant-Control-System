% 修改后的卡尔曼滤波

function    [state_nom_est,P_k] = Kalmanfilter_hhy_modified_1(mpc_params,...
                                                                               state_nom_measure,...
                                                                               state_nom_prior,...
                                                                               angular_v_des_KF,...
                                                                               P_kmin1,...
                                                                               i)
%% Set parameters
Ad = mpc_params.Ac;
Bd = mpc_params.Bc;
Cd = mpc_params.Cc;

H = [1,0,0,0,0,0,0;
     0,1,0,0,0,0,0;
     0,0,1,0,0,0,0;
     0,0,0,1,0,0,0;
     0,0,0,0,1,0,0;
     0,0,0,0,0,1,0;
     0,0,0,0,0,0,1];

Q = [0.01,0,0,0,0,0,0;
     0,0.01,0,0,0,0,0;
     0,0,0.01,0,0,0,0;
     0,0,0,0.01,0,0,0;
     0,0,0,0,0.01,0,0;
     0,0,0,0,0,0.01,0;
     0,0,0,0,0,0,0.01]; 

R = [0.05,0,0,0,0,0,0;
     0,0.05,0,0,0,0,0;
     0,0,0.05,0,0,0,0;
     0,0,0,0.05,0,0,0;
     0,0,0,0,0.05,0,0;
     0,0,0,0,0,0.05,0;
     0,0,0,0,0,0,0.05]; 

%% kalman Filter

x_k_pri = state_nom_prior;
XX_k_pri_cell = {x_k_pri;angular_v_des_KF}; 
XX_k_pri_mat = cell2mat(XX_k_pri_cell);

AAd_cell = {Ad,Cd;zeros(1,6),1};
AAd_mat = cell2mat(AAd_cell);

P_k_pri = AAd_mat * P_kmin1 *AAd_mat' + Q;

Kalman_gain = P_k_pri * H' / (H * P_k_pri * H' + R);

z_k = state_nom_measure;
ZZ_k_cell = {state_nom_measure;angular_v_des_KF};
z_k_modified = cell2mat(ZZ_k_cell);
x_k = XX_k_pri_mat + Kalman_gain * (z_k_modified - H * XX_k_pri_mat);

P_k = (eye(7) - Kalman_gain * H) * P_k_pri;

%% 
x_k_cell  = mat2cell(x_k,[6 1]);
x_k = cell2mat(x_k_cell(1));
state_nom_est = x_k;