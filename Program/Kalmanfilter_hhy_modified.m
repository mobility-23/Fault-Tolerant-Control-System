function    [matrix_state_est,P_k,r_k_hat] = Kalmanfilter_hhy_modified(mpc_params,...
                                                                               matrix_state_measure,...
                                                                               matrix_state_est,...
                                                                               state_nom,...
                                                                               angular_v_des_KF,...
                                                                               P_kmin1,...
                                                                               i)

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
x_k_pri = matrix_state_est;
XX_k_pri_cell = {x_k_pri;angular_v_des_KF}; 
XX_k_pri_mat = cell2mat(XX_k_pri_cell);
AAd_cell = {Ad,Cd;zeros(1,6),1};
AAd_mat = cell2mat(AAd_cell);

P_k_pri = AAd_mat * P_kmin1 *AAd_mat' + Q;

Kalman_gain = P_k_pri * H' / (H * P_k_pri * H' + R);

z_k = matrix_state_measure;
ZZ_k_cell = {matrix_state_measure;angular_v_des_KF};
z_k_modified = cell2mat(ZZ_k_cell);
x_k = XX_k_pri_mat + Kalman_gain * (z_k_modified - H * XX_k_pri_mat);

P_k = (eye(7) - Kalman_gain * H) * P_k_pri;

x_k_cell  = mat2cell(x_k,[6 1]);
x_k = cell2mat(x_k_cell(1));
matrix_state_est = x_k;

%% Residual normalization
r_k = matrix_state_est - state_nom;
r_k_modified_cell = {r_k;0};
r_k_modified_mat  = cell2mat(r_k_modified_cell);

Cov_r_k = H*P_k_pri*H' + R; 
Cov_r_k_mhalf = sqrtm(inv(Cov_r_k));    
r_k_hat_modified = Cov_r_k_mhalf * r_k_modified_mat;    

r_k_hat_cell = mat2cell(r_k_hat_modified,[6 1]);
r_k_hat = cell2mat(r_k_hat_cell(1));
