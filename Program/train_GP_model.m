function d_GP = train_GP_model(log)
Nsim = length(log.time);

nz = 4;
nd = 2;
var_w = diag([(1/3)^2  (deg2rad(3)/3)^2]);

gp_n = nz;
gp_p = nd;

% GP hyperparameters
var_f   = repmat(0.01,[gp_p,1]);    
var_n   = diag(var_w/3);            
M       = repmat(diag([1e0,1e0,1e0,1e0].^2),[1,1,gp_p]);  
maxsize = Nsim; 

d_GP = GP(gp_n, gp_p, var_f, var_n, M, maxsize);

Dd = [1 0; 0 0; 0 1; 0 0; 0 0; 0 0];
Dz_x = [1 0 0 0 0 0; 0 0 1 0 0 0];
Dz_u = [1 0;0 1];

  for k = Nsim-51:Nsim-1
    d_est = Dd \ (log.xhat(:,k+1) - log.xnom(:,k+1));
    zhat = [Dz_x * log.xhat(:,k); Dz_u * log.u(:,k) ];
    d_GP.add(zhat,d_est');
  end
d_GP.updateModel();
d_GP.optimizeHyperParams('fmincon')

d_GP.M
d_GP.var_f
d_GP.var_n
end