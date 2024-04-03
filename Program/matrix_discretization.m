function [Ad, Bd, Cd] = matrix_discretization(Ac, Bc, Cc, Ts)
  [m,n] = size(Ac);
  [m,nb] = size(Bc); 
  [m,nc] = size(Cc);
  s = expm([[Ac Bc Cc]*Ts; zeros(nb,n+nb+nc); zeros(nc,n+nb+nc)]);
  Ad = s(1:n,1:n);
  Bd = s(1:n,n+1:n+nb);
  Cd = s(1:n,n+nb+1:n+nb+nc);
end