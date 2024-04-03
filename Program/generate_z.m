function z = generate_z(xk,uk)
    Dz_x = [1 0 0 0 0 0;0 0 1 0 0 0];
    Dz_u = diag(1,1);
    z = [Dz_x * xk;
         Dz_u * uk];
end