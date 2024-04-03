function  command = solve_mpc_problem(A, B, C,matrix_q,matrix_r,...
                    lower, upper, ref_state, horizon, control,...
                    matrix_state,control_state)

q=size(A,1);
matrix_aa=zeros(size(A,1),horizon * size(A,2));
for i=1:horizon
    matrix_aa(1:q,(i-1)*q+1:(i-1)*q+q)=A^(i); 
end

K_cell=cell(horizon,horizon);
for r=1:horizon
    for c=1:horizon
        if c<=r
        K_cell{r,c}=A^(r-c)*B;
        else
        K_cell{r,c}=zeros(size(B,1),size(B,2));
        end
    end
end
matrix_k=cell2mat(K_cell);

matrix_cc=zeros(horizon*size(C,1),size(C,2));
c_r=size(C,1);
matrix_cc(1:c_r,1)=C;
for j=2:horizon
     matrix_cc((j-1)*c_r+1:(j-1)*c_r+c_r,1)=A^(j-1)*C+...
         matrix_cc((j-2)*c_r+1:(j-2)*c_r+c_r,1);
end

matrix_Q = cell(horizon,horizon);
matrix_R = cell(horizon,horizon);
for i=1:horizon
    for j= 1:horizon
        if i == j
            matrix_Q{i,j}=matrix_q;
            matrix_R{i,j}=matrix_r;
        else 
            matrix_Q{i,j}=zeros(size(matrix_q,1),...
                size(matrix_q,2));
            matrix_R{i,j}=zeros(size(matrix_r,1),...
                size(matrix_r,2));
        end
    end
end
matrix_qq=cell2mat(matrix_Q);
matrix_rr=cell2mat(matrix_R); 

matrix_LL=cell(horizon,1);
matrix_UU=cell(horizon,1);
for i=1:horizon
    matrix_LL{i,1}=lower;
    matrix_UU{i,1}=upper;
end
matrix_ll = cell2mat(matrix_LL); 
matrix_uu = cell2mat(matrix_UU); 

matrix_M=cell(horizon,1);
for i=1:horizon
    matrix_M{i,1}=A^(i)*matrix_state;
end

matrix_m=cell2mat(matrix_M);
matrix_t = zeros(size(B,1)*horizon,1); 


matrix_V = cell(horizon,1); 
for i=1:horizon
    matrix_V{i,1} = control_state;
end
matrix_v = cell2mat(matrix_V);

matrix_m1 = matrix_k' * matrix_qq * matrix_k+matrix_rr;          
matrix_m1 = (matrix_m1+matrix_m1')/2;                              
matrix_m2 = matrix_k' * matrix_qq * (matrix_m+matrix_cc+matrix_t);


[l_row,l_col]                  = size(matrix_ll);
matrix_inequality_constrain_ll = eye(horizon*control,horizon*control);
u_row                          = size(matrix_uu,1); 
matrix_inequality_constrain_uu = eye(horizon*control,1);


options=optimset('Algorithm','interior-point-convex',...
   'Display', 'off');
[matrix_v,fval,exutflag,message]=quadprog(matrix_m1,matrix_m2,...
    matrix_inequality_constrain_ll,...
    matrix_inequality_constrain_uu,...
    [],[],matrix_ll,matrix_uu,[],options);

command=matrix_v(1,1);


