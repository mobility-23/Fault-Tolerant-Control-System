function  command = solve_mpc_problem_QP(A, B, C,matrix_q,matrix_r,...
    lower, upper, ref_state, horizon, control,...
    matrix_state, angular_v_des_seq)

N = horizon;

AA_cell=cell(N,1);
for i=1:N
    AA_cell{i,1} = A^i;
end
AA = cell2mat(AA_cell);

BB_cell=cell(N,N);
for i=1:N
    for j=1:N
        if j<=i
        BB_cell{i,j}=A^(i-j)*B;
        else
        BB_cell{i,j}=zeros(size(B,1),size(B,2));
        end
    end
end
BB=cell2mat(BB_cell);

I = eye(size(A,1));
CC_cell=cell(N,N);
for i=1:N
    for j=1:N
        if j<=i
        CC_cell{i,j}=A^(i-j)*I;
        else
        CC_cell{i,j}=zeros(size(A,1),size(A,2));
        end
    end
end
CC=cell2mat(CC_cell);

cc_cell=cell(N,1);
for i = 1:N
    cc_cell{i,:} = C * angular_v_des_seq(i,1);
end
cc=cell2mat(cc_cell);

CC = CC * cc;

Q_cell = cell(N,N);
R_cell = cell(N,N);
for i=1:horizon
    for j= 1:horizon
        if i == j
            Q_cell{i,j}=matrix_q;
            R_cell{i,j}=matrix_r;
        else 
            Q_cell{i,j}=zeros(size(matrix_q,1),...
                size(matrix_q,2));
            R_cell{i,j}=zeros(size(matrix_r,1),...
                size(matrix_r,2));
        end
    end
end
Q = cell2mat(Q_cell); 
R = cell2mat(R_cell); 

matrix_LL=cell(horizon,1);
matrix_UU=cell(horizon,1);
for i=1:horizon
    matrix_LL{i,1}=lower;
    matrix_UU{i,1}=upper;
end
matrix_ll = cell2mat(matrix_LL);
matrix_uu = cell2mat(matrix_UU);

%% QP solver paramters
H = BB' * Q * BB + R;
H = (H + H') / 2;
f = (AA * matrix_state + CC)' * Q * BB;
options=optimset('Algorithm','interior-point-convex',...
   'Display', 'off');
[matrix_v,fval,exutflag,message]=quadprog(H,f,...
    [],[],...
    [],[],matrix_ll,matrix_uu,[],options);
command=matrix_v(1:2,1);
end