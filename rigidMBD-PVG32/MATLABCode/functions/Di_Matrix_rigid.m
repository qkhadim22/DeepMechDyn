function D_i = Di_Matrix_rigid(i,R_I)

R_i = R_I(:,i);

I_til = [0 -1;
          1 0];
D_i = eye(3);
D_i(1:2,3) = I_til*R_i;

