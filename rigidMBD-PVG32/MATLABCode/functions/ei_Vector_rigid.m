function e_i = ei_Vector_rigid(i,R_I,dPHI)

R_i = R_I(:,i);
dphi = dPHI(i);
e_i = zeros(3,1);
e_i(1:2,1) = -dphi^2*R_i;
