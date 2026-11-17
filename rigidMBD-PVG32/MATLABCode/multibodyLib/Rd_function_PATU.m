function Rd = Rd_function_PATU(rj)
% Rd(3*(i-1)+1:3*i, i) = [r_iy; -r_ix; 1] for every body i (rj holds the joint positions as columns)
nb = size(rj,2);
Rd = zeros(3*nb,nb);
for i = 1:nb
    Rd(3*(i-1)+1:3*i,i) = [rj(2,i); -rj(1,i); 1];
end
