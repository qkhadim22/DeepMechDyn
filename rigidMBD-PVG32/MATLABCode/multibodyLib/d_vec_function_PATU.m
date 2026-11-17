function d = d_vec_function_PATU(dphi,rj,param)
% d(3*(i-1)+1:3*(i-1)+2) = -r_i*(dphi_parent^2 - dphi_i^2), third entry 0 (parent = ground: dphi_parent = 0)
if nargin < 3
    parent = [0 1 2 1];          % PATU default
else
    parent = param.parent;
end
nb = size(rj,2);
d  = zeros(3*nb,1);
for i = 1:nb
    if parent(i) == 0
        dphi_p = 0;
    else
        dphi_p = dphi(parent(i));
    end
    d(3*(i-1)+1:3*(i-1)+2) = -rj(:,i)*(dphi_p^2 - dphi(i)^2);
end
