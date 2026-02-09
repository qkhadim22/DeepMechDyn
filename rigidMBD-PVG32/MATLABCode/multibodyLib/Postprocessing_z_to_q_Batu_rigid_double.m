function [q, dq,p_hyd] = Postprocessing_z_to_q_Batu_rigid_double(t,y,param,bodies,param_bod,joints)
z_i = y(:,1:param.tot_dof);
dz_i = y(:,param.tot_dof+1:2*param.tot_dof);
p_hyd = y(:,2*param.tot_dof+1:end);

for i = 1:length(t)   
        
        [z(i,:),dz(i,:)] = fun_constraintposition_semi_PATU(t(i),z_i(i,:)',dz_i(i,:)',joints,param,param_bod);
        
        phi(i,:) = [z(i,1), z(i,1)+z(i,2), z(i,1)+z(i,2)+z(i,3), z(i,1)+z(i,4)];
        dphi(i,:) = [dz(i,1), dz(i,1)+dz(i,2), dz(i,1)+dz(i,2)+dz(i,3), dz(i,1)+dz(i,4)];

end

for i = 1:length(t)
    [R_i,dR_i,~] = Recursive_position_PATU(phi(i,:),dphi(i,:),param_bod);
    for j = 1:length(bodies)
        R(i,2*(j-1)+1:2*j) = R_i(:,j);
        dR(i,2*(j-1)+1:2*j) = dR_i(:,j);
        q(i,3*(j-1)+1:3*(j-1)+2) = R(i,2*(j-1)+1:2*j);
        q(i,3*(j-1)+3,:) = phi(i,j);
        dq(i,3*(j-1)+1:3*(j-1)+2) = dR(i,2*(j-1)+1:2*j);
        dq(i,3*(j-1)+3,:) = dphi(i,j);
    end
end

