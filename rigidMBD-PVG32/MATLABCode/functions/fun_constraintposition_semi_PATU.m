function [z,dz,Rz] = fun_constraintposition_semi_PATU(t,z_ind,dz_ind,joints,param,param_bod)

% fun_constraintposition_semi_PATU----->Implementing position and velocity problem

AbsErr=10^(-6);
MaxIter=100;
Niter=0;
ok=0;

z_dep0 = [2.5862;-2.5501];

while ((ok==0)&&(Niter<MaxIter))
    
    Niter=Niter+1;

    z( param.Index_ind,1) = z_ind;                      
    z( param.Index_dep,1) = z_dep0;                     
    phi_i = [z(1) z(1)+z(2)  z(1)+z(2)+z(3) z(1)+z(4)];    % Converting relative coordinates into global coordinates 
    [R_i,rj] = Recursive_position_vector_PATU(phi_i,param_bod);    
    
    CC = Constraint_quations_postition_PATU_semi(R_i,phi_i,joints); %norm(CC)
    Cz = Constraint_closed_loop_Jacobian_quations_PATU_semi(R_i,phi_i,rj,joints);
    Cz_d = Cz(:, param.Index_dep);
     if (max(abs(CC))<AbsErr)
        ok=1;
    end     

    z_dep=z_dep0-Cz_d\CC;
    z_dep0=z_dep;
    
end

z( param.Index_ind,1) = z_ind;
z( param.Index_dep,1) = z_dep;
phi_i = [z(1) z(1)+z(2)  z(1)+z(2)+z(3) z(1)+z(4)];
[R_i,rj] = Recursive_position_vector_PATU(phi_i,param_bod);   

Cz = Constraint_closed_loop_Jacobian_quations_PATU_semi(R_i,phi_i,rj,joints);

Czd = Cz(:,param.Index_dep);           
Czi = Cz(:,param.Index_ind);   

Rz = [1 0;
     -Czd^(-1)*Czi;
     0 1];
dz = Rz*dz_ind;