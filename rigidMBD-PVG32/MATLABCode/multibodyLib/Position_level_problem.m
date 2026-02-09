function z = Position_level_problem(z_i)
         
        L2      = parameters.L2;
        L3      = parameters.L3;
        L4      = parameters.L4;
        l_c     = parameters.l_c;
         d1     = parameters.d1;
         z      = [zi(1);zd(1);zd(2);zi(2);zd(3)];

   % Satisfying the constraints at position level:
        C       = Cfun(L2,L3,L4,d1,l_c,z(1),z(2),z(3),z(4));
        norm_C  = norm(C);
    
    while norm_C > KinematicSolver.ztol && KinematicSolver.iters < KinematicSolver.max_iter
    
          Cz    = Czfun(L2,L3,L4,d1,l_c,z(1),z(2),z(3),z(4));
         Czd    = [Cz(:,2),Cz(:,3)];
      deltazd   = Czd\(-C);
     
        z(1)    = z(1);
        z(2)    = z(2)  +  deltazd(1);
        z(3)    = z(3)  +  deltazd(2);
        z(4)    = z(4);
        z(5)    = z(5);

          C     = Cfun(L2,L3,L4,d1,l_c,z(1),z(2),z(3),z(4));
        norm_C  = norm(C);
        iters   = KinematicSolver.iters+1;
    
    if iters >= KinematicSolver.max_iter
    warning('Max number of iterations (%z) reached!',KinematicSolver.max_iter);
    end
    end
          zout  = z(:);
end