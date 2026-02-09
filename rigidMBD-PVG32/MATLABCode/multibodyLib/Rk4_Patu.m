function [state,Z] = Rk4_Patu(fun, dt,tNow,y0,U1, U2, p_P,p_t, i )


[f1,~] = fun(tNow       , y0,U1, U2, p_P,p_t,i );
k1      = dt*f1;

[f2, ~]  = fun(tNow+0.5*dt, y0  + 0.5*k1,U1, U2, p_P,p_t, i);
k2      = dt*f2;

[f3, ~] =  fun(tNow+0.5*dt, y0  + 0.5*k2,U1, U2, p_P,p_t, i );
k3      = dt*f3;

[f4, Z]   = fun(tNow+dt    , y0  +     k3,U1, U2, p_P,p_t,i );

k4      = dt*f4;
yOut    = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4);


% System states
state   =  yOut(:);
Z       =  Z(:);
end