function [state,Z] = Rk4_Patu(fun, dt,tNow,y0 )


[f1,~] = fun(tNow       , y0 );
k1      = dt*f1;

[f2, ~]  = fun(tNow+0.5*dt, y0  + 0.5*k1);
k2      = dt*f2;

[f3,~]  =  fun(tNow+0.5*dt, y0  + 0.5*k2 );
k3      = dt*f3;

[f4, Z]   = fun(tNow+dt    , y0  +     k3 );

k4      = dt*f4;
yOut    = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4);


% System states
state   =  yOut(:);
Z       =  Z(:);
end