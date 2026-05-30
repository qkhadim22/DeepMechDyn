function state = Rk4(fun, dt,tNow,y0,optionalVec, param)

k1      = dt*fun(tNow       , y0, optionalVec,param);
k2      = dt*fun(tNow+0.5*dt, y0  + 0.5*k1, optionalVec,param);
k3      = dt*fun(tNow+0.5*dt, y0  + 0.5*k2,  optionalVec,param);
k4      = dt*fun(tNow+dt    , y0  +     k3, optionalVec,param);

yOut    = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4);


% System states
state   =  yOut(:);
end