function state = Rk4(fun,tNow,y0, simulationSettings)

dt      = simulationSettings.TimeStep;

k1      = dt*fun(tNow       , y0);
k2      = dt*fun(tNow+0.5*dt, y0  + 0.5*k1);
k3      = dt*fun(tNow+0.5*dt, y0  + 0.5*k2);
k4      = dt*fun(tNow+dt    , y0  +     k3);

yOut    = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4);


% System states
state   =  yOut(:);
end