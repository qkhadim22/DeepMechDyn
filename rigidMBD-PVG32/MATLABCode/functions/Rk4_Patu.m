function state = Rk4_Patu(fun, dt,tNow,y0,U1, U2, p_P,p_t )

k1      = dt*fun(tNow       , y0,U1, U2, p_P,p_t );
k2      = dt*fun(tNow+0.5*dt, y0  + 0.5*k1,U1, U2, p_P,p_t );
k3      = dt*fun(tNow+0.5*dt, y0  + 0.5*k2,U1, U2, p_P,p_t );
k4      = dt*fun(tNow+dt    , y0  +     k3,U1, U2, p_P,p_t );

yOut    = y0 + (1/6)*(k1 + 2*k2 + 2*k3 + k4);


% System states
state   =  yOut(:);
end