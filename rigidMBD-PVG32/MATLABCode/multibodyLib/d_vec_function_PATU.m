function d = d_vec_function_PATU(dphi,rj)
r1x = rj(1,1);
r1y = rj(2,1);
r2x = rj(1,2);
r2y = rj(2,2);
r3x = rj(1,3);
r3y = rj(2,3);
r4x = rj(1,4);
r4y = rj(2,4);
dphi1 = dphi(1);
dphi2 = dphi(2);
dphi3 = dphi(3);
dphi4 = dphi(4);

d =[          dphi1^2*r1x;
              dphi1^2*r1y;
                        0;
 -r2x*(dphi1^2 - dphi2^2);
 -r2y*(dphi1^2 - dphi2^2);
                        0;
 -r3x*(dphi2^2 - dphi3^2);
 -r3y*(dphi2^2 - dphi3^2);
                        0;
 -r4x*(dphi1^2 - dphi4^2);
 -r4y*(dphi1^2 - dphi4^2);
                        0];