function Q_wei = External_weight(phi,param,pos)

I_2 = [1 0;
       0 1];
I_til = [0 -1;
         1 0];
     

F_wei = [0;
         -param.m_weight*param.g];


Rot_A4 = [cos(phi(4)) -sin(phi(4));
          sin(phi(4)) cos(phi(4))];
L_e4 = [I_2 Rot_A4*I_til*pos.bar_weig_4];
Q_wei4 = L_e4'*F_wei;

Q_wei = [zeros(9,1);
            Q_wei4];