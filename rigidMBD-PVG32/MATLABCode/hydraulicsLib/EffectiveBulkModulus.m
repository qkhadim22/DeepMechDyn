function [Be1,Be2] = EffectiveBulkModulus(s,l1, l2) 

param = Get_parameters_Patu();
Aa1     = param.C1_A1;
Ab1     = param.C1_A2;
Vh1     = param.VA;
Vh2     = param.VA;

Bh= param.Bh;
Bc = param.Bc;
Bo = param.Bo;

V_c_1   = Vh1+ Aa1*(l1);
V_c_2   = Vh2+Ab1*(l2);

            

Be1 = 1/((1/Bo)+(Aa1*s/(V_c_1*Bc))+(Vh1/(V_c_1*Bh)));     
Be2 = 1/((1/Bo)+(Ab1*s/(V_c_2*Bc))+(Vh2/(V_c_2*Bh)));


end 