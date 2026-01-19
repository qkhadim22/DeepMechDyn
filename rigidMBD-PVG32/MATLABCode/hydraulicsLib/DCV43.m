function [Q_A, Q_B,Q_p] = DCV43( U,p_h1, p_h2,p_P, p_t, CV1,CV2)  

% U: user control signal
% p_h1: hydraulic pressure on piston side
% p_h2: hydraulic pressure on piston-rod side
% p_P: pump pressure
% p_t: tank pressure


param = Get_parameters_Patu();

if U >= 1e-6
    
     Leak1  = param.cu_1*(sqrt(abs(p_P-p_h1))*sign(p_P-p_h1)-sqrt(abs(p_h1-p_t))*sign(p_h1-p_t));
     Leak2  = param.cu_2*(sqrt(abs(p_P-p_h2))*sign(p_P-p_h2)-sqrt(abs(p_h2-p_t))*sign(p_h2-p_t));
     Q_A = CV1*(U)*sqrt(abs(p_P-p_h1))*sign(p_P-p_h1)+Leak1;
     Q_B = CV2*(U)*sqrt(abs(p_h2-p_t))*sign(p_h2-p_t)+Leak2;
     Q_p = Q_A;

elseif -1e-6 < U && U < 1e-6 % U == 0,
    Leak1  = param.cu_1*(sqrt(abs(p_P-p_h1))*sign(p_P-p_h1)-sqrt(abs(p_h1-p_t))*sign(p_h1-p_t));
    Leak2  = param.cu_2*(sqrt(abs(p_P-p_h2))*sign(p_P-p_h2)-sqrt(abs(p_h2-p_t))*sign(p_h2-p_t));
    Q_A = Leak1;
    Q_B = Leak2;
    Q_p = Q_A;

else
    Leak1  = param.cu_1*(sqrt(abs(p_P-p_h1))*sign(p_P-p_h1)-sqrt(abs(p_h1-p_t))*sign(p_h1-p_t));
    Leak2  = param.cu_2*(sqrt(abs(p_P-p_h2))*sign(p_P-p_h2)-sqrt(abs(p_h2-p_t))*sign(p_h2-p_t));
    Q_A = CV2*(U)*sqrt(abs(p_h1-p_t))*sign(p_h1-p_t)+Leak1;
    Q_B = CV1*(U)*sqrt(abs(p_P-p_h2))*sign(p_P-p_h2)+Leak2;
    Q_p = Q_B;
end
