clear all

%% Set parameters for the bodies
param.g     = 9.8066;       % Gravitational acceleration
nb          = 2  ;          % Number of bodies
dt          = 0.05;         % Time step
tini        = 0;            % Initial time
tend        = 25;           % End time
tspan       = tini:dt:tend; % Time span
x           = zeros(6*nb, numel(tspan)); % This is the vector containining [qA, qB, dqA,dqB]


% Body A
param.m_A   = 1;            %Mass
param.L_A   = 1;            %Length
theta_A     = 315*(pi/180); % Initial angle
param.Rx_A  = (param.L_A/2)*cos(theta_A);  % Initial X-position of local CS
param.RY_A  = (param.L_A/2)*sin(theta_A);            % Initial Y-position of local CS

% Body B
param.m_B   = 1;            %Mass
param.L_B   = 1;            %Length
theta_B     = 45*(pi/180); % Initial angle
param.Rx_B  = param.L_A*cos(theta_A)+ (param.L_B/2)*cos(theta_B);  % Initial X-position of local CS
param.RY_B  = param.L_A*sin(theta_A)+ (param.L_B/2)*sin(theta_B);            % Initial Y-position of local CS


%Initial coordinates
q0          = [param.Rx_A;param.RY_A; theta_A;param.Rx_B;param.RY_B; theta_B];
dq0         =  [0;0;0;0;0;0];
x(:,1)      = [q0;dq0];

 

%% Equation of motion

function y = eom(t,x,param)

M_A         = [param.m_A, 0,0;
                0,param.m_A, 0;
                0, 0, (1/12)*(param.m_A)*(param.L_A)^2];
Qe_A        = [0 -param.m_A*param.g 0];

M_B         = [param.m_B, 0,0;
                0,param.m_B, 0;
                0, 0, (1/12)*(param.m_B)*(param.L_B)^2];
Qe_B        = [sin(t) -param.m_B*param.g 0];

%Step 01: Find mass matrix, M.
M           = blkdiag(M_A, M_B);

%Step 02: Find jacobian matrix, Cq.
Cq = [1, 0,  (param.L_A/2)*sin(x(3)),       0,  0,                        0;
           0, 1, -(param.L_A/2)*cos(x(3)),       0,  0,                        0;
           1, 0, -(param.L_A/2)*sin(x(3)),      -1,  0, -(param.L_B/2)*sin(x(6));
           0, 1,  (param.L_A/2)*cos(x(3)),       0, -1,  (param.L_B/2)*cos(x(6));
           0, 0,  0,                              0,  1,  (param.L_B/2)*cos(x(6))];

% Step 03: Find force vector, Qe.
Qe = [Qe_A Qe_B]';

% Step 04: Find force vector, Qv.
Qv = zeros(6,1);


%Step 05: Find force vector, Qc.
Qc = [-x(9)^2*(param.L_A/2)*cos(x(3));
              -x(9)^2*(param.L_A/2)*sin(x(3));
              x(9)^2*(param.L_A/2)*cos(x(3))+x(12)^2*(param.L_B/2)*cos(x(6));
              x(9)^2*(param.L_A/2)*sin(x(3))+x(12)^2*(param.L_B/2)*sin(x(6));
              x(12)^2*(param.L_B/2)*sin(x(6))];

sysM    = [M Cq';Cq  zeros(size(Cq, 1),size(Cq, 1))];

sysQ    =  [Qe+Qv;Qc];



dx = sysM\sysQ;

y(1:6)  = x(7:12); % Velocity from the previous step
y(7:12) = dx(1:6);    % Acceleration (1:6). Lagrange multipliers (7:11)
y       = y(:);
end 



% Integrator
[t, x] = ode45(@(t,x) eom(t,x,param), tspan, x(:,1));


%% Animation

figure; hold on; grid on; axis equal
xlabel('X'); ylabel('Y'); title('Dynamic analysis')

% draw limits
axis([min([x(:,1); x(:,4)])-2 max([x(:,1); x(:,4)])+2  min([x(:,2); x(:,5)])-2 max([x(:,2); x(:,5)])+2]);

% two link graphics
hA = plot([0 0],[0 0],'b-','LineWidth',3);   % Body A
hB = plot([0 0],[0 0],'r-','LineWidth',3);   % Body B

for k = 1:length(t)

    % Body A endpoints
    ALx = x(k,1) - (param.L_A/2)*cos(x(k,3));
    ALy = x(k,2) - (param.L_A/2)*sin(x(k,3));
    ARx = x(k,1) + (param.L_A/2)*cos(x(k,3));
    ARy = x(k,2) + (param.L_A/2)*sin(x(k,3));

    % Body B endpoints
    BLx = x(k,4) - (param.L_B/2)*cos(x(k,6));
    BLy = x(k,5) - (param.L_B/2)*sin(x(k,6));
    BRx = x(k,4) + (param.L_B/2)*cos(x(k,6));
    BRy = x(k,5) + (param.L_B/2)*sin(x(k,6));

    % Update links
    set(hA,'XData',[ALx ARx],'YData',[ALy ARy]);
    set(hB,'XData',[BLx BRx],'YData',[BLy BRy]);

    drawnow;
end
