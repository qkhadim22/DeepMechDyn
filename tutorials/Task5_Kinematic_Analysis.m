clear all
%% Set parameters for the body

param.L     = 1;            %Length
param.Rx_A  = param.L/2;    % Initial X-position of local CS
param.RY_A  = 0;            % Initial Y-position of local CS
theta_A     = 315*(pi/180); % Initial angle

%Initial coordinates
q0          = [param.Rx_A;param.RY_A; theta_A];
dq0         =  [0;0;0];
ddq0        =  [0;0;0];

%% Step 01: Formulate the constraint equations vector and jacobian matrix


C = @(q,t) [q(1) - (param.L/2)*cos(q(3)) - sin(t)/4 + 0.5;
            q(2) - (param.L/2)*sin(q(3));
            q(1) + (param.L/2)*cos(q(3))];


Cq =@(q,t) [1, 0,  (param.L/2) * sin(q(3));
            0, 1, -(param.L/2) * cos(q(3));
            1, 0, -(param.L/2) * sin(q(3))];

% Required for velocity analysis
Ct = @(t) [-1/4 * cos(t);
                0;
                0];

% Required for acceleration analysis
Ctt = @(t) [1/4 * sin(t);
                0;
                0];

Cqq = @(q,dq) [(param.L/2)*cos(q(3)) * dq(3)^2;
     (param.L/2)*sin(q(3)) * dq(3)^2;
    -(param.L/2)*cos(q(3)) * dq(3)^2];

Cqt = [0; 0; 0];

%% Step 02: Position Analysis

% Solver settings
maxiter     = 50;          % maximum Newton iterations per time step
eps_tol     = 1e-10;       % convergence tolerance on state correction ||Δq||

cnt         = 0;            % Counter
dt          = 0.05;         % Time step
tini        = 0;            % Initial time
tend        = 25;           % End time
q           = zeros(3,1);
dq          = zeros(3,1);
ddq          = zeros(3,1);

q(:,1)      = q0;
dq(:,1)     = dq0;
ddq(:,1)    = ddq0;
tsim(1)     = tini;


% Time stepping and Newton–Raphson iterations
for t = tini:dt:tend

    % Column counter to form the result matrices
    cnt = cnt + 1;

    nloop    = 0;      % iteration count in this time step
    nconverg = 0;      % successful convergence counter (optional/stat)

    % Start from previous converged configuration (or q0 for cnt==1)
    if cnt == 1
        qk = q0(:);
    else
        qk = q(:,cnt-1);
    end

    % Newton loop
    while nloop < maxiter
        % Evaluate constraints C(q,t) and Jacobian Cq(q)

        Ck   = C(qk, t);
        Cqk  = Cq(qk,t);
        

        % Newton difference: solve Cq * Δq = -C
        % Use backslash; fall back to pinv if ill-conditioned
        % (guards against singular/near-singular Jacobian)
        try
            deltq = -Cqk \ Ck;
        catch
            deltq = -pinv(Cqk) * Ck;
        end

        % Coordinate update
        qk = qk + deltq;

        % Iteration bookkeeping
        nloop    = nloop + 1;
        nconverg = nconverg + 1;

        % Convergence criteria (||Δq||_inf < eps)
        maxdx = max(abs(deltq));
        if maxdx < eps_tol
            break
        end
    end

    % Store converged configuration at this time step
    q(:,cnt)   = qk;

    %% Step 03: Velocity Analysis
    Cqk  = Cq(qk,t);
    Ctk  = Ct(t);    

    dq(:,cnt)  = Cqk \ (-Ctk);   % update with your velocity solver

     %% Step 04: Acceleration Analysis
     Cttk       = Ctt(t);
     Cqqqk      = Cqq(q(:,cnt), dq(:,cnt));

    ddq(:,cnt)  = Cqk \ ( -Cttk - Cqqqk - Cqt );  % update with your acceleration solver


    tsim(cnt)  = t;

    % (Optional) warn if not converged
    if nloop >= maxiter
        warning('Newton did not converge at t = %.6f (||Δq||_inf = %.3e)', t, maxdx);
    end
end




%% Animation

figure; hold on; grid on;
axis equal
xlabel('X'); ylabel('Y');
title('Mechanism Animation');

% Set reasonable axis limits:
x_min = min(q(1,:)) - param.L;
x_max = max(q(1,:)) + param.L;
y_min = min(q(2,:)) - param.L;
y_max = max(q(2,:)) + param.L;
axis([x_min x_max y_min y_max]);

h_link = plot([0 0],[0 0],'LineWidth',3);     % bar graphic
h_point = plot(0,0,'ro','MarkerSize',6,'MarkerFaceColor','r');

for k = 1:length(tsim)

    x = q(1,k);
    y = q(2,k);
    th = q(3,k);

    % Endpoints of the rigid body
    Ax = x - (param.L/2)*cos(th);
    Ay = y - (param.L/2)*sin(th);

    Px = x + (param.L/2)*cos(th);
    Py = y + (param.L/2)*sin(th);

    % Update bar and center point
    set(h_link,'XData',[Ax Px],'YData',[Ay Py]);
    set(h_point,'XData',x,'YData',y);

    drawnow;
end