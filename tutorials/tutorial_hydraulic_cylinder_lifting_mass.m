clear all

%% Path to shared_tools functions

addpath('../shared_tools/hydraulicsLib/');
addpath('../shared_tools/integrators/');
addpath('../rigidMBD-PVG32/MATLABCode/data/');


%% Lift mass parameters

% Physical paramters
param          = Get_cylinder_parameters();

% load data
filePath       = "../rigidMBD-PVG32/MATLABCode/data";
exp_data       = filePath +"/" + "measurements.mat";  
data           = importdata(exp_data);

%% Simulation parameters

Ts             = 1e-3;                                  %Time step
tend           = 40;                                    %End time
tspan          = 0:Ts:tend;                             %Time span
nSteps         = length(tspan);                         % Number of steps
x              = zeros(5,nSteps);
optionalVec    = zeros(3,nSteps);
 U             = data(11,1:nSteps);
 p_P           = data(13,1:nSteps);

% Initial parameters
s_0            = 0.9430;                                %Initial actuator position
ds_0           = 0;                                     %Initial actuator velocity
p_A_0          = 6.5936e+06;                            %Piston side pressure
p_B_0          = 6.7956e+04;                            %Piston-rod side pressure
u_0            = 5.8 ;                                     %Initial control signal
x(:,1)         = [s_0;0;p_A_0;p_B_0;u_0];           %Initial configuartion

% Solution
for k = 2:nSteps
    optionalVec(:,k-1) = [U(:,k-1);p_P(:,k-1);param.p_t];
    x(:,k)             = Rk4(@ode_double_acting_hydraulics,Ts, tspan(k),x(:,k-1),optionalVec(:,k-1), param );

    %ode_double_acting_hydraulics
    % input : actuator position, velocity, ph1, ph2, U (at the step k-1)
    % output: actuator position, velocity, ph1, ph2, U (at the step k)

end 


%% Plotting
fontSize  = 10;
LineWidth = 2;

 % ---- global figure defaults (set ONCE) ----
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

% ---- plotting parameters ----
basicColor = [0.6 0.8 1];

asymColor  = [0.00 0.45 0.20];
measColor  = [0 0.27   0.55];
nStart   = 5000;
nSignals = 5;
tspan    = 0:Ts:Ts*(nSteps-nStart);    

for i = 1:nSignals

    % ---- initialize figure ----
    figure()
    set(gcf, 'PaperUnits', 'centimeters');
    set(gcf, 'PaperPosition', [0 0 10 8]);
    set(gcf, 'Renderer','painters');
    hold on

    % ---- default empty signals ----
    yOld   = [];
    yBasic = [];
    yAsym  = [];
    ym  = [];

    % ---- set signal data, labels, limits ----
    switch i
        case 1
           fname  = 's1_position';
            %yOld   = 1000*cyl_PVG32_old(i-8,nStart:nSteps);
            % yBasic = 1000*cyl_PVG32_flow_basic(i-8,:);
            yAsym  = 1000*x(i,nStart:nSteps);
            ym  = 1000*data(9,nStart:nSteps);
            ylabelStr = '$s_1\ \mathrm{[mm]}$';
            yLim = [900 1250];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};
             %legendEntries = {'Old model','Asymmetric DB','Experimental'};

        case 2
            fname  = 'dots1_position';
            yAsym   = 1000*x(i,nStart:nSteps);
            % yBasic = 1000*cyl_PVG32_flow_basic(i-8,:);
            % yAsym  = 1000*cyl_PVG32_flow_assymetricDB(i-8,nStart:nSteps);
            ylabelStr = '$\dot{s}_1\ \mathrm{[mm/s]}$';
            yLim = [-65 20];
             legendEntries = {'Asymmetric DB'};
        case 3
            fname  = 'p1_pressure';
            %yOld   = x_PVG32_old(i,nStart:nSteps);
            % yBasic = x_PVG32_flow_basic(i,:);
            yAsym  = x(i,nStart:nSteps);
            ym  = data(5,nStart:nSteps);
            ylabelStr = '$p_1\ \mathrm{[Pa]}$';
            yLim = [44e5 80e5];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};
             %legendEntries = {'Old model','Asymmetric DB','Experimental'};
        case 4
            fname  = 'p2_pressure';
            %yOld   = x_PVG32_old(i,nStart:nSteps);
            % yBasic = x_PVG32_flow_basic(i,:);
            yAsym  = x(i,nStart:nSteps);
            ym  = data(6,nStart:nSteps);
            ylabelStr = '$p_2\ \mathrm{[Pa]}$';
            yLim = [0e5 5e5];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};
             %legendEntries = {'Old model','Asymmetric DB','Experimental'};
      
        case 5
            fname  = 'U1_control signal';
            ym  = data(11,nStart:nSteps);
            ylabelStr = '$U_1\ \mathrm{[V]}$';
            yLim = [3.5 7.5];
            legendEntries = {'Experimental'};

        case 6
            fname  = 'pP_pressure';
            ym  = data(13,nStart:nSteps);
            ylabelStr = '$p_P\ \mathrm{[Pa]}$';
            yLim = [88e5 96e5];
            legendEntries = {'Experimental'};
        case 7
            fname  = 'pT_pressure';
            ym  = data(14,nStart:nSteps);
            ylabelStr = '$p_T\ \mathrm{[Pa]}$';
            yLim = [0.80e5 1.2e5];
            legendEntries = {'Experimental'};
    end

    % ---- plotting all available signals ---
    if isempty(yOld) == 0
        plot(tspan, yOld, 'Color', 'r', 'LineWidth', LineWidth)
    end 
    hold on
    if isempty(yBasic) == 0
        plot(tspan, yBasic, 'Color', basicColor, 'LineWidth', LineWidth)
    end
    hold on
    if isempty(yAsym) == 0
        
        plot(tspan, yAsym, 'Color', asymColor, 'LineWidth', LineWidth)
    end 
    if  isempty(ym) == 0
         plot(tspan, ym, 'Color', measColor, 'LineWidth', LineWidth)
    end
    % ---- axes, labels, grid ----
    xlabel('Time [s]')
    xLim = [0 35];
    ylabel(ylabelStr, 'Interpreter','latex')
    ylim(yLim)
    xlim(xLim);
    yticks(linspace(yLim(1), yLim(2), 6))
    xticks(linspace(0, tspan(end), 6))
    grid on
    grid minor
     
    set(gca, 'LineWidth', LineWidth)

    % After you finish plotting and setting limits:
    box(gca, 'on');                    % draw top+right borders as well

    legend(legendEntries, 'FontName','Times New Roman','FontSize',fontSize)

end