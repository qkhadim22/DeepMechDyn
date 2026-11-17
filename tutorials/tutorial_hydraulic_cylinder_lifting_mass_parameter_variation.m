clear all

%% Path to shared_tools functions

addpath('../shared_tools/hydraulicsLib/');
addpath('../shared_tools/integrators/');
addpath('../rigidMBD-PVG32/MATLABCode/data/');


%% Lift mass parameters

% Physical paramters
paramVar       = [900e2;900e3;900e4;900e6]; % U_N parameter
results        = cell(length(paramVar),1);
paramList      = cell(length(paramVar),1);

% load data
filePath       = "../rigidMBD-PVG32/MATLABCode/data";
exp_data       = filePath +"/" + "measurements.mat";  
data           = importdata(exp_data);

%% Simulation parameters

Ts             = 1e-3;                                  %Time step
tend           = 14;                                    %End time
tspan          = 0:Ts:tend;                             %Time span
nSteps         = length(tspan);                         % Number of steps
optionalVec    = zeros(3,nSteps);
 U             = data(11,1:nSteps);
 p_P           = data(13,1:nSteps);

% Initial parameters
s_0            = 0.9430;                                %Initial actuator position
ds_0           = 0;                                     %Initial actuator velocity
p_cyl_0        = 6.5936e+06;                            %Piston side pressure
p_rod_0        = 6.7956e+04;                            %Piston-rod side pressure
u_0            = 5.8 ;                                     %Initial control signal
x_0            = [s_0;0;p_cyl_0;p_rod_0;u_0];           %Initial configuartion


%% Calling parameter variation function

for i = 1:length(paramVar)

    % parameters to change
    param      = Get_cylinder_parameters();
    param.BeO  = paramVar(i);
   
    % simulation parameters
    optionalVec= [U; p_P; param.p_t*ones(1,nSteps)];


    results{i} = runHydraulicSim(param, x_0,Ts, tspan, optionalVec);
    paramList{i} = paramVar(i); 
end


%% Solution with parameter variation function
function x = runHydraulicSim(param,x_0,Ts, tspan, optionalVec)

    nSteps = length(tspan);
    nStates = length(x_0);
    x       = zeros(nStates,nSteps);
    x(:,1)  = x_0;

    % Time stepping
    for k = 2:nSteps

        x(:,k) = Rk4(@ode_double_acting_hydraulics, Ts, tspan(k), x(:,k-1), optionalVec(:,k-1),param);
    end

    % ode_double_acting_hydraulics
    % input : actuator position, velocity, ph1, ph2, U (at the step k-1)
    % output: actuator position, velocity, ph1, ph2, U (at the step k)


end

%% Plotting
fontSize  = 10;
LineWidth = 2;

set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

% ---- plotting parameters ----
basicColor = [0.6 0.8 1];

asymColor  = [0.00 0.45 0.20];
measColor  = [0 0.27   0.55];
nStart   = 5000;
nSignals = 5;
tspan    = 0:Ts:Ts*(nSteps-nStart);    

nCases = length(results);
colors = lines(nCases);   % automatic colors for simulations

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
    ym     = [];

    legendEntries = {};

    % ---- set measurement + labels ----
    switch i
        case 1
            fname  = 's1_position';
            ym  = 1000*data(9,nStart:nSteps);
            ylabelStr = '$s_1\ \mathrm{[mm]}$';
            yLim = [900 1250];

        case 2
            fname  = 'dots1_position';
            ylabelStr = '$\dot{s}_1\ \mathrm{[mm/s]}$';
            yLim = [-65 20];

        case 3
            fname  = 'p1_pressure';
            ym  = data(5,nStart:nSteps);
            ylabelStr = '$p_1\ \mathrm{[Pa]}$';
            yLim = [40e5 90e5];

        case 4
            fname  = 'p2_pressure';
            ym  = data(6,nStart:nSteps);
            ylabelStr = '$p_2\ \mathrm{[Pa]}$';
            yLim = [-10e5 40e5];

        case 5
            fname  = 'U1_control signal';
            ym  = data(11,nStart:nSteps);
            ylabelStr = '$U_1\ \mathrm{[V]}$';
            yLim = [3.5 7.5];

        case 6
            fname  = 'pP_pressure';
            ym  = data(13,nStart:nSteps);
            ylabelStr = '$p_P\ \mathrm{[Pa]}$';
            yLim = [88e5 96e5];

        case 7
            fname  = 'pT_pressure';
            ym  = data(14,nStart:nSteps);
            ylabelStr = '$p_T\ \mathrm{[Pa]}$';
            yLim = [0.80e5 1.2e5];
    end


    % ==========================================================
    % ---- Plot ALL simulation results -------------------------
    % ==========================================================
    for kCase = 1:nCases

        xCase = results{kCase};

        switch i
            case 1
                yModel = 1000*xCase(i,nStart:nSteps);

            case 2
                yModel = 1000*xCase(i,nStart:nSteps);

            case 3
                yModel = xCase(i,nStart:nSteps);

            case 4
                yModel = xCase(i,nStart:nSteps);

            otherwise
                yModel = [];   % no simulation signal for cases 5–7
        end

        if isempty(yModel) == 0
            plot(tspan, yModel, ...
                'Color', colors(kCase,:), ...
                'LineWidth', LineWidth)

            % ---- Legend using parameter values ----
            if exist('paramVar','var')
                legendEntries{end+1} = ['U_N = ' num2str(paramVar(kCase))];
            else
                legendEntries{end+1} = ['Simulation ' num2str(kCase)];
            end
        end

    end


    % ==========================================================
    % ---- Plot Measurement -----------------------------------
    % ==========================================================
    if isempty(ym) == 0
        plot(tspan, ym, ...
            'Color', measColor, ...
            'LineWidth', LineWidth, ...
            'LineStyle','--')

        legendEntries{end+1} = 'Experimental';
    end


    % ==========================================================
    % ---- Axes + Styling -------------------------------------
    % ==========================================================
    xlabel('Time [s]')
    xLim = [0 35];

    ylabel(ylabelStr, 'Interpreter','latex')

    ylim(yLim)
    xlim(xLim)

    yticks(linspace(yLim(1), yLim(2), 6))
    xticks(linspace(0, tspan(end-1), 6))

    grid on
    grid minor

    set(gca, 'LineWidth', LineWidth)
    box(gca,'on')

    legend(legendEntries, ...
        'FontName','Times New Roman', ...
        'FontSize',fontSize)

end
