
addpath('./data/');
addpath('./functions/');
addpath('../figures/');
addpath('./hydraulicsLib/');

%One change

[param]     = Get_parameters_Patu();
tEnd        = 32;

filePath    = "data";
exp_data    = filePath +"/" + "measurements.mat";  

data        = importdata(exp_data);

for i = 1:numel(param.valve_models)
    x = [];
    z = [];
    [Time, x, z] = true_hydraulics_PATU(data, tEnd, i );

    modelName = param.valve_models(i);
    modelData = x;
    cylData = z;
    fullFileName = fullfile(filePath, modelName + ".mat");
    save(fullFileName, "modelData", "cylData");

end

save(fullfile(filePath, "Time.mat"), "Time");

tEnd = load(filePath+"/"+"Time.mat");
PVG32_old = load(filePath+"/"+ "PVG32_old.mat");
PVG32_flow_basic = load(filePath+"/"+"PVG32_flow_basic.mat");
PVG32_flow_assymetricDB = load(filePath+"/"+ "PVG32_flow_assymetricDB.mat");
exp = load(filePath+"/"+"measurements.mat");

tEnd = tEnd.Time;
x_PVG32_old   = PVG32_old.modelData;
cyl_PVG32_old = PVG32_old.cylData;
x_PVG32_flow_basic = PVG32_flow_basic.modelData;
cyl_PVG32_flow_basic = PVG32_flow_basic.cylData;
x_PVG32_flow_assymetricDB = PVG32_flow_assymetricDB.modelData;
cyl_PVG32_flow_assymetricDB = PVG32_flow_assymetricDB.cylData;
y_meas = exp.measurements;

%++++++++++++++++++++++++++++++++++++++++++
      %PLOTTING
%++++++++++++++++++++++++++++++++++++++++++

fontSize  = 10;
LineWidth = 2;

thisDir = fileparts(mfilename('fullpath'));
figDir  = fullfile(thisDir,'..','figures');

 % ---- global figure defaults (set ONCE) ----
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

% ---- plotting parameters ----
basicColor = [0.6 0.8 1];

asymColor  = [0.00 0.45 0.20];
measColor  = [0 0.27   0.55];

Ts       = 1e-3;
nSteps   = numel(tEnd);
nStart   = 5000;
nSignals = 16;
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
            fname  = 'z1_angle';
            %yOld   = rad2deg(x_PVG32_old(i,nStart:nSteps));
            % yBasic = rad2deg(x_PVG32_flow_basic(i,:));
            yAsym  = rad2deg(x_PVG32_flow_assymetricDB(i,nStart:nSteps));
            ym  = y_meas(i,nStart:nSteps);
            ylabelStr = '$z_1\ \mathrm{[deg]}$';
            yLim = [-5 55];
            % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             % legendEntries = {'Old model','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};

        case 2
            fname  = 'z2_angle';
            % yOld   = rad2deg(x_PVG32_old(i,nStart:nSteps) + x_PVG32_old(i-1,nStart:nSteps));
            % yBasic = rad2deg(x_PVG32_flow_basic(i,:) + x_PVG32_flow_basic(i-1,:));
            yAsym  = rad2deg(x_PVG32_flow_assymetricDB(i,nStart:nSteps) + x_PVG32_flow_assymetricDB(i-1,nStart:nSteps));
            ym  = y_meas(i,nStart:nSteps);
            ylabelStr = '$z_2\ \mathrm{[deg]}$';
            yLim = [-60 36];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             % legendEntries = {'Old model','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};

        case 3
            fname  = 'z1_angular_velocity';
            % yOld   = rad2deg(x_PVG32_old(i,nStart:nSteps));
            % yBasic = rad2deg(x_PVG32_flow_basic(i,:));
            yAsym  = rad2deg(x_PVG32_flow_assymetricDB(i,nStart:nSteps));
            ym  = y_meas(i,nStart:nSteps);
            ylabelStr =  '$\dot{z}_1\ \mathrm{[deg/s]}$';
            yLim = [-18 9];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};
             %legendEntries = {'Old model','Asymmetric DB','Experimental'};
        case 4
            fname  = 'z2_angular_velocity';
            % yOld   = rad2deg(x_PVG32_old(i,nStart:nSteps) + x_PVG32_old(i-1,nStart:nSteps));
            % yBasic = rad2deg(x_PVG32_flow_basic(i,:) + x_PVG32_flow_basic(i-1,:));
            yAsym  = rad2deg(x_PVG32_flow_assymetricDB(i,nStart:nSteps) + x_PVG32_flow_assymetricDB(i-1,nStart:nSteps));
            ym  = y_meas(i,nStart:nSteps);
            ylabelStr =  '$\dot{z}_2\ \mathrm{[deg/s]}$';
            yLim = [-18 9];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};
             %legendEntries = {'Old model','Asymmetric DB','Experimental'};
        case 5
            fname  = 'p1_pressure';
            %yOld   = x_PVG32_old(i,nStart:nSteps);
            % yBasic = x_PVG32_flow_basic(i,:);
            yAsym  = x_PVG32_flow_assymetricDB(i,nStart:nSteps);
            ym  = y_meas(i,nStart:nSteps);
            ylabelStr = '$p_1\ \mathrm{[Pa]}$';
            yLim = [40e5 90e5];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};
             %legendEntries = {'Old model','Asymmetric DB','Experimental'};
        case 6
            fname  = 'p2_pressure';
            %yOld   = x_PVG32_old(i,nStart:nSteps);
            % yBasic = x_PVG32_flow_basic(i,:);
            yAsym  = x_PVG32_flow_assymetricDB(i,nStart:nSteps);
            ym  = y_meas(i,nStart:nSteps);
            ylabelStr = '$p_2\ \mathrm{[Pa]}$';
            yLim = [-10e5 40e5];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};
             %legendEntries = {'Old model','Asymmetric DB','Experimental'};
        case 7
            fname  = 'p3_pressure';
            %yOld   = x_PVG32_old(i,nStart:nSteps);
            % yBasic = x_PVG32_flow_basic(i,:);
            yAsym  = x_PVG32_flow_assymetricDB(i,nStart:nSteps);
            ym  = y_meas(i,nStart:nSteps);
            ylabelStr = '$p_3\ \mathrm{[Pa]}$';
            yLim = [0e5 15e5];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};
             %legendEntries = {'Old model','Asymmetric DB','Experimental'};
        case 8
            fname  = 'p4_pressure';
            %yOld   = x_PVG32_old(i,nStart:nSteps);
            % yBasic = x_PVG32_flow_basic(i,:);
            yAsym  = x_PVG32_flow_assymetricDB(i,nStart:nSteps);
            ym  = y_meas(i,nStart:nSteps);
            ylabelStr = '$p_4\ \mathrm{[Pa]}$';
            yLim = [25e5 40e5];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};
             %legendEntries = {'Old model','Asymmetric DB','Experimental'};
        case 9
            fname  = 's1_position';
            %yOld   = 1000*cyl_PVG32_old(i-8,nStart:nSteps);
            % yBasic = 1000*cyl_PVG32_flow_basic(i-8,:);
            yAsym  = 1000*cyl_PVG32_flow_assymetricDB(i-8,nStart:nSteps);
            ym  = 1000*y_meas(i,nStart:nSteps);
            ylabelStr = '$s_1\ \mathrm{[mm]}$';
            yLim = [900 1250];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};
             %legendEntries = {'Old model','Asymmetric DB','Experimental'};
        case 10
            fname  = 's2_position';
            %yOld   = 1000*cyl_PVG32_old(i-8,nStart:nSteps);
            % yBasic = 1000*cyl_PVG32_flow_basic(i-8,:);
            yAsym  = 1000*cyl_PVG32_flow_assymetricDB(i-8,nStart:nSteps);
            ym  = 1000*y_meas(i,nStart:nSteps);
            ylabelStr = '$s_2\ \mathrm{[mm]}$';
            yLim = [1200 1350];
             % legendEntries = {'Old model','Basic flow','Asymmetric DB','Experimental'};
             legendEntries = {'Asymmetric DB','Experimental'};
             %legendEntries = {'Old model','Asymmetric DB','Experimental'};
        case 11
            fname  = 'dots1_position';
            yOld   = 1000*cyl_PVG32_old(i-8,nStart:nSteps);
            % yBasic = 1000*cyl_PVG32_flow_basic(i-8,:);
            yAsym  = 1000*cyl_PVG32_flow_assymetricDB(i-8,nStart:nSteps);
            ylabelStr = '$\dot{s}_1\ \mathrm{[mm/s]}$';
            yLim = [-65 20];
             legendEntries = {'Old model','Asymmetric DB'};
        case 12
            fname  = 'dots2_position';
            yOld   = 1000*cyl_PVG32_old(i-8,nStart:nSteps);
            % yBasic = 1000*cyl_PVG32_flow_basic(i-8,:);
            yAsym  = 1000*cyl_PVG32_flow_assymetricDB(i-8,nStart:nSteps);
            ylabelStr = '$\dot{s}_2\ \mathrm{[mm/s]}$';
            yLim = [-60 30];
            legendEntries = {'Old model','Asymmetric DB'};

        case 13
            fname  = 'U1_control signal';
            ym  = y_meas(i-2,nStart:nSteps);
            ylabelStr = '$U_1\ \mathrm{[V]}$';
            yLim = [3.5 7.5];
            legendEntries = {'Experimental'};

        case 14
            fname  = 'U2_control signal';
            ym  = y_meas(i-2,nStart:nSteps);
            ylabelStr = '$U_2\ \mathrm{[V]}$';
            yLim = [3.5 7.5];
            legendEntries = {'Experimental'};
        case 15
            fname  = 'pP_pressure';
            ym  = y_meas(i-2,nStart:nSteps);
            ylabelStr = '$p_P\ \mathrm{[Pa]}$';
            yLim = [88e5 96e5];
            legendEntries = {'Experimental'};
        case 16
            fname  = 'pT_pressure';
            ym  = y_meas(i-2,nStart:nSteps);
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
    xticks(linspace(0, tspan(end-1), 6))
    grid on
    grid minor
     
    set(gca, 'LineWidth', LineWidth)

    % After you finish plotting and setting limits:
    box(gca, 'on');                    % draw top+right borders as well

    legend(legendEntries, 'FontName','Times New Roman','FontSize',fontSize)

    % ---- save figure ----
    exportgraphics(gcf, fullfile(figDir, fname + ".pdf"), 'ContentType','vector')
end
