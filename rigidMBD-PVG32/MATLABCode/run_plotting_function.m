%++++++++++++++++++++++++++++++++++++++++++
      %PLOTTING
%++++++++++++++++++++++++++++++++++++++++++


tEnd = load(filePath+"/"+"Time.mat");
% PVG32_old = load(filePath+"/"+ "PVG32_old.mat");
% PVG32_flow_basic = load(filePath+"/"+"PVG32_flow_basic.mat");
PVG32_flow_assymetricDB = load(filePath+"/"+ "PVG32_flow_assymetricDB.mat");
exp = load(filePath+"/"+"measurements.mat");

tEnd = tEnd.Time;
% x_PVG32_old   = PVG32_old.modelData;
% cyl_PVG32_old = PVG32_old.cylData;
% x_PVG32_flow_basic = PVG32_flow_basic.modelData;
% cyl_PVG32_flow_basic = PVG32_flow_basic.cylData;
x_PVG32_flow_assymetricDB = PVG32_flow_assymetricDB.modelData;
cyl_PVG32_flow_assymetricDB = PVG32_flow_assymetricDB.cylData;
y_meas = exp.measurements;

thisDir = fileparts(mfilename('fullpath'));
figDir  = fullfile(thisDir,'..','figures');

% ---- global figure defaults ----
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');

% ---- plotting parameters ----
basicColor = [0.6 0.8 1];
asymColor  = [0 0 1];
measColor  = [1 0 0];

Ts       = 1e-3;
nSteps   = numel(tEnd);
nStart   = 5000;
nSignals = 12;
tspan    = 0:Ts:Ts*(nSteps-nStart);

%% Figure # 9
width = 6;
Height= 5;

%Figure 1: s vs F_h
figure()
set(gcf,'PaperUnits','centimeters');
set(gcf,'PaperPosition',[2 2 width Height]);
set(gcf,'Renderer','painters');
hold on
fname = 'F_h';
xData = cyl_PVG32_flow_assymetricDB(1,nStart:nSteps);
yData = cyl_PVG32_flow_assymetricDB(7,nStart:nSteps);
plot(xData, yData, ...
     'Color', asymColor, 'LineWidth', LineWidth)
xlabel('Position \mathrm{[m]}')
ylabel('$F_{\mathrm{h_1}}\ \mathrm{[N]}$', 'Interpreter','latex')
xLim = [-0.06 0.06];
yLim = [-1800 1800];
xlim(xLim)
ylim(yLim)
xticks(linspace(xLim(1), xLim(2), 6))
yticks(linspace(yLim(1), yLim(2), 6))
ax = gca;
grid(ax,'on')
set(gca, 'FontSize',fontSize, 'LineWidth', LineWidth)
ax.GridAlpha = 0.12;
ax.GridColor = [0.6 0.6 0.6];
ax.GridLineStyle = '-';
ax.LineWidth = LineWidth;
box(ax,'on')
legend({'Brown-McPhee friction'}, ...
       'FontName','Times New Roman', ...
       'FontSize',fontSize, ...
       'Location', 'southeast');   
exportgraphics(gcf, fullfile(figDir, [fname, '.pdf']), ...
               'ContentType', 'vector', ...
               'BackgroundColor','none'); 


%% ---- Settings ----
fontSize  = 16;
LineWidth = 2;

width = 14;
Height= 12;
    

%% ---- Loop through signals ----
for i = 1:nSignals

    % ---- initialize figure ----
    figure();
    figure()
    set(gcf,'PaperUnits','centimeters');
    set(gcf,'PaperPosition',[2 2 width Height]);
    set(gcf,'Renderer','painters');
    hold on

    % ---- default empty signals ----
    yOld = [];
    yBasic = [];
    yAsym = [];
    ym = [];

    % ---- set signal data, labels, limits ----
    switch i
        case 1
            fname = 'z1_angle';
            yAsym = rad2deg(x_PVG32_flow_assymetricDB(i,nStart:nSteps));
            ym    = y_meas(i,nStart:nSteps);
            ylabelStr = '$z_1\ \mathrm{[^\circ]}$';
            yLim = [-11 61];
            legendEntries = {'Asymmetric DB','Experimental'};
        case 2
            fname = 'z2_angle';
            yAsym = rad2deg(x_PVG32_flow_assymetricDB(i,nStart:nSteps) + ...
                            x_PVG32_flow_assymetricDB(i-1,nStart:nSteps));
            ym    = y_meas(i,nStart:nSteps);
            ylabelStr = '$z_2\ \mathrm{[^\circ]}$';
            yLim = [-60 40];
            legendEntries = {'Asymmetric DB','Experimental'};
        case 3
            fname = 'z1_angular_velocity';
            yAsym = rad2deg(x_PVG32_flow_assymetricDB(i,nStart:nSteps));
            ym    = y_meas(i,nStart:nSteps);
            ylabelStr = '$\dot{z}_1\,\mathrm{[^\circ/s]}$';
            yLim = [-15 15];
            legendEntries = {'Asymmetric DB','Experimental'};
        case 4
            fname = 'z2_angular_velocity';
            yAsym = rad2deg(x_PVG32_flow_assymetricDB(i,nStart:nSteps) + ...
                            x_PVG32_flow_assymetricDB(i-1,nStart:nSteps));
            ym    = y_meas(i,nStart:nSteps);
            ylabelStr = '$\dot{z}_2\,\mathrm{[^\circ/s]}$';
            yLim = [-15 15];
            legendEntries = {'Asymmetric DB','Experimental'};
        case 5
            fname = 'p1_pressure';
            yAsym = x_PVG32_flow_assymetricDB(i,nStart:nSteps);
            ym    = y_meas(i,nStart:nSteps);
            ylabelStr = '$p_1\ \mathrm{[Pa]}$';
            yLim = [40e5 90e5];
            legendEntries = {'Asymmetric DB','Experimental'};
        case 6
            fname = 'p2_pressure';
            yAsym = x_PVG32_flow_assymetricDB(i,nStart:nSteps);
            ym    = y_meas(i,nStart:nSteps);
            ylabelStr = '$p_2\ \mathrm{[Pa]}$';
            yLim = [0e5 10e5];
            legendEntries = {'Asymmetric DB','Experimental'};
        case 7
            fname = 'p3_pressure';
            yAsym = x_PVG32_flow_assymetricDB(i,nStart:nSteps);
            ym    = y_meas(i,nStart:nSteps);
            ylabelStr = '$p_3\ \mathrm{[Pa]}$';
            yLim = [0e5 20e5];
            legendEntries = {'Asymmetric DB','Experimental'};
        case 8
            fname = 'p4_pressure';
            yAsym = x_PVG32_flow_assymetricDB(i,nStart:nSteps);
            ym    = y_meas(i,nStart:nSteps);
            ylabelStr = '$p_4\ \mathrm{[Pa]}$';
            yLim = [20e5 50e5];
            legendEntries = {'Asymmetric DB','Experimental'};
        case 9
            fname = 's1_position';
            yAsym = 1000*cyl_PVG32_flow_assymetricDB(i-8,nStart:nSteps);
            ym    = 1000*y_meas(i,nStart:nSteps);
            ylabelStr = '$s_1\ \mathrm{[mm]}$';
            yLim = [900 1500];
            legendEntries = {'Asymmetric DB','Experimental'};
        case 10
            fname = 's2_position';
            yAsym = 1000*cyl_PVG32_flow_assymetricDB(i-8,nStart:nSteps);
            ym    = 1000*y_meas(i,nStart:nSteps);
            ylabelStr = '$s_2\ \mathrm{[mm]}$';
            yLim = [1200 1350];
            legendEntries = {'Asymmetric DB','Experimental'};
        case 11
            fname = 'dots1_position';
            yAsym = 1000*cyl_PVG32_flow_assymetricDB(i-8,nStart:nSteps);
            ylabelStr = '$\dot{s}_1\ \mathrm{[mm/s]}$';
            yLim = [-60 30];
            legendEntries = {'Asymmetric DB'};
        case 12
            fname = 'dots2_position';
            yAsym = 1000*cyl_PVG32_flow_assymetricDB(i-8,nStart:nSteps);
            ylabelStr = '$\dot{s}_2\ \mathrm{[mm/s]}$';
            yLim = [-60 30];
            legendEntries = {'Asymmetric DB'};
    end

    % ---- plot signals ----
    if ~isempty(yOld)
        plot(tspan, yOld,'r','LineWidth',LineWidth);
    end
    if ~isempty(yBasic)
        plot(tspan, yBasic,'Color',basicColor,'LineWidth',LineWidth);
    end
    if ~isempty(yAsym)
        plot(tspan, yAsym,'Color',asymColor,'LineWidth',LineWidth);
    end
    if ~isempty(ym)
        plot(tspan, ym,'Color',measColor,'LineWidth',0.8);
    end

    % ---- axes labels, limits, grid ----
    if i == 13 || i == 14
        xlabel('Velocity [m/s]');
        xLim = [-0.1 0.1];
    else
        xlabel('Time [s]');
        xLim = [0 tEnd(end)-5];
    end

    ylabel(ylabelStr,'Interpreter','latex');
    xlim(xLim);
    ylim(yLim);
    xticks(linspace(xLim(1),xLim(2),6));
    yticks(linspace(yLim(1),yLim(2),6));
    grid on;
    box on;
    set(gca,'FontSize',fontSize,'LineWidth',LineWidth);
    legend(legendEntries,'FontName','Times New Roman','FontSize',fontSize);

    % matlab2tikz(fullfile(figDir,[fname,'.tex']), ...
    %             'width','6cm','height','4cm', ...
    %             'standalone',false, ...
    %             'parseStrings',false, ...
    %             'showInfo',false);

    exportgraphics(gcf, fullfile(figDir, [fname, '.pdf']), ...
               'ContentType', 'vector', ...
               'BackgroundColor','none');
end



%Figure 2: dots vs F_mu
figure()
set(gcf,'PaperUnits','centimeters');
set(gcf,'PaperPosition',[2 2 width Height]);
set(gcf,'Renderer','painters');
hold on
fname = 'F_mu';
xData = cyl_PVG32_flow_assymetricDB(3,nStart:nSteps);
yData = cyl_PVG32_flow_assymetricDB(5,nStart:nSteps);
plot(xData, yData, ...
     'Color', asymColor, 'LineWidth', LineWidth)
xlabel('Velocity [m/s]')
ylabel('$F_{\mathrm{\mu_1}}\ \mathrm{[N]}$', 'Interpreter','latex')
xLim = [-0.06 0.06];
yLim = [-1800 1800];
xlim(xLim)
ylim(yLim)
xticks(linspace(xLim(1), xLim(2), 6))
yticks(linspace(yLim(1), yLim(2), 6))
ax = gca;
grid(ax,'on')
set(gca, 'FontSize',fontSize, 'LineWidth', LineWidth)
ax.GridAlpha = 0.12;
ax.GridColor = [0.6 0.6 0.6];
ax.GridLineStyle = '-';
ax.LineWidth = LineWidth;
box(ax,'on')
legend({'Brown-McPhee friction'}, ...
       'FontName','Times New Roman', ...
       'FontSize',fontSize, ...
       'Location', 'southeast');   
exportgraphics(gcf, fullfile(figDir, [fname, '.pdf']), ...
               'ContentType', 'vector', ...
               'BackgroundColor','none'); 

% T = table(xData(:), yData(:), 'VariableNames', {'x','y'});
% csvFileName = fullfile(figDir, [fname,'.csv']);
% writetable(T, csvFileName, 'Delimiter',',');
% 
% 
% matlab2tikz(fullfile(figDir,[fname,'.tex']), ...
%     'externalData', true, ...
%     'width', '\figurewidth', ...
%     'height', '\figureheight', ...
%     'standalone', false, ...
%     'parseStrings', false, ...
%     'showInfo', false);


