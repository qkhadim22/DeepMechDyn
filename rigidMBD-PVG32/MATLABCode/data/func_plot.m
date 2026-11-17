function [plots] = func_plot(simData, modelNames, expData, plots, colors, tspan, nStart, nSteps, figDir,mode)
% ---- global figure defaults ----
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
for varIdx = 1:length(plots.names)
    figure('Units','centimeters','Position',[2 2 14 8],'Color','w');
    hold on
    legendHandles = [];
    legendLabels  = strings(0);
for k = 1:numel(modelNames)
switch plots.names(varIdx)
case  'Fmu1'
                x_data   = 1e-3*simData(modelNames(k)).Hycyls(3,nStart:nSteps);
                y_data   = simData(modelNames(k)).Hycyls(5,nStart:nSteps);
                 h       =  plot(x_data, y_data,'Color',colors(k,:),'LineWidth',2);
                legendHandles(end+1) = h;
                legendLabels(end+1)  = modelNames(k);
                T        = table(x_data(:), y_data(:), 'VariableNames', {'x','y'});
                csvFileName = fullfile(figDir,plots.names(varIdx) + "_" + modelNames(k) + "_" + mode + ".csv");
                writetable(T, csvFileName, 'Delimiter',',');
case  'Fmu2'
                x_data   = 1e-3*simData(modelNames(k)).Hycyls(4,nStart:nSteps);
                y_data   = simData(modelNames(k)).Hycyls(6,nStart:nSteps);
                h       =  plot(x_data, y_data,'Color',colors(k,:),'LineWidth',2);
                legendHandles(end+1) = h;
                legendLabels(end+1)  = modelNames(k);
                T        = table(x_data(:), y_data(:), 'VariableNames', {'x','y'});
                csvFileName = fullfile(figDir,plots.names(varIdx) + "_" + modelNames(k) + "_" + mode + ".csv");
                writetable(T, csvFileName, 'Delimiter',',');
case  'Be1'
               x_data   =  1e-5*simData(modelNames(k)).states(5,nStart:nSteps);
               y_data   = 1e-6*simData(modelNames(k)).Hycyls(9,nStart:nSteps);
               h       =  plot(x_data, y_data,'Color',colors(k,:),'LineWidth',2);
               legendHandles(end+1) = h;
               legendLabels(end+1)  = modelNames(k);
               T        = table(x_data(:), y_data(:), 'VariableNames', {'x','y'});
                csvFileName = fullfile(figDir,plots.names(varIdx) + "_" + modelNames(k) + "_" + mode + ".csv");
               writetable(T, csvFileName, 'Delimiter',',');
case 'Be2'
                 x_data =  1e-5*simData(modelNames(k)).states(6,nStart:nSteps);
                 y_data   = 1e-6*simData(modelNames(k)).Hycyls(10,nStart:nSteps);
                 h       =  plot(x_data, y_data,'Color',colors(k,:),'LineWidth',2);
                 legendHandles(end+1) = h;
                 legendLabels(end+1)  = modelNames(k);
                 T        = table(x_data(:), y_data(:), 'VariableNames', {'x','y'});
                csvFileName = fullfile(figDir,plots.names(varIdx) + "_" + modelNames(k) + "_" + mode + ".csv");
                 writetable(T, csvFileName, 'Delimiter',',');
case  {"z1", "dotz1"}
                 x_data   = tspan;
                 y_data    = rad2deg(simData(modelNames(k)).states(varIdx,nStart:nSteps));
                 h       =  plot(x_data, y_data,'Color',colors(k,:),'LineWidth',2);
                 legendHandles(end+1) = h;
                 legendLabels(end+1)  = modelNames(k);
                 T        = table(x_data(:), y_data(:), 'VariableNames', {'x','y'});
                csvFileName = fullfile(figDir,plots.names(varIdx) + "_" + modelNames(k) + "_" + mode + ".csv");
                 writetable(T, csvFileName, 'Delimiter',',');
                 hold on
                 hExp= plot(x_data, expData(varIdx,nStart:nSteps),'Color','k','LineWidth',2);
                 legendHandles(end+1) = hExp;
                 legendLabels(end+1)  = "Experimental";
case  { "z2", "dotz2"}
                 x_data   = tspan;
                 y_data    = rad2deg(simData(modelNames(k)).states(varIdx-1,nStart:nSteps))+...
                                    rad2deg(simData(modelNames(k)).states(varIdx,nStart:nSteps));
                 h       =  plot(x_data, y_data,'Color',colors(k,:),'LineWidth',2);
                 legendHandles(end+1) = h;
                 legendLabels(end+1)  = modelNames(k);
                 T        = table(x_data(:), y_data(:), 'VariableNames', {'x','y'});
                csvFileName = fullfile(figDir,plots.names(varIdx) + "_" + modelNames(k) + "_" + mode + ".csv");
                 writetable(T, csvFileName, 'Delimiter',',');
                 hold on
                 hExp= plot(x_data, expData(varIdx,nStart:nSteps),'Color','k','LineWidth',2);
                 legendHandles(end+1) = hExp;
                 legendLabels(end+1)  = "Experimental";
case  {"p1" "p2" "p3" "p4"}
            if k == 1                                   % write experimental data once, not per model
                Texp = table(x_data(:), expData(varIdx,nStart:nSteps).', 'VariableNames', {'x','y'});
                csvExp = fullfile(figDir, plots.names(varIdx) + "_experimental_" + mode + ".csv");
                writetable(Texp, csvExp, 'Delimiter',',');
            end
                 x_data   = tspan;
                 y_data    = (simData(modelNames(k)).states(varIdx,nStart:nSteps));
                 h       =  plot(x_data, y_data,'Color',colors(k,:),'LineWidth',2);
                 legendHandles(end+1) = h;
                 legendLabels(end+1)  = modelNames(k);
                 T        = table(x_data(:), y_data(:), 'VariableNames', {'x','y'});
                csvFileName = fullfile(figDir,plots.names(varIdx) + "_" + modelNames(k) + "_" + mode + ".csv");
                 writetable(T, csvFileName, 'Delimiter',',');
                 hold on
                 hExp= plot(x_data, expData(varIdx,nStart:nSteps),'Color','k','LineWidth',2);
                 legendHandles(end+1) = hExp;
                 legendLabels(end+1)  = "Experimental";
case  {"s1" "s2"}
                    x_data    = tspan;
                    y_data    = (simData(modelNames(k)).Hycyls(varIdx-8,nStart:nSteps));
                    h         =  plot(x_data, y_data,'Color',colors(k,:),'LineWidth',2);
                    legendHandles(end+1) = h;
                    legendLabels(end+1)  = modelNames(k);
                    T        = table(x_data(:), y_data(:), 'VariableNames', {'x','y'});
                    csvFileName = fullfile(figDir,plots.names(varIdx) + "_" + modelNames(k) + "_" + mode + ".csv");
                    writetable(T, csvFileName, 'Delimiter',',');
                    hold on
                    hExp= plot(x_data, 1000*expData(varIdx,nStart:nSteps),'Color','k','LineWidth',2);
                    legendHandles(end+1) = hExp;
                    legendLabels(end+1)  = "Experimental";
otherwise
                 x_data   = tspan;
                 y_data    = (simData(modelNames(k)).Hycyls(varIdx-8,nStart:nSteps));
                 h       =  plot(x_data, y_data,'Color',colors(k,:),'LineWidth',2);
                legendHandles(end+1) = h;
                legendLabels(end+1)  = modelNames(k);
                T        = table(x_data(:), y_data(:), 'VariableNames', {'x','y'});
                csvFileName = fullfile(figDir,plots.names(varIdx) + "_" + modelNames(k) + "_" + mode + ".csv");
                writetable(T, csvFileName, 'Delimiter',',');
end
end
    % xLim = (plots.xlim{varIdx});
    % yLim = (plots.ylim{varIdx});
    axis tight;                                              % CHANGED: limits from the plotted data, so any load case works
    xLim = xlim;                                             % CHANGED: x axis = exact data range
    yLim = ylim;                                             % CHANGED
    if diff(yLim) == 0, yLim = yLim + [-1 1]; end            % CHANGED: flat signal -> give the axis some height
    yLim = yLim + [-1 1]*0.05*diff(yLim);                    % CHANGED: 5 % margin above and below
    xlabel(plots.xlabels(varIdx),'Interpreter','latex');
    ylabel(plots.ylabels(varIdx),'Interpreter','latex');
    xlim(xLim);
    ylim(yLim);
    xticks(linspace(xLim(1),xLim(2),6));
    yticks(linspace(yLim(1),yLim(2),6));
    grid on;
    box on;
    set(gca,'FontSize',12,'LineWidth',2);
    legend(legendHandles, legendLabels,'Interpreter','none', ...
'FontName','Times New Roman', ...
'FontSize',6, ...
'Location','northeast');
 exportgraphics(gcf, fullfile(figDir, plots.names(varIdx) + "_" + mode + ".pdf"), ...
        'ContentType','vector', 'BackgroundColor','none');
% exportgraphics(gcf, fullfile(figDir, [plots.names(varIdx), '.pdf']), ...
%                                  'ContentType', 'vector', 'BackgroundColor','none');
end
end