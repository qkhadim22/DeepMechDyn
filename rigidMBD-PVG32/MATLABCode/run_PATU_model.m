
addpath('./data/');
addpath('./functions/');
addpath('../figures/');
addpath('./hydraulicsLib/');

%One change

[param]                       = Get_parameters_Patu();

if param.DCV
    tEnd        = 29.9;
    exp_data    = 'data/data_DCV.txt';
else
    tEnd        = 40;
    exp_data    ='data/data_PVG32.txt'; 
end


data        = importdata(exp_data);

[tspan, x,y] = true_hydraulics_PATU(data, tEnd);


%++++++++++++++++++++++++++++++++++++++++++
      %PLOTTING
%++++++++++++++++++++++++++++++++++++++++++

fontSize  = 10;
LineWidth = 1.5;


 for i = 1:size(x,1)  % iterate over rows (signals)
   
    if i <= 8
        figure()
        set(groot, 'defaultAxesTickLabelInterpreter','latex');
        set(groot, 'defaultLegendInterpreter','latex');
        set(gcf, 'PaperUnits', 'centimeters');
        set(gcf, 'PaperPosition', [0 0 10 8]);
        set(gcf,'renderer','painters')

        hold on
        switch i
            case 1
                plot(tspan, rad2deg(x(i,:)), 'r', 'LineWidth', LineWidth)
                hold on
                plot(tspan, y(i,:), 'Color', [0.6 0.8 1], 'LineWidth', LineWidth)
                ylabel('$z_1\ \mathrm{[deg]}$', 'Interpreter', 'latex')
                legend('Simulation', 'Experimental','FontName','Times New Roman','FontSize',fontSize)

                if param.DCV
                   yMin  = 5;
                   yMax  = 30;
                else
                   yMin  = -15;
                   yMax  = 60;
                end

            case 2
                plot(tspan, rad2deg(x(i,:))+rad2deg(x(i-1,:)), 'r', 'LineWidth', LineWidth)
                hold on
                plot(tspan, y(i,:), 'Color', [0.6 0.8 1], 'LineWidth', LineWidth)
                if param.DCV
                   yMin  = -60;
                   yMax  = -15;
                else
                    yMin  = -60;
                    yMax  = 60;
                end

                ylabel('$z_2\ \mathrm{[deg]}$', 'Interpreter', 'latex')
                legend('Simulation', 'Experimental','FontName','Times New Roman','FontSize',fontSize)
            case 3
                plot(tspan, rad2deg(x(i,:)), 'r', 'LineWidth', LineWidth)
                hold on
                plot(tspan, y(i,:), 'Color', [0.6 0.8 1], 'LineWidth', LineWidth)
                ylabel('Angular velocity [deg/sec]')
                if param.DCV
                   yMin  = -12;
                   yMax  = 12;
                else
                    yMin  = -12;
                    yMax  = 12;
                end
                legend('Simulation', 'Experimental','FontName','Times New Roman','FontSize',fontSize)

            case 4
                plot(tspan, rad2deg(x(i,:))+rad2deg(x(i-1,:)), 'r', 'LineWidth', LineWidth)
                hold on
                plot(tspan, y(i,:), 'Color', [0.6 0.8 1], 'LineWidth', LineWidth)
                ylabel('Angular velocity [deg/sec]')

                if param.DCV
                   yMin  = -20;
                   yMax  = 20;
                else
                    yMin  = -12;
                    yMax  = 12;
                end
                legend('Simulation', 'Experimental','FontName','Times New Roman','FontSize',fontSize)
            case 5
                plot(tspan, x(i,:), 'r', 'LineWidth', LineWidth)
                hold on
                plot(tspan, y(i,:), 'Color', [0.6 0.8 1], 'LineWidth', LineWidth)
                ylabel('$p_1\ \mathrm{[Pa]}$', 'Interpreter', 'latex')

                 if param.DCV
                   yMin  = 110e5;
                   yMax  = 170e5;
                else
                    yMin  = 40e5;
                    yMax  = 90e5;
                 end
                legend('Simulation', 'Experimental','FontName','Times New Roman','FontSize',fontSize)
            case 6
                plot(tspan, x(i,:), 'r', 'LineWidth', LineWidth)
                hold on
                plot(tspan, y(i,:), 'Color', [0.6 0.8 1], 'LineWidth', LineWidth)
                if param.DCV
                   yMin  = 0e5;
                   yMax  = 85e5;
                else
                    yMin  = -10e5;
                    yMax  = 40e5;
                end

                ylabel('$p_2\ \mathrm{[Pa]}$', 'Interpreter', 'latex')
                legend('Simulation', 'Experimental','FontName','Times New Roman','FontSize',fontSize)

            case 7
                plot(tspan, x(i,:), 'r', 'LineWidth', LineWidth)
                hold on
                if param.DCV
                   yMin  = 10e5;
                   yMax  = 50e5;
                else
                    yMin  = 0e5;
                    yMax  = 15e5;
                end

                plot(tspan, y(i,:), 'Color', [0.6 0.8 1], 'LineWidth', LineWidth)
                ylabel('$p_3\ \mathrm{[Pa]}$', 'Interpreter', 'latex')
                legend('Simulation', 'Experimental','FontName','Times New Roman','FontSize',fontSize)
            case 8
                plot(tspan, x(i,:), 'r', 'LineWidth', LineWidth)
                hold on
                if param.DCV
                   yMin  = 60e5;
                   yMax  = 130e5;
                else
                    yMin  = 25e5;
                    yMax  = 50e5;
                end
                plot(tspan, y(i,:), 'Color', [0.6 0.8 1], 'LineWidth', LineWidth)
                ylabel('$p_4\ \mathrm{[Pa]}$', 'Interpreter', 'latex')
                legend('Simulation', 'Experimental','FontName','Times New Roman','FontSize',fontSize)

        end
        hold off

        set(gca, 'YLim', [yMin yMax])
        set(gca, 'YTick', linspace(yMin, yMax, 6))
        set(gca, 'XTick', linspace(0, tspan(end), 6))
        set(gca, 'LineWidth', LineWidth)

        xlabel('Time [s]')
        grid minor
    else
        figure()
        set(groot, 'defaultAxesTickLabelInterpreter','latex');
        set(groot, 'defaultLegendInterpreter','latex');
        set(gcf, 'PaperUnits', 'centimeters');
        set(gcf, 'PaperPosition', [0 0 10 8]);
        set(gcf,'renderer','painters')
        grid minor
    end
end



