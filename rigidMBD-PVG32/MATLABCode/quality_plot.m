
function quality_plot(color, x_data, y_data,xLim, yLim, xlabel, ylabel)



 plot(x_data, y_data,'Color',color,'LineWidth',2);

 xlabel(xlabel);
 ylabel(ylabel);
 xlim(xLim);
 ylim(yLim);
 xticks(linspace(xLim(1),xLim(2),6));
 yticks(linspace(yLim(1),yLim(2),6));
 grid on;
 box on;
 set(gca,'FontSize',fontSize,'LineWidth',LineWidth);
 legend(legendEntries,'FontName','Times New Roman','FontSize',fontSize); 
 exportgraphics(gcf, fullfile(figDir, [fname, '.pdf']), ...
                                'ContentType', 'vector', 'BackgroundColor','none');


end

