% Redefinition of the Matlab "plot" function, with more options available
% and with the possibility of printing the figure to a .pdf file.
%
%
function [] = custom_plot(x, y, label_xaxis, label_yaxis, label_title, ...
                          lineSize, labels_legend, figNumber, printFigure, useDots)
   
   % generate nice plots
   figure(figNumber)                   
  
   if useDots     
       
       plot(x,y,'o','Markersize',10)
   else
       plot(x,y,'Linewidth',lineSize)
   end
   
   xlabel(label_xaxis)
   ylabel(label_yaxis)
   title(label_title)
   legend(labels_legend,'Location','Best')
   grid on
   hold all
   
   if printFigure    
        print(gcf, [label_title, '.png'], '-dpng','-r600')
   end
end
