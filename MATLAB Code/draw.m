% To call the function , type 'draw (Array1, start_point, draw_this_long);'

function y = draw( array, start_point, draw_this_long)

end_point = start_point + draw_this_long ;
xaxis = [ start_point : end_point ];
y = plot(xaxis, array (start_point:end_point)  , 'linewidth', 1.5);
title('Array1')

end


