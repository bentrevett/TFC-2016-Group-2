% To call the function , type '' draw2 (Array1, Array2, start_point, draw_this_long); ''

function [y,z] = draw_sub(array1, array2, start_point,draw_this_long)

end_point = start_point + draw_this_long ;
xaxis = [ start_point : end_point ];

subplot (2,1,1)
y = plot(xaxis, array1 (start_point:end_point) , 'linewidth', 1.5 );
title('Array1')

subplot (2,1,2)
z = plot(xaxis, array2 (start_point:end_point) , 'linewidth', 1.5 );
title('Array2')

end


