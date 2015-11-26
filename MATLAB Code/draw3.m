% To call the function , type '' draw2 (Array1, Array2, Array3, start_point, draw_this_long); ''

function [q,y,z] = draw3(array1, array2, array3, start_point,draw_this_long)

end_point = start_point + draw_this_long ;
xaxis = [ start_point : end_point ];

figure;
y = plot(xaxis, array1 (start_point:end_point) , 'linewidth', 1.5);

hold on;
z = plot(xaxis, array2 (start_point:end_point), 'r' , 'linewidth', 1.5);
q = plot(xaxis, array3 (start_point:end_point), 'g' , 'linewidth', 1.5);
legend('DATA1','DATA2','DATA3');
hold off;
end
