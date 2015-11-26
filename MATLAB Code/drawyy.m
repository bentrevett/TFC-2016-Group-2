% To call the function , type 'drawyy (Array1, Array2, start_point, draw_this_long);'

function y = drawyy(array1, array2, start_point, draw_this_long)

end_point = start_point + draw_this_long ;
xaxis = [ start_point : end_point ];
y = plotyy (xaxis, array1 (start_point:end_point) , xaxis , array2 (start_point:end_point) );
legend('DATA1','DATA2');
 
end


