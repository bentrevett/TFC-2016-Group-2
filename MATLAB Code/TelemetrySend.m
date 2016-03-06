%-----------------------------%
if (exist('bt','var'))
    fclose(bt);
    clear bt;
end;
close all;
clc;
%-----------------------------%
display('Connecting to Bluetooth');
bt = Bluetooth ('Freescalextric',1);  % Define the name of the particular BT module used!
fopen(bt);                            % Connect to the device
display('Connected to Bluetooth');
%-----------------------------%

%fwrite(obj,A) writes the binary data A to the device connected to the serial port object, obj.

%fwrite(obj,A,'precision') writes binary data with precision specified by precision.

%precision controls the number of bits written for each value and the interpretation of those bits as integer, floating-point, or character values. If precision is not specified, uchar (an 8-bit unsigned character) is used. The supported values for precision are listed in Tips.

%fwrite(obj,A,'mode') writes binary data with command-line access specified by mode. If mode is sync, A is written synchronously and the command line is blocked. If mode is async, A is written asynchronously and the command line is not blocked. If mode is not specified, the write operation is synchronous.

%fwrite(obj,A,'precision','mode') writes binary data with precision specified by precision and command-line access specified by mode.
   
%fwrite(bt,n,'int8','async');

variable_string=' ';

while(strcmp(variable_string,'EXIT')==0)

    variable_string = input('Name of variable: ','s');
    if(strcmp(variable_string,'EXIT')==0)
    value = input('Name of value: ');

    switch(variable_string)
        case 'SPEED'
            variable_number = 255;
        case 'PID_P'
            variable_number = 254;
        case 'PID_I'
            variable_number = 253;
        case 'PID_D'
            variable_number = 252;
        case 'LED'
            variable_number = 251;
        otherwise
            error('Please enter a valid variable name - SPEED, PID_P, PID_I, PID_D, LED, EXIT');
    end

    
        for n = 1:1
            fwrite(bt,-128); 
            fwrite(bt,variable_number);
            disp(variable_string);
            pause(1);
            fwrite(bt,value);
            disp(value);
            pause(1);
        end
    end

end

disp('EXITING PROGRAM')

fclose(bt);
