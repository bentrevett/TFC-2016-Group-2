%-----------------------------%
if (exist('bt','var'))
    fclose(bt);
    clear bt;
end
clear all
close all
clc

disp('Attempting to connect to Bluetooth');
bt = Bluetooth('GROUP2',1);% Define the name of the particular BT module used!
fopen(bt); % Connect to the device
disp('Connected to Bluetooth!');

data_per_read = 129*2; %read in up to 2 camera readings at once to scan through
no_of_data_in=0;
data_in_array = ones(1,data_per_read);

while 1
    
    
    while(no_of_data_in<data_per_read)
        disp('Reading data in...');
        [data_in_array(1:data_per_read),no_of_data_in] = fread(bt, data_per_read, 'int8');
    end
    
    disp('2 sets of camera values read successfully, searching for start flag');
    
    i=1;
    
    if(data_in_array(1,i)~=-128)
        while (data_in_array(1,i) ~= -128) %wait for start flag
            i=i+1;
            if(i>=data_per_read)
                error('Start flag not found!');
            end
        end
    end
    
    disp('Start flag found!');
    start_position = i+1;
    
    plot(1:1:128,data_in_array(start_position:1:start_position+127));
 
    xlabel('linescan[x]');
    ylabel('light level/100');
    title('please work');
    ylim([0,50]);
    xlim([1,128]);
    drawnow;
 
    no_of_data_in=0;
    
end


fclose(bt)

