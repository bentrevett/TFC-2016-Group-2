%-----------------------------%
if (exist('bt','var'))
    fclose(bt);
    clear bt;
end
clear all
close all
clc
%-----------------------------%

bt = Bluetooth ('Freescalextric',1);              % Define the name of the particular BT module used!
fopen(bt);                               % Connect to the device

%---User Settings-- ----------%
number_of_cycles = 6000;
data_per_read = 140;       % optimally should be multiple of (data_variables + 1)
data_variables = 13;
%-----------------------------%

array_position = ones(data_variables, 1, 'uint32');
i = 1;
position = 0;
converter = zeros(1,1,'single');

plot_1_len = zeros(1,1, 'uint32');
plot_2_len = zeros(1,1, 'uint32');
plot_3_len = zeros(1,1, 'uint32');
plot_4_len = zeros(1,1, 'uint32');

min_len = zeros(1,1, 'uint32');

captured_data = zeros(number_of_cycles*data_per_read, 1, 'int8');
                                                                                   
TrackCentre = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'int8');   % 1 Track Centre from Camera (pixels)
TargetSpeed = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'single'); % 2 Target Speed (TFC speed units)
SpeedR = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'single');      % 3 Speed - Right Wheel (TFC speed units)
SpeedL = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'single');      % 4 Speed - Left Wheel (TFC speed units)
PWM_R = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'single');       % 5 PWM - Right Wheel (%)
PWM_L = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'single');       % 6 PWM - Left Wheel (%)
CurrentL = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'single');    % 7 Current - Right Motor (A)
CurrentR = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'single');    % 8 Current - Left Motor (A)
ServoValue = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'single');  % 9 Servo Value (TFC %; 100% = 45 degrees)
EdgeType = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'single');    % 10 Edge Type: None 50, Track 0, Left 100, Right -100
Exposure = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'single');    % 11 Camera exposure time (ms)
LoopTime = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'single');    % 12 Processing Loop Time (ms)
LineDistance = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 1, 'int8');  % 13 Line Distance detected by second camera (pixels)

% DataX = zeros(floor(number_of_cycles*data_per_read/(data_variables+1)), 2, 'int8');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clf
Fs = 200;
AxesWidth = 5; % Axes Width (s)

subplot(4,1,1)
stripchart(Fs,AxesWidth, 2);

subplot(4,1,2)
stripchart(Fs,AxesWidth, 3);

subplot(4,1,3)
stripchart(Fs,AxesWidth, 2);

subplot(4,1,4)
stripchart(Fs,AxesWidth, 2);

%stripchart(Fs,AxesWidth, 2); -check the 3rd parameter again

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for loop_counter = 1:number_of_cycles
    
    array_start = (loop_counter-1)*data_per_read + 1;
    
    captured_data(array_start : loop_counter*data_per_read) = fread(bt, data_per_read, 'int8');
    
    
    array_init_position = array_position;
    
     
    if(position == 0 && captured_data(i) ~= -128)    
        while (captured_data(i) ~= -128)
            i = i + 1;
        end
        i = i + 1;
        position = 1;
    end
    
    while i <= loop_counter*data_per_read
        
        if (i <= loop_counter*data_per_read && position == 1)
            TrackCentre(array_position(1)) = captured_data(i);
            array_position(1) = array_position(1) + 1;
            i = i + 1;
            position = 2;
        end
        if (i <= loop_counter*data_per_read && position == 2)
            if(captured_data(i) >= 0)
                converter = single(captured_data(i));
            else
                converter = 256 + single(captured_data(i));
            end
            TargetSpeed(array_position(2)) = converter/8;
            array_position(2) = array_position(2) + 1;
            i = i + 1;
            position = 3;
        end
        if (i <= loop_counter*data_per_read && position == 3)
            if(captured_data(i) >= 0)
                converter = single(captured_data(i));
            else
                converter = 256 + single(captured_data(i));
            end            
            SpeedR(array_position(3)) = converter/8;
            array_position(3) = array_position(3) + 1;
            i = i + 1;
            position = 4;
        end
        if (i <= loop_counter*data_per_read && position == 4)
            if(captured_data(i) >= 0)
                converter = single(captured_data(i));
            else
                converter = 256 + single(captured_data(i));
            end              
            SpeedL(array_position(4)) = converter/8;
            array_position(4) = array_position(4) + 1;
            i = i + 1;
            position = 5;
        end
        if (i <= loop_counter*data_per_read && position == 5)
            converter = single(captured_data(i));
            PWM_R(array_position(5)) = converter*100/126;
            array_position(5) = array_position(5) + 1;
            i = i + 1;
            position = 6;
        end
        if (i <= loop_counter*data_per_read && position == 6)
            converter = single(captured_data(i));
            PWM_L(array_position(6)) = converter*100/126;
            array_position(6) = array_position(6) + 1;
            i = i + 1;
            position = 7;
        end
        if (i <= loop_counter*data_per_read && position == 7)
            if(captured_data(i) >= 0)
                converter = single(captured_data(i));
            else
                converter = 256 + single(captured_data(i));
            end              
            CurrentL(array_position(7)) = converter/60;
            array_position(7) = array_position(7) + 1;
            i = i + 1;
            position = 8;
        end
        if (i <= loop_counter*data_per_read && position == 8)
            if(captured_data(i) >= 0)
                converter = single(captured_data(i));
            else
                converter = 256 + single(captured_data(i));
            end              
            CurrentR(array_position(8)) = converter/60;
            array_position(8) = array_position(8) + 1;
            i = i + 1;
            position = 9;
        end
        if (i <= loop_counter*data_per_read && position == 9)
            converter = single(captured_data(i));
            ServoValue(array_position(9)) = converter*100/126;
            array_position(9) = array_position(9) + 1;
            i = i + 1;
            position = 10;
        end
        if (i <= loop_counter*data_per_read && position == 10)
            EdgeType(array_position(10)) = captured_data(i);
            array_position(10) = array_position(10) + 1;
            i = i + 1;
            position = 11;
        end
        if (i <= loop_counter*data_per_read && position == 11)
            if(captured_data(i) >= 0)
                converter = single(captured_data(i));
            else
                converter = 256 + single(captured_data(i));
            end
            Exposure(array_position(11)) = converter*40/1000;
            array_position(11) = array_position(11) + 1;
            i = i + 1;
            position = 12;
        end
        if (i <= loop_counter*data_per_read && position == 12)
            converter = single(captured_data(i));
            LoopTime(array_position(12)) = converter/10;
            array_position(12) = array_position(12) + 1;
            i = i + 1;
            position = 13;
        end    
        if (i <= loop_counter*data_per_read && position == 13)
            LineDistance(array_position(13)) = captured_data(i);
            array_position(13) = array_position(13) + 1;
            i = i + 1;
            position = 0;
        end   
        
        if (i <= loop_counter*data_per_read && position == 0)
            i = i + 1;
            position = 1;
        end
        
    end    
 

   min_len = array_position(1) - array_init_position(1);
   if (min_len > (array_position(10) - array_init_position(10)))
       min_len = array_position(10) - array_init_position(10);
   end
   plot_1_len = min_len - 1;
   
   min_len = array_position(2) - array_init_position(2);
   if (min_len > (array_position(3) - array_init_position(3)))
       min_len = array_position(3) - array_init_position(3);
   end
   if(min_len > (array_position(4) - array_init_position(4))) 
       min_len = array_position(4) - array_init_position(4);
   end
   plot_2_len = min_len - 1;
   
   min_len = array_position(5) - array_init_position(5);
   if (min_len > (array_position(6) - array_init_position(6)))
       min_len = array_position(6) - array_init_position(6);
   end
   plot_3_len = min_len - 1;
   
   min_len = array_position(7) - array_init_position(7);
   if (min_len > (array_position(8) - array_init_position(8)))
       min_len = array_position(8) - array_init_position(8);
   end
   plot_4_len = min_len - 1;
   
   subplot(4,1,1)   % TrackCentre, EdgeType
   stripchart([TrackCentre(array_init_position(1):array_init_position(1)+plot_1_len), EdgeType(array_init_position(10):array_init_position(10)+plot_1_len)]);
   ylabel('Track Centre');
   ylim([-110,110]);
   
   subplot(4,1,2)   % Speed T,L,R
   stripchart([TargetSpeed(array_init_position(2):array_init_position(2)+plot_2_len), SpeedL(array_init_position(4):array_init_position(4)+plot_2_len), SpeedR(array_init_position(3):array_init_position(3)+plot_2_len)]);
   ylabel('Speeds');
   
   subplot(4,1,3)   % PWM L,R
   stripchart([PWM_L(array_init_position(6):array_init_position(6)+plot_3_len), PWM_R(array_init_position(5):array_init_position(5)+plot_3_len)]);
   ylabel('PWMs');
   ylim([-110,110]);
   
   subplot(4,1,4)   % Current L,R
   stripchart([CurrentL(array_init_position(8):array_init_position(8)+plot_4_len), CurrentR(array_init_position(7):array_init_position(7)+plot_4_len)]);
   ylabel('Currents');
   
   drawnow;
 
end

fclose(bt)

