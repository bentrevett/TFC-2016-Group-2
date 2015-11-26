%% Example
% Create data
Fs = 1000;                      % Sample rate
Ns = Fs*10;                      % Make 5 seconds worth of data
t = (0:1:Ns-1)'/Fs;
A = sqrt(t);
A(1:Ns/2) = A(1:Ns/2);
A(end:-1:Ns/2+1) = A(1:Ns/2);
s = A.*sin(2*pi*t*1);

%%
% Initialize stripchart
clf
AxesWidth = 2;                  % Axes Width (s)
stripchart(Fs,AxesWidth);

%%
% Update stripchart
N = 50;
ind = 1:N:Ns;

for ii = ind
    stripchart(s(ii:ii+N-1,:));
    drawnow;pause(0.005);
end;