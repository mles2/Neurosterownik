% Start height
starth = 0.1;      % Initial height in meters
kout = 0.0001;
Ts = 0.2;          % Sampling period (sec)

% Parameters for Cone tank model
conealpha = 20;    % Cone angle Degrees
conegain = 1/(tan(conealpha*pi/180)^2*pi);
maxh = 0.5;        % Cone height (meters)
startQ = kout*sqrt(starth); %Stable initial inflow

h0 = 0.2;          % Operating point
Qin0 = kout*sqrt(h0);

a = -2*conegain*Qin0*h0^-3 + 1.5*kout*conegain*h0^-(5/2);
b = conegain*h0^-2;

[nn,dd]=c2dm(b,[1 -a],Ts,'zoh');

