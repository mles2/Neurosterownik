%*******************************************************
%*-----------------------------------------------------*
%*      PARAMETERS FOR THE PNEUMATIC SERVO             *
%*      This MATLAB command file initialises           *
%*      the values for the parameters of the digitally *
%*      controlled linear pneumatic servo.             *
%*                                                     *
%*-----------------------------------------------------*
%*******************************************************

X0      =  0;           % Initial position
                        % relative to the equal volume pos.
X1      =  0.1;        % Step size


V1      =  4.908e-4;    % Initial volume in chamber 1.
V2      =  4.123e-4;    % Initial volume in chamber 2.
M       =  20.;          % Total inertial mass.
g       =  9.81;        % Gravitational acceleration.
B1       =  30;          % Viscous friction coefficient.
Fdry    =  0;           % Dry friction.

A1      =  19.63e-4;    % Area of large side of piston.
A2      =  16.49e-4;    % Area of small side of piston.

rho1     =  1.208;       % Gas density.
T1       =  293;         % Gas absolute temperature.
R1       =  281.3;       % Gas constant.
PS      =  6e5;         % supply pressure.
PR      =  1e5;         % Return pressure (1 atm.).

k1       =  1.4;         % Isentropic Exponent.
zkrit   =  0.528;       % critical pressure drop.
kkrit   =  1.3154;      % critical flow gain
kvalve  =  2.857e-9;    % valve flow gain.


P10     =  4.e5;        % Initial pressure.
%P20     =  4.82e5;      % Initial pressure.
P20=(M*g+P10*A1-PR*(A1-A2))/A2;

KP      =  20;          % Proportional gain.
TS      =  .100;        % Sample time

Wp      =  rho1*120./60000.; % Supply mass flow.
Vsupply =  1e-2;        % Supply compressor accumulator.

