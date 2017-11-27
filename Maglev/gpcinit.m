% -------------------------------> GPCINIT.M <------------------------------

% ----------      Switches       -----------
regty      ='gpc';           % Controller type (apc, pid, none)
refty      ='siggener';      % Reference signal (siggener/<var. name>)
simul      ='simulink';      % Control object spec. (simulink/matlab/nnet)


mlevpar

% ----------   Initializations   -----------
Ts = 1/100;                    % Sampling period (in seconds)
samples = 300 ;              % Number of samples in simulation
u_0      = il;                % Initial control input
y_0      = yl;                % Initial output
ulim_min = 0;             % Minimum control input
ulim_max = 5;              % Maximum control input


% --  System to be Controlled (SIMULINK) --
integrator= 'ode45';         % Name of dif. eq. solver (f. ex. ode45 or ode15s)
sim_model = 'maglev';          % Name of SIMULINK model


% ---  System to be Controlled (MATLAB)  --
mat_model = 'springm';       % Name of MATLAB model
model_out = 'smout';         % Output equation (function of the states)
x0        = [0;0];           % Initial states


% ------------ Reference filter ---------------
Am = [1];                    % Denominator of filter
Bm = [1];                    % Numerator of filter (starts in q^{-1})


% ------------ Linear model ---------------
A=[1 -2.0867 1];
B = [-0.7260 -0.7260]*1e-3;


% ---------- GPC initializations -----------
N1 = 1;                      % Minimum prediction horizon (typically=nk)
N2 = 10;                     % Maximum prediction horizon (>= nb)
Nu = 2;                      % Control horizon
rho = 1e-4;                  % Weight factor on differenced control signal


% ----------- Reference signal -------------
dc      = 0.023;             % DC-level
sq_amp  = 0.016;             % Amplitude of square signals (row vector)
sq_freq = 1;                 % Frequencies of square signals (column vector)
sin_amp = [0];               % Amplitude of sine signals (row vector)
sin_freq= [0]';              % Frequencies of sine signals (column vector)
Nvar  = 0';                  % Variance of white noise signal


% -- Constant Gain Controller Parameters -- 
K=0.8;                       % PID parameters
Td=0.8;                      % PID
alf=0.1;                     % PID
Wi=0.2;                      % PID (1/Ti)


% ------ Choose Data to be Plotted  -------
% plot_a and plot_b must be cell structures containing the vector names in strings
plot_a = {'ref_data','y_data'};
plot_b = {'u_data'};
