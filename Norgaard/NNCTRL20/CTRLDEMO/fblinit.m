% ------------------------------>  FBLINIT.M  <------------------------------

% ----------      Switches       -----------
regty      ='fbl';           % Controller type (fbl, pid, none)
refty      ='myref';         % Reference signal (siggener/<var. name>)
simul      ='simulink';      % Control object spec. (simulink/matlab/nnet)
if exist('simulink')~=5,
  simul      ='matlab';      % Simulink not present
end


% ------    General Initializations  -------
Ts = 0.20;                   % Sampling period (in seconds)
samples = 300 ;              % Number of samples to be simulated
u_0 = 0;                     % Initial control input
y_0 = 0;                     % Initial output
ulim_min = -10;             % Minimum control input
ulim_max = Inf;              % Maximum control input


% --  System to be Controlled (SIMULINK) --
integrator= 'ode45';         % Name of dif. eq. solver (f. ex. ode45 or ode15s)
sim_model = 'spm1';          % Name of SIMULINK model


% ---  System to be Controlled (MATLAB)  --
mat_model = 'springm';       % Name of MATLAB model
model_out = 'smout';         % Output equation (function of the states)
x0        = [0;0];           % Initial states
 

% ----- Neural Network Specification ------
% "nnfile" must contain the following variables which together define
% the network model:
% NN, NetDeff, NetDefg, W1f, W2f, W1g, W2g
% (i.e. regressor structure, architecture definitions, and weight matrices)
nnfile = 'forward3';         % Name of file


% ---- Desired characteristic polynomial ---
Am = [1 -1.4 0.49];          % Characteristic polynomial


% ------------ Reference signal ------------
dc      = 0;                 % DC-level
sq_amp  = 1;                 % Amplitude of square signals (row vector)
sq_freq = 0.05;              % Frequency of square signals (column vector)
sin_amp = [0];               % Amplitude of sine signals  (row vector)
sin_freq= [0]';              % Frequency of sine signals   (column vector)
Nvar  = 0';                  % Variance white noise signal
myref =[0.3*ones(50,1);-0.3*ones(50,1);];
myref =[myref;1*ones(50,1);-1*ones(50,1)];
myref=[myref;2*ones(50,1);-2*ones(50,1)];

% --------  Linear Controller Parameters  --------- 
K=8;                         % PID parameters
Td=0.8;                      % PID
alf=0.1;                     % PID
Wi=0.2;                      % PID (1/Ti)


% ------- Specify data vectors to plot --------
% plot_a and plot_b must be cell structures containing the vector names in strings
plot_a = {'ref_data','y_data','ym_data'};
plot_b = {'u_data'};
