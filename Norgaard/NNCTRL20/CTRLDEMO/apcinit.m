% -------------------------------> APCINIT.M <------------------------------

% ----------      Switches       -----------
regty      ='apc';           % Controller type (apc, pid, none)
refty      ='myref';         % Reference signal (siggener/<var. name>)
simul      ='nnet';      % Control object spec. (simulink/matlab/nnet)
if exist('simulink')~=5,
  simul      ='matlab';      % Simulink not present
end


% ----------   Initializations   -----------
Ts = 0.2;                    % Sampling period (in seconds)
samples = 300 ;              % Number of samples in simulation
u_0 = 0;                     % Initial control input
y_0 = 0;                     % Initial output
ulim_min = -Inf;             % Minimum control input
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
% a NNARX model:
% NN, NetDef, W1, W2
% (i.e. regressor structure, architecture definition, and weight matrices)
nnfile  = 'forward2';        % Name of file containing neural network model


% ------------ Reference filter ---------------
Am = [1];                    % Denominator of filter
Bm = [1];                    % Numerator of filter


% ---------- APC initializations -----------
N1 = 1;                      % Minimum prediction horizon (typically=nk)
N2 = 7;                      % Maximum prediction horizon (>= nb)
Nu = 2;                      % Control horizon
rho = 0.03;                  % Weight factor on differenced control signal


% ----------- Reference signal -------------
dc      = 0;                 % DC-level
sq_amp  = 3;                 % Amplitude of square signals (row vector)
sq_freq = 0.05;              % Frequencies of square signals (column vector)
sin_amp = [0];               % Amplitude of sine signals (row vector)
sin_freq= [0]';              % Frequencies of sine signals (column vector)
Nvar  = 0';                  % Variance of white noise signal
myref =[0.3*ones(50,1);-0.3*ones(50,1);];
myref =[myref;1*ones(50,1);-1*ones(50,1)];
myref=[myref;2*ones(50,1);-2*ones(50,1)];

% -- Constant Gain Controller Parameters -- 
K=0.8;                       % PID parameters
Td=0.8;                      % PID
alf=0.1;                     % PID
Wi=0.2;                      % PID (1/Ti)


% ------ Choose Data to be Plotted  -------
% plot_a and plot_b must be cell structures containing the vector names in strings
plot_a = {'ref_data','y_data'};
plot_b = {'u_data'};
