% ------------------------------>  OPTINIT.M  <------------------------------

% ----------      Switches       -----------
regty      ='opt';           % Controller type (opt, pid, none)
refty      ='siggener';      % Reference signal (siggener/<var. name>)
simul      ='simulink';      % Control object spec. (simulink/matlab/nnet)


% ------    General Initializations  -------
Ts = 0.20;                   % Sampling period (in seconds)
samples  = 200;              % Number of samples to be simulated
u_0      = 0;                % Initial control input
y_0      = 0;                % Initial output
ulim_min = -Inf;             % Minimum control input
ulim_max = Inf;              % Maximum control input


% --  System to be Controlled (SIMULINK) --
integrator= 'ode45';         % Name of dif. eq. solver (f. ex. ode45 or ode15s)
sim_model = 'spm1';          % Name of SIMULINK model


% ---  System to be Controlled (MATLAB)  --
mat_model = 'springm';       % Name of MATLAB model
model_out = 'smout';         % Output equation (function of the states)
x0        = [0;0];           % Initial states
 

% -- System to be Controlled (neural net) -
% The file must contain: NN, NetDeff, W1f, W2f
nnforw = 'forward';          % Name of file


% ----- Network Controller Specification ------
% The file must contain the variables:
% NN, NetDefc, W1c, W2c
% (i.e. regressor structure, architecture definition, and weight matrices)
nnctrl= 'optctrl';           % Name of file


% ------------ Reference Filter ---------------
Am = [1];                    % Filter denominator
Bm = [1];                    % Filter numerator (starts in z^0)


% ------------ Reference signal ------------
dc      = 0;                 % DC-level
sq_amp  = 1;                 % Amplitude of square signals (row vector)
sq_freq = 0.1;               % Frequency of square signals (column vector)
sin_amp = [0];               % Amplitude of sine signals  (row vector)
sin_freq= [0]';              % Frequency of sine signals   (column vector)
Nvar    = 0';                % Variance of white noise signal


% --------  Linear Controller Parameters  --------- 
K=1;                         % PID parameters
Td=10;                       % PID
alf=0.1;                     % PID
Wi=1/0.2;                    % PID (1/Ti)


% ------- Specify data vectors to plot --------
% plot_a and plot_b must be cell structures containing the vector names in strings
plot_a = {'ref_data','y_data'};
plot_b = {'u_data'};
