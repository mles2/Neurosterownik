% ------------------------------>  INVINIT2.M  <------------------------------
% Initialization file for the program "special2"


% ----------      Switches       -----------
simul      = 'simulink';     % System specification (simulink/matlab/nnet)
method     = 'ff';           % Training algorithm (ff/ct/efra)
refty      = 'siggener';     % Reference signal (siggener/<var. name>)


% ------    General Initializations  -------
Ts = 0.20;                   % Sampling period (in seconds)
samples  = 200;              % Number of samples in each epoch
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
 

% ----- Neural Network Specification ------
% The "forward model file" must contain the following variables which together
% define a NNARX-model:
% NN, NetDeff, W1f, W2f
% and the "inverse model file" must contain
% NN, NetDefi, W1i, W2i
% (i.e. regressor structure, architecture definition, and weight matrices)
nnforw = 'forward';          % Name of file containing forward model
nninv  = 'inverse';          % Name of file containing inverse model


% ------------ Reference Model ---------------
Am = [1 -0.7];               % Model denominator
Bm = [0.3];                  % Model numerator (starts in z^-1)


% ------------ Training parameters -----------
maxiter = 8;                 % # of "epochs"

% --- Forgetting factor algorithm (ff) ---
% trparms = [lambda p0]
%    lambda = forgetting factor (suggested value 0.995)
%    p0     = Covariance matrix diagonal (1-10)
%    
% --- Constant trace algorithm (ct) ---
% trparms = [lambda alpha_max alpha_min]
%    lambda = forgetting factor (suggested value 0.995)
%    alpha_max = Max. eigenvalue of covariance matrix (100)
%    alpha_min = Min. eigenvaule of covariance matrix (0.001)
%    
% --- Exponential Forgetting and Restting Algorithm (efra) ---
% trparms = [alpha beta delta lambda]
%    Suggested values:
%    alpha = 0.5-1
%    beta = 0.001
%    delta = 0.001
%    lambda = 0.98
trparms = [0.995 10];
%trparms = [0.995 100 0.001];
%trparms = [1 0.001 0.001 0.98];


% ------------ Reference signal ------------
% Reference generated by the signal generator
dc      = 0;                 % DC-level
sq_amp  = 1;                 % Amplitude of square signals (row vector)
sq_freq = 0.1;               % Frequency of square signals (column vector)
sin_amp = [0];               % Amplitude of sine signals  (row vector)
sin_freq= [0]';              % Frequency of sine signals   (column vector)
Nvar  = 0;                   % Variance of white noise signal


% ------- Specify data vectors to plot --------
% plot_a and plot_b must be cell structures containing the vector names in strings
plot_a = {'ref_data','y_data','ym_data','yhat_data'};
plot_b = {'u_data'};
