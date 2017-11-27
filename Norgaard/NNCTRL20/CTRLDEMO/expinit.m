% ------------------------------> EXPINIT.M <------------------------------

% ----------      Switches       -----------
regty      ='none';          % Controller type (rst, pid, none)
refty      ='predef';        % Reference signal (siggener/none/<var. name>)
probety    ='none';          % Probing signal (none/<var. name>)
simul      ='simulink';      % Control object spec. (SIMULINK/MATLAB)
if exist('simulink')~=5,
  simul      ='matlab';      % Simulink not present
end


% ------    General Initializations  -------
Ts = 0.20;                   % Sampling period (in seconds)
samples = 1000 ;             % Number of samples to be simulated


% --- System to be Controlled (SIMULINK) ---
integrator= 'ode45';         % Name of dif. eq. solver (f. ex. ode45 or ode15s)
sim_model = 'spm1';          % Name of SIMULINK model
u_0 = 0;                     % Initial control input
y_0 = 0;                     % Initial output
ulim_min = -Inf;             % Minimum control input
ulim_max = Inf;              % Maximum control input


% ---  System to be Controlled (MATLAB)  ---
mat_model = 'springm';       % Name of MATLAB model
model_out = 'smout';         % Output equation (function of the states)
x0        = [0;0];           % Initial states


% ------------ Reference Signal ------------
dc        = 0;               % DC-level
sq_amp    = 2;               % Amplitude of square signals (row vector)
sq_freq   = 0.05;            % Frequency of square signals (column vector)
sin_amp   = [0];             % Amplitude of sine signals  (row vector)
sin_freq  = [0]';            % Frequency of sine signals   (column vector)
Nvar  = 0';                  % Variance of normal dist. white noise signal
ownref = prs(samples,10,0.15);


% ----------- Probing Signal -----------
% Use in case of closed-loop identification
predef = prs(samples,12,0.85);


% --------  Linear Controller Parameters  --------- 
K=8;                         % PID parameters
Td=0.8;                      % PID
alf=0.1;                     % PID
Wi=0.2;                      % PID (1/Ti)

r0=0.0609;                   % RST parameters
r1=0.0514;                   % RST
t0=0.2;                      % RST
s0=0.802;                    % RST
s1=-0.602;                   % RST
R = [r0 r1];
S = [s0 s1];
T = t0;

% ------- Specify data vectors to plot --------
% plot_a and plot_b must be cell structures containing the vector names in strings
plot_a = {'ref_data','y_data'};
plot_b = {'u_data'};
