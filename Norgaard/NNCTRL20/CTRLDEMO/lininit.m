% -------------------------------> LININIT.M <-------------------------------

% ----------      Switches       -----------
regty      ='rst';           % Controller type (rst/pid/none)
design     ='ppaz';          % Controller design (ppnz/ppaz/mv1/off)
refty      ='myref';         % Reference signal (siggener/<var. name>)
simul      ='simulink';      % Control object spec. (simulink/matlab/nnet)
if exist('simulink')~=5,
  simul      ='matlab';      % Simulink not present
end
if pp==2, design     ='ppnz'; end %(This line is only required to run the demo)


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
% an NNARX model:
% NN, NetDef, W1, W2
% (i.e. regressor structure, architecture definition, and weight matrices)
nnfile  = 'forward2';        % Name of file containing neural network model


% ----------- Reference signal -------------
dc      = 0;                 % DC-level
sq_amp  = 01;                % Amplitude of square signals (row vector)
sq_freq = 0.05;              % Frequencies of square signals (column vector)
sin_amp = [0];               % Amplitude of sine signals (row vector)
sin_freq= [0]';              % Frequencies of sine signals (column vector)
Nvar    = 0';                % Variance of white noise signal
myref =[0.3*ones(50,1);-0.3*ones(50,1);];
myref =[myref;1*ones(50,1);-1*ones(50,1)];
myref=[myref;2*ones(50,1);-2*ones(50,1)];


% -- Design parameters in pole placement --
% deg(Am)=deg(A)+deg(Ar)
% deg(Ao)=deg(A)-1         if no zeros are cancled
% deg(Ao)=deg(A)-deg(B)-1  if all zeros are canceled
Am = [1.0 -1.4 0.49 0];      % Denominator of desired model
Bm = [0.09];                 % Numerator of desired model (starts in z^{-1})
Ao = [1];                    % Observer polynomial
Ar = [1 -1];                 % Pre-specified factor of R. Ar must contain
                             % [1 -1] as a factor (=integrator).
As = 1;                      % Pre-specified factor of S (eg. notch filter)
if pp==2, Ao = [1 0];end     %(This line is only needed to run the demo)

% -------- Design parameters in MV1 --------
delta = 0.002;               % Penalty factor on squared control dif. controls


% --------  Constant Controller Parameters  --------- 
K=8;                         % PID parameters
Td=0.8;                      % PID
alf=0.1;                     % PID
Wi=0.2;                      % PID (1/Ti)


% ------ Specify data vectors to plot  -------
% plot_a and plot_b must be cell structures containing the vector names in strings
plot_a = {'ref_data','y_data','ym_data'};
plot_b = {'u_data'};
