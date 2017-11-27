% -------------------------------> NPCINIT.M <------------------------------

% ----------      Switches       -----------
regty      ='npc';           % Controller type (npc, pid, none)
refty      ='siggener';      % Reference signal (siggener/<var. name>)
simul      ='simulink';      % Control object spec. (simulink/matlab/nnet)


% ----------   Initializations   -----------
Ts = 0.2;                    % Sampling period (in seconds)
samples  = 300;              % Number of samples in simulation
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
% "nnfile" must contain the following variables which together define
% an NNARX model:
% NN, NetDef, W1, W2
% (i.e. regressor structure, architecture definition, and weight matrices)
nnfile  = 'forward';         % Name of file containing neural network model


% ------------ Reference filter ---------------
Am = [1];                    % Denominator of filter
Bm = [1];                    % Numerator of filter


% ---------- GPC initializations -----------
N1 = 1;                      % Min. prediction horizon (must equal time delay!)
N2 = 7;                      % Max. prediction horizon (>= nb)
Nu = 2;                      % Control horizon
rho = 0.03;                  % Penalty factor on differenced control signal


% -- Minimization algorithm initialzations -
% maxiter: Maxiumum number of Levenberg-Marquardt/Quasi-Newton iteratons.
% delta  : If the 2-norm of the difference to the previous iterate is smaller
%          than 'delta', the algorithm is terminated.
% initval: A string which will be evaluated by npccon1/2. To avoid problems with
%          local minima, different initial values of the most future component
%          of the 'sequence of future controls' can be initialized differenly.
%          I.e. the minimization algorithm is executed once for each element
%          in 'initval'. Use initval = '[upmin(Nu)]' as default.          
maxiter = 5;                 % Max. number of iterations to determine u 
delta = 1e-4;                % Norm-of-control-change-stop criterion
initval = '[upmin(Nu)]';     % Initial guess on last control in sequence


% ----------- Reference signal -------------
dc      = 0;                 % DC-level
sq_amp  = 3;                 % Amplitude of square signals (row vector)
sq_freq = 0.1;               % Frequencies of square signals (column vector)
sin_amp = [0];               % Amplitude of sine signals (row vector)
sin_freq= [0]';              % Frequencies of sine signals (column vector)
Nvar  = 0';                  % Variance of white noise signal


% -- Constant Gain Controller Parameters -- 
K=8;                         % PID parameters
Td=0.8;                      % PID
alf=0.1;                     % PID
Wi=0.2;                      % PID (1/Ti)


% ------ Specify data vectors to plot  -------
% plot_a and plot_b must be cell structures containing the vector names in strings
plot_a = {'ref_data','y_data','yhat_data'};
plot_b = {'u_data'};
