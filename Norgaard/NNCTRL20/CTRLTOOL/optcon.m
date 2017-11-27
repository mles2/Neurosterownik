% ---------------------------------    OPTCON     --------------------------------
%
%  Program for simulating optimal control of nonlinear processes.
%
%  All design parameters must be defined in the file 'optinit.m'
%
%  Programmed by Magnus Norgaard IAU, Technical University of Denmark.
%  LastEditDate: Jan. 23, 2000


%----------------------------------------------------------------------------------
%-------------------         >>>  INITIALIZATIONS  <<<        ---------------------
%----------------------------------------------------------------------------------

%>>>>>>>>>>>>>>>>>>>>>>      READ VARIABLES FROM FILE       <<<<<<<<<<<<<<<<<<<<<<<
clear plot_a plot_b
global ugl
optinit
eval(['load ' nnctrl]);                  % Load controller network


% >>>>>>>>>>>>>>>>>>>>>>>>   DETERMINE REGRESSOR STRUCTURE   <<<<<<<<<<<<<<<<<<<<<
na = NN(1);                              % # of past y's to be used in TDL
nb = NN(2);                              % # of past u's to be used in TDL
nk = NN(3);                              % Time delay in system
nab      = na+sum(nb);                   % Number of "signal inputs" to each net
outputs  = 1;                            % # of outputs
inputs   = nab-1;                        % # of inputs
phi = zeros(inputs,1);                   % Initialize regressor vector


% >>>>>>>>>>>>>>>>>   DETERMINE STRUCTURE OF FORWARD MODEL    <<<<<<<<<<<<<<<<<<<<
if strcmp(simul,'nnet'),
  eval(['load ' nnforw]);                % Load forward neural model of system
  hiddenf   = length(NetDeff(1,:));      % Number of hidden neurons
  L_hiddenf = find(NetDeff(1,:)=='L')';  % Location of linear hidden neurons
  H_hiddenf = find(NetDeff(1,:)=='H')';  % Location of tanh hidden neurons
  L_outputf = find(NetDeff(2,:)=='L')';  % Location of linear output neurons
  H_outputf = find(NetDeff(2,:)=='H')';  % Location of tanh output neurons
  y1f       = [zeros(hiddenf,1);1];      % Hidden layer outputs
end


% >>>>>>>>>>>>>>>   DETERMINE STRUCTURE OF CONTROLLER NETWORK    <<<<<<<<<<<<<<<<<
hiddenc   = length(NetDefc(1,:));        % Number of hidden neurons
L_hiddenc = find(NetDefc(1,:)=='L')';    % Location of linear hidden neurons
H_hiddenc = find(NetDefc(1,:)=='H')';    % Location of tanh hidden neurons
L_outputc = find(NetDefc(2,:)=='L')';    % Location of linear output neurons
H_outputc = find(NetDefc(2,:)=='H')';    % Location of tanh output neurons
y1c       = [zeros(hiddenc,1);1];        % Hidden layer outputs


%>>>>>>>>>>>>>>>>>>>>>>>        INITIALIZE VARIABLES        <<<<<<<<<<<<<<<<<<<<<<
% Determine length of polynomials
nam = length(Am);
nbm = length(Bm);

% Initialization of past signals
maxlength = 6;                              % IT MIGHT BE NECESSARY TO INCREASE maxlength
ref_old  = repmat(y_0,maxlength,1);         % FOR HIGH ORDER SYSTEMS !!
y_old    = repmat(y_0,maxlength,1);
u_old    = repmat(u_0,maxlength,1);

% Initialization of PID parameters
if strcmp(regty,'pid'),
  B1 = K*(1+Ts*Wi/2);
  A1 = Ts*Wi;
  B2 = (2*Td+Ts)/(2*alf*Td+Ts);
  A2 = 2*Ts/(2*alf*Td+Ts);
  I1 = 0;
  I2 = 0;
  uimin = -10; uimax = 10;
end


% Miscellanous initializations
t    = -Ts;
u    = u_0;
y    = y_0;
yhat = y_0;


% Initialization of Simulink system
if strcmp(simul,'simulink')
  simoptions = simset('Solver',integrator,'MaxRows',0); % Set integrator opt.
  eval(['[sizes,x0] = ' sim_model '([],[],[],0);']);    % Get initial states
end

%>>>>>>>>>>>>>>>>>    CALCULATE REFERENCE SIGNAL & FILTER IT     <<<<<<<<<<<<<<<<<<
if strcmp(refty,'siggener'),
  ref = zeros(samples+1,1);
  for i = 1:samples+1,
    ref(i) = siggener(Ts*(i-1),sq_amp,sq_freq,sin_amp,sin_freq,dc,sqrt(Nvar));
    ref_old  = shift(ref_old,ref(i));
  end
elseif strcmp(refty,'none'),
  ref = zeros(samples+1,1);
else
  eval(['ref = ' refty ';']);
  ref=ref(:);
  i=length(ref);
  if i>samples+1,
    ref=ref(1:samples+1);
  else
    ref=[ref;ref(i)*ones(samples+1-i,1)];
  end
end
ref=filter(Bm,Am,ref);

% Initialization of data vectors
ref_data    = ref(1:samples);
u_data      = zeros(samples,1);
y_data      = zeros(samples,1);
t_data      = zeros(samples,1);
fighandle=progress;

%---------------------------------------------------------------------------------
%-------------------          >>>   MAIN LOOP   <<<            -------------------
%---------------------------------------------------------------------------------
for i=1:samples,
  t = t + Ts;
   

%>>>>>>>>>>>>>>>>>>>   READ OUTPUT FROM THE PHYSICAL SYSTEM     <<<<<<<<<<<<<<<<<<
  if strcmp(simul,'simulink')
    utmp=[t-Ts,u_old(1);t,u_old(1)];
    simoptions.InitialState=x0;
    [time,x0,y] = sim(sim_model,[t-Ts t],simoptions,utmp);
    x0 = x0(size(x0,1),:)';
    y  = y(size(y,1),:)';
  elseif strcmp(simul,'matlab')
    ugl = u_old(1);
    [time,x] = ode45(mat_model,[t-Ts t],x0);
    x0 = x(length(time),:)';
    eval(['y = ' model_out '(x0);']);
  elseif strcmp(simul,'nnet')
    phi = [y_old(1:na);u_old(1:nb);1];
    h1f = W1f*phi;
    y1f(H_hiddenf)  = pmntanh(h1f(H_hiddenf));
    y1f(L_hiddenf)  = h1f(L_hiddenf);    
    h2f = W2f*y1f;
    y(H_outputf) = pmntanh(h2f(H_outputf));
    y(L_outputf) = h2f(L_outputf);
  end


%>>>>>>>>>>>>>>>>>>>>>>      DETERMINE CONTROL SIGNAL       <<<<<<<<<<<<<<<<<<<<<<
  e = ref(i) - y;
  
  % Control using the network
  if strcmp(regty,'opt'),
    phi = [ref(i+1);y;y_old(1:na-1);u_old(1:nb-1);1];
    h1  = W1c*phi;  
    y1c(H_hiddenc) = pmntanh(h1(H_hiddenc));
    y1c(L_hiddenc) = h1(L_hiddenc);    
    h2 = W2c*y1c;
    u(H_outputc)  = pmntanh(h2(H_outputc));
    u(L_outputc)  = h2(L_outputc);


  % PID controller
    elseif strcmp(regty,'pid'),
    ui = B1*e + I1;
    um = ui;
    if ui<uimin, um=uimin; end
    if ui>uimax, um=uimax; end
    u = (um-I2)*B2 + I2;
    I1 = I1 + (K*e - (ui - um))*A1;
    I2 = I2 + (um - I2)*A2;
  
  % No controller
  else
     u = ref(i);
  end
  
  % Make sure control inputs is within limits
  if u>ulim_max,
     u=ulim_max;
  elseif u<ulim_min
     u=ulim_min;
  end

  
 %>>>>>>>>>>>>>>>>       COPY DATA INTO THE DATA VECTORS        <<<<<<<<<<<<<<<
  u_data(i)       = u;
  y_data(i)       = y;
  t_data(i)       = t;


%>>>>>>>>>>>>>>>>>>>>>>>         TIME OPDATES          <<<<<<<<<<<<<<<<<<<<<<<<
  y_old    = shift(y_old,y);
  u_old    = shift(u_old,u);
  ref_old  = shift(ref_old,ref(i));


%>>>>>>>>>>>>>>>       PRINT %-AGE OF SIMULATION COMPLETED       <<<<<<<<<<<<<<
  progress(fighandle,floor(100*i/samples));
end
%------------------------------------------------------------------------------
%----------------         >>>   END OF MAIN LOOP   <<<        ----------------
%------------------------------------------------------------------------------
 
%>>>>>>>>>>>>>>>>>>>>>>            DRAW PLOTS           <<<<<<<<<<<<<<<<<<<<<<<
figure(gcf);clf
set(gcf,'DefaultTextInterpreter','none');

% Plot A
  if(exist('plot_a')==1),
   a_plots=length(plot_a);            % Number of plots in plot A
   plmat = zeros(samples,a_plots);    % Collect vectors in plmat
   for nn = 1:a_plots, 
     plmat(:,nn) = eval(plot_a{nn});   
   end
   subplot(2,1,1);
   plot([0:samples-1],plmat);           % Plot plmat
   xlabel('Samples');
   set(gca,'Xlim',[0 samples-1]);       % Set x-axis
   if regty(1)=='o',
     title('Optimal control with a neural network');
   elseif regty(1)=='p',
     title('Constant gain PID controller');
   else
     title('Open-loop simulation');
   end
   grid on
   legend(plot_a{:});
  end
  
 % Plot B
  if(exist('plot_b')==1),
   b_plots=length(plot_b);              % Number of plots in plot B
   plmat = zeros(samples,b_plots);      % Collect vectors in plmat
   for nn = 1:b_plots, 
     plmat(:,nn) = eval(plot_b{nn});   
   end
   subplot(2,1,2);
   plot([0:samples-1],plmat);           % Plot plmat
   xlabel('Samples'); 
   set(gca,'Xlim',[0 samples-1]);       % Set x-axis
   grid on
   legend(plot_b{:});
  end
set(gcf,'DefaultTextInterpreter','tex');
subplot(111)


