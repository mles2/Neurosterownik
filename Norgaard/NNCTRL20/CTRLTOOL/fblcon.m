% ---------------------------------    FBLCON     --------------------------------
%
%  Program for simulating control of nonlinear processes using feedback
%  linearization.
%
%  All design parameters must be defined in the file 'fblinit.m'

%  Wriiten by Magnus Norgaard, IAU, Technical University of Denmark.
%  LastEditDate: Jan. 23, 2000


%----------------------------------------------------------------------------------
%-------------------         >>>  INITIALIZATIONS  <<<        ---------------------
%----------------------------------------------------------------------------------

%>>>>>>>>>>>>>>>>>>>>>>      READ VARIABLES FROM FILE       <<<<<<<<<<<<<<<<<<<<<<<
clear plot_a plot_b
global ugl
fblinit
eval(['load ' nnfile]);
 

% >>>>>>>>>>>>>>>>>>>>>>>>   DETERMINE REGRESSOR STRUCTURE   <<<<<<<<<<<<<<<<<<<<<<   
na = NN(1);                             % # of past y's to be used in TDL
nb = NN(2);                             % # of past u's to be used in TDL
nk = NN(3);                             % Time delay in system
nmax        = max(na,nb+nk-1);
nab         = na+sum(nb);               % Number of inputs to each net
outputs  = 1;                           % # of outputs
inputs   = nab-1;                       % # of inputs
phi = zeros(inputs,1);                  % Initialize regressor vector


% >>>>>>>>>>>>>>>>>>>      DETERMINE NETWORK ARCHITECTURE      <<<<<<<<<<<<<<<<<<<<<
% ---------- f-net architecture ----------
hiddenf = length(NetDeff(1,:));         % Number of hidden neurons in f-net
L_hiddenf = find(NetDeff(1,:)=='L')';   % Location of linear hidden neurons
H_hiddenf = find(NetDeff(1,:)=='H')';   % Location of tanh hidden neurons
L_outputf = find(NetDeff(2,:)=='L')';   % Location of linear output neurons
H_outputf = find(NetDeff(2,:)=='H')';   % Location of tanh output neurons
y1f       =[zeros(hiddenf,1);1];        % Hidden layer outputs
f         = zeros(outputs,1);           % Network output

% ---------- g-net architecture ----------
hiddeng = length(NetDefg(1,:));         % Number of hidden neurons in g-net
L_hiddeng = find(NetDefg(1,:)=='L')';   % Location of linear hidden neurons
H_hiddeng = find(NetDefg(1,:)=='H')';   % Location of tanh hidden neurons
L_outputg = find(NetDefg(2,:)=='L')';   % Location of linear output neurons
H_outputg = find(NetDefg(2,:)=='H')';   % Location of tanh output neurons
y1g       =[zeros(hiddeng,1);1];        % Hidden layer outputs
g         = 0;                          % g network output
f         = y_0;                        % f network output


%>>>>>>>>>>>>>>>>>>>>>>>        INITIALIZE VARIABLES        <<<<<<<<<<<<<<<<<<<<<<
% Determine length of polynomials
nam = length(Am);
if (nam-1)~=na,
  fprintf('\nWrong order of desired characteristic polynomial\n');
end


% Initialization of past signals
maxlength = 5;                          % MIGHT BE NECESSARY TO INCREASE maxlength
ref_old   = repmat(y_0,maxlength,1);    % FOR HIGH-ORDER SYSTEMS!!!
y_old     = repmat(y_0,maxlength,1);
ym_old    = repmat(y_0,maxlength,1);
u_old     = repmat(u_0,maxlength,1);

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
t = 0;
u    = u_0;
yhat = y_0;
fighandle=progress;

% Initialization of Simulink system
if strcmp(simul,'simulink')
  simoptions = simset('Solver',integrator,'MaxRows',0); % Set integrator opt.
  eval(['[sizes,x0] = ' sim_model '([],[],[],0);']);    % Get initial states
end

% Initialization of data vectors
ref_data    = zeros(samples,1);
u_data      = zeros(samples,1);
y_data      = zeros(samples,1);
yhat_data   = zeros(samples,1);
ym_data     = zeros(samples,1);
t_data      = zeros(samples,1);


% A predefined vector contains the reference
if ~(strcmp(refty,'siggener')|strcmp(refty,'none')),
  eval(['ref_data = ' refty ';']);
  ref_data=ref_data(:);
  i=length(ref_data);
  if i>=samples,
    ref_data=ref_data(1:samples);
  else
    ref_data=[ref_data;ref_data(i)*ones(samples-i,1)];
  end
end

%------------------------------------------------------------------------------
%-------------------         >>>   MAIN LOOP   <<<           ------------------
%------------------------------------------------------------------------------

for i=1:samples,
  t = t + Ts;
  
%>>>>>>>>>>>>>>>>>>>>>     GENERATE REFERENCE SIGNAL      <<<<<<<<<<<<<<<<<<<<<
  if strcmp(refty,'siggener')
    ref = siggener(t,sq_amp,sq_freq,sin_amp,sin_freq,dc,sqrt(Nvar));
  else                  % Predfined reference
    ref = ref_data(i);
  end


%>>>>>>>>>>>>>>>>>>>   COMPUTE OUTPUT FROM DESIRED SYSTEM  <<<<<<<<<<<<<<<<<<<<
  ym = sum(- Am(2:nam)*ym_old(1:nam-1)) + sum(Am)*ref_old(1);
  


%>>>>>>>>>>>>>>>>>>>   OUTPUT PREDICTED BY THE NEURAL NET    <<<<<<<<<<<<<<<<<<
  yhat = f + g*u_old(1);



%>>>>>>>>>>>>>>>>>>>  READ OUTPUT FROM THE PHYSICAL SYSTEM   <<<<<<<<<<<<<<<<<<
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
    y=yhat;
  end



%>>>>>>>>>>>>>>    COMPUTE OUTPUT PREDICTED BY THE NEURAL NET    <<<<<<<<<<<<<<
  phi = [y;y_old(1:na-1);u_old(1:nb-1);1];
  h1f = W1f*phi;  
  y1f(H_hiddenf) = pmntanh(h1f(H_hiddenf));
  y1f(L_hiddenf) = h1f(L_hiddenf);    
  h2f = W2f*y1f;
  f(H_outputf)   = pmntanh(h2f(H_outputf));
  f(L_outputf)   = h2f(L_outputf);

  h1g = W1g*phi;  
  y1g(H_hiddeng) = pmntanh(h1g(H_hiddeng));
  y1g(L_hiddeng) = h1g(L_hiddeng);    
  h2g = W2g*y1g;
  g(H_outputg)   = pmntanh(h2g(H_outputg));
  g(L_outputg)   = h2g(L_outputg);



%>>>>>>>>>>>>>>>>>>>>>>     DETERMINE CONTROL SIGNAL      <<<<<<<<<<<<<<<<<<<<<<
  e = ref - y;

  % Feedback Linearizing Controller
  if strcmp(regty,'fbl'),
    w  = sum(Am)*ref - sum(Am(2:nam)*[y;y_old(1:nam-2)]);
    u = (w - f)/g;
 
    
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
     u = ref;
  end
  
  % Make sure control input is within limits
  if u>ulim_max,
     u=ulim_max;
  elseif u<ulim_min
     u=ulim_min;
  end
 
  
 %>>>>>>>>>>>>>>>>       COPY DATA INTO THE DATA VECTORS        <<<<<<<<<<<<<<<
  ref_data(i)     = ref;
  u_data(i)       = u;
  y_data(i)       = y;
  yhat_data(i)    = yhat;
  ym_data(i)      = ym;
  t_data(i)       = t;


%>>>>>>>>>>>>>>>>>>>>>>>         TIME OPDATES          <<<<<<<<<<<<<<<<<<<<<<<<
  y_old    = shift(y_old,y);
  u_old    = shift(u_old,u);
  ref_old  = shift(ref_old,ref);
  ym_old   = shift(ym_old,ym);


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
   if regty(1)=='f',
     title('Control by feedback linearization');
   elseif regty(1)=='p',
     title('Constant gain PID controller');
   else
     title('Open-loop simulation');
   end
   grid on
   legend(plot_a{:})
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
   legend(plot_b{:})
   end
set(gcf,'DefaultTextInterpreter','tex'); 
subplot(111)

