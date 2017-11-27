function [y_data,u_data] = experim
%  EXPERIM
%          Function for generating data for neural network training.
%          Call:
%                [Y,U]=experim;
%          Data can be generated using models specified in either SIMULINK or
%          as a set of differential equations in a Matlab-finction. The
%          function returns the input and output sequences in the vectors
%          U and Y.
%
%          All design parameters must be defined in the file 'expinit.m'
%
%          NB NB NB
%          For historical reasons (I have now forgotten), U and Y are
%          returned as column vectors. This is incompatible with the
%          network training functions, which all require the data to be
%          passed as row vectors. Thus, remember to transpose U and Y!
 
% Written by Magnus Norgaard. LastEditDate Mar. 30, 2003.

%----------------------------------------------------------------------------------
%-------------------          >>>  INITIALIZATIONS  <<<         -------------------
%----------------------------------------------------------------------------------

%>>>>>>>>>>>>>>>>>>>>>>      READ VARIABLES FROM FILE       <<<<<<<<<<<<<<<<<<<<<<<
clear plot_a plot_b
global ugl
expinit;


% >>>>>>>>>>>>>>>>>>>>>>>        INITIALIZE VARIABLES        <<<<<<<<<<<<<<<<<<<<<<
% Initialization of past signals
maxlength = 5;                          % MIGHT BE NECESSARY TO INCREASE maxlength
ref_old   = repmat(y_0,maxlength,1);    % FOR HIGH-ORDER SYSTEMS!!!
y_old     = repmat(y_0,maxlength,1);
u_old     = repmat(u_0,maxlength,1);
uc_old    = repmat(u_0,maxlength,1);


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
t      = -Ts;
u      = u_0;
yhat   = y_0;

fighandle=progress;

if strcmp(simul,'simulink')
  simoptions = simset('Solver',integrator,'MaxRows',0); % Set integrator opt.
  eval(['[sizes,x0] = ' sim_model '([],[],[],0);']);    % Get initial states
end


% Initialization of data vectors
ref_data    = zeros(samples,1);
probe_data  = zeros(samples,1);
u_data      = zeros(samples,1);
y_data      = zeros(samples,1);
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

% Initialize probing signal
if ~strcmp(probety,'none'),
  eval(['probe_data = ' probety ';']);
  probe_data=probe_data(:);
  i=length(ref_data);
  if i>=samples,
    probe_data=probe_data(1:samples);
  else
    probe_data=[probe_data;probe_data(i)*ones(samples-i,1)];
  end
end


%------------------------------------------------------------------------------
%-------------------         >>>   MAIN LOOP   <<<           ------------------
%------------------------------------------------------------------------------
for i=1:samples,
  t = t + Ts;

  
%>>>>>>>>>>>>>>>>>>>>>    CALCULATE REFERENCE SIGNAL     <<<<<<<<<<<<<<<<<<<<<<
  if strcmp(refty,'siggener')
    ref = siggener(t,sq_amp,sq_freq,sin_amp,sin_freq,dc,sqrt(Nvar));
  else                  % Predefined reference
    ref = ref_data(i);
  end

%>>>>>>>>>>>>>>>>>>>>>    GENERATE PERTURBATION SIGNAL    <<<<<<<<<<<<<<<<<<<<<
  probe = probe_data(i);


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
  end


%>>>>>>>>>>>>>>>>>>>>>>     DETERMINE CONTROL SIGNAL      <<<<<<<<<<<<<<<<<<<<<<
  e = ref - y;

  % RST controller
  if strcmp(regty,'rst'),
    nr = length(R);
    ns = length(S);
    nt = length(T);
    u1 = sum(T*[ref;ref_old(1:nt-1)]);
    u2 = sum(S*[y;y_old(1:ns-1)]);
    u3 = sum(R(2:nr)*uc_old(1:nr-1));
    uc = ( u1 - u2 - u3 ) / R(1);
    
  % PID controller
  elseif strcmp(regty,'pid'),
    ui = B1*e + I1;
    um = ui;
    if ui<uimin, um=uimin; end
    if ui>uimax, um=uimax; end
    uc = (um-I2)*B2 + I2;
    I1 = I1 + (K*e - (ui - um))*A1;
    I2 = I2 + (um - I2)*A2;
  
  % No controller
  else
     uc = ref;
  end
 
  
  % Add pertubation signal
  u = uc + probe;
  
  % Make sure control inputs is within limits
  if u>ulim_max,
     u=ulim_max;
  elseif u<ulim_min
     u=ulim_min;
  end
 

%>>>>>>>>>>>>>>>>>       COPY DATA INTO THE DATA VECTORS        <<<<<<<<<<<<<<<
  ref_data(i)     = ref;
  u_data(i)       = u;
  y_data(i)       = y;
  t_data(i)       = t;


%>>>>>>>>>>>>>>>>>>>>>>>         TIME OPDATES          <<<<<<<<<<<<<<<<<<<<<<<<
  y_old    = shift(y_old,y);
  uc_old   = shift(uc_old,uc);
  u_old    = shift(u_old,u);
  ref_old  = shift(ref_old,ref);


%>>>>>>>>>>>>>>>       PRINT %-AGE OF SIMULATION COMPLETED       <<<<<<<<<<<<<<
  progress(fighandle,floor(100*i/samples));
end
%------------------------------------------------------------------------------
%----------------         >>>   END OF MAIN LOOP   <<<        -----------------
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
  title('Data generated in the experiment');
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
