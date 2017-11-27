% ----------------------------------   LINCON.M    --------------------------------
%
%  Program for simulating a control system based on the instantaneous
%  linearization technique. An "approximate" pole placement design and
%  an "approximate" minimum variance type of design have been implemented.  
%
%  All design parameters must be defined in the file 'lininit.m'

%  Programmed by Magnus Norgaard IAU, Technical University of Denmark.
%  LastEditDate: Jan. 22, 2000


%----------------------------------------------------------------------------------
%-------------------         >>>  INITIALIZATIONS  <<<        ---------------------
%----------------------------------------------------------------------------------

%>>>>>>>>>>>>>>>>>>>>>>      READ VARIABLES FROM FILE       <<<<<<<<<<<<<<<<<<<<<<<
clear plot_a plot_b
global ugl
lininit
eval(['load ' nnfile]);                % Load neural network


% >>>>>>>>>>>>>>>>>>>>>>>>   DETERMINE REGRESSOR STRUCTURE   <<<<<<<<<<<<<<<<<<<<<<   
na      = NN(1);                       % # of past y's to be used in TDL
nb      = NN(2);                       % # of past u's to be used in TDL
nk      = NN(3);                       % Time delay in system
nab     = na+sum(nb);                  % Number of inputs to each net
outputs = 1;                           % # of outputs is 1 (SISO system)
inputs  = nab;                         % # of inputs
phi     = zeros(inputs,1);             % Initialize regressor vector


% >>>>>>>>>>>>>>>>>    DETERMINE STRUCTURE OF NETWORK MODEL     <<<<<<<<<<<<<<<<<<<
hidden   = length(NetDef(1,:));        % Number of hidden neurons
L_hidden = find(NetDef(1,:)=='L')';    % Location of linear hidden neurons
H_hidden = find(NetDef(1,:)=='H')';    % Location of tanh hidden neurons
L_output = find(NetDef(2,:)=='L')';    % Location of linear output neurons
H_output = find(NetDef(2,:)=='H')';    % Location of tanh output neurons
y1       = [zeros(hidden,1)];          % Hidden layer outputs
yhat     = zeros(outputs,1);           % Network output
y        = 0;


%>>>>>>>>>>>>>>>>>>>>>>>        INITIALIZE VARIABLES        <<<<<<<<<<<<<<<<<<<<<<
% Determine length of polynomials
nam = length(Am);
nbm = length(Bm);

% Initialization of past signals
maxlen = 5;                            % MIGHT BE NECESSARY TO INCREASE maxlen
ref_old  = repmat(y_0,maxlen,1);       % FOR HIGH-ORDER SYSTEMS!!!
y_old    = repmat(y_0,maxlen,1);
ym_old   = repmat(y_0,maxlen,1);
yhat_old = repmat(y_0,maxlen,1);
u_old    = repmat(u_0,maxlen,1);

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

% Miscellaneous initializations
t = -Ts;
yhat = y_0;
A     = [1 zeros(1,na)];
B     = zeros(1,nb);
fighandle=progress;

% Initialization of Simulink system
if strcmp(simul,'simulink')
  simoptions = simset('Solver',integrator,'MaxRows',0); % Set integrator opt.
  eval(['[sizes,x0] = ' sim_model '([],[],[],0);']);    % Get initial states
end

% Initializations of vectors used to store old data
ref_data    = zeros(samples,1);
u_data      = zeros(samples,1);
y_data      = zeros(samples,1);
yhat_data   = zeros(samples,1);
ym_data     = zeros(samples,1);
t_data      = zeros(samples,1);
A_data      = zeros(samples,na+1);
B_data      = zeros(samples,nb);

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
  
  
%>>>>>>>>>>>>>>>>>>>>>    CALCULATE REFERENCE SIGNAL     <<<<<<<<<<<<<<<<<<<<<<
  if strcmp(refty,'siggener')
    ref = siggener(t,sq_amp,sq_freq,sin_amp,sin_freq,dc,sqrt(Nvar));
  else                  % Predefined reference
    ref = ref_data(i);
  end

%>>>>>>>>>>>>>>>>>>   CALCULATE OUTPUT FROM DESIRED MODEL   <<<<<<<<<<<<<<<<<<<
  ym = sum(- Am(2:nam)*ym_old(1:nam-1)) + Bm(1:nbm)*ref_old(nk:nk+nbm-1);


%>>>>>>>>>>>>>>>>>>>>>>>>    READ OUTPUT FROM PLANT     <<<<<<<<<<<<<<<<<<<<<<<
  if strcmp(simul,'simulink')
    utmp=[t-Ts,u_old(1);t,u_old(1)];
    simoptions.InitialState=x0;
    [time,x0,y] = sim(sim_model,[t-Ts t],simoptions,utmp);
    x0 = x0(size(x0,1),:)';
    y  = y(size(y,1),:)';
  elseif strcmp(simul,'matlab')
    ugl = u_old(nk);
    [time,x] = ode45(mat_model,[t-Ts t],x0);
    x0 = x(length(time),:)';
    eval(['y = ' model_out '(x0);']);
  end
  ey = y - yhat;                          % prediction error (a priori)


%>>>>>>>>>>>>>>>>>>>>>>     CALCULATE CONTROL SIGNAL     <<<<<<<<<<<<<<<<<<<<<<
  e = ref - y;
  
  % RST controller
  if strcmp(regty,'rst'),
    if i==1, u=0;
    else
      ns=length(S);
      nr=length(R);
      nt=length(T);
      u = S(1)*y + sum(S(2:ns)*y_old(1:ns-1)) + sum(R(2:nr)*u_old(1:nr-1));
      u = ( T(1)*ref + sum(T(2:nt)*ref_old(1:nt-1))- u) / R(1);
    end


  % PID controller
  elseif strcmp(regty,'pid'),
    ui = B1*e + I1;
    um = ui;
    if ui<uimin, um=uimin; end
    if ui>uimax, um=uimax; end
    u = (um-I2)*B2 + I2;
    I1 = I1 + (K*e - (ui - um))*A1;
    I2 = I2 + (um - I2)*A2;
  
  % No control
  else
     u = ref;
  end
  
  % Make sure control inputs is within limits
  if u>ulim_max,
     u=ulim_max;
  elseif u<ulim_min
     u=ulim_min;
  end
  
 %>>>>>>>>>>>>>>>>>>>       STORE DATA IN DATA VECTORS      <<<<<<<<<<<<<<<<<<<
  ref_data(i)     = ref;
  u_data(i)       = u;
  y_data(i)       = y;
  yhat_data(i)    = yhat;
  ym_data(i)      = ym;
  t_data(i)       = t;
  A_data(i,:)     = A;
  B_data(i,:)     = B;


%>>>>>>>>>>>>>>>>>>>>>>>>>>       TIME UPDATES        <<<<<<<<<<<<<<<<<<<<<<<<<
  y_old    = shift(y_old,y);
  u_old    = shift(u_old,u);
  ref_old  = shift(ref_old,ref);
  ym_old   = shift(ym_old,ym);


%------------------------------------------------------------------------------
%-----------      >>>   DESIGN CONTROLLER FOR NEXT SAMPLE   <<<      ----------
%------------------------------------------------------------------------------
%>>>>>>>>>>>>>>>>>>>>>  CALCULATE OUTPUT PREDICTED BY NN   <<<<<<<<<<<<<<<<<<<<
   phi      = [y_old(1:na);u_old(nk:nk+nb-1)];
   h1 = W1(:,1:inputs)*phi + W1(:,inputs+1);  
   y1(H_hidden) = pmntanh(h1(H_hidden)); 
   y1(L_hidden) = h1(L_hidden);
   h2 = W2(:,1:hidden)*y1 + W2(:,hidden+1);
   yhat(H_output) = pmntanh(h2(H_output));
   yhat(L_output) = h2(L_output);
   if strcmp(simul,'nnet')
     y = yhat;
   end


%>>>>>>>>>>>>>>>>>>>>>>   GET LINEAR PARAMETERS FROM NN  <<<<<<<<<<<<<<<<<<<<<<
   % Matrix consisting of the partial derivatives of each output with
   % respect to each of the outputs from the hidden neurons
   d21 = W2;
   for j = H_output',
     d21(j,:) = (1-yhat(j)*yhat(j))*W2(j,:);
   end

   % Matrix with partial derivatives of the output from each hidden neurons
   % with respect to each input:
   d10 = W1;
   for j = H_hidden',
     d10(j,:) = (1-y1(j)*y1(j))*W1(j,:);
   end

   % Matrix with partial derivative of each output with respect to each input
   d20 = d21(1:hidden)*d10;

   A = [1 -d20(1,1:na)];
   B = d20(1,na+1:nab);


%>>>>>>>>>>>>>>>>>>>>>>>>>      CONTROLLER DESIGN      <<<<<<<<<<<<<<<<<<<<<<<<

   % Pole placement without no zeros canceled
   if strcmp(design,'ppnz')==1,
     [R,S,T] = dio(A,B,nk,Am,1,Ao,Ar,As);
     T       = T*sum(Am)/sum(B);

   % Pole placement with all zeros canceled
   elseif strcmp(design,'ppaz')==1,
     [R,S,T] = dio(A,1,nk,Am,Bm,Ao,Ar,As);
     R       = conv(B,R);

   % MV1 design
   elseif strcmp(design,'mv1')==1,
     AAr     = conv(A,Ar);
     [R,S]   = diophant(AAr,1,nk,1);  
     R       = conv(B,R);
     R(1)    = R(1) + delta;
     R       = conv(R,Ar);    
     T       = 1;
   end


%>>>>>>>>>>>>>>>>>>      WRITE % OF SIMULATION COMPLETED      <<<<<<<<<<<<<<<<<
  progress(fighandle,floor(100*i/samples));
end
%------------------------------------------------------------------------------
%------------------        >>>   END OF MAIN LOOP   <<<       -----------------
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
   plot([0:samples-1],plmat);             % Plot plmat
   xlabel('Samples');
   set(gca,'Xlim',[0 samples-1]);         % Set x-axis
   if regty(1)=='r',
     if design(3)=='n', title('Pole placement without zero cancellation');
     elseif design(3)=='a', title('Pole placement with all zeros canceled');
     elseif design(1)=='m', title('MV1 controller');
     end
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
   plot([0:samples-1],plmat);             % Plot plmat
   xlabel('Samples'); 
   set(gca,'Xlim',[0 samples-1]);         % Set x-axis
   grid on
   legend(plot_b{:})
 end
set(gcf,'DefaultTextInterpreter','tex');
subplot(111)

