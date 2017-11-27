% ----------------------------------   GPCCON.M    --------------------------------
%
% Program for simulating generalized predictive controllers (GPC)
%
% All design parameters must be defined in the file 'gpcinit.m'
%
%
% Written by Magnus Norgaard, IAU, Technical University of Denmark.
% LastEditDate: Jan. 23, 2000

%----------------------------------------------------------------------------------
%-------------------         >>>  INITIALIZATIONS  <<<        ---------------------
%----------------------------------------------------------------------------------

%>>>>>>>>>>>>>>>>>>>>>>      READ VARIABLES FROM FILE       <<<<<<<<<<<<<<<<<<<<<<<
clear plot_a plot_b
global ugl
gpcinit


%>>>>>>>>>>>>>>>>>>>>>>>        INITIALIZE VARIABLES        <<<<<<<<<<<<<<<<<<<<<<
% Determine length of reference filter polynomials
na = length(A)-1;
nb = length(B);
nk = N1;
nam = length(Am);
nbm = length(Bm);

% Initialization of past signals
maxlength = 5;                          % MIGHT BE NECESSARY TO INCREASE maxlength
ref_old   = repmat(y_0,maxlength,1);    % FOR HIGH-ORDER SYSTEMS!!!
y_old     = repmat(y_0,maxlength,1);
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

% Initialization of APC polynomials and matrices
F = zeros(1,na+1);
E = [1 zeros(1,N2-1)];
G = zeros(1,N2+nb-1);
GAMMA   = zeros(N2-nk+1,Nu);
PHI   = zeros(N2,1);
F0    = zeros(N2,1);
rhoI  = rho*eye(Nu);
invHG = zeros(Nu,N2-N1+1);

% Miscellaneous initializations
t    = -Ts;

fighandle=progress;

% Initialization of Simulink system
if strcmp(simul,'simulink')
  simoptions = simset('Solver',integrator,'MaxRows',0); % Set integrator opt.
  eval(['[sizes,x0] = ' sim_model '([],[],[],0);']);    % Get initial states
end


%>>>>>>>>>>>>>>>>>    CALCULATE REFERENCE SIGNAL & FILTER IT     <<<<<<<<<<<<<<<<<<
if strcmp(refty,'siggener'),
  ref = zeros(samples+N2,1);
  for i = 1:samples+N2,
    ref(i) = siggener(Ts*(i-1),sq_amp,sq_freq,sin_amp,sin_freq,dc,sqrt(Nvar));
  end
elseif strcmp(refty,'none'),
  ref = zeros(samples+N2,1);
else
  eval(['ref = ' refty ';']);
  ref=ref(:);
  i=length(ref);
  if i>samples+N2,
    ref=ref(1:samples+N2);
  else
    ref=[ref;ref(i)*ones(samples+N2-i,1)];
  end
end
ref=filter(Bm,Am,ref);


%>>>>>>>>>>>   INITIALIZATION OF VECTORS USED FOR STORING PAST DATA    <<<<<<<<<<<<
ref_data    = ref(1:samples);
u_data      = zeros(samples,1);
y_data      = zeros(samples,1);
yhat_data   = zeros(samples,1);
t_data      = zeros(samples,1);


%------------------------------------------------------------------------------
%-------------------         >>>   MAIN LOOP   <<<           ------------------
%------------------------------------------------------------------------------
for i=1:samples,
  t = t + Ts;
  

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


%>>>>>>>>>>>>>>>>>>>>>>     CALCULATE CONTROL SIGNAL     <<<<<<<<<<<<<<<<<<<<<<
  e = ref(i) - y;
 
  % Predictive controller
  if strcmp(regty,'gpc'),
    PHI = PHI+F0*y;         % Complete calculation of the vector PHI
    U=invHG*(ref(i+N1-1:i+N2-nk)-PHI(N1:N2));
    u = U(1)+u_old(1);
        
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
     u = ref(i);
  end
  
  % Make sure control inputs is within limits
  if u>ulim_max,
     u=ulim_max;
  elseif u<ulim_min
     u=ulim_min;
  end
 

%>>>>>>>>>>>>>>>>>>>>>>>>>>       TIME UPDATES        <<<<<<<<<<<<<<<<<<<<<<<<<
  y_old    = shift(y_old,y);
  u_old    = shift(u_old,u);
  ref_old  = shift(ref_old,ref(i));


%>>>>>>>>>>>>>>>>>>>       STORE DATA IN DATA VECTORS      <<<<<<<<<<<<<<<<<<<
  u_data(i)       = u;
  y_data(i)       = y;
  t_data(i)       = t;


%------------------------------------------------------------------------------
%-----------      >>>   DESIGN CONTROLLER FOR NEXT SAMPLE   <<<      ----------
%------------------------------------------------------------------------------

%>>>>>>>>>>>>>>>>>>>       SOLVE DIOPHANTINE EQUATIONS     <<<<<<<<<<<<<<<<<<<
  Atilde  = [A 0];                         %              -1
  Atilde(2:na+2)=Atilde(2:na+2)-A(1:na+1); % Atilde = (1-q  )A
  F       = -Atilde(2:na+2);               % F(k) = q(1-Atilde)
  G = [B zeros(1,N2-1)];                   % G(1) = B
  GAMMA(1,1) = G(1);
  F0(1) = F(1);                            % Store F(1)_0
  PHI(1)= G(3-nk:nb)*(u_old(1:nb+nk-2)-u_old(2:nb+nk-1))+...
             F(2:na+1)*y_old(1:na);        % F(k)y(t)-F(k)_0*y(t) 
  for k=1:N2-1,
    G(k+1:k+nb) = G(k+1:k+nb)+B*F(1);      %  G(k+1)    = G(k)+ q^{-1}*B*F(k)_0 
    F      = [F(2:na+1) 0]-F(1)*Atilde(2:na+2); % F(k+1)= q(F(k)-Atilde*F(k)_0)
    PHI(k+1) = G(k-nk+3:k+nb)*(u_old(1:nb+nk-2)-u_old(2:nb+nk-1))+...
             F(2:na+1)*y_old(1:na);        % (G-??)u(t)+F(k)y(t)-F(k)_0*y(t)
    F0(k+1)  = F(1);                       % Store F(k)_0
  end
  
  % Insert G coefficients in GAMMA
  for j=1:Nu,
    GAMMA(j:N2-nk+1,j) = G(1:N2-nk+2-j)';
  end
  H = GAMMA(N1:N2,:)'*GAMMA(N1:N2,:)+rhoI;
  invHG=inv(H)*GAMMA(N1:N2,:)';


%>>>>>>>>>>>>>>>>>>      WRITE % OF SIMULATION COMPLETED      <<<<<<<<<<<<<<<<<
  progress(fighandle,floor(100*i/samples));
end
%------------------------------------------------------------------------------
%------------------        >>>   END OF MAIN LOOP   <<<       -----------------
%------------------------------------------------------------------------------

%>>>>>>>>>>>>>>>>>>>>>>            DRAW PLOTS           <<<<<<<<<<<<<<<<<<<<<<<
figure(gcf);clf;
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
   if regty(1)=='a',
     title('Approximate Predictive Control');
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
