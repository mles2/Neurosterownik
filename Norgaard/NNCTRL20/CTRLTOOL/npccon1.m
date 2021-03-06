% ----------------------------------   NPCCON1.M    -------------------------------
%
%  This program simulates a predictive controller applied to a process
%  modelled by a nonlinear neural network model. 
%
%  Minimization of the predictive control criterion:
%
%                     N2                              Nu
%                    ---         ^        2           ---                    2
%       J(t,U)  =    >   (r(t+k)-y(t+k|t))   +  rho * >   (u(t+k-1)-u(t+k-2))
%                    ---                              --- 
%                    k=N1                             k=1
%
%  is done with a Quasi-Newton method using the BFGS-algorithm for
%  approximating the inverse Hessian. The algorithm is implemented with
%  the "soft" line search described in:
%                   K. Madsen: "Optimering uden bibetingelser." (in Dansih)
%                   IMM, Technical University of Denmark, 1984
%
%  The proposed algorithm is inspired by the considerations in:
%    J.E. Dennis & J.J More: "Quasi-Newton Methods, Motivation and Theory"
%    SIAM Review 19, pp. 46-89, 1977
%
%  All design parameters must be defined in the file 'npcinit.m'
%
%  Programmed by Magnus Norgaard, IAU/IMM, Technical Univ. of Denmark
%  LastEditDate: Jan. 23, 2000


%----------------------------------------------------------------------------------
%-------------------         >>>  INITIALIZATIONS  <<<        ---------------------
%----------------------------------------------------------------------------------

%>>>>>>>>>>>>>>>>>>>>>>      READ VARIABLES FROM FILE       <<<<<<<<<<<<<<<<<<<<<<<
clear plot_a plot_b
global ugl
npcinit
eval(['load ' nnfile]);                % Load neural network


% >>>>>>>>>>>>>>>>>>>>>>>>   DETERMINE REGRESSOR STRUCTURE   <<<<<<<<<<<<<<<<<<<<<<   
na      = NN(1);                       % # of past y's to be used in TDL
nb      = NN(2);                       % # of past u's to be used in TDL
nk      = NN(3);                       % Time delay in system
d       = NN(3);                       % Time delay in addition to the usual 1 
N1      = d;                           % N1<>d not fully implemented yet. 
inputs  = na+sum(nb);                  % Total number of inputs to network
outputs = 1;                           % # of outputs is 1 (SISO system)
phi     = zeros(inputs,1);             % Initialize regression vector


% >>>>>>>>>>>>>>>>>    DETERMINE STRUCTURE OF NETWORK MODEL     <<<<<<<<<<<<<<<<<<<
hidden   = length(NetDef(1,:));        % Number of hidden neurons
L_hidden = find(NetDef(1,:)=='L')';    % Location of linear hidden neurons
H_hidden = find(NetDef(1,:)=='H')';    % Location of tanh hidden neurons
y1       = zeros(hidden,N2-N1+1);      % Hidden layer outputs
yhat     = zeros(outputs,1);           % Network output


%>>>>>>>>>>>>>>>>>>>>>>>        INITIALIZE VARIABLES        <<<<<<<<<<<<<<<<<<<<<<
% Determine length of reference filter polynomials
nam = length(Am);
nbm = length(Bm);

% Initialization of past signals
maxlength = 5;                          % MIGHT BE NECESSARY TO INCREASE maxlength
ref_old   = repmat(y_0,maxlength,1);    % FOR HIGH-ORDER SYSTEMS!!!
y_old     = repmat(y_0,N2,1);
yhat_old  = repmat(y_0,N2,1);
u_old     = repmat(u_0,maxlength,1);

% Initialization of constant gain PID parameters
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
df  = ones(hidden,1);
dUtilde_dU = eye(Nu);
dUtilde_dU(1:Nu-1,2:Nu)=dUtilde_dU(1:Nu-1,2:Nu)-eye(Nu-1);
dY_dU      = zeros(Nu,N2-N1+1);  % Initialize matrix of partial derivatives
u          = u_0;                % The controls up to time t<=0
up         = u(ones(Nu,1));      % Initial future controls
upmin      = up;
beta0      = 0.9;                % Associated with the line search
t = -Ts;
delta2     = delta*delta;
fighandle=progress;

% Initialization of Simulink system
if strcmp(simul,'simulink')
  simoptions = simset('Solver',integrator,'MaxRows',0); % Set integrator opt.
  eval(['[sizes,x0] = ' sim_model '([],[],[],0);']);    % Get initial states
end


%>>>>>>>>>>>>>>>>    CALCULATE REFERENCE SIGNAL & FILTER IT     <<<<<<<<<<<<<<<<<<
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


% Initializations of vectors used for storing old data
ref_data    = ref(1:samples);
u_data      = zeros(samples,1);
y_data      = zeros(samples,1);
yhat_data   = zeros(samples,1);
t_data      = zeros(samples,1);
e_data      = zeros(samples,1);


% Data vectors used in control algorithm
% u_vec : First element is u(t-d+N1-nb+1), last element is u(t-d+N2)
% Index to time t is thus: tiu=d-N1+nb ("t index u_vec")
u_vec = repmat(u,N2-N1+nb,1);
tiu   = d-N1+nb;
upi   = [1:Nu-1 Nu(ones(1,N2-d-Nu+2))];     % [1 2 ... Nu Nu ... Nu] 
uvi   = [tiu:N2-N1+nb];

% y_vec: First element is y(t-na), last element is y(t-1)
% Index to time t is: tiy=na+1
y_vec = repmat(y_0,na,1);
tiy   = na+1;

% yhat_vec: First element is yhat(t), last element is yhat(t+N2)
% Index to time t is: tiyh=1
yhat_vec = repmat(y_0,N2+1,1);
tiyh = 1;


%------------------------------------------------------------------------------
%-------------------         >>>   MAIN LOOP   <<<           ------------------
%------------------------------------------------------------------------------
for i=1:samples,
  t = t + Ts;
  
%>>>>>>>>>>>>>>>>>>>>>>>>    PREDICT OUTPUT FROM PLANT  <<<<<<<<<<<<<<<<<<<<<<<
  phi            = [y_vec(na:-1:1);u_old(d:d+nb-1)];
  h1             = W1(:,1:inputs)*phi + W1(:,inputs+1);  
  y1(H_hidden,1) = pmntanh(h1(H_hidden)); 
  y1(L_hidden,1) = h1(L_hidden);
  yhat           = W2(:,1:hidden)*y1(:,1) + W2(:,hidden+1);
  yhat_vec(tiyh) = yhat;


%>>>>>>>>>>>>>>>>>>>>>>>>    READ OUTPUT FROM PLANT     <<<<<<<<<<<<<<<<<<<<<<<
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
    y = yhat;
  end

 
  %--------------------------------------------------------------------------------
  %----------     >>>  FIND NEW CONTROL SIGNAL BY OPTIMIZATION    <<<    ----------
  %--------------------------------------------------------------------------------
  upmin0   = upmin([2:Nu Nu]);
  einitval = eval(initval);     % Evaulate inival string
  for tr=1:length(einitval),
    up=upmin0;                  % Initial value for numerical search for a new u  
    up(Nu)=einitval(tr);
    u_vec(uvi) = up(upi);  
    dw = 1;                     % Flag specifying that up is new
    lambda = 0.1;               % Initialize Levenberg-Marquardt parameter
  
  
  %>>>>>>>>>>>>>>> COMPUTE PREDICTIONS FROM TIME t+N1 TO t+N2 <<<<<<<<<<<<<<<<
    for k=N1:N2,
      %----- Determine prediction yhat(t+k) -----
      phi              = [yhat_vec(tiyh+k-1:-1:tiyh+k-min(k,na)) ; ...
               y_vec(tiy-1:-1:tiy-max(na-k,0)) ; u_vec(tiu-d+k:-1:tiu-d+1-nb+k)];
      h1               = W1(:,1:inputs)*phi + W1(:,inputs+1);  
      y1(H_hidden,k-N1+1) = pmntanh(h1(H_hidden)); 
      y1(L_hidden,k-N1+1) = h1(L_hidden);
      yhat_vec(tiyh+k) = W2(:,1:hidden)*y1(:,k-N1+1) + W2(:,hidden+1);
    end
  
  
  %>>>>>>>>>>>>>>>>>>>>>>    EVALUATE CRITERION    <<<<<<<<<<<<<<<<<<<<<<
    duvec = u_vec(tiu:tiu+Nu-1)-u_vec(tiu-1:tiu+Nu-2);
    evec  = ref(i+N1:i+N2) - yhat_vec(tiyh+N1:tiyh+N2);
    J = evec'*evec + rho*(duvec'*duvec);
    
    
  %>>>>>>>>>>>>>>>>>>>>>>>>      DETERMINE dyhat/du       <<<<<<<<<<<<<<<<<<<<<<<<<
    for k=N1:N2
      % tanh'(x)
      df(H_hidden)  = (1-y1(H_hidden,k-N1+1).*y1(H_hidden,k-N1+1));
      for l=0:min(k-d,Nu-2)
         %                                 min(k-d-l,na)
         %                                     ---        dy(t+k-1)
         % h(k,l,j) =    w              +      >   w      --------
         %                j,na+k-d-l+1         ---   j,i   du(t+l)
         %                                     i=1
         % 
         imax1 = min(k-d-l,na);
         if l>=k-d-nb+1,
          if imax1>=1, 
            hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)'...
                          + W1(:,na+k-d-l+1);;
          else
            hj_vec=W1(:,na+k-d-l+1);
          end
         else
           hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)';
         end
    
         %          hidden     
         % dy(t+k)   ---
         % ------- =  >  W  * f'(y1(j)) * h(k,l,j)
         % du(t+l)   ---  j
         %           j=1
         dY_dU(l+1,k-N1+1)  = W2(1:hidden)*(df.*hj_vec);
      end
    
      if k>=Nu
         l=Nu-1;
         %       min(k-d-Nu+2,nb)          min(k-d-l,na)
         %             ---                    ---        dy(t+k-1)
         % h(k,l,j) =  >   w           +      >   w      --------
         %             ---  j,na+i            ---   j,i   du(t+l)
         %             i=1                    i=1
         % 
         imax1 = min(k-d-l,na);
         imax2 = min(k-d-Nu+2,nb);
         if imax2>1,
           if k==Nu,
             hj_vec = sum(W1(:,na+1:na+imax2)')';
           else
             hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)'...
                                                    + sum(W1(:,na+1:na+imax2)')';
           end
         else
           if k==Nu,
             hj_vec = W1(:,na+1:na+imax2);
           else
             hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)'...
                                                    + W1(:,na+1:na+imax2);
           end
         end
                                                
                                                    
         %          hidden     
         % dy(t+k)   ---
         % ------- =  >  W  * f'(y1(j)) * h(k,l,j)
         % du(t+l)   ---  j
         %           j=1     
         dY_dU(l+1,k-N1+1)  = W2(1:hidden)*(df.*hj_vec);
       end 
     end

  %>>>>>>>>>>>>>>>>>>>>>>>>>>>>    DETERMINE dJ/du     <<<<<<<<<<<<<<<<<<<<<<<<<<<<
  dJdu   = 2*(-dY_dU*evec + rho*(dUtilde_dU*duvec));


  %>>>>>>>>>>>>>>>>>>>>>>    DETERMINE INVERSE HESSIAN    <<<<<<<<<<<<<<<<<<<<<<<<<
  B = eye(Nu);                  % Initialize Hessian to I


  %>>>>>>>>>>>>>>>>>>>>>>>     BEGIN SEARCH FOR MINIMUM      <<<<<<<<<<<<<<<<<<<<<<    
  for m = 1:maxiter,            % Max. number of iteration
  
  
    %>>>>>>>>>>>>>>>>>>>>>>>   DETERMINE SEARCH DIRECTION   <<<<<<<<<<<<<<<<<<<<<<<
    f = -B*dJdu;

     
     %===============  DETERMINE STEP SIZE BY SIMPLE LINE SEARCH  =================
     mu = 1; b1 = 0; b2 = 1; beta = beta0; Jb1 = J; Gb1 = dJdu;

  
     while 2>1   
       %==============  COMPUTE PREDICTIONS FROM TIME t+N1 TO t+N2  ===============
       up_mu = up + mu*f;                % A priori iteration
       u_vec(uvi) = up_mu(upi);          % Insert updated controls
     
       %----- Determine prediction yhat(t+k,up_mu) -----
       for k=N1:N2,
         phi              = [yhat_vec(tiyh+k-1:-1:tiyh+k-min(k,na)) ; ...
                  y_vec(tiy-1:-1:tiy-max(na-k,0)) ; u_vec(tiu-d+k:-1:tiu-d+1-nb+k)];
         h1               = W1(:,1:inputs)*phi + W1(:,inputs+1);  
         y1(H_hidden,k-N1+1) = pmntanh(h1(H_hidden)); 
         y1(L_hidden,k-N1+1) = h1(L_hidden);
         yhat_vec(tiyh+k) = W2(:,1:hidden)*y1(:,k-N1+1) + W2(:,hidden+1);
       end
       
       %========================     EVALUATE CRITERION     =======================
       duvec = u_vec(tiu:tiu+Nu-1)-u_vec(tiu-1:tiu+Nu-2);
       evec  = ref(i+N1:i+N2) - yhat_vec(tiyh+N1:tiyh+N2);
       J_mu = evec'*evec + rho*(duvec'*duvec);


       %====================    DETERMINE dyhat(t+k,up_mu)/du   ===================
       for k=N1:N2
       % tanh'(x)
       df(H_hidden)  = (1-y1(H_hidden,k-N1+1).*y1(H_hidden,k-N1+1));

      for l=0:min(k-d,Nu-2)
         imax1 = min(k-d-l,na);
         if l>=k-d-nb+1,
          if imax1>=1, 
           hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)'...
                          + W1(:,na+k-d-l+1);;
          else
              hj_vec=W1(:,na+k-d-l+1);
          end
         else
           hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)';
         end
       dY_dU(l+1,k-N1+1)  = W2(1:hidden)*(df.*hj_vec);
      end
      
      if k>=Nu
         l=Nu-1;
         imax1 = min(k-d-l,na);
         imax2 = min(k-d-Nu+2,nb);
         if imax2>1,
           if k==Nu,
             hj_vec = sum(W1(:,na+1:na+imax2)')';
           else
             hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)'...
                                                    + sum(W1(:,na+1:na+imax2)')';
           end
         else
           if k==Nu,
             hj_vec = W1(:,na+1:na+imax2);
           else
             hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)'...
                                                    + W1(:,na+1:na+imax2);
           end
         end                                    
 
         dY_dU(l+1,k-N1+1)  = W2(1:hidden)*(df.*hj_vec);
       end 
     end
       
     
      %=========================     DETERMINE dJ/du     =========================
       dJdu_mu   = 2*(-dY_dU*evec + rho*(dUtilde_dU*duvec));
       Jb2 = J_mu; Gb2 = dJdu_mu;
     
     
       %========================  CHECK THE TWO CONDITIONS   ======================
       flag1 = 0;
       if (J_mu <= J + delta*mu*f'*dJdu),
         % step 2
         if (f'*dJdu_mu >= beta*f'*dJdu),
           break
         end
       else
         % Step 3
         flag1 = 1;
         break
       end
     
       % Step 4
       b1 = mu; b2 = 2*mu; mu=2*mu; Jb1 = Jb2; Gb1 = Gb2;
     end
   
     while flag1==1,
       % Step 5
       bb = b1-b2;
       p2 = (Jb2 - Jb1 + f'*Gb1*bb) / (bb*bb);  
       p1 = f'*Gb1 -2*p2*b1;
       beta = -0.5*p1/p2;

       % Step 6
       if ( min(beta-b1, b2-beta) >= 0.1*(b2-b1) )
         mu = beta;
       else
         mu = (b1+b2)/2;
       end
   
       % Step 7
       %==============  COMPUTE PREDICTIONS FROM TIME t+N1 TO t+N2  ===============
       up_mu = up + mu*f;
       u_vec(uvi) = up_mu(upi);          % Insert updated controls
       for k=N1:N2,
         phi              = [yhat_vec(tiyh+k-1:-1:tiyh+k-min(k,na)) ; ...
                  y_vec(tiy-1:-1:tiy-max(na-k,0)) ; u_vec(tiu-d+k:-1:tiu-d+1-nb+k)];
         h1               = W1(:,1:inputs)*phi + W1(:,inputs+1);  
         y1(H_hidden,k-N1+1) = pmntanh(h1(H_hidden)); 
         y1(L_hidden,k-N1+1) = h1(L_hidden);
         yhat_vec(tiyh+k) = W2(:,1:hidden)*y1(:,k-N1+1) + W2(:,hidden+1);
       end
       
       %========================     EVALUATE CRITERION     =======================
       duvec = u_vec(tiu:tiu+Nu-1)-u_vec(tiu-1:tiu+Nu-2);
       evec  = ref(i+N1:i+N2) - yhat_vec(tiyh+N1:tiyh+N2);
       J_mu = evec'*evec + rho*(duvec'*duvec);
       
       %===========================    DETERMINE dy/du   ==========================
       for k=N1:N2,
         % tanh'(x)
         df(H_hidden)  = (1-y1(H_hidden,k-N1+1).*y1(H_hidden,k-N1+1));

         for l=0:min(k-d,Nu-2)
           imax1 = min(k-d-l,na);
           if l>=k-d-nb+1,
          if imax1>=1, 
           hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)'...
                          + W1(:,na+k-d-l+1);;
          else
              hj_vec=W1(:,na+k-d-l+1);
          end
         else
           hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)';
         end
         dY_dU(l+1,k-N1+1)  = W2(1:hidden)*(df.*hj_vec);
        end
      
      if k>=Nu
         l=Nu-1;
         imax1 = min(k-d-l,na);
         imax2 = min(k-d-Nu+2,nb);
         if imax2>1,
           if k==Nu,
             hj_vec = sum(W1(:,na+1:na+imax2)')';
           else
             hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)'...
                                                    + sum(W1(:,na+1:na+imax2)')';
           end
         else
           if k==Nu,
             hj_vec = W1(:,na+1:na+imax2);
           else
             hj_vec = W1(:,1:imax1)*dY_dU(l+1,k-N1:-1:k-imax1-N1+1)'...
                                                    + W1(:,na+1:na+imax2);
           end
         end                                    
 
         dY_dU(l+1,k-N1+1)  = W2(1:hidden)*(df.*hj_vec);
       end 
     end
       
       
       %=========================    DETERMINE dJ/du     ==========================
       dJdu_mu   = 2*(-dY_dU*evec + rho*(dUtilde_dU*duvec));
     
     
       % Step 8
       %=======================   CHECK THE TWO CONDITIONS   ======================
       if (J_mu <= J + delta*mu*f'*dJdu),
         if (f'*dJdu_mu >= beta*f'*dJdu),
           break
         end
       
         % Step 9a
         b1 = mu;
         Jb1 = J_mu;
         Gb1 = dJdu_mu;
       else
         % Step 9b
         b2 = mu;
         Jb2 = J_mu;
         Gb2 = dJdu_mu;
       end
     end  
       
     % Step 10, Accept the iteration
     J = J_mu;
     dJdu_old = dJdu;
     dJdu = dJdu_mu;
   
  
     %>>>>>>>>>>>>>>>>>>>>>>>>   UPDATE FUTURE CONTROLS   <<<<<<<<<<<<<<<<<<<<<<<<<
     up_old = up;
     up = up_mu; 
     
     
     %>>>>>>>>>>>>>>>>>>>>>>>>     CHECK STOP CONDITION     <<<<<<<<<<<<<<<<<<<<<<<
     dup = up-up_old;
     if dup'*dup < delta2,
       break;
     end 
       
       
     %>>>>>>>>>>>>>>>>>>>     BFGS UPDATE OF INVERSE HESSIAN    <<<<<<<<<<<<<<<<<<
     dG  = dJdu - dJdu_old;
     BdG = B*dG;
     dupdG = dup'*dG;
     fac = 1/dupdG;
     diff = dup - BdG;
     dupfac=dup*fac;
     diffdup = diff*(dupfac'); 
     B = B + diffdup + diffdup' - (diff'*dG)*(dupfac*dupfac');
  end


    %>>>>>>>>>>>>>>>>>>>>>>>     SELECT BEST MINIMUM     <<<<<<<<<<<<<<<<<<<<<<<<<
    if tr==1,
      Jmin_old = J;
      upmin = up;
    else
      if J<Jmin_old,
        upmin = up;
      end
    end
  end


%>>>>>>>>>>>>>>>>>>>>>>     CALCULATE CONTROL SIGANL     <<<<<<<<<<<<<<<<<<<<<<
  e = ref(i) - y;
 
  % Predictive Controller
  if strcmp(regty,'npc'),
    u=upmin(1);

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
  
  % Make sure control input is within limits
  if u>ulim_max,
     u=ulim_max;
  elseif u<ulim_min
     u=ulim_min;
  end
  upmin(1) = u;
  
  
 %>>>>>>>>>>>>>>>>>>>       STORE DATA IN DATA VECTORS      <<<<<<<<<<<<<<<<<<<
  u_data(i)       = u;
  y_data(i)       = y;
  yhat_data(i)    = yhat_vec(tiyh);
  t_data(i)       = t;
  e_data(i)       = J;


%>>>>>>>>>>>>>>>>>>>>>>>>>>       TIME UPDATES        <<<<<<<<<<<<<<<<<<<<<<<<<
  y_old    = shift(y_old,y);
  u_old    = shift(u_old,u);
  u_vec(uvi) = upmin(upi);
  u_vec    = [u_vec(2:length(u_vec)) ; upmin(length(up))];
  y_vec    = [y_vec(2:length(y_vec)) ; y];
  yhat_vec(1:length(yhat_vec)-1) = yhat_vec(2:length(yhat_vec));


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
   plot([0:samples-1],plmat);           % Plot plmat
   xlabel('Samples');
   set(gca,'Xlim',[0 samples-1]);       % Set x-axis
   if regty(1)=='n',
     title('Nonlinear Predictive Control');
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

