% ---------------------------------     SPECIAL2     ------------------------------
%
%  Program for training an inverse model of a process by so-called specialized
%  training (see Psaltis, Sideris & Yamamura: "A Multilayered Neural Network
%  Controller").
%  
%  The inverse model is trained with a recursive Gauss-Newton algorithm
%  developed in the spirit of "recursive pseudo-linear regression methods"
%  in the sense that the regressors dependency on the network weights are
%  disregarded. Three different versions have been implemented: Exponential
%  forgetting, constant trace, and the exponential forgetting and resetting
%  algorithm (EFRA).
%
%  The user must provide a neural network model of the process to be controlled
%  and an initial inverse model. This can be created by choosing the weights at
%  random or by training the network with general training. The latter is
%  recommended when possible. 
%
%  All parameters associated with the training procedure are set in the
%  file 'invinit2.m'
%
%  See the program "special3" for a true "recursive prediction error method"
%  implementation.
%
%  Written by Magnus Norgaard IAU, Technical University of Denmark.
%  LastEditDate: Jan. 23, 2000. 

%----------------------------------------------------------------------------------
%-------------------         >>>  INITIALIZATIONS  <<<        ---------------------
%----------------------------------------------------------------------------------

%>>>>>>>>>>>>>>>>>>>>>>      READ VARIABLES FROM FILE       <<<<<<<<<<<<<<<<<<<<<<<
clear plot_a plot_b
global ugl
invinit2                                 % Run user sepcified initializations
eval(['load ' nninv]);                   % Load inverse neural model


% >>>>>>>>>>>>>>>>>    DETERMINE STRUCTURE OF FORWARD MODEL    <<<<<<<<<<<<<<<<<<<<
outputs   = 1;                           % # of outputs
eval(['load ' nnforw]);                  % Load forward neural model of system
hiddenf   = length(NetDeff(1,:));        % Number of hidden neurons
L_hiddenf = find(NetDeff(1,:)=='L')';    % Location of linear hidden neurons
H_hiddenf = find(NetDeff(1,:)=='H')';    % Location of tanh hidden neurons
L_outputf = find(NetDeff(2,:)=='L')';    % Location of linear output neurons
H_outputf = find(NetDeff(2,:)=='H')';    % Location of tanh output neurons
y1f       = [zeros(hiddenf,1);1];        % Hidden layer outputs
yhat      = zeros(outputs,1);            % Network output


% >>>>>>>>>>>>>>>>>>>>>>>>   DETERMINE REGRESSOR STRUCTURE   <<<<<<<<<<<<<<<<<<<<<<   
na        = NN(1);                       % # of past y's to be used in TDL
nb        = NN(2);                       % # of past u's to be used in TDL
nk        = NN(3);                       % Time delay in system
nab       = na+sum(nb);                  % Number of "signal inputs" to each net
inputs    = nab;                         % # of inputs
phi       = zeros(inputs+1,1);           % Initialize regressor vector


% >>>>>>>>>>>>>>>>>>    DETERMINE STRUCTURE OF INVERSE MODEL    <<<<<<<<<<<<<<<<<<<
hiddeni   = length(NetDefi(1,:));        % Number of hidden neurons
L_hiddeni = find(NetDefi(1,:)=='L')';    % Location of linear hidden neurons
H_hiddeni = find(NetDefi(1,:)=='H')';    % Location of tanh hidden neurons
L_outputi = find(NetDefi(2,:)=='L')';    % Location of linear output neurons
H_outputi = find(NetDefi(2,:)=='H')';    % Location of tanh output neurons
y1i       = [zeros(hiddeni,1);1];        % Hidden layer outputs
d21       = W2f(1:hiddenf);              % Derivative if linear output
d10       = W1f(:,na+1);                 % Derivative if linear hidden units
d20       = 0;                           % Derivative of output w.r.t. control


%>>>>>>>>>>>>>>>>>    CALCULATE REFERENCE SIGNAL & FILTER IT     <<<<<<<<<<<<<<<<<<
if strcmp(refty,'siggener'),
  ref = zeros(samples+1,1);
  for ii = 1:samples,
    ref(ii) = siggener(Ts*(ii-1),sq_amp,sq_freq,sin_amp,sin_freq,dc,sqrt(Nvar));
  end
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
ym = filter(Bm,Am,ref);               % Filter the reference
ym(samples+1) = ym(1);                % Necessary because the reference is repeated
ref(samples+1) = ref(1);


%>>>>>>>>>>>>>>>>>>>>>>>        INITIALIZE VARIABLES        <<<<<<<<<<<<<<<<<<<<<<<
% Initialization of vectors containing past signals
maxlength = 5;                          % MIGHT BE NECESSARY TO INCREASE maxlength
y_old     = repmat(y_0,maxlength,1);    % FOR HIGH-ORDER SYSTEMS!!!
u_old     = repmat(u_0,maxlength,1);

% Variables associated with the weight update algorithm
SSE = 0;                                % Sum of squared error in current epoch
epochs = 0;                             % Epoch counter
first = max(na,nb+nk-1)+10;             % Update weights when iteration>first
index = (hiddeni+1) + 1 + [0:hiddeni-1]*(inputs+1); % A useful vector!
parameters1= hiddeni*(inputs+1);        % # of input-to-hidden weights
parameters2= (hiddeni+1);               % # of hidden-to-output weights
parameters = parameters1 + parameters2; % Total # of weights
PSI        = zeros(parameters,1);       % Deriv. of each output w.r.t. each weight
                                        % Parametervector containing all weights
theta = [reshape(W2i',parameters2,1) ; reshape(W1i',parameters1,1)];
index3= 1:(parameters+1):(parameters^2);% Yet another useful vector
if strcmp(method,'ff'),                 % Forgetting factor method
  mflag     = 1;                        % Method flag
  lambda    = trparms(1);               % Forgetting factor
  p0        = trparms(2);               % Diagonal element of covariance matrix
  P         = p0 * eye(parameters);     % Initialize covariance matrix
elseif strcmp(method,'ct'),             % Constant trace method
  mflag     = 2;                        % Method flag
  lambda    = trparms(1);               % Forgetting factor
  alpha_max = trparms(2);               % Max. eigenvalue
  alpha_min = trparms(3);               % Min. eigenvalue
  P      = alpha_max * eye(parameters); % Initialize covariance matrix
  Pbar      = P;                        % Initialize covariance matrix
elseif strcmp(method,'efra'),           % EFRA method
  mflag     = 3;                        % Method flag
  alpha     = trparms(1);               % EFRA parameters
  beta      = trparms(2);
  delta     = trparms(3);
  lambda    = trparms(4);
  gamma     = (1-lambda)/lambda;
  maxeig = gamma/(2*delta)*(1+sqrt(1+4*beta*delta/(gamma*gamma)));% Max. eigenvalue
  P      = maxeig * eye(parameters);    % Initialize covariance matrix
  betaI     = beta*eye(parameters);     % Useful diagonal matrix
end

% Initialization of Simulink system
if strcmp(simul,'simulink')
  simoptions = simset('Solver',integrator,'MaxRows',0); % Set integrator  opt.
  eval(['[sizes,x0] = ' sim_model '([],[],[],0);']);    % Get initial states
end

% Initializations of vectors used for storing old data
ref_data    = [ref(1:samples)];
ym_data     = [ym(1:samples)];
u_data      = zeros(samples,1);
y_data      = zeros(samples,1);
yhat_data   = zeros(samples,1);

% Miscellanous initializations
maxiter = maxiter*samples;              % Number of iterations
t    = -Ts;
u    = u_0;
y    = y_0;
yhat = y_0;
i   = 0;                                % Iteration in current epoch counter
fighandle=progress;

%----------------------------------------------------------------------------------
%---------------------         >>>   MAIN LOOP   <<<           --------------------
%----------------------------------------------------------------------------------
for iter=1:maxiter,
  i = i+1;
  t = t + Ts;


  %>>>>>>>>>>>>>>  PREDICT OUTPUT OF SYSTEM USING THE FORWARD MODEL   <<<<<<<<<<<<<
  phi = [y_old(1:na);u_old(1:nb);1];
  h1f = W1f*phi;
  y1f(H_hiddenf)  = pmntanh(h1f(H_hiddenf));
  y1f(L_hiddenf)  = h1f(L_hiddenf);    
  h2f = W2f*y1f;
  yhat(H_outputf) = pmntanh(h2f(H_outputf));
  yhat(L_outputf) = h2f(L_outputf);


  %>>>>>>>>>>>>>>>>>>>>  READ OUTPUT FROM THE PHYSICAL SYSTEM   <<<<<<<<<<<<<<<<<<<
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


  %>>>>>>>>>>>>>>>>>>>>>>>    CALCULATE PREDICTION ERROR    <<<<<<<<<<<<<<<<<<<<<<<
  ey = ym(i) - y;


  %>>>>>>>>>>>>  COMPUTE DERIVATIVE OF PREDICTED OUTPUT W.R.T. CONTROL <<<<<<<<<<<<
  if iter >first,                          % wait a few samples before updating
   % Matrix containing the partial derivative of the output w.r.t
   % each of the outputs from the hidden units
   if H_outputf,
     d21 = (1-yhat*yhat)*W2f(1:hiddenf);
   end

   % Matrix containing partial derivatives of the output from each hidden unit
   % w.r.t the most recent control input:
   d10(H_hiddenf) = (1-y1f(H_hiddenf).*y1f(H_hiddenf)).*W1f(H_hiddenf,na+1);

   % Partial derivative of output w.r.t the most recent control input
   d20 = d21(1:hiddenf)*d10;

  
   %>>>>>>>>>>>>>>  COMPUTE DERIVATIVE OF CONTROL W.R.T. EACH WEIGHT  <<<<<<<<<<<<<<
    % ==========   Elements corresponding to the linear output units   ============
    if L_outputi'

      % -- The part of PSI corresponding to hidden-to-output layer weights --
      index1 = 1;
      PSI(index1:index1+hiddeni) = y1i;
      % ---------------------------------------------------------------------
 
      % -- The part of PSI corresponding to input-to-hidden layer weights ---
      for j = L_hiddeni',
        PSI(index(j):index(j)+inputs) = W2i(j)*phii;
      end
      
      for j = H_hiddeni',
        PSI(index(j):index(j)+inputs) = W2i(j)*(1-y1i(j)*y1i(j))*phii;
      end 
      % ---------------------------------------------------------------------    

    % ============  Elements corresponding to the tanh output units   =============
    elseif H_outputi',
      % -- The part of PSI corresponding to hidden-to-output layer weights --
      index1 = 1;
      PSI(index1:index1+hiddeni,i) = y1i * (1 - u*u);
      % ---------------------------------------------------------------------
       
      % -- The part of PSI corresponding to input-to-hidden layer weights ---
      for j = L_hiddeni',
        PSI(index(j):index(j)+inputs) = W2i(j)*(1-u*u) * phii;
      end
      
      for j = H_hiddeni',
        PSI(index(j):index(j)+inputs) = W2i(j)*(1-y1i(j)*y1i(j))*(1-u*u) * phii;
      end
      % ---------------------------------------------------------------------
    end
 
 
    %>>>>>>>>>>>  COMPUTE DERIVATIVE OF PREDICTED OUTPUT W.R.T. WEIGHT  <<<<<<<<<<<
    PSI_red = PSI*d20;


    %>>>>>>>>>>>>>>>>>>>>>>>>>>>    UPDATE THE WEIGHTS    <<<<<<<<<<<<<<<<<<<<<<<<<
    
    % ---------- Forgetting factor method ----------
    if mflag==1,
      % -- Update P matrix --
      P = (P - P*PSI_red/(lambda + PSI_red'*P*PSI_red)*PSI_red'*P ) / lambda;

      % -- Update Parameters --
      theta = theta + P*PSI_red*ey;
  
      % ----------  Constant trace method   ---------- 
    elseif mflag == 2,
      % -- Correction factor --
      K = P*PSI_red /(lambda + PSI_red'*P*PSI_red);
      
      % -- Measurement update of P matrix --
      Pbar = (P - P*PSI_red/(1 + PSI_red'*P*PSI_red)*PSI_red'*P )/lambda;

      % -- Update Parameters --
      theta = theta + K*ey;

      % -- Time update of P matrix --
      P         = ((alpha_max-alpha_min)/trace(Pbar))*Pbar;
      P(index3) = P(index3)+alpha_min;
      
    % ----------       EFRA method        ---------- 
    else 
      % -- Correction factor --
      K = P*PSI_red * (alpha/(1 + PSI_red'*P*PSI_red));

      % -- Update Parameters --
      theta = theta + K*ey;
      
      % -- Update P --
      P = P/lambda - K*PSI_red'*P + betaI-delta*P*P;
    end 
  
    SSE = SSE + ey*ey;                    % Update performance index (SSE)
  
    % -- Put parameters back into weight matrices --
    W1i = reshape(theta(parameters2+1:parameters),inputs+1,hiddeni)';
    W2i = reshape(theta(1:parameters2),hiddeni+1,1)';
  end


  %>>>>>>>>>>>>>>>>>>>>>>>    DETERMINE CONTROL SIGNAL     <<<<<<<<<<<<<<<<<<<<<<<<  
  % Control using the inverse model
  phii= [ref(i+1);y;y_old(1:na-1);u_old(1:nb-1);1];
  h1i = W1i*phii;  
  y1i(H_hiddeni) = pmntanh(h1i(H_hiddeni));
  y1i(L_hiddeni) = h1i(L_hiddeni);    
  h2i = W2i*y1i;
  u(H_outputi)   = pmntanh(h2i(H_outputi));
  u(L_outputi)   = h2i(L_outputi);
  
  % Make sure control inputs is within limits
  if u>ulim_max,
     u=ulim_max;
  elseif u<ulim_min
     u=ulim_min;
  end

  
  %>>>>>>>>>>>>>>>>>>       COPY DATA INTO THE DATA VECTORS       <<<<<<<<<<<<<<<<<
  u_data(i)    = u;
  y_data(i)    = y;
  yhat_data(i) = yhat;


  %>>>>>>>>>>>>>>>>>>>>>>>>         TIME OPDATES          <<<<<<<<<<<<<<<<<<<<<<<<<
  y_old = shift(y_old,y);
  u_old = shift(u_old,u);


  %>>>>>>>>>>>>>>>>>>>>      PRINT %-AGE OF EPOCH COMPLETED      <<<<<<<<<<<<<<<<<<
  progress(fighandle,floor(100*i/samples));
  

  %>>>>>>>>>>>>>>>>>>>>>>>>>>>        DRAW PLOTS       <<<<<<<<<<<<<<<<<<<<<<<<<<<<
  if i==samples,
    epochs = epochs+1;
    figure(gcf)
    
    % Plot A
    if(exist('plot_a')==1),
      if epochs==1,
        a_plots=length(plot_a);            % Number of plots in plot A
        plmata = zeros(samples,a_plots);    % Collect vectors in plmat
      end
      for nn = 1:a_plots, 
        plmata(:,nn) = eval(plot_a{nn});   
      end
      subplot(2,1,1);
      plot([0:samples-1],plmata);          % Plot plmat
      xlabel('Samples');
      set(gca,'Xlim',[0 samples-1]);       % Set x-axis
      title(['Specialized Training  (SSE = ' num2str(SSE) ...
                                         ',    epoch = ' num2str(epochs) ')']);
      grid on
    end
  
    % Plot B
    if(exist('plot_b')==1),
      if epochs==1,
           b_plots=length(plot_b);              % Number of plots in plot B
           plmatb = zeros(samples,b_plots);     % Collect vectors in plmat
      end
      for nn = 1:b_plots, 
         plmatb(:,nn) = eval(plot_b{nn});   
      end
      subplot(2,1,2);
      plot([0:samples-1],plmatb);          % Plot plmat
      xlabel('Samples'); 
      set(gca,'Xlim',[0 samples-1]);       % Set x-axis
      grid on
    end
    figure(gcf); drawnow
    i   = 0;
    SSE = 0;
    if iter<maxiter, fighandle=progress; end
  end
end
%----------------------------------------------------------------------------------
%----------------           >>>   END OF MAIN LOOP   <<<          -----------------
%----------------------------------------------------------------------------------
subplot(111)

