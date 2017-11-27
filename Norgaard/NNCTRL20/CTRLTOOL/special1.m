% ---------------------------------     SPECIAL1     ------------------------------
%
%  Program for training an inverse model of a system by so-called specialized
%  training (see Psaltis, Sideris & Yamamura: "A Multilayered Neural Network
%  Controller").
%  
%  The inverse model is trained with a recursive back-propagation algorithm.
%
%  The user must provide a neural network model of the process to be controlled
%  and an initial inverse model. This can be created by choosing the weights at
%  random or by training the network with general training. The latter is
%  recommended when possible. 
%
%  All parameters associated with the training procedure are set in the
%  file 'invinit1.m'
%
%  Written by Magnus Norgaard IAU, Technical University of Denmark.
%  LastEditDate: Jan. 23, 2000. 


%----------------------------------------------------------------------------------
%-------------------         >>>  INITIALIZATIONS  <<<        ---------------------
%----------------------------------------------------------------------------------

%>>>>>>>>>>>>>>>>>>>>>>      READ VARIABLES FROM FILE       <<<<<<<<<<<<<<<<<<<<<<<
clear plot_a plot_b
global ugl
invinit1                                 % Run user sepcified initializations
eval(['load ' nninv]);                   % Load inverse neural model


% >>>>>>>>>>>>>>>>>>   DETERMINE STRUCTURE OF FORWARD MODEL    <<<<<<<<<<<<<<<<<<<<
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
nab       = na+sum(nb);                  % Number of "inputs" to each net
inputs    = nab;                         % # of inputs
phi       = zeros(inputs+1,1);           % Initialize regressor vector


% >>>>>>>>>>>>>>>>>>    DETERMINE STRUCTURE OF INVERSE MODEL    <<<<<<<<<<<<<<<<<<<
hiddeni   = length(NetDefi(1,:));        % Number of hidden neurons
L_hiddeni = find(NetDefi(1,:)=='L')';    % Location of linear hidden neurons
H_hiddeni = find(NetDefi(1,:)=='H')';    % Location of tanh hidden neurons
L_outputi = find(NetDefi(2,:)=='L')';    % Location of linear output neurons
H_outputi = find(NetDefi(2,:)=='H')';    % Location of tanh output neurons
y1i       = [zeros(hiddeni,1);1];        % Hidden layer outputs
delta1    = zeros(hiddeni,1);            % "Back-propagated error"
delta2    = zeros(1,1);                  % "Back-propagated error"
d21 = W2f(1:hiddenf);                    % Derivative if linear output
d10 = W1f(:,na+1);                       % Derivative if linear hidden units
d20       = 0;                           % Derivative of output w.r.t. control

%>>>>>>>>>>>>>>>>>    CALCULATE REFERENCE SIGNAL & FILTER IT     <<<<<<<<<<<<<<<<<<
if strcmp(refty,'siggener'),
  ref = zeros(samples+1,1);
  for ii = 1:samples+1,
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


%>>>>>>>>>>>>>>>>>>>>>>>>        INITIALIZE VARIABLES        <<<<<<<<<<<<<<<<<<<<<<
% Initialization of vectors containing past signals
maxlength = 5;                        % MIGHT BE NECESSARY TO INCREASE maxlength
y_old     = repmat(y_0,maxlength,1);  % FOR HIGH-ORDER SYSTEMS!!!
u_old     = repmat(u_0,maxlength,1);

% Initialization of Simulink system
if strcmp(simul,'simulink')
  simoptions = simset('Solver',integrator,'MaxRows',0); % Set integrator opt.
  eval(['[sizes,x0] = ' sim_model '([],[],[],0);']);    % Get initial states
end

% Initializations of vectors used for storing old data
ref_data    = [ref(1:samples)];
ym_data     = [ym(1:samples)];
u_data      = zeros(samples,1);
y_data      = zeros(samples,1);
yhat_data   = zeros(samples,1);


% Miscellanous initializations
maxiter = maxiter*samples;            % Number of iterations
t    = -Ts;
u    = u_0;
y    = y_0;
yhat = y_0;
i      = 0;                           % Iteration in current epoch counter
SSE    = 0;                           % Sum of squared error in current epoch
first  = max(na,nb+nk-1)+10;          % Update weights when iteration>first
epochs = 0;                           % Epoch counter
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
  ey = ym(i) - y;                          % prediction error (a priori)


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


    %>>>>>>>>>>>>>>>>>>>>>>   UPDATE WEIGHTS BY BACK-PROP    <<<<<<<<<<<<<<<<<<<<<<
    E = d20*ey;                           % "Virtual" error on control signal
                                          % Delta for output layer
    delta2(H_outputi) = (1-u(H_outputi).*u(H_outputi)).*E(H_outputi);
    delta2(L_outputi) = E(L_outputi);
                                          % delta for hidden layer
    E1 = W2i(:,1:hiddeni)'*delta2; 
    delta1(H_hiddeni) = (1-y1i(H_hiddeni).*y1i(H_hiddeni)).*E1(H_hiddeni);
    delta1(L_hiddeni) = E1(L_hiddeni);
   
    W2i = W2i + eta*delta2*y1i';          % Update weights between hidden and ouput
    W1i = W1i + eta*delta1*phii';         % Update weights between input and hidden
    
   SSE = SSE + ey*ey;                     % Update performance index (SSE)
  end


  %>>>>>>>>>>>>>>>>>>>>>>     DETERMINE CONTROL SIGNAL      <<<<<<<<<<<<<<<<<<<<<<< 
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
      plot([0:samples-1],plmata);          % Plot plmata
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
      plot([0:samples-1],plmatb);          % Plot plmatb
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
%------------------         >>>   END OF MAIN LOOP   <<<        -------------------
%----------------------------------------------------------------------------------
subplot(111)



