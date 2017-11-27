function [W1,W2,PI_vector,iteration,lambda]=general(NetDef,NN,W1,W2,trparms,Y,U)
%  GENERAL
%  -------
%          Train a neural network as an inverse model of a dynamic system by using
%          the Marquardt method. The inverse model is found with the
%          "generalized" training scheme. Use one of the functions 'special1', 
%          'special2', or 'special3' for "specialized" training.
%
%          system  :  y(t) = f(y(t-1),..,y(t-na),u(t-nk),...,u(t-nk-nb+1))
%          NN model:  u_hat(t) = g(y(t+nk),..,y(t+nk-na),u(t-1),...,u(t-nb+1))
%
%  INPUTS:
%  U       : Control signal. (1 | # of data)
%  Y       : Output data. (1 | # of data)
%  NN      : NN=[na nb nk].
%            na = # of past outputs used to determine prediction
%            nb = # of past inputs used to determine prediction
%            nk = time delay (usually 1)
%  NetDef  : Structure of network 
%  W1,W2   : Input-to-hidden layer and hidden-to-output layer weights.
%            If they are passed as [], they will be initialized automatically
%  trparms : Data structure with parameters associated with the
%            training algorithm (optional). Use the function SETTRAIN if
%            you do not want to use the default values.
%
%  See the function "marq" for more information about the input+return arguments.
%                                                                                 
%  Written by : Magnus Norgaard
%  LastEditDate  : Jan. 15, 2000

% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   INITIALIZATIONS   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<   

% -- Get regressor structure --
[nu,N]      = size(U); 
na = NN(1);
nb = NN(2:1+nu);
nk = NN(2+nu:1+2*nu);
nmax        = max(na,nb+nk-1);
nab         = na+sum(nb);


% -- Initialize weights if nescessary --
if isempty(W1) | isempty(W2),
  hidden = length(NetDef(1,:));    % Number of hidden neurons
  W1 = rand(hidden,nab+1)-0.5;
  W2 = rand(1,hidden+1)-0.5;
end

% -- Initialize 'trparms' if nescessary --
if isempty(trparms), trparms=[]; end


% >>>>>>>>>>>>>>>>>>>>  CONSTRUCT THE REGRESSION MATRIX PHI   <<<<<<<<<<<<<<<<<<<<<
PHI = zeros(nab,N-nmax);
jj  = nmax+1:N;
for k = 1:na+1, PHI(k,:)    = Y(jj-k+1); end
index = na+1;
for kk = 1:nu,
  for k = 2:nb(kk), PHI(k+index-1,:) = U(kk,jj-k-nk(kk)+1); end
  index = index + nb(kk);
end


% >>>>>>>>>>>>>>>>>>>>         CALL TRAINING FUNCTION         <<<<<<<<<<<<<<<<<<<<<
[W1,W2,PI_vector,iteration,lambda]=marq(NetDef,W1,W2,PHI,U(nmax-nk(kk)+1:N-nk(kk)),trparms);
