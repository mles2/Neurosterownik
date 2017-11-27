function [Uhat,PI]=invsim(NetDef,NN,W1,W2,Y,U)
%  INVSIM
%  ------
%          Evaluate a neural network trained to model the inverse of a dynamic
%          system.
%
%          The following plots a made:
%          - Actual control signal together with prediction
%          - Prediction error
%          - Auto correlation function of prediction error
%          - Extracted linear parameters
%
%  Call:
%           [Uhat,PI] = nnsim(NetDef,NN,W1,W2,Y,U)

%  Programmed by : Magnus Norgaard
%  LastEditDate  : Oct. 14, 1994

% >>>>>>>>>>>>>>>>>>>>>>>>>>>>     INITIALIZATIONS     <<<<<<<<<<<<<<<<<<<<<<<<<<<< 
Ndat     = length(Y);                   % # of data
na = NN(1);
[nu,N] = size(U); 
nb = NN(2:1+nu); 
nk = NN(2+nu:1+2*nu);


% --------- Common initializations --------
nmax     = max([na,nb+nk-1]);           % 'Oldest' signal used as input to the model
N        = Ndat - nmax;                 % Size of training set
nab      = na+sum(nb);                  % na+nb
outputs  = 1;                           % Only MISO models considered
L_hidden = find(NetDef(1,:)=='L')';     % Location of linear hidden neurons
H_hidden = find(NetDef(1,:)=='H')';     % Location of tanh hidden neurons
L_output = find(NetDef(2,:)=='L')';     % Location of linear output neurons
H_output = find(NetDef(2,:)=='H')';     % Location of tanh output neurons
[hidden,inputs] = size(W1);
inputs          = inputs-1;
E        = zeros(outputs,N);
y1       = zeros(hidden,N);
Uhat     = zeros(outputs,N);


% >>>>>>>>>>>>>>>>>>>>  CONSTRUCT THE REGRESSION MATRIX PHI   <<<<<<<<<<<<<<<<<<<<<
PHI = zeros(nab,N);
jj  = nmax+1:Ndat;
for k = 1:na+1, PHI(k,:)    = Y(jj-k+1); end
index = na+1;
for kk = 1:nu,
  for k = 2:nb(kk), PHI(k+index-1,:) = U(kk,jj-k-nk(kk)+1); end
  index = index + nb(kk);
end


% >>>>>>>>>>>>>>>>>>>>>>>>>>   COMPUTE NETWORK OUTPUT   <<<<<<<<<<<<<<<<<<<<<<<<<<<
U  = U(nmax-nk(1)+1:Ndat-nk(1));
h1 = W1*[PHI;ones(1,N)];  
y1(H_hidden,:) = pmntanh(h1(H_hidden,:));
y1(L_hidden,:) = h1(L_hidden,:);
    
h2 = W2*[y1;ones(1,N)];
Uhat(H_output,:) = pmntanh(h2(H_output,:));
Uhat(L_output,:) = h2(L_output,:);

E      = U - Uhat;                       % Error between U and Uhat
SSE    = E*E';                           % Sum of squared errors (SSE)
PI     = SSE/(2*N);                      % Performance index



% >>>>>>>>>>>>>>>>>>>>>>>>>>      PLOT THE RESULTS      <<<<<<<<<<<<<<<<<<<<<<<<<<<

% ---------- Output, Prediction and Prediction error ----------
figure
subplot(211)
plot(U,'b-'); hold on
plot(Uhat,'r--');hold off
xlabel('time (samples)')
title('Actual control signal (dashed) and prediction (solid)')
grid

subplot(212)
plot(E);
title('Prediction error (u-uhat)')
xlabel('time (samples)')
grid
subplot(111)
drawnow


% --------- Correlation functions ----------
figure
subplot(211)
M=min(25,N-1);
Eauto=crossco(E,E,M);
Eauto=Eauto(M+1:2*M+1);
conf=1.96/sqrt(N);
plot([0:M],Eauto(1:M+1),'b-'); hold on
plot([0 M],[conf -conf;conf -conf],'m--');hold off
xlabel('lag')
title('Auto correlation function of prediction error')
grid
Ecov=cov(E);
drawnow


% ---------- Extract linear model from network ----------
dy2dx=zeros(outputs*(inputs),N);

% Matrix with partial derivative of each output with respect to each of the
% outputs from the hidden neurons
for t=1:N,
  dy2dy1 = W2(:,1:hidden);

  % Matrix with partial derivatives of the output from each hidden neurons with
  % respect to each input:
  dy1dx = W1(:,1:inputs);
  for j = H_hidden',
    dy1dx(j,:) = W1(j,1:inputs)*(1-y1(j,t).*y1(j,t));
  end

  % Matrix with partial derivative of each output with respect to each input
  dl       = (dy2dy1 * dy1dx)';
  dy2dx(:,t) = dl(:);
end

subplot(212)
plot(dy2dx')
title('Linearized network parameters')
xlabel('time (samples)')
grid
drawnow
subplot(111)


