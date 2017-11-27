% PROGRAM DEMONTRATION OF DIRECT INVERSE TRAINING
%
% Programmed by Magnus Norgaard, IAU/IMM, Technical Univ. of Denmark
% LastEditDate: Jan. 23, 2000
close all
StopDemo=0;
figure
guihand=gcf;
for k=1:1, %dummy loop

  % >>>>>>>>>>>>>>>>  BUILD GUI INTERFACE  <<<<<<<<<<<<<<<<<
  [guihand,edmulti,contbut,quitbut]=pmnshow;
  set(guihand,'Name','Direct inverse control demo');

  % >>>>>>>>>>>>>>>>  SCREEN 1  <<<<<<<<<<<<<<<<<
  s0='1';
  s1='The purpose of this demo is to show different methods';
  s2='for training a neural network to act as the inverse';
  s3='of a nonlinear process. Subsequently the inverse';
  s4='model is used for direct inverse control.';
  s5='    The process under consideration is a spring-mass-';
  s6='damper system with a hardening spring:';
  s7=' y"(t) + y''(t) + y(t) + y(t)^{3} = u(t)';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7);
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end


% >>>>>>>>>>>>>>>>  SCREEN 2  <<<<<<<<<<<<<<<<<
  % -- Generate data --
  figure('Units','Centimeters','Position',[1.5 1.5 10 1.5]);
  [Y,U]=experim;close
  N2=length(U);
  N1=floor(N2/2);
  Y1 = Y(1:N1)';
  U1 = U(1:N1)';
  Y2 = Y(N1+1:N2)';
  U2 = U(N1+1:N2)';
  s0='2';
  s1='We began by generating a set of data by performing a';
  s2='small experiment. The data set is then split into two';
  s3='portions: one for training and one for testing.';
  s4='The function ''experim'' was used for simulating the';
  s5='experiment. The design parameters has been initialized in';
  s6='the file ''expinit''.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6);

  subplot(411)
  plot(U); hold on
  plot([N1 N1],[min(U) max(U)],'m--'); hold off
  axis([0 N2 min(U) max(U)])
  title('Input sequence (training and test set, respectively)')
  subplot(412)
  plot(Y); hold on
  plot([N1 N1],[min(Y) max(Y)],'m--'); hold off
  axis([0 N2 min(Y) max(Y)])
  title('Output sequence')
  xlabel('time (samples)')
  drawnow
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end


  % >>>>>>>>>>>>>>>>  SCREEN 3  <<<<<<<<<<<<<<<<<
  subplot(411);delete(gca);subplot(412);delete(gca)
  subplot('position',[0.1 0.5 0.6 0.5])
  NetDefi  = ['HHHHH';'L----'];
  NN      = [2 2 1];
  drawnet(ones(5,5),ones(1,6),eps,{'y(t+1)' '  y(t)' 'y(t-1)' 'u(t-1)'},{'u(t)'});
  title('Network architecture')
  s0='3';
  s1='We will now train an inverse neural network model of the';
  s2='process by general training. Since it''s a 2nd order process';
  s3='we will use the regressor structure shown above. The';
  s4='network has 5 hidden ''tanh'' units and one linear output';
  s5='unit. The function ''general'' implements general'; 
  s6='training with a Levenberg-Marquardt algorithm.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6);
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end


  % >>>>>>>>>>>>>>>>  SCREEN 4  <<<<<<<<<<<<<<<<<
  % ----- Train network -----
  s0='4';
  s1=[];
  s2='    >> Training process in action!! <<';
  s3=[];
  s4=[];
  s5='We run up to 500 iterations so you may have to';
  s6='wait for a while.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6);
  set(edmulti,'String',smat);
  drawnow
  [W1i,W2i,PI_vector,iteration,lambda]=general(NetDefi,NN,[],[],[],Y1,U1);

  % >>>>>>>>>>>>>>>>  SCREEN 5  <<<<<<<<<<<<<<<<<
  % ----- Evaluate network -----
  s0='5';
  s1='The network has now been trained and we will then';
  s2='validate it on a fresh data set.';
  smat=str2mat(s0,s1,s2);
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end 
  [Yhat,PI] = invsim(NetDefi,NN,W1i,W2i,Y2,U2);
  figure(1)

  % >>>>>>>>>>>>>>>>  SCREEN 6  <<<<<<<<<<<<<<<<<
  s0='6';
  s1='Let''s just say we''re satisfied with the network';
  s2='model and let''s save architecture definition ';
  s3='and weights in a file.';
  s4=[];
  s5='The network can now be used as controller in a';
  s6='''Direct inverse control'' concept';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6); 
  drawnet(W1i,W2i,eps,{'y(t+1)' '  y(t)' 'y(t-1)' 'u(t-1)'},{'u(t)'});
  title('Network after training')
  save inverse W1i W2i NetDefi NN
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end 

  % >>>>>>>>>>>>>>>>  SCREEN 7  <<<<<<<<<<<<<<<<<
  close(2); close(3);
  figure('Units','Centimeters','Position',[1.5 1.5 10 1.5]);
  invcon;
  close
  subplot(411)
  plot([0:samples-1],[ref_data y_data]); grid
  axis([0 samples -2 2])
  title('Reference and output signal')
  subplot(412)
  plot([0:samples-1],u_data);
  axis([0 samples min(u_data) max(u_data)]); grid
  title('Control signal')
  xlabel('time (samples)')
  drawnow
  s0='7';
  s1='Reasonable model-following has been achieved but';
  s2='there is obviously a small overshoot when the reference';
  s3='changes. From the plot showing the control signal it is';
  s4='evident that the model is used outside the region in';
  s5='which it was operated during the initial experiment. To';
  s6='fine-tune the network directly on the square wave';
  s7='reference trajectory we will use specialized training.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
 % >>>>>>>>>>>>>>>>  SCREEN 8  <<<<<<<<<<<<<<<<<
  s0='8';
  s1='Before continuing with specialized training it is necessary';
  s2='to identify a "forward" model of the process. This model is';
  s3='required by the specialized training algorithm. We will use';
  s4='the function "nnarx" from the NNSYSID-toolbox for this. The';
  s5='model is inferred from the same data set as was used for';
  s6='training the inverse model.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  s8='    >> Training process in action!! <<';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,[],s8);
  set(edmulti,'String',smat);
  trparms = settrain;
  trparms = settrain(trparms,'maxiter',200);
  NetDeff  = ['HHHHH';'L----'];
  
  [W1f,W2f,PI_vector,iteration,lambda]=nnarx(NetDeff,NN,[],[],trparms,Y1,U1);
  save forward W1f W2f NetDeff NN
  

  % >>>>>>>>>>>>>>>>  SCREEN 9  <<<<<<<<<<<<<<<<<
  s0='9';
  s1='The function "special2" will be used for performing the';
  s2='specialized training. We choose a recursive Gauss-Newton';
  s3='algorithm with exponential forgetting for updating the weights.';
  s4='The reference trajectory (consisting of 200 samples) will be';
  s5='repeated eight times while updating the weights.';
  smat=str2mat(s0,s1,s2,s3,s4,s5); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end 
  s7='    >> Specialized training process in action!! <<';
  smat=str2mat(s0,s1,s2,s3,s4,s5,[],s7);
  set(edmulti,'String',smat);
  figure(2)
  special2
  close(2)
  save inverse W1i W2i NetDefi NN
  
  % >>>>>>>>>>>>>>>>  SCREEN 10  <<<<<<<<<<<<<<<<
  s0='10';
  s1='The training is now finished and the new weights are saved';
  s2='in a file. To conclude the session we will simulate the entire';
  s3='closed-loop system again.';
  smat=str2mat(s0,s1,s2,s3); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  figure('Units','Centimeters','Position',[1.5 1.5 10 1.5]);
  invcon
  close(2)
  subplot(411)
  plot([0:samples-1],[ref_data y_data]); grid
  axis([0 samples -2 2])
  title('Reference and output signal')
  subplot(412)
  plot([0:samples-1],u_data);
  axis([0 samples min(u_data) max(u_data)]); grid
  title('Control signal')
  xlabel('time (samples)')
  drawnow
  
  % >>>>>>>>>>>>>>>>  SCREEN 11  <<<<<<<<<<<<<<<<<
  s1=[];
  s2='                  >> THE END <<';
  smat=str2mat(s1,s1,s1,s1,s2);
  set(edmulti,'String',smat);
  drawnow
end
