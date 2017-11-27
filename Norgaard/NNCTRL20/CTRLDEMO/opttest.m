% PROGRAM FOR DEMONTRATING OPTIMAL CONTROL
%
% Written by Magnus Norgaard, IAU/IMM, Technical Univ. of Denmark
% LastEditDate: Jan. 22, 2000
close all
StopDemo=0;
figure
guihand=gcf;
for k=1:1, %dummy loop

  % >>>>>>>>>>>>>>>>  BUILD GUI INTERFACE  <<<<<<<<<<<<<<<<<
  [guihand,edmulti,contbut,quitbut]=pmnshow;
  set(guihand,'Name','Demonstration of optimal control');

  % >>>>>>>>>>>>>>>>  SCREEN 1  <<<<<<<<<<<<<<<<<
  s0='1';
  s1='The purpose of this program is to demonstrate a simple';
  s2='type of optimal control applied to a nonlinear process.';
  s3='The process in question is a spring-mass-damper system';
  s4='with a hardening spring:';
  s5=[];
  s6='      y"(t) + y''(t) + y(t) + y(t)^{3} = u(t)';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6);
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end


  % >>>>>>>>>>>>>>>>  SCREEN 2  <<<<<<<<<<<<<<<<<
  % -- Generate data --
  load expdata;
  N2=length(U);
  N1=floor(N2/2);
  Y1 = Y(1:N1)';
  U1 = U(1:N1)';
  Y2 = Y(N1+1:N2)';
  U2 = U(N1+1:N2)';
  s0='2';
  s1='Before we can apply the controller design we need a neural';
  s2='network model of the process. To create this we must make';
  s3='an experiment and collect a set of data describing the';
  s4='process over its entire range of operation. Such an';
  s5='experiment has been simulated in advance with the function';
  s6='"experim." The plot above shows the data set.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6);

  subplot(411)
  plot(U1); grid
  axis([0 N1 min(U1) max(U1)])
  title('Input and output sequence')
  subplot(412)
  plot(Y1); grid
  axis([0 N1 min(Y1) max(Y1)])
  xlabel('time (samples)')
  drawnow
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  
  % >>>>>>>>>>>>>>>>  SCREEN 3  <<<<<<<<<<<<<<<<<
  s0='3';
  s1='The optimal controller network is trained on-line in a';
  s2='fashion similar to "specialized" training of inverse';
  s3='models. Thus, we must first identify a "forward" model ';
  s4='of the process. This is done with the function "nnarx"';
  s5='from the NNSYSID-toolbox. We will use the network';
  s6='architecture shown above. It has five hidden "tanh" units';
  s7='and a linear output unit. We will use as regressors two';
  s8='past outputs and two past controls.';
  subplot(411);delete(gca);subplot(412);delete(gca)
  subplot('position',[0.1 0.55 0.45 0.38]);
  drawnet(ones(5,5),ones(1,6),eps,{'y(t-1)' 'y(t-2)' 'u(t-1)' 'u(t-2)'},{'yhat(t)'});
  title('Network architecture')
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7,s8); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  
  % >>>>>>>>>>>>>>>>  SCREEN 4  <<<<<<<<<<<<<<<<<
  % ----- Train network -----
  s0='4';
  s1=[];
  s2='    >> Training process in action!! <<';
  s3=[];
  s4=[];
  s5='We run up to 200 iterations so you may have to';
  s6='wait for a while.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6);
  set(edmulti,'String',smat);
  drawnow
  trparms = settrain;
  trparms = settrain(trparms,'maxiter',200);
  NetDeff  = ['HHHHH';'L----'];
  NN=[2 2 1];
  [W1f,W2f]=nnarx(NetDeff,NN,[],[],trparms,Y1,U1);
  save forward W1f W2f NetDeff NN
  delete(gca);
  subplot('position',[0.1 0.55 0.45 0.38]);
  drawnet(W1f,W2f,eps,{'y(t-1)' 'y(t-2)' 'u(t-1)' 'u(t-2)'},{'yhat(t)'});
  title('Trained "forward" model')
  if StopDemo==1, close all, break; end

  % >>>>>>>>>>>>>>>>  SCREEN 5  <<<<<<<<<<<<<<<<<
  s0='5';
  s1='We are now ready to execute the training algorithm. This is';
  s2='done by invoking the function "opttrain". This function';
  s3='implements a modified recursive Gauss-Newton algorithm.';
  s4='We will train the controller on a square wave reference';
  s5='trajectory. The trajectory will be repeated seven times';
  s6='while updating the weights. The penalty on squared controls';
  s7='are selected to 0.001. The different design parameters are';
  s8='defined in the file "opttrinit".';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7,s8); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  
  % >>>>>>>>>>>>>>>>  SCREEN 6  <<<<<<<<<<<<<<<<<
  s0='6';
  s1='The applied network architecture is displayed above.';
  s2='The initial weights are simply chosen by random.';
  delete(gca)
  subplot('position',[0.1 0.5 0.6 0.5])
  drawnet(ones(7,5),ones(1,8),eps,{'r(t+1)' '  y(t)' 'y(t-1)' 'u(t-1)'},{'u(t)'});
  title('Controller network architecture')
  smat=str2mat(s0,s1,s2); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  s3='        >> Training process in action!! <<';
  smat=str2mat(s0,s1,s2,[],[],[],s3);
  set(edmulti,'String',smat);
  figure(2)
  opttrain
  close(2)
  save optctrl W1c W2c NetDefc NN
  
   
  % >>>>>>>>>>>>>>>>  SCREEN 7  <<<<<<<<<<<<<<<<
  s0='7';
  s1='The training session is now finished and the weights';
  s2='are saved in a file. We will then conclude this';
  s3='demonstration by simulating the entire closed-loop';
  s4='system.';
  smat=str2mat(s0,s1,s2,s3,s4); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  delete(gca)
  figure('Units','Centimeters','Position',[1.5 1.5 10 1.5]);
  optcon
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
  
  % >>>>>>>>>>>>>>>>  SCREEN 8  <<<<<<<<<<<<<<<<<
  s0='8';
  s1='The result is a reasonable reference tracking with a';
  s2='less active control signal as seen in direct';
  s3='inverse control. Unfortunately the response shows a';
  s4='steady-state error (which depends on the penalty factor).';
  s5='To avoid this one would have to penalize squared';
  s6='differenced controls: [u(t)-u(t-1)] rather than just the';
  s7='squared controls [u(t)].';
  smat=str2mat(s1,s2,s3,s4,s5,s6,s7);
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  drawnow
 
 
  % >>>>>>>>>>>>>>>>  SCREEN 9  <<<<<<<<<<<<<<<<<
  s1=[];
  s2='                  >> THE END <<';
  smat=str2mat(s1,s1,s1,s1,s2);
  set(edmulti,'String',smat);
  drawnow
end

