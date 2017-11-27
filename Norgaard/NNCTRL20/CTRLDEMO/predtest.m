% PROGRAM DEMONTRATION OF NEURAL NETWORK BASED PREDICTIVE CONTROL
%
% Programmed by Magnus Norgaard, IAU/IMM, Technical Univ. of Denmark
% LastEditDate: Feb. 20, 1996
close all
StopDemo=0;
figure
guihand=gcf;
for k=1:1, %dummy loop

  % >>>>>>>>>>>>>>>>  BUILD GUI INTERFACE  <<<<<<<<<<<<<<<<<
  [guihand,edmulti,contbut,quitbut]=pmnshow;
  set(guihand,'Name','Predictive control');

  % >>>>>>>>>>>>>>>>  SCREEN 1  <<<<<<<<<<<<<<<<<
  s0='1';
  s1='The purpose of this program is to demonstrate';
  s2='neural network based predictive control of a';
  s3='nonlinear process. Two types of controller designs';
  s4='will be investigated: "true" nonlinear generalized';
  s5='predictive control and approximate generalized predictive';
  s6='control. The process in question is a spring-mass-damper';
  s7='system with a hardening spring:';
  s8=' y"(t) + y''(t) + y(t) + y(t)^{3} = u(t)';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7,s8);
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
  plot(Y); grid
  axis([0 N1 min(Y1) max(Y1)])
  xlabel('Time (samples)')
  drawnow
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  
  % >>>>>>>>>>>>>>>>  SCREEN 3  <<<<<<<<<<<<<<<<<
  s0='3';
  s1='To identify the neural network model we will use the';
  s2='function "nnarx" from the NNSYSID-toolbox. Since it''s';
  s3='a second order process we will use as regressors two';
  s4='past outputs and two past controls. Furthermore we choose';
  s5='a network architecture with five hidden "tanh" units and';
  s6='one linear output.';
  subplot(411);delete(gca);subplot(412);delete(gca)
  subplot('position',[0.1 0.55 0.45 0.38]);
  drawnet(ones(7,5),ones(1,8),eps,{'y(t-1)' 'y(t-2)' 'u(t-1)' 'u(t-2)'},{'yhat(t)'});
  title('Network architecture')
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
  s5='We run up to 200 iterations so you may have to';
  s6='wait for a while.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6);
  set(edmulti,'String',smat);
  drawnow
  trparms = settrain;
  trparms = settrain(trparms,'maxiter',200);
  NetDef  = ['HHHHH';'L----'];
  NN=[2 2 1];
  [W1,W2]=nnarx(NetDef,NN,[],[],trparms,Y1,U1);
  save forward2 W1 W2 NetDef NN
  delete(gca);
  subplot('position',[0.1 0.55 0.45 0.38]);
  drawnet(W1,W2,eps,{'y(t-1)' 'y(t-2)' 'u(t-1)' 'u(t-2)'},{'yhat(t)'});
  title('The trained network')
  if StopDemo==1, close all, break; end
  
  % >>>>>>>>>>>>>>>>  SCREEN 5  <<<<<<<<<<<<<<<<<
  s0='5';
  s1='The network has now been trained and we are ready to';
  s2='simulate nonlinear predictive control of the process.';
  s3='We will use the program "npcinit1" which implements';
  s4='a Quasi-Newton search for the minimum of the GPC criterion';
  s5='The design parameters are selected as follows:';
  s6='Minimum and maximum output horizon: N1=1 and N2=7';
  s7='Control horizon and penalty factor: Nu=1 and rho=0.03';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7);
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  % >>>>>>>>>>>>>>>>  SCREEN 6  <<<<<<<<<<<<<<<<<
  figure('Units','Centimeters','Position',[1.5 1.5 10 1.5]);
  npccon1
  close
  subplot(411)
  plot([0:samples-1],[ref_data y_data]); grid
  axis([0 samples -2.2 2.2])
  title('Reference and output signal')
  subplot(412)
  plot([0:samples-1],u_data);
  axis([0 samples min(u_data) max(u_data)]); grid
  title('Control signal')
  xlabel('Time (samples)')
  drawnow
  s0='6';
  s1='It appears that we obtain a reasonably close';
  s2='tracking of the reference trajectory. However, you';
  s3='probably noticed that the simulation was extremely';
  s4='time consuming. The reason for this is the iterative';
  s5='minimization algorithm executed at each sample to';
  s6='determine the optimal control. Alternatively we may thus';
  s7='apply the instantaneous linearization principle to reduce';
  s8='the amount of computations. This is simulated with "apccon"';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7,s8); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  % >>>>>>>>>>>>>>>>  SCREEN 7  <<<<<<<<<<<<<<<<<
  figure('Units','Centimeters','Position',[1.5 1.5 10 1.5]);
  apccon
  close
  subplot(411)
  plot([0:samples-1],[ref_data y_data]); grid
  axis([0 samples -2.2 2.2])
  title('Reference and output signal')
  subplot(412)
  plot([0:samples-1],u_data);
  axis([0 samples min(u_data) max(u_data)]); grid
  title('Control signal')
  xlabel('time (samples)')
  drawnow
  s0='7';
  s1='The simulation was obviously considerably faster than';
  s2='before but the reference tracking still looks OK.';
  s3='The response differs slightly from the one produced by';
  s4='the nonlinear predictive controller. This is not alone';
  s5='due to the linearization but also has to do with the future';
  s6='predictions being calculated in a different fashion';
  s7='(see the manual).';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end 
  
  % >>>>>>>>>>>>>>>>  SCREEN 9  <<<<<<<<<<<<<<<<<
  s0='9';
  s1=[];
  s2='                  >> THE END <<';
  smat=str2mat(s1,s1,s1,s1,s2);
  set(edmulti,'String',smat);
  drawnow
end
