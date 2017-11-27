% PROGRAM FOR DEMONTRATING CONTROL BASED ON INSTANTANEOUS LINEARIZATION
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
  set(guihand,'Name','Control based on instantaneous linearization');

  % >>>>>>>>>>>>>>>>  SCREEN 1  <<<<<<<<<<<<<<<<<
  s0='1';
  s1='The purpose of this demo is to show how the instantaneous';
  s2='linearization principle can be used in the design of';
  s3='control systems for nonlinear processes. Two types of';
  s4='designs will be investigated: pole placement with zero';
  s5='cancellation and pole placement without zero cancellation.';
  s6='The process in question is a spring-mass-damper system';
  s7='with a hardening spring: y"(t) + y''(t) + y(t) + y(t)^{3} = u(t)';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7);
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
  title('Trained network')
  if StopDemo==1, close all, break; end
  
  % >>>>>>>>>>>>>>>>  SCREEN 5  <<<<<<<<<<<<<<<<<
  s0='5';
  s1='The network has now been trained and we are ready to';
  s2='simulate the control system. First we will try a';
  s3='pole placement strategy where we cancel the zero.';
  s4='Let''s choose the desired closed loop system as:';
  s5='                0.09z^{3}';
  s6='H(z)=----------------------------';
  s7='        z^{3} - 1.4z^{2} + 0.49z';
  s8='corresponding to two poles in z=0.7 and one in z=0.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7,s8);
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  % >>>>>>>>>>>>>>>>  SCREEN 6  <<<<<<<<<<<<<<<<<
  figure('Units','Centimeters','Position',[1.5 1.5 10 1.5]);
  pp=1;
  lincon
  close
  subplot(411)
  plot([0:samples-1],[ref_data y_data ym_data]); grid
  axis([0 samples -2 2])
  title('Reference, output and desired output')
  subplot(412)
  plot([0:samples-1],u_data);
  axis([0 samples min(u_data) max(u_data)]); grid
  title('Control signal')
  xlabel('time (samples)')
  drawnow
  s0='6';
  s1='Very nice model-following is clearly achieved. However';
  s2='the control signal is very active because the zero we';
  s3='cancel is close to -1. To avoid this we instead';
  s4='choose not to let the controller cancel the zero.';
  s5='Let''s use the same desired model as before but introduce';
  s6='a dead-beat observer polynomial for causality reasons:';
  s7='Ao(z)=z';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  % >>>>>>>>>>>>>>>>  SCREEN 7  <<<<<<<<<<<<<<<<<
  figure('Units','Centimeters','Position',[1.5 1.5 10 1.5]);
  pp=2;
  lincon
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
  s1='It is seen that the response looks more or less as';
  s2='before but that the control signal now is much more';
  s3='smooth.';
  smat=str2mat(s0,s1,s2,s3); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  % >>>>>>>>>>>>>>>>  SCREEN 8  <<<<<<<<<<<<<<<<<
  subplot(411)
  plot([0:samples-1],B_data); grid
  axis([0 samples min(min(B_data(3:samples,:))) max(max(B_data))])
  title('Numerator (above) and denominator (below) coefficients')
  subplot(412)
  plot([0:samples-1],A_data(:,2:3));
  axis([0 samples min(min(A_data(3:samples,2:3))) max(max(A_data))]); grid
  title('Control signal')
  xlabel('time (samples)')
  drawnow
  s0='8';
  s1='A very nice feature of instantaneous linearization is';
  s2='that it provides an excellent understanding of the';
  s3='dynamics of the process. As shown above we can, for';
  s4='example, plot the coefficients of the extracted linear';
  s5='models.';
  s6='   Another possibility is to plotpoles and zeros in';
  s7='the complex plane.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  % >>>>>>>>>>>>>>>>  SCREEN 9  <<<<<<<<<<<<<<<<<
  subplot(411);delete(gca);subplot(412);delete(gca)
  subplot('position',[0.1 0.55 0.45 0.38]);
  Zmat = zeros(samples-2,nb-1);
  for k=3:samples,
    Zmat(k-2,:)=roots(B_data(k,:))';
  end
  ZPmat = zeros(samples-2,na);
  for k=3:samples,
    ZPmat(k-2,:)=roots(A_data(k,:))';
  end
  plot(ZPmat,'x');
  title('Poles (x) and zeros (o) of extracted linear models');grid
  axis([-1 1 -1 1])
  axis('equal')
  hold
  plot(Zmat,Zmat*0,'o');
  t=-pi:0.05:pi;
  plot(sin(t),cos(t))
  hold off
  drawnow
  s0='9';
  s1='From this plot it is quite clear why there were problems';
  s2='when the controller canceled the zero. Obviously it is';
  s3='very close to the unity circle. We can also see that the';
  s4='poles varies quite a lot. To get a better understanding';
  s5='of how the pole locations affect the behavior of the';
  s6='process we can calculate damping factor and natural';
  s7='frequency.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  % >>>>>>>>>>>>>>>>  SCREEN 10  <<<<<<<<<<<<<<<<<
  Cmat = zeros(samples-2,na-1);
  for k=3:samples,
    r=roots(A_data(k,:))';
    [n,m]=sort(real(r));
    [MAG,a1,a2]=ddamp(r(m)',0.2);
    Cmat(k-2,1)=a1(1);
    Cmat(k-2,2)=a2(1);
  end
  Cmat=real(Cmat);
  subplot(411)
  plot(2:samples-1,Cmat(:,1));grid
  title('Natural frequency (rad/s) and damping factor')
  axis([0 samples min(Cmat(:,1)) max(Cmat(:,1))])
  subplot(412)
  plot(2:samples-1,Cmat(:,2));grid
  axis([0 samples 0 1])
  xlabel('time (samples)')
  drawnow
  s0='10';
  s1='Obviously the process becomes less damped and the';
  s2='natural frequency larger when the magnitude of';
  s3='the output increases.';
  s4=[];
  s5=[];
  s6='              >>  THE END <<';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6); 
  set(edmulti,'String',smat);
  drawnow
end



