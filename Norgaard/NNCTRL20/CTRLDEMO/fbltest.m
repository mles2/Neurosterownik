% PROGRAM DEMONTRATION OF CONTROL USING FEEDBACK LINEARIZATION
%
% Written by Magnus Norgaard, IAU/IMM, Technical Univ. of Denmark
% LastEditDate: Jan. 23, 2000
close all
StopDemo=0;
figure
guihand=gcf;
for k=1:1, %dummy loop

  % >>>>>>>>>>>>>>>>  BUILD GUI INTERFACE  <<<<<<<<<<<<<<<<<
  [guihand,edmulti,contbut,quitbut]=pmnshow;
  set(guihand,'Name','Control by feedback linearization');

  % >>>>>>>>>>>>>>>>  SCREEN 1  <<<<<<<<<<<<<<<<<
  s0='1';
  s1='The purpose of this demo is to show how a simple kind';
  s2='of discrete feedback linearization can be used for';
  s3='controlling a nonlinear process. The feedback linearizes';
  s4='the process by introduction of a "virtual" control input';
  s5='and assigns the closed-loop poles to a desired location.';
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
  s6='"experim." The plots above show the data set.';
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
  s1='To perform discrete feedback linearization we require';
  s2='that the process can be described by a particular model';
  s3='structure:';
  s4='             y(t)=f(phi(t)) + g(phi(t))*u(t-1)';
  s5='where';
  s6='             phi(t)=[y(t-1),..,y(t-n),u(t-2),..,u(t-m)].';
  s7='When the process is unknown we can let two neural';
  s8='networks model "f" and "g", respectively.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7,s8); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  
  % >>>>>>>>>>>>>>>>  SCREEN 4  <<<<<<<<<<<<<<<<<
  s0='4';
  s1='The NNSYSID-toolbox contains a function called "nniol"';
  s2='which does this. Let''s use a network with five hidden units';
  s3='for approximating "f" and a network with three hidden units for';
  s4='approximating "g". Since we are dealing with a second';
  s5='order process we will use as regressors two past outputs';
  s6='and two past controls.';
  subplot(411);delete(gca);subplot(412);delete(gca)
  subplot('position',[0.1 0.60 0.40 0.38]);
  drawnet(ones(5,4),ones(1,6),[],{'y(t-1)' 'y(t-2)' 'u(t-2)'},{'fhat(t)'});
  subplot('position',[0.50 0.45 0.40 0.38]);
  drawnet(ones(3,4),ones(1,4),[],{'y(t-1)' 'y(t-2)' 'u(t-2)'},{'ghat(t)'});
  title('Network architectures')
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6); 
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  % >>>>>>>>>>>>>>>>  SCREEN 5  <<<<<<<<<<<<<<<<<
  % ----- Train network -----
  s0='5';
  s1=[];
  s2='    >> Training process in action!! <<';
  s3=[];
  s4=[];
  s5='We run up to 100 iterations so you may have to';
  s6='wait for a while.';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6);
  set(edmulti,'String',smat);
  drawnow
  trparms = settrain;
  trparms = settrain(trparms,'maxiter',100);
  NN=[2 2 1];
  NetDeff = ['HHHHH'       
           'L----'];
  NetDefg = ['HHH'
           'L--'];
  [W1f,W2f,W1g,W2g]=...
       nniol(NetDeff,NetDefg,NN,[],[],[],[],trparms,Y1,U1);      
  save forward3 NetDeff W1f W2f NN NetDefg W1g W2g
  delete(gca); delete(gca);
  subplot('position',[0.1 0.60 0.40 0.38]);
  drawnet(W1f,W2f,eps,{'y(t-1)' 'y(t-2)' 'u(t-2)'},{'fhat(t)'});
  subplot('position',[0.50 0.45 0.40 0.38]);
  drawnet(W1g,W2g,eps,{'y(t-1)' 'y(t-2)' 'u(t-2)'},{'ghat(t)'});
  title('Trained network')
  if StopDemo==1, close all, break; end

  % >>>>>>>>>>>>>>>>  SCREEN 6  <<<<<<<<<<<<<<<<<
  s0='6';
  s1='The network has now been trained and we are ready to';
  s2='simulate the control system. Let''s select as our';
  s3='desired characteristic polynomial:';
  s4=[];
  s5='        Am(z)=z^{2} - 1.4z + 0.49';
  s6=[];
  s7='corresponding to two poles in z=0.7';
  smat=str2mat(s0,s1,s2,s3,s4,s5,s6,s7);
  pmnshow(smat,guihand,edmulti,contbut,quitbut);
  if StopDemo==1, close all, break; end
  
  % >>>>>>>>>>>>>>>>  SCREEN 7  <<<<<<<<<<<<<<<<<
  figure('Units','Centimeters','Position',[1.5 1.5 10 1.5]);
  pp=1;
  fblcon
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
  s0='7';
  s1='Obviously we have achieved a reasonably accurate';
  s2='model-following.';
  s3='              >>  THE END <<';
  smat=str2mat(s0,s1,s2,[],[],[],s3);
  set(edmulti,'String',smat);
  drawnow
end

