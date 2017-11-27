function prsout = prs(N,maxval,alpha)
% PRS
%     prsout = prs(N,maxval,alpha)
%     generates a "level change at random instances" signal.
%     The signal is defined in the following way:
%     u(0)=randn
%     for t>0:
%                      / u(t-1) with probability alpha
%              u(t) = |
%                      \ randn  with propability 1-alpha  
%
%     I.e., at random instances the signal is changed to a new random
%     value. The "shift frequency" is determined by 'alpha' which is a
%     variabel between 0 and 1. alpha=0 generates a constant signal while
%     alpha=1 generates a Gaussian white noise signal. The signal is scaled
%     so that it contains values between -maxval and +maxval. N is the
%     length of the signal.
%
%     The signal has the following auto correlation function:
%                   -tau
%     R(tau) = alpha 

% Programmed by Magnus Norgaard. LastEditDate: Oct. 12, 1994
prsvec = [randn zeros(1,N-1)];
for i=2:N,
 if rand>(1-alpha)
   prsvec(i) = prsvec(i-1);
 else
   prsvec(i) = randn;
 end
end
prsvec = prsvec-(max(prsvec)+min(prsvec))/2;
prsvec = maxval/max(abs(prsvec))*prsvec;
if nargout==0
 plot(prsvec);
else prsout=prsvec;
end
