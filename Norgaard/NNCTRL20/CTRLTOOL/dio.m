function [R,S,T] = dio(A,B,d,Am,Bm,Ao,Ar,As);
% DIO
% ---
%       This function represents the interface to the generic diophantine
%       equation solver DIOPHANT.
%
%       CALL [R,S,T] = dio(A,B,d,Am,Bm,Ao,Ar,As);
%  
%       The system to be controlled is described by y(t)=(B/A)u(t-d)
%       The controller is: u(t) = (T*ref(t) - S*y(t))/R
%       The desired model is ym(t)=(Bm/Am)ref(t-d)
%       Ao is the oberserver polynomial while Ar and As represent terms
%       to be forced into R and S, respectively

% Copyright Magnus Norgaard, IAU/IMM, Technical University of Denmark
% LastEditDate Aug. 20, 1996
AmAo = conv(Am,Ao);
AAr  = conv(A,Ar);
BAs  = conv(B,As);

[R,S]=diophant(AAr,BAs,d,AmAo);

R = conv(R,Ar);
S = conv(S,As);
T = conv(Bm,Ao);
