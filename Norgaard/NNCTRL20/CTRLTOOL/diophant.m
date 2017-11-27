function [F,G]=diophant(A,B,d,C)
% 
% [F,G] = diophant(A,B,d,C)
%                                                    -d
% The function solves the Diophantine identity AF + q   BG = C with respect
% to the F and G polynomials. A and C must be monic. 

na = length(A)-1;     % Order of A
nb = length(B)-1;     % Order of B
nc = length(C)-1;     % Order of C

ng = na-1;            % Order of G
nf = nb+d-1;          % Order of F
if nc>ng+nf+1,
  nf = nc-na;
end

% -- Create the Sylvester matrix --
SV = zeros(nf+ng+1,nf+ng+1);
SV(1:nf,1:nf) = eye(nf);
for i=1:nf,
  SV(1+i:na+i,i) = A(2:na+1)';
end
for i=1:ng+1,
  SV(d+i-1:d+i+nb-1,nf+i) = B';
end

% -- Create the right side of the identity --
ac = zeros(nf+ng+1,1);
ac(1:nc) = C(2:nc+1)';
ac(1:na) = ac(1:na)-A(2:na+1)';

% -- Solve the Diophantine identity --
FG = SV\ac;
F = [1 FG(1:nf)'];
G = FG(nf+1:ng+nf+1)';
