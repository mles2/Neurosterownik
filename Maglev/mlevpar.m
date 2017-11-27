%=================== System constants ====================
g=9.81;						% Gravity
mb=0.0682;					% Mass of metal ball
a=0.2042;
b=72.988;
Ts = 1/100;

%===== Define operating point and linearize system =====
yl = 0.02;
il = sqrt(mb*g)*(a+b*yl);

k=2*il/(mb*(a+b*yl)^2);
maglevpole=sqrt(abs(k*il*b/(a+b*yl)));

snum = -k;
sden = [1 0 -maglevpole^2];

% Discretize
[dnum,dden]=c2dm(snum,sden,Ts,'zoh');
