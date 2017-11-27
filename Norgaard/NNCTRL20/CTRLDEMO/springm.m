function xdot=springm(t,x)
global ugl;
xdot = [x(2) ; -x(1)-x(2)-x(1)*x(1)*x(1)+ugl];
