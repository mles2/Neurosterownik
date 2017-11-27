clear all
%A = importdata('sunspot.asc.txt')
%u = 1:size(A,1)
%y = A(:,2)

%plot(u,y)
y = sim('actuator_model')