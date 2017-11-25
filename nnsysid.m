close all;
clear all;
clc
cd NNSYSID20
load spmdata
whos

%narysowanie przebiegów uczących

przebiegiUczace = figure;

subplot(211)
plot(u1)
title('Training data - Input sequence')

subplot(212)
plot(y1)
title('Training data - Output sequence')

przebiegiSprawdzajace = figure;

subplot(211)
plot(u2)
title('Validating data - Input sequence')

subplot(212)
plot(y2)
title('Validating data - Output sequence')

%przeskaluj dane -średnia 0 , wariancja 1:
[ u1s , uscales ] = dscale ( u1 ) ;
[ y1s , yscales ] = dscale ( y1 ) ;
u2s = dscale ( u2 , uscales ) ;
y2s = dscale ( y2 , yscales ) ;

%ustal rząd systemu
close all
OrderIndicies = lipschit(u1s,y1s, 1:5, 1:5);

% %dopasuj model liniowy OE (Output Error) - korzystając z System
% %identification Toolboxa
% 
% th = oe ( [ y1' u1' ] , [ 2 2 1 ] ) ;
% 
% present(th)
% close all
% figure, compare([y2' u2'],th,1);
% figure, resid ([y2' u2'] ,th);
% 
% %model nieliniowy NNOE
% NetDef = ['HHHHHHHHHH';...
%     'L---------'];
% NN = [2 2 1];
% 
% %naucz sieć
% trparms = settrain ;
% trparms = settrain ( trparms , 'maxiter',300 , 'D',1e-3 , 'skip',10 ) ;
% [ W1 , W2 , NSSEvec ] = nnoe ( NetDef , NN , [] , [] , trparms , y1s , u1s ) ;
% 
% %przeskaluj wagi
% [ w1 , w2 ] = wrescale ( 'nnoe' , W1 , W2 , uscales , yscales , NN ) ;
% 
% %przetestuj na zbiorze uczącym 
% 
% close all % zamykam poprzednie okienka
% %[ yhat , NSSE ] = nnvalid ( 'nnoe' , NetDef , NN , w1 , w2 , y1 , u1 ) ;
% 
% %przetestuj na zbiorze sprawdzającym
% close all % zamykam poprzednie okienka
% %[ yhat , NSSE ] = nnvalid ( 'nnoe' , NetDef , NN , w1 , w2 , y2 , u2 ) ;
% 
% % optymalna liczba wag w sieci
% et ( gca , 'Ylim' , [ 0 0.25 ] ) ;
% [mintev,index] =min(tev(pv));
% index = pv ( index )
% 
% close all % zamykam poprzednie okienka
% [ W1 , W2 ] = netstruc ( NetDef , thd , index ) ;
% 
% close all % zamykam poprzednie okienka
% [ W1 , W2 ] = netstruc ( NetDef , thd , index ) ;
