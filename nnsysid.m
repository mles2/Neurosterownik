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
