load actuator.mat
p = p';
u = u';

daneUczace = figure;

subplot(211), plot(u), ylabel('u'); title('input: valve opening');
subplot(212), plot(p), ylabel('p'); title('output: oil pressure');
xlabel('time')

[p ,ps] = mapminmax(p);
[u, us] = mapminmax(u);

ps
us

pUcz = con2seq(p(1,1:512));
uUcz = con2seq(u(1,1:512));
pSpr = con2seq(p(1,513:1024));
uSpr = con2seq(u(1,513:1024));

uDelays = [1 2 ]; % opóźnienia wejścia (pobudzenie)
pDelays = [1 2 3]; %opóźnienia wyjścia (odpowiedź)

liczbaNeuronow = input ('Podaj liczbę neuronów w warstwie ukrytej sieci: ');
siecNARXprzed = narxnet(uDelays,pDelays, liczbaNeuronow)

%przygotowanie danych uczących:
siecNARXprzed.divideFcn = '';
siecNARXprzed.trainParam.min_grad = 1e-10;

[PsekwencjaUcz, PiUcz, AiUcz, tUcz] = preparets(siecNARXprzed, uUcz, {}, pUcz);
[PsekwencjaSpr, PiSpr, AiSpr, tSpr] = preparets(siecNARXprzed, uSpr, {}, pSpr);

%naucz sieć
siecNARXpo = train(siecNARXprzed, PsekwencjaUcz, tUcz, PiUcz);

%sprawdzenie sieci:
yUczNARX = sim ( siecNARXpo , PsekwencjaUcz , PiUcz ) ; % na zbiorze uczącym
ySprNARX = sim ( siecNARXpo , PsekwencjaSpr , PiSpr ) ; % na zbiorze sprawdzającym
view(siecNARXpo);

odpSieciNARXoneStep = figure ;
subplot ( 2,1,1 ), plot ( cell2mat(yUczNARX) , 'b' ), hold on
plot ( cell2mat(tUcz ) , 'r' )
title ( 'NARX - One-step ahead prediction - training data' )
subplot ( 2,1,2 ), plot ( cell2mat(ySprNARX) , 'b' ), hold on 
plot ( cell2mat(tSpr ) , 'r' )
title ( 'NARX - One-step ahead prediction - validating data' )


% zamknięcie w pętli sprzężenia:
siecNARXpoClosed = closeloop(siecNARXpo);
view(siecNARXpoClosed);

%przygotowanie danych do symulacji:
[ PsekwencjaUcz, PiUcz, AiUcz, tUcz ] = preparets ( siecNARXpoClosed, uUcz, { } , pUcz ) ;
[ PsekwencjaSpr, PiSpr, AiSpr, tSpr ] = preparets ( siecNARXpoClosed, uSpr, { } , pSpr ) ;
%symulacja:
yUczNARXclosed = sim ( siecNARXpoClosed, PsekwencjaUcz, PiUcz ) ; % na uczącym
ySprNARXclosed = sim ( siecNARXpoClosed, PsekwencjaSpr, PiSpr ) ; % na sprawdzającym
%wyniki symulacji:
odpSieciNARXmultiStep = figure ;
% na uczącym:
subplot ( 2,1,1 ), plot ( cell2mat(yUczNARXclosed) , 'b' ), hold on
plot ( cell2mat(tUcz ) , 'r' )
title ( 'NARX - Multi-step ahead prediction (simulation) - training data' )
% na sprawdzającym:
subplot ( 2,1,2 ), plot ( cell2mat(ySprNARXclosed) , 'b' ), hold on
plot ( cell2mat(tSpr ) , 'r' )
title ( 'NARX - Multi-step ahead prediction (simulation) - validating data' )


