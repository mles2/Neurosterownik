clear all;
load actuator.mat;

%definicje wejść i wyjść docelowej sieci
wejscie_sieci = [ 0  u' 0  0 ;
                  0  0  u' 0 ;
                  0  p' 0  0 ;
                  0  0  p' 0 ;
                  0  0  0  p']';

wyjscie_sieci = [ p' 0 0 0]' ;

%ucięcie zer - redundantnych parametrów
wejscie_sieci ( 1025 : 1027 , : ) = [];
wejscie_sieci (    1 :    3 , : ) = [];

wyjscie_sieci ( 1025 : 1027 , : ) = [];
wyjscie_sieci ( 1 : 3 , : ) = [];

%uczenie wsadowe:
PuczWsadowe = wejscie_sieci(1 : 511, : )';
TuczWsadowe = wyjscie_sieci(1 : 511, : )';

PsprWsadowe = wejscie_sieci(512 : 1021, :)';
TsprWsadowe = wyjscie_sieci(512 : 1021, :)';

%uczenie przyrostowe:
PuczPrzyrostowe = num2cell(wejscie_sieci(1 : 511, : )' , 1);
TuczPrzyrostowe = num2cell(wyjscie_sieci(1 : 511, : )' , 1);

PsprPrzyrostowe = num2cell(wejscie_sieci(512 : 1021, :)',1);
TsprPrzyrostowe = num2cell(wyjscie_sieci(512 : 1021, :)',1);

%ucz wsadowo:
siecARXprzed = newlin ( PuczWsadowe , TuczWsadowe ) ;

disp('Uczę sieć ARX - wsadowo...')
disp(' ')
siecARXpo = train (siecARXprzed, PuczWsadowe, TuczWsadowe);

%ucz przyrostowo:
siecARX = siecARXprzed;
siecARX.adaptParam.passes = 511 ;
liczba_epok = 10;
bladMSE = zeros(1,liczba_epok);

disp('Uczę sieć - przyrostowo...')
disp(' ')

for i = 1:liczba_epok ,
     [siecARX,yUczARX,eUczARX] = adapt(siecARX, PuczPrzyrostowe, ...
                                                TuczPrzyrostowe);
     bladMSE(i) = mse(eUczARX);
     disp(sprintf('Krok %5d - błąd: %8.6f' , i , bladMSE(i) ) )
     
end
siecARXpo = siecARX;

figure
plot(bladMSE)
title('Training record')
xlabel('epoch')
ylabel('MSE error')

%sprawdzenie działania modelu - metodą one step ahead prediction na zbiorze
%uczącym

figure
subplot(3,1,1), plot(TuczWsadowe, 'r'), hold on
plot(cell2mat(yUczARX), 'b')
title('ARX - One-step ahead prediction - training data')
xlabel('Time (samples)')

%sprawdzenie działania modelu - one-step ahead prediction - na zbiorze
%sprawdzającym
disp('')
disp('Sprawdzam sieć ARX na zbiorze sprawdzającym...')
disp('')

ySprARX = sim(siecARXpo, PsprWsadowe);
subplot(312), plot(TsprWsadowe, 'r'), hold on
              plot(ySprARX,'b')
title('ARX - One-step ahead prediction - validating data')
xlabel ('Time (samples)')

% sprawdzenie działania modelu - metoda symulacji - na zbiorze
% sprawdzającym
disp('')
disp('Symuluję siećARX na zbiorze sprawdzającym...')
disp('')
ySimARX = zeros(1,510);
for i = 4:510 ,
    xSim = [PsprWsadowe(1:2, i); ...
        ySimARX(1,i-1); ...
        ySimARX(1,i-2); ...
        ySimARX(1, i-3)];
    ySimARX (1,i) = sim(siecARXpo, xSim);
end

subplot(313), plot(TsprWsadowe, 'r'),hold on
              plot(ySimARX, 'b')
title('ARX - Simulation - validating data')
xlabel('Time(samples)')
                  