clear
close all

addpath('..')

Y = [];
Phi = [];
lambdas = [];
sigmas = [];

datapath = 'Data3/';
filepath = datapath;
files = dir([filepath '*.mat']);

filelist = files;
for i=1:length(filelist)
    
loadRegressor;

% normalization
kn = 1/max(y);
Y = kn*y;
Phi = kn*phi;

% identificazione
[lambda sigma Phi Y] = LSIdentification(Phi,Y, 2, 10);

data(i).y = Y;
data(i).phi = Phi;
data(i).sigma = sigma;

lambda
sigma

lambdas = [lambdas lambda];
sigmas = [sigmas sigma];

pause


end

%%



[lambdaML var_lambdaML] = MLIdentification(data, 3, 5);

lambdaML
s = sqrt(var_lambdaML)

figure(10)
subplot(2,1,1)
hist(lambdas(1,:)')
subplot(2,1,2)
plot(lambdas(1,:),sigmas,'.')


% figure(11)
% subplot(2,1,1)
% hist(lambdas(2,:)')
% subplot(2,1,2)
% plot(lambdas(2,:),sigmas,'.')

% figure(12)
% subplot(2,1,1)
% hist(lambdas(3,:)')
% subplot(2,1,2)
% plot(lambdas(3,:),sigmas,'.')