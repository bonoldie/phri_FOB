addpath('..')

theta_m = [];
dtheta_m = [];
ddtheta_m = [];
theta_e = [];
dtheta_e = [];
current = [];

filepath = [datapath '/Validation/'];
files = dir([filepath '*.mat']);

loadRegressor;

% Modello che approssima i dati sperimentali
y_model = phi * lambda;

% Errore fra modello ed esperimento
error = y - y_model;

% Calcolo della deviazione standard dell'errore
sigma = sqrt(var(error))

figure(3)
hold on
% Esperimento
plot(y)
% Modello (verde)
plot(y_model,'g')
% Errore (rosso)
plot(error,'r')
hold off




