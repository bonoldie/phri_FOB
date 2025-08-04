clear
addpath('..')

mean_vel = [];
mean_cur = [];



datapath = 'DataLoad/';
filepath = datapath;
filelist = dir([filepath '*.mat']);

start = 5 * 3000;%wait n second



k = 3.30890;

for i=1:length(filelist)

eval(['load ' filepath filelist(i).name])
filelist(i).name

theta_m = Position(start:end);
theta_e = AuxiliaryPosition(start:end);
tau = k*(theta_m - theta_e);

% if abs(mean(pVelocity))<50 %because of outliers
    mean_vel = [mean_vel; mean(pAuxiliaryVelocity(start:end))]
    mean_cur = [mean_cur; mean(tau)];

% end
% pause
end

figure(3)
plot(mean_vel,mean_cur,'.')

%% model identification

dtheta_m = mean_vel
y = mean_cur;

[dtheta_m_pos dtheta_m_neg sign_dtheta_m_pos...
    sign_dtheta_m_neg] = frictionModes(dtheta_m,0.2);

% phi = [dtheta_m ];
phi = [dtheta_m sign(dtheta_m)];
% phi = [dtheta_m_pos dtheta_m_neg sign_dtheta_m_pos sign_dtheta_m_neg];%.225

[lambda sigma] = LSIdentification(phi,y, 10, 1);

lambda
sigma


% Modello che approssima i dati sperimentali
y_model = phi * lambda;
% Errore fra modello ed eserimento
error = y - y_model;

figure(999)
clf
hold on
% Esperimento
plot(mean_vel,y,'.')
% Modello (verde)
plot(mean_vel,y_model,'g.')
% Errore (rosso)
% plot(mean_vel,error,'r.')
hold off



