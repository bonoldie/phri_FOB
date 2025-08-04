
eval(['load ' filepath files(i).name])
files(i).name

Velocity = pVelocity;
Timesec = TimeSec;

figure(1)
title('data')
subplot(4,1,1)
plot(Timesec,ActiveCurrentA);
legend('current')
subplot(4,1,2)
plot(Timesec,Position);
legend('pos')
subplot(4,1,3)
plot(Timesec,Velocity);
legend('vel')
subplot(4,1,4)
plot(diff(Velocity) ./ diff(Timesec));
legend('acc')



[B,A] = butter(2,0.01);
ActiveCurrentA = filter(B,A,ActiveCurrentA);
Position = filter(B,A,Position);
Velocity = filter(B,A,Velocity);

figure(2)
title('filtered data')
subplot(4,1,1)
plot(Timesec,ActiveCurrentA,'r');
legend('current')
subplot(4,1,2)
plot(Timesec,Position,'r');
legend('pos')
subplot(4,1,3)
plot(Timesec,Velocity,'r');
legend('vel')
subplot(4,1,4)
plot( (diff(Velocity) ./ diff(Timesec)));
legend('acc')

current = [ActiveCurrentA];
theta_m = [Position];
dtheta_m = [Velocity];
ddtheta_m = [0; (diff(Velocity) ./ diff(Timesec))];

% pause

% figure(3)
% subplot(4,1,1)
% plot(current);
% subplot(4,1,2)
% plot(theta_m);
% subplot(4,1,3)
% plot(dtheta_m);
% subplot(4,1,4)
% plot(ddtheta_m);
% legend('acc')

[dtheta_m_pos dtheta_m_neg sign_dtheta_m_pos...
    sign_dtheta_m_neg] = frictionModes(dtheta_m);

y = current;

% direct drive motor
% y = current ...
%     - (0.00211*dtheta_m_pos + 0.002174*dtheta_m_neg + ...
%     0.144*sign_dtheta_m_pos + 0.16078*sign_dtheta_m_neg);

%geared motor
% y = current ...
% - (0.0382*dtheta_m_pos + 0.0263*dtheta_m_neg + ...
%     0.4971*sign_dtheta_m_pos + 0.5453*sign_dtheta_m_neg);
	
% geared DC fahulaber motor
y = current ...
- (0.0120*dtheta_m_pos + 0.0113*dtheta_m_neg + ...
    0.0959*sign_dtheta_m_pos + 0.1000*sign_dtheta_m_neg);

Ktr = 1;% 1.43333;
y = y * Ktr; % this alows to find the REFLECTED inertia

phi = [ddtheta_m];%.23
% phi = [ddtheta_m sign(dtheta_m)];%.225
% phi = [ddtheta_m dtheta_m sign(dtheta_m)];%.225

%phi = [ddtheta_m dtheta_m_pos dtheta_m_neg sign(dtheta_m)];%.225
%phi = [ddtheta_m dtheta_m_pos dtheta_m_neg sign_dtheta_m_pos sign_dtheta_m_neg];%.225
%phi = [ddtheta_m dtheta_m sign_dtheta_m_pos sign_dtheta_m_neg];%.225


