function [lambda var_lambda] = MLIdentification(data, confidence, max_iteration_number)
% Least Square identification
% A least square procedure that iterates several times and eliminates
% outliers
% confidence: for removing outliers
% max_iteration_number: the maximum number of iteration
%
% EXAMPLES:
% [lambda sigma] = LSIdentification(phi,y, 3, 14)
% does 14 cycles and eliminates ouliers out of 3 sigma confidence
y = [];
phi = [];
phi_s = [];

for i=1:length(data)
    
y = [y; data(i).y];

phi = [phi; data(i).phi];
phi_s = [phi_s, (data(i).phi'/data(i).sigma)]; % phiT * Sigma^-1

end

retain_indexes = 1:length(phi);

% CICLO IDENTIFICAZIONE
for i=1:max_iteration_number 
    
    y = y(retain_indexes');
    phi = phi(retain_indexes,:);
    phi_s = phi_s(:,retain_indexes);
    
    % minimi quadrati
    lambda = (phi_s * phi) \ phi_s * y;
    var_lambda = inv(phi_s * phi);
    % Modello che approssima i dati sperimentali
    y_model = phi * lambda;
    
    % Errore fra modello ed esperimento
    error = y - y_model;

    % Calcolo della deviazione standard dell'errore
    sigma = sqrt(var(error));
    
    % Con ipotesi di rumore gaussiano elimino i dati fuori dall'intervallo
    % di probabilit�
    retain_indexes = find(abs(error) < sigma*confidence);
    
end

figure(998)
% Esperimento
plot(y)
hold on
% Modello (verde)
plot(y_model,'g')
% Errore (rosso)
plot(error,'r')
hold off