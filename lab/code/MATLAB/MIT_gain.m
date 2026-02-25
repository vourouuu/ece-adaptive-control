clear;
clc;
close all;

% Initialization
gammas = [0.1, 0.5, 1, 4]; % 4 different adaptation gains
dt = 0.01;              % time step
T  = 20;                 % total time
t = 0:dt:T;

k0   = 1;
gain = 4.49;
pole = 2.59;
am = 10;
bm = 10;

yp_all    = zeros(length(gammas), length(t));
ym_all    = zeros(length(gammas), length(t));
theta_all = zeros(length(gammas), length(t));
e_all     = zeros(length(gammas), length(t));

for idx = 1:length(gammas) % loop for each γ_i
    gamma = gammas(idx); % γ_i
    
    yp    = 0;
    ym    = 0;
    theta = 0;
    S     = 0;
    
    Yp    = zeros(size(t));
    Ym    = zeros(size(t));
    Theta = zeros(size(t));
    E     = zeros(size(t));
    
    for k = 1:length(t)
        r = sin(t(k));  % reference
        e  = yp - ym;    % error
        u  = theta * r; % controller
        
        % Derivatives
        dyp = gain * u - pole * yp;
        dym = bm * r - am * ym;
        dS  = bm * r - am * S;
        
        % MIT rule
        dtheta = -gamma * e * S;

        % Euler integration
        yp    = yp + dyp * dt;
        ym    = ym + dym * dt;
        S     = S + dS * dt;
        theta = theta + dtheta * dt;

        Yp(k)    = yp;
        Ym(k)    = ym;
        Theta(k) = theta;
        E(k)     = e;
    end
    
    yp_all(idx,:)    = Yp;
    ym_all(idx,:)    = Ym;
    theta_all(idx,:) = Theta;
    e_all(idx,:)     = E;
end

% Plots
colors = ['b', 'r', 'g', 'k'];
figure;

% 1st plot: outputs
subplot(3,1,1);
hold on;
plot(t, ym_all(1,:), 'k--');
for idx = 1:length(gammas)
    plot(t, yp_all(idx,:), colors(idx));
end
hold off;
xlabel('Time (sec)');
ylabel('Outputs');
legendStrings = cell(1, length(gammas)+1);
legendStrings{1} = 'Model output (y_m)';
for idx = 1:length(gammas)
    legendStrings{idx+1} = ['Plant output (y_p), \gamma=' num2str(gammas(idx))];
end
legend(legendStrings, 'Location', 'best');
title('Model Reference Adaptive Control - MIT Rule');

% 2nd plot: error signal
subplot(3,1,2);
hold on;
for idx = 1:length(gammas)
    plot(t, e_all(idx,:), colors(idx));
end
hold off;
xlabel('Time (sec)');
ylabel('e(t)');
legendStrings = cell(1,length(gammas));
for idx = 1:length(gammas)
    legendStrings{idx} = ['\gamma=' num2str(gammas(idx))];
end
legend(legendStrings, 'Location', 'best');

% 3rd plot: theta
subplot(3,1,3);
hold on;
for idx = 1:length(gammas)
    plot(t, theta_all(idx,:), colors(idx));
end
hold off;
xlabel('Time (sec)');
ylabel('\theta(t)');
legend(legendStrings, 'Location', 'best');
