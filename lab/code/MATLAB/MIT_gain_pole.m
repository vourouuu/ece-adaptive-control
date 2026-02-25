clear;
clc;
close all;

% Initialization
gamma = 2; % single adaptation gain
dt = 0.01; % time step
T = 20;    % total time
t = 0:dt:T;

gain = 4.49;
pole = 2.59;
am = 10;
bm = 10;

yp     = 0;
ym     = 0;
theta1 = 0; % θ_1
theta2 = 0; % θ_2
S1     = 0; % sensitivity for θ_1
S2     = 0; % sensitivity for θ_2

Yp     = zeros(size(t));
Ym     = zeros(size(t));
Theta1 = zeros(size(t));
Theta2 = zeros(size(t));
E      = zeros(size(t));

for i = 1:length(t)
    r = sin(t(i));                % reference
    e = yp - ym;                  % error
    u = theta1 * r - theta2 * yp; % controller with feedback

    % Derivatives
    dyp =  gain * u  - am * yp;
    dym =  bm   * r  - am * ym;
    dS1 =  gain * r  - am * S1;
    dS2 = -gain * yp - am * S2;

    % MIT rule
    dtheta1 = -gamma * e * S1;
    dtheta2 = -gamma * e * S2;

    % Euler integration
    yp     = yp + dyp * dt;
    ym     = ym + dym * dt;
    S1     = S1 + dS1 * dt;
    S2     = S2 + dS2 * dt;
    theta1 = theta1 + dtheta1 * dt;
    theta2 = theta2 + dtheta2 * dt;

    Yp(i)     = yp;
    Ym(i)     = ym;
    Theta1(i) = theta1;
    Theta2(i) = theta2;
    E(i)      = e;
end

% Plots
figure;

% 1st plot: outputs
subplot(4,1,1);
plot(t, Yp, t, Ym, 'r--');
xlabel('Time (sec)');
ylabel('Outputs');
legend('Model output (y_m)', 'Plant output (y_p)');
title('Model Reference Adaptive Control - MIT Rule');

% 2nd plot: error signal
subplot(4,1,2);
plot(t, E);
xlabel('Time (sec)');
ylabel('e(t)');

% 3rd plot: theta_1
subplot(4,1,3);
plot(t, Theta1);
xlabel('Time (sec)');
ylabel('\theta_1(t)');

% 4th plot: theta_2
subplot(4,1,4);
plot(t, Theta2);
xlabel('Time (sec)');
ylabel('\theta_2(t)');