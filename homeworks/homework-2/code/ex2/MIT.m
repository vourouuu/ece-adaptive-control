clear;
clc;
close all;

%% Initializations
gamma = 1;

dt = 0.001; % time step
T = 30;    % simulation time
t = 0:dt:T;

%% New Plant: P(s) = b/s
b = 4;
yp = 0; dyp = 0;

%% New Reference Model: M(s) = b_m / (s + a_m)
am = 1;
bm = 2;
ym = 0; dym = 0;

%% Amplitude and angular vel. for reference signal
A = 2.4;
w_r = 4.3212;

%% MIT Rule parameters
% Control law parameters: u = θ1*r - θ2*yp
theta1 = 0; dtheta1 = 0;
theta2 = 0; dtheta2 = 0;

% Sensitivity functions
S1 = 0; dS1 = 0;
S2 = 0; dS2 = 0;

%% Data storage
R = zeros(size(t));
E = zeros(size(t));
U = zeros(size(t));
Yp = zeros(size(t));
Ym = zeros(size(t));
Theta1 = zeros(size(t));
Theta2 = zeros(size(t));

%% MIT Rule
for i = 1:length(t)
    % r = A;
    r = A * sin(w_r * t(i));  % reference input
    e = yp - ym;              % tracking error
    u = theta1*r - theta2*yp; % control signal
        
    % Plant model dynamics
    dyp = b*u;
    yp = yp + dyp*dt;
    
    % Reference model dynamics
    dym = -am*ym + bm*r;
    ym = ym + dym * dt;
    
    % Sensitivity function    
    dS1 = b * (r - theta2 * S1);
    S1 = S1 + dS1*dt;

    dS2 = -theta2*b*S2 - b*yp;
    S2 = S2 + dS2*dt;
    
    % MIT adaptation law
    den = 1 + S1^2 + S2^2;
    % den = 1;
    dtheta1 = -gamma * e * S1 / den;
    dtheta2 = -gamma * e * S2 / den;
    
    theta1 = theta1 + dtheta1*dt;
    theta2 = theta2 + dtheta2*dt;
    
    R(i) = r;
    E(i) = e;
    U(i) = u;
    Yp(i) = yp;
    Ym(i) = ym;
    Theta1(i) = theta1;
    Theta2(i) = theta2;
end

%% Plots
figure(1);
subplot(4,1,1);
plot(t, Yp, t, Ym, 'r--')
xlabel('Time (sec)');
ylabel('y_p / y_m');
title('MIT Rule');
legend('Plant: y_p','Model: y_m');

subplot(4,1,2);
plot(t, E)
xlabel('Time (sec)');
ylabel('error')

subplot(4,1,3);
plot(t, Theta1)
xlabel('Time (sec)');
ylabel('\theta_1')

subplot(4,1,4);
plot(t, Theta2)
xlabel('Time (sec)');
ylabel('\theta_2')
