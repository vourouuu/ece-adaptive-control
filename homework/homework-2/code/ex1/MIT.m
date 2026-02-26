clear;
clc;
close all;

%% Initializations
gamma = 0.0009;

dt = 0.001; % time step
T = 100;    % total simulation time
t = 0:dt:T;

%% Plant: P(s) = 1 / (s*(s+a))
a = 1; % unknown plant parameter
b = 1; % gain
yp = 0; dyp = 0; ddyp = 0;

%% Reference Model: M(s) = ω^2 / (s^2 +2ζωs + ω^2)
z = 0.707;
w = 2;

ym = 0; dym = 0; ddym = 0;

%% Amplitude and angular vel. for reference signal
A = 10;
w_r = 1;

%% MIT Rule parameters
% Control law parameters: u = θ1*r - θ2*yp
theta1 = 0; dtheta1 = 0;
theta2 = 0; dtheta2 = 0;

% Sensitivity functions: S1 = ∂(e0)/∂(θ1), S2 = ∂(e0)/∂(θ2)
S1 = 0; dS1 = 0; ddS1 = 0; 
S2 = 0; dS2 = 0; ddS2 = 0;

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
    % r = A * sin(w_r * t(i)); % reference signal 
    r = A;
    e = yp - ym;               % tracking error
    u = theta1*r - theta2*yp;  % control input
    
    % Plant model dynamics
    ddyp = -a*dyp + b*u;
    dyp  = dyp + ddyp*dt;
    yp   = yp + dyp*dt;
    
    % Reference model dynamics
    ddym = -(2*z*w)*dym - (w^2)*ym + (w^2)*r;
    dym  = dym + ddym*dt;
    ym   = ym + dym*dt;
    
    % Sensitivity functions
    ddS1 = - a*dS1 - theta2*S1 + r;
    ddS2 = - a*dS2 - theta2*S2 - yp;
    
    dS1 = dS1 + ddS1*dt;
    dS2 = dS2 + ddS2*dt;
    
    S1 = S1 + dS1*dt;
    S2 = S2 + dS2*dt;
    
    % MIT adaptation law
    dtheta1 = -gamma * e * S1;
    dtheta2 = -gamma * e * S2;
    
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
xlabel('Time (sec)')
ylabel('\theta_2')
