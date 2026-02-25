clear;
clc;
close all;

%% Projection function
function proj = projection_op(theta, y, theta_max, epsilon_theta)
    % 1. f(θ)
    norm_theta_sq = sum(theta(:).^2);
    f = (norm_theta_sq - theta_max^2) / (epsilon_theta * theta_max^2);
    
    % 2. Gradient
    grad_f = (2 * theta) / (epsilon_theta * theta_max^2);
    
    % 3. Conditions: f(theta) > 0 && y.T * grad_f > 0
    if (f > 0) && (sum(y .* grad_f, 'all')) > 0
        % 4. Definition
        term1 = grad_f * sum(grad_f .* y, 'all') / sum(grad_f.^2, 'all');
        proj = y - term1 * f;
    else
        proj = y;
    end
end

%% Initializations
J = 4.6 * 1e-5;  % inertia torque (kgr*m^2)
Km = 0.19;       % constant of motor's torque (N*m/A)
Bf = 8.0 * 1e-4; % coefficient of friction (N*m*sec/rad)
N = 50;          % num of engine notches

% We have α% uncertainty
alpha = 0.75;
J_real  = J  * (1 + alpha*(2*rand-1));
Bf_real = Bf * (1 + alpha*(2*rand-1));
Km_real = Km * (1 + alpha*(2*rand-1));

dt = 0.001; % time step
T = 30;     % total simulation time
time_vec = 0:dt:T;

%% Nominal model
A_nom = [0 1 ; 0 -Bf/J];
B_nom = [0 ; Km/J];

%% Real model
A_real= [0 1 ; 0 -Bf_real/J_real];
B_real = [0 ; Km_real/J_real];

%% Reference Model
Ar = [0 1 ; -24 -10];
Br = [0 ; 24];

Q = eye(2);
P = lyap(Ar', Q);

%% Nominal values of feedback gains
Kx_hat_nom = [-24/(Km/J) -10/(Km/J)];
Kr_hat_nom = 24/(Km/J);

%% Adaptive values of feedback gains
Dx_hat = [0 0];
Dr_hat = 0;
Di_hat = 0;

%% Controller
X = zeros(2, length(time_vec));
Xr = zeros(2, length(time_vec));
U = zeros(1, length(time_vec));
Dx = zeros(2, length(time_vec));
Dr = zeros(1, length(time_vec));
Di = zeros(1, length(time_vec));

gamma_x = [0.5 0 ; 0 0.5];
gamma_r = 0.2;
gamma_i = 1;

% State vector
theta = 0; omega = 0;
x = [theta ; omega];

% Reference state vector
theta_r = 0; omega_r = 0;
xr = [theta_r ; omega_r];

% Integral model
z = 0;

% Reference signal r
theta_c = 0;

for i = 1:length(time_vec)
    t = time_vec(i);

    if (mod(t, 3) < dt)
        theta_c = theta_c + 5*pi/180;
        z = 0;
    end

    % 1. State error
    e = x - xr;
   
    % 2. Control law
    Fx = Kx_hat_nom;
    Fr = Kr_hat_nom;
    u_nom = Fx * x + Fr * theta_c;
    u = u_nom + Dx_hat * x + Dr_hat * theta_c + Di_hat * z;
    
    % 3. Euler integration
    z_dot = theta_c - x(1); % z_dot = θc - θ
    z = z + z_dot * dt;

    x_dot = A_real * x + B_real * u;
    x = x + x_dot * dt;

    xr_dot = Ar * xr + Br * theta_c;
    xr = xr + xr_dot * dt;
    
    % 4. Adaptive laws MRAC
    proj_Dx = projection_op(Dx_hat.', -x * e.' * P * B_nom, 2, 0.05);
    proj_Dr = projection_op(Dr_hat, -theta_c * e.' * P * B_nom, 2, 0.05);
    proj_Di = projection_op(Di_hat, -z * e.' * P * B_nom, 2, 0.05);
    
    Dx_hat_dot = gamma_x * proj_Dx;
    Dr_hat_dot = gamma_r * proj_Dr;
    Di_hat_dot = gamma_i * proj_Di;
    
    Dx_hat = Dx_hat + Dx_hat_dot.' * dt;
    Dr_hat = Dr_hat + Dr_hat_dot * dt;
    Di_hat = Di_hat + Di_hat_dot * dt;
    
    X(:, i) = x;
    Xr(:, i) = xr;
    U(i) = u;
    Dx(:, i) = Dx_hat;
    Dr(i) = Dr_hat;
    Di(i) = Di_hat;
end

%% Plots
f1 = figure(1);
subplot(2,1,1);
plot(time_vec, X(1,:));
hold on;
plot(time_vec, Xr(1,:));
ylabel('\theta (rad)');
legend('θ', 'θ_r');
title('Angle tracking');

subplot(2,1,2);
plot(time_vec, X(2,:));
hold on;
plot(time_vec, Xr(2,:));
ylabel('\omega (rad/s)');
xlabel('Time (sec)');
legend('ω', 'ω_{r}');
title('Angular velocity tracking');

saveas(f1, 'tracking-75-uncertainty.png');

f2 = figure(2);
subplot(3,1,1);
plot(time_vec, Dx(1,:));
hold on;
plot(time_vec, Dx(2,:));
legend({'$\hat{D}_{x_1}$', '$\hat{D}_{x_2}$'}, 'Interpreter', 'latex');
title('Adaptive Gains');

subplot(3,1,2);
plot(time_vec, Dr, 'r');
legend({'$\hat{D}_r$'}, 'Interpreter', 'latex');

subplot(3,1,3);
plot(time_vec, Di, 'g');
xlabel('Time (sec)');
legend({'$\hat{D}_i$'}, 'Interpreter', 'latex');

saveas(f2, 'gains-75-uncertainty.png');