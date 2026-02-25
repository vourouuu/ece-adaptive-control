clear;
clc;
close all;

%% Parameters
params.gamma = 21;
params.dt    = 0.001; % simulation time step
params.T     = 100;   % total simulation time

%% Constants
params.a = 1;
params.b = 1;
params.z = 0.707;
params.w = 2;

%% A and ω ranges
A_vec     = linspace(0.1, 10, 100);
omega_vec = linspace(0.1, 20, 100);

length_A = length(A_vec);
length_W = length(omega_vec);

stable = zeros(length_W, length_A);
metric = zeros(length_W, length_A);

%% Stability criterion
jump = 20;              % jump
check_time_window = 40; % first 20 seconds of sim
tol = 1e-2;             % threshold

%% Diagram
for idx_A = 1:length_A
    for idx_w = 1:length_W

        A = A_vec(idx_A);         % current A
        omega = omega_vec(idx_w); % current ω

        try
            [t, Yp, Ym, E, ~, ~] = runMIT(A, omega, params);
        catch
            stable(idx_w, idx_A) = 0;
            metric(idx_w, idx_A) = Inf;
            continue;
        end

        % Early exit: catastrophic jump
        if (max(abs(Yp)) > jump)
            stable(idx_w, idx_A) = 0;
            metric(idx_w, idx_A) = Inf;
            continue; % continue with next pair of (A,ω)
        end

        % check last seconds
        idx_time = t > check_time_window;

        error = max(abs(E(idx_time)));
        metric(idx_w, idx_A) = error;

        if (error < tol)
            stable(idx_w, idx_A) = 1;
            fprintf("idx_w = %d, idx_A = %d\n", idx_w, idx_A);
        else
            stable(idx_w, idx_A) = 0;
        end
    end
end

%% Plot stability diagram
figure(2);
imagesc(A_vec, omega_vec, stable);
axis xy;
colorbar;
xlabel('Amplitude A');
ylabel('Frequency \omega');
title('Stability diagram (\gamma = 25)');
