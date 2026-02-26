% ---------------------------------------------
% MIT Rule simulation
% Inputs:
%   A     : amplitude of reference signal r(t)
%   w_r : frequency of reference signal r(t)
%   params: gamma, dt, T, b, am, bm
% Outputs:
%   - t       : time vector
%   - Yp      : plant output
%   - Ym      : model output
%   - E       : error
%   - Theta1,2: theta histories
% ---------------------------------------------

function [t, Yp, Ym, E, Theta1, Theta2] = runMIT(A, w_r, params)
    gamma = params.gamma;
    dt    = params.dt;
    T     = params.T;
    t = 0:dt:T;
    
    b = params.b; % reference model damping
    am = params.am; % plant parameter a
    bm = params.bm; % plant gain
    
    yp = 0; dyp = 0; % plant model
    ym = 0; dym = 0; % referene model
    
    theta1 = 0; dtheta1 = 0; % control parameter 1
    theta2 = 0; dtheta2 = 0; % control parameter 2
    
    S1 = 0; dS1 = 0; % sensitivity function 1
    S2 = 0; dS2 = 0; % sensitivity function 2
    
    %% Data storage
    R = zeros(size(t));
    E = zeros(size(t));
    U = zeros(size(t));
    Yp = zeros(size(t));
    Ym = zeros(size(t));
    Theta1 = zeros(size(t));
    Theta2 = zeros(size(t));
    
    % MIT Rule
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
        % den = 1 + S1^2 + S2^2;
        den = 1;
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
end