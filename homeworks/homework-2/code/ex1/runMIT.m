% ---------------------------------------------
% MIT Rule simulation
% Inputs:
%   A     : amplitude of reference signal r(t)
%   w_r : frequency of reference signal r(t)
%   params: gamma, dt, T, a, b, z, w
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
    
    a = params.a; % plant parameter a
    b = params.b; % plant gain
    z = params.z; % reference model damping
    w = params.w; % reference model natural frequency
    
    yp = 0; dyp = 0; ddyp = 0; % plant model
    ym = 0; dym = 0; ddym = 0; % referene model
    
    theta1 = 0; dtheta1 = 0; % control parameter 1
    theta2 = 0; dtheta2 = 0; % control parameter 2
    
    S1 = 0; dS1 = 0; ddS1 = 0; % sensitivity function 1
    S2 = 0; dS2 = 0; ddS2 = 0; % sensitivity function 2
    
    N = length(t);
    Yp = zeros(1,N);
    Ym = zeros(1,N);
    Theta1 = zeros(1,N);
    Theta2 = zeros(1,N);
    E = zeros(1,N);
    
    % MIT Rule
    for i = 1:N
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
        dtheta1 = -gamma * S1 * e;
        dtheta2 = -gamma * S2 * e;
        
        theta1 = theta1 + dtheta1*dt;
        theta2 = theta2 + dtheta2*dt;
        
        Yp(i) = yp;
        Ym(i) = ym;
        Theta1(i) = theta1;
        Theta2(i) = theta2;
        E(i) = e;
    end
end