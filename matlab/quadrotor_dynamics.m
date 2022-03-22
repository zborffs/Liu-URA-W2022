function out = quadrotor_dynamics(t, state, p)
    % 
    
    M = p.M;
    m = p.m;
    L = p.L;
    l = p.l;
    g = p.g;

    x = state(1);
    y = state(2);
    z = state(3);
    xdot = state(4);
    ydot = state(5);
    zdot = state(6);
    alpha = state(7);
    beta = state(8);
    gamma = state(9);
    alphadot = state(10);
    betadot = state(11);
    gammadot = state(12);
    
%     u = get_control();
    
    k_beta = 1.25;
    k_betadot = 1.25 * 1.5;
    
    k_gamma = 1.25;
    k_gammadot = 1.25 * 1.5;
    
    u = zeros(4,1);
    
    % simultaneous equations: (1) m * l / ((M + 4 * m) * L) * (sum(u)) == -k_alphadot * alphadot - k_alpha * alpha;
    %                         (2) sum(u.^2)) / (M+4*m) * cos(beta) * cos(gamma) == g
    
    u(3) = sqrt((M + 4 * m) * g / (cos(beta) * cos(gamma)) / 4); % random. must be changed later
    u(1) = sqrt(u(3)^2 - k_beta * beta - k_betadot * betadot); % beta direction
    
    u(4) = -sqrt((M + 4 * m) * g / (cos(beta) * cos(gamma)) / 4); % random. must be changed later
    u(2) = -sqrt(u(4)^2 - k_gamma * gamma - k_gammadot * gammadot); % alpha direction
    
    assert(imag(u(1)) == 0);
    assert(imag(u(2)) == 0);
    assert(imag(u(3)) == 0);
    assert(imag(u(4)) == 0);
    
    a = 1 / (M + 4 * m) * sum(u.^2);
    
    out = [
        xdot;
        ydot;
        zdot;
        a * (cos(alpha) * sin(beta) * cos(gamma) - sin(alpha) * sin(gamma));
        a * (sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma));
        a * cos(beta) * cos(gamma) - g;
        alphadot;
        betadot;
        gammadot;
        m * l / ((M + 4 * m) * L) * sum(u);
        1 / m * (u(1)^2 - u(3)^2);
        1 / m * (u(2)^2 - u(4)^2)
    ];
end