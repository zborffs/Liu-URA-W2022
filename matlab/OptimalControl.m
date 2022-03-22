addpath("../OptimTraj");
clear; clc;

% Physical parameters of the pendulum
p.k = 1;  % Normalized gravity constant
p.c = 0.1;  % Normalized damping constant

% User-defined dynamics and objective functions
problem.func.dynamics = @(t,x,u)( dynamics(x,u,p) );
problem.func.pathObj = @(t,x,u)( (1.0 - sqrt(angdiff(x(1,:), pi/2*ones(size(x(1,:)))).^2)) );
% problem.func.pathCst= @(t,x,u)( pathCst(x) );

% Problem bounds
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 0.5;
problem.bounds.finalTime.upp = 5.0;

problem.bounds.state.low = [-2*pi; -inf];
problem.bounds.state.upp = [2*pi; inf];
% problem.bounds.initialState.low = [0.1;0];
% problem.bounds.initialState.upp = [0.1;0];
problem.bounds.initialState.low = [2*pi-0.1;0];
problem.bounds.initialState.upp = [2*pi-0.1;0];
problem.bounds.finalState.low = [pi;0];
problem.bounds.finalState.upp = [pi;0];

problem.bounds.control.low = -5; %-inf;
problem.bounds.control.upp = 5; %inf;

% Guess at the initial trajectory
problem.guess.time = [0, 1];
problem.guess.state = [0, pi; pi pi];
problem.guess.control = [0, 0];

% Select a solver:
problem.options.method = 'trapezoid';
problem.options.defaultAccuracy = 'medium';

% Solve the problem
soln = optimTraj(problem);
t = soln.grid.time;
q = soln.grid.state(1,:);
dq = soln.grid.state(2,:);
u = soln.grid.control;

% Plot the solution:
figure(1); clf;

subplot(3,1,1)
plot(t,q)
ylabel('q')
title('Single Pendulum Swing-Up');

subplot(3,1,2)
plot(t,dq)
ylabel('dq')

subplot(3,1,3)
plot(t,u)
ylabel('u')

figure(2); clf;
rod_length = 1;
x = rod_length * sin(q);
y = -rod_length * cos(q);
p = [x; y];
drawPoleTraj(t, p, 10);

% val = p;
% xLow = min(val(1,:));
% xUpp = max(val(1,:));
% yLow = min(val(2,:));
% yUpp = max(val(2,:));
% drawFun = @(t,p)( drawPoleAnim(t,p,xLow, xUpp, yLow, yUpp) );
% P.plotFunc = drawFun;
% P.figNum = 3;
% P.frameRate = 24;
% P.speed = 2;
% P.fileName = 'cartPoleAnimation';
% animate(t, p, P);

function dx = dynamics(x,u,p)
% dx = dynamics(x,u,p)
%
% Computes the dynamics for the simple pendulum
%

q = x(1,:);
dq = x(2,:);

q = wrapTo2Pi(q);

k = p.k;    c = p.c;
ddq = -c*dq - k*sin(q) + u;
dx = [dq;ddq];

end