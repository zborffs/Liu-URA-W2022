addpath("src");

p = struct('m', 0.075, 'M', 0.1, 'l', 0.04, 'L', 0.1, 'g', 9.8);

model = @(t,state) quadrotor_dynamics(t,state,p);



init_conds = zeros(12,1); init_conds(8) = 0.1; init_conds(11) = 0.1; init_conds(9) = -0.2; init_conds(12) = -0.4;
tspan = [0 20];
[t,out] = ode45(model, tspan, init_conds);

x = out(:,1);
y = out(:,2);
z = out(:,3);
xdot = out(:,4);
ydot = out(:,5);
zdot = out(:,6);
alpha = out(:,7);
beta = out(:,8);
gamma = out(:,9);
alphadot = out(:,10);
betadot = out(:,11);
gammadot = out(:,12);


figure;
subplot(3,1,1);
plot(t,alpha); grid on;
subplot(3,1,2);
plot(t,beta); grid on;
subplot(3,1,3);
plot(t,gamma); grid on;

figure;
subplot(3,1,1);
% plot3(x,y,z); grid on;
plot(t,x); grid on;
subplot(3,1,2);
plot(t,y); grid on;
subplot(3,1,3);
plot(t,z); grid on;

figure;
subplot(3,1,1);
% plot3(x,y,z); grid on;
plot(t,xdot); grid on;
subplot(3,1,2);
plot(t,ydot); grid on;
subplot(3,1,3);
plot(t,zdot); grid on;
