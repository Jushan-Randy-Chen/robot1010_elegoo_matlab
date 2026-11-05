%
% sim_distance_control.m
%
% purpose: simulating distance control
%

% initialization

clear all; close all;

% set constants

x_star = 50; % target distance (mm)
k_max = 50; % maximum # of iterations
k_p = 8; % proportional feedback gain
t_s_star = 0.2; % assumed sampling period
delta_t_s = 0.0; % bound of t_s variation
n_s_star = 5; % sensor noise
delta_n_s = 1; % bound of n_s variation
x(1) = 100; % initial distance

% control loop 

for k = 1: k_max
    n_s(k) = n_s_star + (rand-0.5)*2*delta_n_s;
    t_s(k) = t_s_star + (rand-0.5)*2*delta_t_s;
    y(k) = x(k) + n_s(k);
    u(k) = k_p * ( y(k) - x_star );
    x(k+1) = x(k) - u(k) * t_s(k);
end

disp(sprintf('Steady State Error: %g',mean(x(end-10:end)-x_star)))
% time vector for plotting
% time = (0:k_max) * t_s;
time = [0 cumsum(t_s)];
% Plot the control input over time
figure;
plot(time(1:k_max), u, 'o-');
xlabel('Time (s)');
ylabel('Control input');
title('Control Input, $u$')
grid on;
% Plot the simulated distance and target value
figure;plot(time, x, 'o-', time, x_star * ones(size(time)), '--');
xlabel('Time (s)');
ylabel('Distance (mm)');
title('Distance to the Wall vs. Target Distance')
legend('Distance', 'Target');
grid on;