%% Initializations
clc; clear; close all;

% Physical constants
g = 9.81;   % The gravitational acceleration [m/s^2]
l =  0.2;   % Distance from the center of mass to each rotor [m]
m =  0.5;   % Total mass of the quadrotor [kg]
I = [1.24, 1.24, 2.48]; % Mass moment of inertia [kg m^2]
mu = 30.0;   % Maximum thrust of each rotor [N]
sigma = 0.01; % The proportionality constant relating thrust to torque [m]

% Parameter vector
p = [g l m I mu sigma];

% Initial conditions
z0 = zeros(12,1);

% External force (r) and moment (n) vectors
r = [0; 0; 0];
n = [0; 0; 0];


%% Simulations Parameters

% Set desired time interval
sim_seconds = 15;
time_step = 0.01;

% Vector of time steps
sim_steps = sim_seconds/time_step;
t = linspace(0, sim_seconds, sim_steps);

% Array of state vectors
z = zeros(sim_steps,12);
z(1,:) = z0;

% Array of control vectors
u = zeros(4,sim_steps);
u0 = [0,0,0,0];
u(:,1) = u0;

% Initial error values
E_prev = [0 0 0 0 0 0];

% Generate intruder trajectory
% wpts = [-5 5; 2 2; 5 5];
% wpts = [-5 3; 2 -2; 0 6];
wpts = [3 -1; -5 4; 10 0];
% wpts = [5 1 0; 2 2 -2; 8 4 8];
% vpts = [0.75 0; 0 -0.75; 0.4 0.4];
% vpts = [0.75 0.75; 0 0; 0 0];
vpts = [0 0; 0.5 0.5; -0.2 -0.4];
% vpts = [-0.5 -0.5 -0.5; 0.1 -0.3 -0.5; -0.3 0 0.3];
tpts = [0 15];
tvec = 0:time_step:sim_seconds;
[q, qd, qdd, pp] = quinticpolytraj(wpts, tpts, tvec, 'VelocityBoundaryCondition', vpts);


%% Simulate Dynamics w/ Controller for Capture Phase

fprintf("Pursuing intruder...\n\n")

for j = 1:sim_steps-1

    % Compute new state vector for each time step
    [t_out,z_out] = ode45(@(t,z) quadrotor(t, z, u(:,j), p, r, n), [t(j) t(j+1)], z(j,:));
    z_curr = z_out(end,:);
    z(j+1,:) = z_curr;

    distance_from_target = norm(z_curr(1:3) - q(:,j)');
    if distance_from_target < l
        fprintf("Captured intruder!\n")
        break
    end

    % Get desired trajectory values for timestep
    z_d = [q(:,j) (q(:,j+1) - q(:,j))/time_step zeros(3,1)];

    % Compute control vector
    u_curr = SMC_Controller(z_curr, z_d, p, "capture");

    % Construct control vector for next time step
    u(:,j+1) = u_curr;

end

capture_time = j; % record capture time

if capture_time == sim_steps-1
    fprintf("Failed to capture intruder.\n\n")
else
    fprintf("    capture time = %f seconds\n\n", t(capture_time))
end


%% Simulate Dynamics w/ Controller for Return Phase

fprintf("Returning to nest...\n\n")

% Reset simulation time
return_time = 10;
sim_steps = return_time/time_step;
t2 = linspace(0, return_time, sim_steps);

% Array of state vectors
z2 = zeros(sim_steps,12);
z2(1,:) = z_curr;

% Generate return trajectory

wpts2 = [z_curr(1) 0 0; z_curr(2) 0 0; z_curr(3) 0.5 0.05];
vpts = [0 0 0; 0 0 -0.1; 0 0 0];
tpts2 = [0 return_time-2 return_time];
tvec2 = 0:time_step:return_time;
[q2, qd2, qdd2, pp2] = quinticpolytraj(wpts2, tpts2, tvec2, 'VelocityBoundaryCondition', vpts);

for j = 1:sim_steps-1

    % Generate force + moment disturbance vectors for next time step
    r_rand = 0.2*rand(3,1);
    n_rand = 0.1*rand(3,1);

    r_rand = [0 0 0]';
    n_rand = [0 0 0]';

    % Compute new state vector for each time step
    [t_out,z_out] = ode45(@(t2,z2) quadrotor(t2, z2, u(:,j), p, r_rand, n_rand), [t2(j) t2(j+1)], z2(j,:));
    z_curr = z_out(end,:);
    z2(j+1,:) = z_curr;

    % Get desired trajectory values for timestep
    z_d = [q2(:,j) qd2(:,j) qdd2(:,j)];

    % Compute control vector
    u_curr = SMC_Controller(z_curr, z_d, p, "return");

    % Construct control vector for next time step
    u(:,j+1) = u_curr;

end

fprintf("Landed.\n\n")


%% Plot the results

for i=1:4
    ax(i) = subplot(2,2,i,'NextPlot','Add','Box','on','XGrid','on','YGrid','on',...
                'Xlim',[t(1), t(capture_time) + t2(end)],...
                'TickLabelInterpreter','LaTeX','FontSize',14);
    xlabel('t','Interpreter','LaTeX','FontSize',14);        
end

plot_time = [t(1:capture_time) (t2+t(capture_time))];
plot_data = cat(1,z(1:capture_time,:),z2(:,:));

plot(ax(1), plot_time, plot_data(:,1:3), 'LineWidth', 1.5);
legend(ax(1), {'$x_1$', '$x_2$', '$x_3$'},... 
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(1), '${\bf x}$','Interpreter','LaTeX','FontSize',14);
xlabel(ax(1), 't','Interpreter','LaTeX','FontSize',14);
xline(ax(1), t(capture_time),'-','Intruder captured');

plot(ax(3), plot_time, plot_data(:,4:6), 'LineWidth', 1.5);
legend(ax(3), {'$\phi$', '$\theta$', '$\psi$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(3), '\boldmath$\alpha$','Interpreter','LaTeX','FontSize',14);
xline(ax(3), t(capture_time),'-','Intruder captured');

plot(ax(2), plot_time, plot_data(:,7:9), 'LineWidth', 1.5);
legend(ax(2), {'$\dot{x}_1$', '$\dot{x}_2$', '$\dot{x}_3$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(2), '$\dot{\bf x}$','Interpreter','LaTeX','FontSize',14);
xline(ax(2), t(capture_time),'-','Intruder captured');

plot(ax(4), plot_time, plot_data(:,10:12), 'LineWidth', 1.5);
legend(ax(4), {'$\omega_1$', '$\omega_2$', '$\omega_3$'},...
    'Interpreter', 'LaTeX', 'FontSize', 14);
title(ax(4), '\boldmath$\omega$','Interpreter','LaTeX','FontSize',14);
xline(ax(4), t(capture_time),'-','Intruder captured');


%% Animation
animation_fig = figure;

airspace_box_length = 10;

animation_axes = axes('Parent', animation_fig,...
    'NextPlot','add','DataAspectRatio',[1 1 1],...
    'Xlim',airspace_box_length*[-0.5 0.5],...
    'Ylim',airspace_box_length*[-0.5 0.5],...
    'Zlim',airspace_box_length*[0 1],...
    'box','on','Xgrid','on','Ygrid','on','Zgrid','on',...
    'TickLabelInterpreter','LaTeX','FontSize',14);

view(animation_axes, 3);

N = 10;
Q = linspace(0,2*pi,N)';
circle = 0.3*l*[cos(Q) sin(Q) zeros(N,1)];
loc = l*[1 0 0; 0 1 0; -1 0 0; 0 -1 0];


silhouette = plot3(0,0,0, '--', 'Color', 0.5*[0.75 0.75 1], 'LineWidth', 1 ,...
    'Parent', animation_axes);
body = plot3(0,0,0, 'Color', [0 0 1], 'LineWidth', 2,...
        'Parent', animation_axes);
for i=1:4
    rotor(i) = plot3(0,0,0, 'Color', [0 0 1], 'LineWidth', 2,...
        'Parent', animation_axes);
end

silhouette_intruder = plot3(0,0,0, '--', 'Color', 0.5*[1 0.75 0.75], 'LineWidth', 1 ,...
    'Parent', animation_axes);
body_intruder = plot3(0,0,0, 'Color', [1 0 0], 'LineWidth', 2,...
        'Parent', animation_axes);
for i=1:4
    rotor_intruder(i) = plot3(0,0,0, 'Color', [1 0 0], 'LineWidth', 2,...
        'Parent', animation_axes);
end

body_return = plot3(0,0,0, 'Color', [0 1 0], 'LineWidth', 2,...
        'Parent', animation_axes);
for i=1:4
    rotor_return(i) = plot3(0,0,0, 'Color', [0 1 0], 'LineWidth', 2,...
        'Parent', animation_axes);
end

tic;
for k=1:capture_time
    
    R = [ cos(z(k,5))*cos(z(k,6)), sin(z(k,4))*sin(z(k,5))*cos(z(k,6)) - cos(z(k,4))*sin(z(k,6)), sin(z(k,4))*sin(z(k,6)) + cos(z(k,4))*sin(z(k,5))*cos(z(k,6));
          cos(z(k,5))*sin(z(k,6)), cos(z(k,4))*cos(z(k,6)) + sin(z(k,4))*sin(z(k,5))*sin(z(k,6)), cos(z(k,4))*sin(z(k,5))*sin(z(k,6)) - sin(z(k,4))*cos(z(k,6));
                     -sin(z(k,5)),                                 sin(z(k,4))*cos(z(k,5)),                                 cos(z(k,4))*cos(z(k,5))];

    for i=1:4
        ctr(i,:) = z(k,1:3) + loc(i,:)*R';
        pose = ones(N,1)*z(k,1:3) + (ones(N,1)*loc(i,:) + circle)*R';
        set(rotor(i), 'XData', pose(:,1), 'YData', pose(:,2),  'ZData', pose(:,3) );
         
    end
    set(silhouette,'XData', [0, z(k,1), z(k,1), z(k,1)],...
        'YData', [0, 0, z(k,2), z(k,2)],...
        'ZData', [0, 0, 0, z(k,3)]);
    set(body, 'XData', [ctr([1 3],1); NaN; ctr([2 4],1)], ...
        'YData', [ctr([1 3],2); NaN; ctr([2 4],2)],...
        'ZData', [ctr([1 3],3); NaN; ctr([2 4],3)] );

    R_intruder = [1 0 0; 0 1 0; 0 0 1];

    for i=1:4
        ctr(i,:) = q(1:3,k)' + loc(i,:)*R_intruder';
        pose = ones(N,1)*q(1:3,k)' + (ones(N,1)*loc(i,:) + circle)*R_intruder';
        set(rotor_intruder(i), 'XData', pose(:,1), 'YData', pose(:,2),  'ZData', pose(:,3) );
         
    end
    set(silhouette_intruder,'XData', [0, q(1,k)', q(1,k)', q(1,k)'],...
        'YData', [0, 0, q(2,k)', q(2,k)'],...
        'ZData', [0, 0, 0, q(3,k)']);
    set(body_intruder, 'XData', [ctr([1 3],1); NaN; ctr([2 4],1)], ...
        'YData', [ctr([1 3],2); NaN; ctr([2 4],2)],...
        'ZData', [ctr([1 3],3); NaN; ctr([2 4],3)] );

    pause(t(k)-toc);
    pause(0.01);
end

set(body, 'Color', [0 1 0]);
set(silhouette, 'Color', 0.5*[0.75 1 0.75])
delete(body_intruder);
delete(silhouette_intruder)
for i=1:4
    set(rotor(i), 'Color', [0 1 0]);
    delete(rotor_intruder(i));    
end

for k=1:length(t2)
    
    R = [ cos(z2(k,5))*cos(z2(k,6)), sin(z2(k,4))*sin(z2(k,5))*cos(z2(k,6)) - cos(z2(k,4))*sin(z2(k,6)), sin(z2(k,4))*sin(z2(k,6)) + cos(z2(k,4))*sin(z2(k,5))*cos(z2(k,6));
          cos(z2(k,5))*sin(z2(k,6)), cos(z2(k,4))*cos(z2(k,6)) + sin(z2(k,4))*sin(z2(k,5))*sin(z2(k,6)), cos(z2(k,4))*sin(z2(k,5))*sin(z2(k,6)) - sin(z2(k,4))*cos(z2(k,6));
                     -sin(z2(k,5)),                                 sin(z2(k,4))*cos(z2(k,5)),                                 cos(z2(k,4))*cos(z2(k,5))];

    for i=1:4
        ctr(i,:) = z2(k,1:3) + loc(i,:)*R';
        pose = ones(N,1)*z2(k,1:3) + (ones(N,1)*loc(i,:) + circle)*R';
        set(rotor(i), 'XData', pose(:,1), 'YData', pose(:,2),  'ZData', pose(:,3) );
         
    end
    set(silhouette,'XData', [0, z2(k,1), z2(k,1), z2(k,1)],...
        'YData', [0, 0, z2(k,2), z2(k,2)],...
        'ZData', [0, 0, 0, z2(k,3)]);
    set(body, 'XData', [ctr([1 3],1); NaN; ctr([2 4],1)], ...
        'YData', [ctr([1 3],2); NaN; ctr([2 4],2)],...
        'ZData', [ctr([1 3],3); NaN; ctr([2 4],3)] );

    pause(t2(k)-toc);
    pause(0.01);
end

delete(silhouette)

