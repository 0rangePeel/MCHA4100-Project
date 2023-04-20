% Run numerical simulation for the Bicycle Dynamics Model
% MCHA4100
% Uses classBicycleDynamics.m


clear all;
close all;
clc;

%% Define motor object and define parameters
bicycle = classBicycleDynamics;     % Obtain the object from classBicycleDynamics.m

bicycle.Lf = 1.4;                 % Length of forward wheel to center of gravity (m)
bicycle.Lr = 1.6;                 % Length of back wheel to center of gravity (m)
bicycle.g = 9.81;               % Acceleration due to Gravity (m/s/s)
bicycle.m = 2000;                % Mass of the vehicle (kg)
bicycle.Calphaf = 12e3;          % Cornering stiffness coefficient for front tyre
bicycle.Calphar = 11e3;          % Cornering stiffness coefficient for rear tyre

bicycle.Iz = 4000;              % Moment of Inertia about the Z-axis



%% Solve the ODE
% Simulation time
dt = 0.1;
t_sim = 0:dt:120;
t = t_sim;

% Initial Conditions
x0 = [5; 5; 0; 0; 0];      % x = [xg, yg, theta, vy, r]


%% Define inputs
Vx = 10;                     % In m/s, velocity of the vehicle
sigma_f = deg2rad(50);       % In radians, converted from degrees


%% Define system ODE --- x = [xg, yg, theta, vy, r] and u = [sigma_f]

sys_wrap = @(t,x) bicycle.state_derivative(x, sigma_f, Vx);

% Solve ODE
options = odeset('RelTol', 1e-6);     % Set relative tolerance
[results.t, results.x] = ode45(sys_wrap,t_sim,x0,options);



%% Plot the results
figure(1)
title("Simulation Bicycle Model Dynamics")
subplot(3,1,1)
plot(results.t, results.x(:,1));        % Plot 'Xg'
ylabel("East Position (m)")
grid on


subplot(3,1,2)
plot(results.t, results.x(:,2));        % Plot 'Yg'
ylabel("North Position (m)")
grid on


subplot(3,1,3)
plot(results.t, results.x(:,3));        % Plot 'theta'
ylabel("Yaw Angle (Radians)")
grid on


figure(2)
title("Simulation Bicycle Model Dynamics")
subplot(2,1,1)
plot(results.t, results.x(:,4));        % Plot 'Vy'
ylabel("Lateral Velocity (m)")
grid on

% Plot 'r' (Yaw rate)
subplot(2,1,2)
plot(results.t, results.x(:,5));        % Plot 'r'
ylabel("Yaw Rate (m)")
grid on


E = results.x(:,1);
N = results.x(:,2);
psi = results.x(:,3);
theta = results.x(:,3);


%% Visualize the vehicle moving on a 2D plane
%% Animation shows center of gravity, back wheel and front wheel positions
fig     = 3;
hf      = figure(fig); clf(fig);
hf.Color = 'w';
ax      = axes(hf, 'FontSize', 14);
hold(ax, 'on');

hFt     = plot(ax, nan(size(E)), nan(size(N)), 'r');
hRt     = plot(ax, nan(size(E)), nan(size(N)), 'g');
hCt     = plot(ax, nan(size(E)), nan(size(N)), 'b');
hP      = plot(ax, nan, nan, 'k-');
hF      = plot(ax, 0, 0, 'r.');
hR      = plot(ax, 0, 0, 'g.');
hC      = plot(ax, 0, 0, 'b.');

tF      = text(ax, 0, 0, ' F', 'FontSize', 10, 'Color', 'r');
tR      = text(ax, 0, 0, ' R', 'FontSize', 10, 'Color', 'g');
tC      = text(ax, 0, 0, ' C', 'FontSize', 10, 'Color', 'b');

hold(ax, 'off');
axis(ax, 'equal');
axis(ax, [min(E) - 2, max(E) + 2, min(N) - 2, max(N) + 2]);
grid(ax, 'on');
xlabel(ax, 'East [m]');
ylabel(ax, 'North [m]');


Se3  = skew([0; 0; 1]);

rCNn = nan(3, length(t));
rFNn = nan(3, length(t));
rRNn = nan(3, length(t));
rBNn = nan(3, length(t));

rFBb = [bicycle.Lf; 0; 0];
rRBb = [-bicycle.Lr; 0; 0];

% Plot the body of the vehicle
r = 0.5;      % Equvalent to the Lf and Lr variables, % North, East, Down
d = 1.5;
% Draw car shape
curve = linspace(3*pi/2, pi/2, 50);

% Plotting and drawing body of the vehicle
rPCb = [ [-d; r; 0], [d; r; 0], [d; -r; 0], [-d; -r; 0], [-d; r; 0], ... % Car Body
    [-d; r+0.25; 0], [-d+1; r+0.25; 0], [-d+1; r; 0], ...  % Car Wheel 1 - Back wheel
    [d; r; 0], [d; r+0.25; 0], [d-1; r+0.25; 0], [d-1; r; 0], [d; r; 0], ...  % Car Wheel 2 - Front wheel (Turning wheel)
    [d; -r; 0], [d; -r-0.25; 0], [d-1; -r-0.25; 0], [d-1; -r; 0], [d; -r; 0], ... % Car Wheel 3 - Front wheel (Turning wheel)
    [-d; -r; 0], [-d; -r-0.25; 0], [-d+1; -r-0.25; 0], [-d+1; -r; 0], ...          % Car Wheel 4 - Back wheel
    [-d; -r; 0], [-d; r; 0], [d; r; 0], -r*[cos(curve)-2.5; sin(curve); zeros(size(curve))]];     % Mad Aerodynamics

                        

for i = 1:length(t)
    Rnb = [sin(theta(i)) cos(theta(i)) 0; cos(theta(i)) -sin(theta(i)) 0; 0 0 1];
    rCNn(:, i) = [N(i); E(i); 0];
    rBNn(:, i) = [N(i); E(i); 0];

    rFNn(:, i) = rBNn(:, i) + [bicycle.Lf*sin(theta(i)); bicycle.Lf*cos(theta(i)); 0];
    rRNn(:, i) = rBNn(:, i) + [-bicycle.Lr*sin(theta(i)); -bicycle.Lr*cos(theta(i)); 0];


    rPNn = rCNn(:, i) + Rnb*rPCb;
     hP.XData = rPNn(2, :);
     hP.YData = rPNn(1, :);
    
    hFt.XData = rFNn(2, :);
    hFt.YData = rFNn(1, :);
    hRt.XData = rRNn(2, :);
    hRt.YData = rRNn(1, :);
    hCt.XData = rCNn(2, :);
    hCt.YData = rCNn(1, :);
    
    hF.XData = rFNn(2, i);
    hF.YData = rFNn(1, i);
    tF.Position = [rFNn(2, i), rFNn(1, i), 0];
    hR.XData = rRNn(2, i);
    hR.YData = rRNn(1, i);
    tR.Position = [rRNn(2, i), rRNn(1, i), 0];
    hC.XData = rCNn(2, i);
    hC.YData = rCNn(1, i);
    tC.Position = [rCNn(2, i), rCNn(1, i), 0];


    pause(0.05);
    drawnow
end





