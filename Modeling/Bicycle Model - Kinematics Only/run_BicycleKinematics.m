% Run numerical simulation for the Bicycle Kinematics Model
% MCHA4100
% Uses classBicycleKinematics.m


clear all;
close all;
clc;

%% Define motor object and define parameters
bicycle = classBicycleKinematics;     % Obtain the object from classBicycleKinematics.m
bicycle.lf = 1;         % In meters (m), length from the front wheel to the center of gravity
bicycle.lr = 1;         % In meters (m), length from the rear wheel to the center of gravity



%% Solve the ODE
% Simulation time
t_sim = [0 10];

% Initial Conditions
x0 = [0; 0; deg2rad(0)];      % x = [N, E, psi]


%% Define inputs
Vin = 5;                     % In m/s, velocity of the vehicle, in the same direction as the wheels
sigma_f = deg2rad(20);       % In radians, converted from degrees


%% Define system ODE --- x = [N, E, psi] and u = [Vin, sigma_f]

% Create function handle for solver input format
sys_wrap = @(t,x) bicycle.state_derivative(x,Vin,sigma_f);


% Solve ODE
options = odeset('RelTol', 1e-6);     % Set relative tolerance
[results.t, results.x] = ode45(sys_wrap,t_sim,x0,options);



%% Plot the results
figure(1)
title("Simulation Bicycle Model Kinematics")
subplot(3,1,1)
plot(results.t, results.x(:,1));        % Plot 'N'
ylabel("North Position (m)")
grid on


subplot(3,1,2)
plot(results.t, results.x(:,2));        % Plot 'E'
ylabel("East Position (m)")
grid on


subplot(3,1,3)
plot(results.t, results.x(:,3));        % Plot 'psi'
ylabel("Yaw Angle From the Vertical (Radians)")
grid on



%% Visualize the vehicle moving on a 2D plane

figure(2)
grid on
xlabel("East Position (m)")
ylabel("North Position (m)")
axis([min(results.x(:,2))-2    max(results.x(:,2))+2       min(results.x(:,1))-2    max(results.x(:,1))+2])

bicycle_model = rectangle('EdgeColor','k','FaceColor','b');

North_pos = results.x(:,1);
East_pos = results.x(:,2);
Psi_angle = results.x(:,3);

for k1 = 1:length(results.x(:,1))

  bicycle_model.Position = [East_pos(k1)  North_pos(k1), 0.4  0.4];
  drawnow
  pause(0.05)

end






