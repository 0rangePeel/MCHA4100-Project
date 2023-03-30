% MPC control for the Kinematics Bicycle Model


clear;
clc;
close all;


%% Create Bicycle Kinematics Object
kinematicBicycle = classBicycleKinematics;


%% Define bicycle object and define parameters
bicycle = classBicycleKinematics;     % Obtain the object from classBicycleKinematics.m
bicycle.lf = 1;         % In meters (m), length from the front wheel to the center of gravity
bicycle.lr = 1;         % In meters (m), length from the rear wheel to the center of gravity



%% Create linearised kinematics bicycle object
bicycleKinematicsLinearized = classBicycleKinematicsLinear;
 
% Define parameters
bicycleKinematicsLinear.lf = 1;         % In meters (m), length from the front wheel to the center of gravity
bicycleKinematicsLinear.lr = 1;         % In meters (m), length from the rear wheel to the center of gravity


%% Open Loop System
% Simulation time
t_sim = [0 10];
 
% Initial Conditions
x0 = [0; 0; deg2rad(0)];      % x = [N, E, psi]

%% Define inputs
Vin = 5;                     % In m/s, velocity of the vehicle, in the same direction as the wheels
sigma_f = deg2rad(20);       % In radians, converted from degrees


%% Define system ODE --- x = [N, E, psi] and u = [Vin, sigma_f]

% % Create function handle for solver input format
sys_wrap = @(t,x) bicycle.state_derivative(x,Vin,sigma_f);


% % Solve ODE
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








%% Create MPC control object for Bicycle Kinematic Model
controllerKinematicBicycle = classMPCcontrol_BicycleKinematics;

% Control period
T = 0.01; % Sampling at 100Hz (100 times a second)

% LQR cost and tuning parameters
Q = diag([0.1 0.1 0.1]);
R = 1;
N = zeros(3,1);


A = bicycleKinematicsLinearized.A(x, Vin, sigma_f);  % Linearized model to design a controller for the Nonlinear system
B = bicycleKinematicsLinearized.B(x, Vin, sigma_f);  % Linearized model to design a controller for the Nonlinear system

controllerKinematicBicycle.T = T;

% Control horizon
N_horizon = 15;

% Actuator limit constraints (Input is [Velocity; Steering angle])
u_max = 20;
u_min = 0;

u_max2 = deg2rad(45);
u_min2 = deg2rad(-45);

u_constraints_max = [u_max; u_max2];
u_constraints_min = [u_min; u_min2];


% Actuator slew constraints (Input is [Velocity; Steering angle])
delta_u_max = 5;
delta_u_min = -5;

delta_u_max_2 = deg2rad(30);
delta_u_min_2 = deg2rad(-30);

delta_u_constraints_max = [delta_u_max; delta_u_max_2];
delta_u_constraints_min = [delta_u_min; delta_u_min_2];


% Initialise controller
@(t,x) controllerKinematicBicycle.controlInit_regulation(A,B,Q,R,N,T,N_horizon, ...
    u_constraints_max,u_constraints_min,delta_u_constraints_max,delta_u_constraints_min)



%% Run simulation
% Define initial conditions x = [N, E, psi]
% Initial Conditions
sim.N0 = 0;
sim.E0 = 0;
sim.psi0 = deg2rad(0);

% Initial Conditions
sim.x0 = [sim.N0; sim.E0; sim.psi0];
 
% Define vector of time points where the controller is updated
sim.time = [0:controllerKinematicBicycle.T:10];


% Set ODE solver options
options = odeset('RelTol',1e-6);


% Run the ODE solver on each defined time interval
res.t = sim.time(1);
res.x = sim.x0.';
res.V = 0;
res.sigma_f = 0;

 
% Iterative loop to solve for each period (T) and update the control signal
% after each period (In this case, updates 200 times every second)
 
for i=1:length(sim.time)-1
x0 = res.x(end,:).';
t = [sim.time(i) sim.time(i)+controllerKinematicBicycle.T];

% Update the input 'Velocity' and 'Steering Angle' using the new control input (Implemented controller)
[V] = controllerKinematicBicycle.control([x0(1); x0(2); x0(3)]);

% Define ODE to be solved
sim.ode = @(t,x) kinematicBicycle.state_derivative(x,V,sigma_f);


% Run ODE solver over ith time interval
[t, x] = ode45(sim.ode,t,x0,options);


% Concatinate solutions
res.t = [res.t; t];            % Grab the last value(s) which correspond to the end of that interval
res.x = [res.x; x];          % Grab the last value(s) which correspond to the end of that interval

res.V = [res.V; V];                         % Save inputs 
res.sigma_f = [res.sigma_f; sigma_f];       % Save inputs
end


%% Plot the results - For Discrete Time - MPC
figure(2)
title("Simulation Bicycle Model Kinematics - MPC Control")
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


















%% MPC Controller with Reference Feed-Forward and Integral Action
% Control period
T = 0.04; % Sampling at 25Hz (25 times a second)

% Output - Regulate 'omega' to a reference value
y_ref = 0;

C_r = [0 0];            % [theta; ptheta]
D_r = [1];              % Regulate the output [omega] (rad/s)

% MPC cost and tuning parameters [This time with integral action]
Q = diag([1e-8 1e-10 1000]);
R = 1e3;
N = [10000; 10; 100];


A = balancingRobotLinearised.A_vel();  % Using the reduced linearized model to design a controller for the Nonlinear system
B = balancingRobotLinearised.B_vel();  % Using the reduced linearized model to design a controller for the Nonlinear system

controllerBalancingRobot.T = T;


% Control horizon
N_horizon = 10;

% Actuator limit constraints
u_max = 20;
u_min = -20;

% Actuator slew constraints (Input is omega - velocity)
delta_u_max = 4;
delta_u_min = -4;

% Initialise controller
controllerBalancingRobot.controlInit_reference_integrator(A,B,Q,R,N,T,N_horizon,u_max,u_min,delta_u_max,delta_u_min,C_r,D_r,y_ref)







%% MPC Controller with Reference Feed-Forward and Integral Action
% Define initial conditions
sim.phi0 = 0;
sim.theta0 = 3*(pi/180);
sim.ptheta0 = 0;

% Angle of incline disturbance
alpha = 0;


% Initial Conditions
sim.x0 = [sim.phi0; sim.theta0; sim.ptheta0];

% Define vector of time points where the controller is updated 
% [FOR 10 SECONDS]
sim.time = [0:controllerBalancingRobot.T:10];


% Set ODE solver options
options = odeset('RelTol',1e-6);


% Run the ODE solver on each defined time interval
res.t = sim.time(1);
res.x = sim.x0.';
res.omega = 0;



% Iterative loop to solve for each period (T) and update the control signal
% after each period (In this case, updates 200 times every second)

for i=1:length(sim.time)-1
x0 = res.x(end,:).';
t = [sim.time(i) sim.time(i)+controllerBalancingRobot.T];

fprintf('theta, ptheta values: %f, %f\n', x0(2), x0(3));        % Debugging

% Update the input 'Omega' using the new control input (Implemented controller)
omega = controllerBalancingRobot.control([x0(2); x0(3)]);



% Define ODE to be solved
sim.ode = @(t,x) balancingRobot.state_derivative(x,omega,alpha);



% Run ODE solver over ith time interval
[t, x] = ode45(sim.ode,t,x0,options);


% Concatinate solutions
res.t = [res.t; t];            % Grab the last value(s) which correspond to the end of that interval
res.x = [res.x; x];          % Grab the last value(s) which correspond to the end of that interval
res.omega = [res.omega; omega];
end





%% Plot the results - MPC Controller with Reference Feed-Forward and Integral Action
figure(1)

subplot(4,1,1)
plot(res.t, res.x(:,1),'LineWidth',2);        % Plot 'phi'
ylabel("{\phi} (radians)",'FontSize',14)
legend('phi','FontSize',14);
grid on
title("MPC Controller - Simulation",'FontSize',18)


subplot(4,1,2)
plot(res.t, res.x(:,2),'LineWidth',2);        % Plot 'theta'
ylabel("{\theta} (radians)",'FontSize',14)
legend('theta','FontSize',14);
grid on

subplot(4,1,3)
plot(res.t, res.x(:,3),'LineWidth',2);        % Plot 'ptheta'
ylabel("P{_\theta}",'FontSize',14)            % Units of momentum in the 'theta' coordinate (m*(rad/s))
legend('ptheta','FontSize',14);
grid on



subplot(4,1,4)
plot(sim.time, res.omega,'LineWidth',2)          % Plot Omega
legend('Input velocity','FontSize',14)
ylabel("{\omega} (rad/s)",'FontSize',14)
xlabel('time (seconds)','FontSize',16)
grid on

























