function xk1 = BicycleDynamics_DT_euler(xk, u, params)
%% Discrete-time nonlinear dynamic model of the bicycle dynamics model
% Integrate the continuous-time plant model from timestep x_{k} to x_{k+1}
% effectively creating a 'discrete-time' non-linear plant model.
% Variable xk1 is the states at time k+1.
%


%% Unpack params
% !! Important: These must exactly match the order they were passed as input
% parameters. It'd be very easy to get this wrong and have bugs.

% Unpack parameters
Lf = params(1);     % Length of forward wheel to center of gravity (m)
Lr = params(2);     % Length of back wheel to center of gravity (m)
g = params(3);      % Acceleration due to Gravity (m/s/s)
m = params(4);      % Mass of the vehicle (kg)
Calphaf = params(5); % Cornering stiffness coefficient for front tyre
Calphar = params(6); % Cornering stiffness coefficient for rear tyre
Iz = params(7);      % Moment of Inertia about the Z-axis
Vx = params(8);     % Constant velocity in the direction of the wheels

Ts = params(9);      % Sample time


% Unpack inputs (Steering angle)
sigma_f = u(1);


%% Forward Euler method
% Forward Euler is used because it's very quick. If system was stiff or 
% higher accuracy needed, any other ODE solver can be used (ode45, ode23, etc)
M = 10;
delta = Ts/M;
xk1 = xk;
for ct=1:M
    xk1 = xk1 + delta*BicycleDynamics(xk1, u, params);
end