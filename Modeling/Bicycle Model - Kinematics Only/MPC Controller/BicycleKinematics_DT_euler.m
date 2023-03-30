function xk1 = BicycleKinematics_DT_euler(xk, u, params)
%% Discrete-time nonlinear dynamic model of the bicycle kinematics model
% Integrate the continuous-time plant model from timestep x_{k} to x_{k+1}
% effectively creating a 'discrete-time' non-linear plant model.
% Variable xk1 is the states at time k+1.
%


%% Unpack params
% !! Important: These must exactly match the order they were passed as input
% parameters. It'd be very easy to get this wrong and have bugs.

% Unpack parameters
Ts = params(1);
lr = params(2);
lf = params(3);


Vin = u(1);
sigma_f = u(2);


%% Forward Euler method
% Forward Euler is used because it's very quick. If system was stiff or 
% higher accuracy needed, any other ODE solver can be used (ode45, ode23, etc)
M = 5;
delta = Ts/M;
xk1 = xk;
for ct=1:M
    xk1 = xk1 + delta*BicycleKinematics(xk1, u, params);
end