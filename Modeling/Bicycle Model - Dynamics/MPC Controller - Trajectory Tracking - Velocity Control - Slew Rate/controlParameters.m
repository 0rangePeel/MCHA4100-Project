function param = controlParameters()

% Horizon
param.nControlHorizon       = 15;            % Length of control horizon
param.nPredictionHorizon    = 15;            % Length of prediction horizon
param.nSubsteps             = 1;            % Number of RK substeps per time step

% Actuator [sigma_f and accel_x] bounds
uLowerBound           = [ deg2rad(-40); -2];     % Lower actuator bound(s) at each time step
uUpperBound           = [ deg2rad(40); 2];     % Upper actuator bound(s) at each time step

% Package Lower and Upper bounds
param.lb = repmat(uLowerBound, param.nControlHorizon,1);
param.ub = repmat(uUpperBound, param.nControlHorizon,1);

% Slew Rate Parameters
param.delta_u_max = [deg2rad(10) 2];
param.delta_u_min = [-deg2rad(10) -2];

m = 2; % Number of Inputs
W = eye(param.nControlHorizon*m) - diag(ones(param.nControlHorizon*m-2,1), -2);
param.A = [W; -W]; % A Inequality Optimisation Parameter

bMax = [param.delta_u_max(1);param.delta_u_max(2)];
bMin = -[param.delta_u_min(1);param.delta_u_min(2)];

for i=2:param.nControlHorizon
    bMax = [bMax;[param.delta_u_max(1);param.delta_u_max(2)]];
    bMin = [bMin;-[param.delta_u_min(1);param.delta_u_min(2)]];
end

param.b = [bMax;bMin]; % b Inequality Optimisation Parameter


% Cost penalties
        param.qr                    = 300;            % Position error penalty
        param.qpsi                  = 5;            % Heading error penalty
        param.ru                    = 0.01;            % Actuator penalty (Steering and Velocity)
