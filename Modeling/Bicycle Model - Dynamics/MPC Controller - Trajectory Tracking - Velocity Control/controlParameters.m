function param = controlParameters()

% Horizon
param.nControlHorizon       = 15;            % Length of control horizon
param.nPredictionHorizon    = 15;            % Length of prediction horizon
param.nSubsteps             = 1;            % Number of RK substeps per time step

% Actuator [sigma_f and accel_x] bounds
param.uLowerBound           = [ deg2rad(-40); -2];     % Lower actuator bound(s) at each time step
param.uUpperBound           = [ deg2rad(40); 2];     % Upper actuator bound(s) at each time step

% Cost penalties
        param.qr                    = 300;            % Position error penalty
        param.qpsi                  = 5;            % Heading error penalty
        param.ru                    = 0.01;            % Actuator penalty (Steering and Velocity)
