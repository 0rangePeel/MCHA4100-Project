% FYI, Linearization is disgusting on super non-linear models 
% Thus non-linear MPC is implemented instead using fmincon optimization to
% solve for the control problem

% Solving for the NLMPC for the Dynamic Bicycle Model

% Jamie Tsang
% Changed on: 20/04/2023

% Housekeeping
clear all
clc; 
close all

%% Define bicycle parameters
params_struct.Lf = 1.4;
params_struct.Lr = 1.6;
params_struct.g = 9.81;
params_struct.m = 2000;
params_struct.Calphaf = 12e3;
params_struct.Calphar = 11e3;
params_struct.Iz = 4000;

Lf = params_struct.Lf;
Lr = params_struct.Lr;


params_struct.Vx = 5;           % Constant velocity in the direction of the wheels


params(1) = params_struct.Lr;
params(2) = params_struct.Lf;
params(3) = params_struct.g;
params(4) = params_struct.m;
params(5) = params_struct.Calphaf;
params(6) = params_struct.Calphar;
params(7) = params_struct.Iz;
params(8) = params_struct.Vx;



%% Define initial input
Vx = params_struct.Vx;
sigma_f = deg2rad(0);        % In radians, steering angle



%% Create Nonlinear MPC Controller
nx = 5; % Five states x = [xg, yg, theta, vy, r]

ny = 2; % Outputs we want to control are y = [Xg, Yg]

nu = 1; % One input u = [delta_f]   - [Steering angle], velocity is kept constant in direction of wheels
nlobj = nlmpc(nx, ny, nu); % Initialise object for nlmpc controller



%% Set sampling rate
Ts                             = 0.1;
nlobj.Ts                       = Ts;


% Create NLMPC options structure and declare expected input parameters
nloptions                      = nlmpcmoveopt;

% These parameters are specified as one large vector for the nlmpc toolbox.
% Ensure the order of the parameters in the StateFcn are 
% unpacked matches the order that they occur in these vectors.
nloptions.Parameters           = {[params, Ts]};


%% Set prediction and control horizon
nlobj.PredictionHorizon        = 15;
nlobj.ControlHorizon           = 15;


%% Specify Nonlinear Plant Model
nlobj.Model.StateFcn           = "BicycleDynamics_DT_euler";
nlobj.Model.IsContinuousTime   = false;
nlobj.Model.NumberOfParameters = 1;


%% Set input limits and slew rates - [Sigma_f]

 % Sigma_f (Steering Angle)
 nlobj.ManipulatedVariables(1).Max = deg2rad(45);
 nlobj.ManipulatedVariables(1).Min = deg2rad(-45);

 nlobj.ManipulatedVariables(1).RateMax = deg2rad(5);
 nlobj.ManipulatedVariables(1).RateMin = deg2rad(-5);


%% Define weights for control cost function
% Assuming all states are measured by default
nlobj.Weights.ManipulatedVariables = [1; 1; 1e3; 1; 1e3]; % Tuning Parameter Weights
nlobj.Weights.ManipulatedVariablesRate = [0.1; 0.1; 1e2; 0.1; 1e2];              % Tuning Parameter Weights
nlobj.Weights.OutputVariables   = [1e1, 1e1];      % Weights for cost function

% Here we inform the controller that we would like to control measured
% outputs, rather than states
nlobj.Model.OutputFcn           = @(x,u,Ts) BicycleDynamicsOutputFunction(x,u,Ts);


%% Set output constraints
% None
%  nlobj.OutputVariables(1).Max = 20;
%  nlobj.OutputVariables(1).Min = -20;




%% Set initial conditions and reference setpoint
x0 = [0; 0; deg2rad(180); 0; deg2rad(40)];
u0  = [sigma_f]; % Start from the equilibrium input [sigma_f]

x  = x0;  % Start the sim at initial conditions
u  = u0;


 yref = [50, 100]; % Reference for the controller [Xg, Yg]


 %% Reference Trajectory...



%% Run the simulation
tsim = 30;
hbar = waitbar(0,'Simulation Progress');
xHist = zeros(nx,tsim/Ts + 1);      % Matrix to store the values of the states
uHist = zeros(nu,tsim/Ts + 1);      % Matrix to store the values of the input
uHist(:,1) = u0; % Adds input to history for t=0
tHist = zeros(1,tsim/Ts+1);


%% Loop to execute the simulation
for t = 1:(tsim/Ts)
    % Update current state of the plant
    xk            = x;
    
%     % Update trajectory at 10 sec
%     % Introduce as reference trajectory over the horizon
%     % for better pre-emptive response by nlmpc controller
%     if t == 10/Ts
%         yref = [deg2rad(10), 0];        % [gamma; alpha]
%         % TODO: Fix dimensions when doing step e
%     end


    if (x(1) >= 50) && (x(2) >= 50)
        params_struct.Vx = 0.01;
        params(8) = params_struct.Vx;
    end


    % To update trajectory at a later time, consider using another if.
    % To update at a later time, lets say at 60 seconds, do something like:
    % if t == 60/Ts
    % We can then use this to change the reference at which the states will
    % go to


    
    % Introduce velocity profile as a measured disturbance
    % trajectory
    
    % Compute optimal control moves
    % Info: retrieving, and re-passing the 'nloptions' object allows the
    % nlmpcmove() function to 'warm-start' using previously optimised
    % trajectories as an initial guess --> much faster solve time!
    
    tic;                 % Start MPC solve-time measurement
    [u, nloptions, info] = nlmpcmove(nlobj, xk, u, yref, [], nloptions);    % Computes the optimal control action
    T_MPC         = toc; %Save the time taken for each MPC calculation ('Tic' and 'Toc' measure the elapsed time between commands


    % Apply first control value and simulate forward to k+1 using ode45
    [~,x]         = ode45(@(t,x)BicycleDynamics(x, u, params), [0, Ts], xk);
    x             = x(end,:);
    
    % Save a hist of states and input to plot
    xHist(:,t+1)  = x;
    uHist(:,t+1)  = u;
    tHist(t)      = T_MPC;
    waitbar(t*Ts/tsim,hbar);
end
close(hbar);



%% Plot simulation output.
tplot = 0:Ts:tsim;
CreateFig;
subplot(3,1,1)


% Order of states is swapped compared to previous aircraft lab
% (state, timestep)
plot(tplot,xHist(1,:))                        % Plotting Xg
title('Closed Loop Bicycle Dynamics Response');
ylabel('Xg (m)')
grid; 

subplot(3,1,2)
plot(tplot,xHist(2,:))                        % Plotting Yg
ylabel('Yg (m)')

subplot(3,1,3)
plot(tplot,rad2deg(xHist(3,:)))             % Plotting Theta
ylabel('Angle (deg)')



CreateFig;
subplot(2,1,1)
plot(tplot,ones(length(tplot),1)*Vx)                        % Plotting Velocity
title('Closed Loop Bicycle Dynamics Inputs');
ylabel('Velocity (m/s)')

subplot(2,1,2)
plot(tplot,rad2deg(uHist(1,:)))             % Plotting Sigma_f
ylabel('Angle of steering (deg)')



CreateFig;
plot(tplot, tHist);
title('Computation time for NLMPC controller');
ylabel('Solve time (sec)')
xlabel('Time (sec)')






N = xHist(2,:);
E = xHist(1,:);
theta = xHist(3,:);


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

rFBb = [Lf; 0; 0];
rRBb = [-Lr; 0; 0];

% Plot the body of the vehicle
r = 0.5;      % Equvalent to the Lf and Lr variables, % North, East, Down
d = 1.5;
% Draw car shape
curve = linspace(3*pi/2, pi/2, 50);

% Plotting and drawing body of the vehicle
rPCb = [ [-d; r; 0], [d; r; 0], [d; -r; 0], [-d; -r; 0], [-d; r; 0], ... % Car Body
    [-d; r+0.25; 0], [-d+1; r+0.25; 0], [-d+1; r; 0], ...  % Car Wheel 1
    [d; r; 0], [d; r+0.25; 0], [d-1; r+0.25; 0], [d-1; r; 0], [d; r; 0], ...  % Car Wheel 2
    [d; -r; 0], [d; -r-0.25; 0], [d-1; -r-0.25; 0], [d-1; -r; 0], [d; -r; 0], ... % Car Wheel 3
    [-d; -r; 0], [-d; -r-0.25; 0], [-d+1; -r-0.25; 0], [-d+1; -r; 0], ...          % Car Wheel 4
    [-d; -r; 0], [-d; r; 0], [d; r; 0], -r*[cos(curve)-2.5; sin(curve); zeros(size(curve))]];     % Mad Aerodynamics

                        

for i = 1:length(tplot)
    Rnb = [sin(theta(i)) cos(theta(i)) 0; cos(theta(i)) -sin(theta(i)) 0; 0 0 1];
    rCNn(:, i) = [N(i); E(i); 0];
    rBNn(:, i) = [N(i); E(i); 0];

    rFNn(:, i) = rBNn(:, i) + [Lf*sin(theta(i)); Lf*cos(theta(i)); 0];
    rRNn(:, i) = rBNn(:, i) + [-Lr*sin(theta(i)); -Lr*cos(theta(i)); 0];


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


    pause(0.01);
    drawnow
end

