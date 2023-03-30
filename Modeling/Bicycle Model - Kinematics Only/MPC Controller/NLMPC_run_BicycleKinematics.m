% FYI, Linearization is disgusting on super non-linear models 
% Thus non-linear MPC is implemented instead using fmincon optimization to
% solve for the control problem

% Solving for the NLMPC for the Kinematics Bicycle Model

% Created by: Jamie Tsang
% Creation date: 26/03/2023

% Housekeeping
clear all
clc; 
close all

%% Define bicycle parameters
params_struct.lf = 1;
params_struct.lr = 1;

params(1) = params_struct.lr;
params(2) = params_struct.lf;


%% Define inputs
Vin = 5;                     % In m/s, velocity of the vehicle, in the same direction as the wheels
sigma_f = deg2rad(20);       % In radians, converted from degrees





%% Create Nonlinear MPC Controller
nx = 3; % Three states x = [North, East, Psi]

ny = 2; % Outputs we want to control are y = [North, East]

nu = 2; % Two inputs u = [V, delta_f]   - [Velocity and steering angle]
nlobj = nlmpc(nx, ny, nu); % Initialise object for nlmpc controller



%% Set sampling rate
Ts                             = 0.1;
nlobj.Ts                       = Ts;


% Create NLMPC options structure and declare expected input parameters
nloptions                      = nlmpcmoveopt;

% These parameters are specified as one large vector for the nlmpc toolbox.
% Ensure the order of the parameters in the StateFcn are 
% unpacked matches the order that they occur in these vectors.
nloptions.Parameters           = {[Ts, params]};


%% Set prediction and control horizon
nlobj.PredictionHorizon        = 10;
nlobj.ControlHorizon           = 10;


%% Specify Nonlinear Plant Model
nlobj.Model.StateFcn           = "BicycleKinematics_DT_euler";
nlobj.Model.IsContinuousTime   = false;
nlobj.Model.NumberOfParameters = 1;


%% Set input limits and slew rates - [Velocity, Sigma_f]

% Velocity
 nlobj.ManipulatedVariables(1).Max = 20;
 nlobj.ManipulatedVariables(1).Min = 0;       % Velocity so the vehicle does not go backwards

 nlobj.ManipulatedVariables(1).RateMax = 5;
 nlobj.ManipulatedVariables(1).RateMin = -5;

 % Sigma_f (Steering Angle)
 nlobj.ManipulatedVariables(2).Max = deg2rad(35);
 nlobj.ManipulatedVariables(2).Min = deg2rad(-35);

 nlobj.ManipulatedVariables(2).RateMax = deg2rad(15);
 nlobj.ManipulatedVariables(2).RateMin = deg2rad(-15);


%% Define weights for control cost
% Assuming all states are measured by default
nlobj.Weights.OutputVariables   = [1e1, 1e0];      % Output values for the states

% Here we inform the controller that we would like to control measured
% outputs, rather than states
nlobj.Model.OutputFcn           = @(x,u,Ts) BicycleKinematicsOutputFunction(x,u,Ts);
% nlobj.Weights.OutputVariables   = [1e1 1e0];        % Control measured outputs, rather than states


%% Set output constraints

%  nlobj.OutputVariables(1).Max = 20;
%  nlobj.OutputVariables(1).Min = -20;




%% Set initial conditions and reference setpoint
x0 = zeros(3,1);
u0  = [Vin; sigma_f]; % Start from the equilibrium input [Velocity; sigma_f]

x  = x0;  % Start the sim at initial conditions
u  = u0;

% yref = [0, 0, 0]; % Set reference for controller
 yref = [100, 100]; % [North, East]


 %% Reference Trajectory...



%% Run the simulation
tsim = 20;
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
    [~,x]         = ode45(@(t,x)BicycleKinematics(x, u, params), [0, Ts], xk);
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
plot(tplot,xHist(1,:))                        % Plotting North
title('Closed Loop Bicycle Kinematics Response');
ylabel('North (m)')
grid; 

subplot(3,1,2)
plot(tplot,xHist(2,:))                        % Plotting East
ylabel('East (m)')

subplot(3,1,3)
plot(tplot,rad2deg(xHist(3,:)))             % Plotting Psi
ylabel('Angle (deg)')



CreateFig;
subplot(2,1,1)
plot(tplot,uHist(1,:))                        % Plotting Velocity
title('Closed Loop Bicycle Kinematics Inputs');
ylabel('Velocity (m/s)')

subplot(2,1,2)
plot(tplot,rad2deg(uHist(2,:)))             % Plotting Sigma_f
ylabel('Angle of steering (deg)')



CreateFig;
plot(tplot, tHist);
title('Computation time for NLMPC controller');
ylabel('Solve time (sec)')
xlabel('Time (sec)')








%% Visualize the vehicle moving on a 2D plane

figure(4)
grid on
xlabel("East Position (m)")
ylabel("North Position (m)")
axis([min(xHist(2,:))-2    max(xHist(2,:))+2       min(xHist(1,:))-2    max(xHist(1,:))+2])

bicycle_model = rectangle('EdgeColor','k','FaceColor','b');

North_pos = xHist(1,:);
East_pos = xHist(2,:);
Psi_angle = xHist(3,:);

for k1 = 1:length(xHist(1,:))

  bicycle_model.Position = [East_pos(k1)  North_pos(k1), 2  2];
  drawnow
  pause(0.05)

end
