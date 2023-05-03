% NLMPC using Bicycle Dynamics instead

% Ensure no unit tests fail before continuing
% results = runtests({ ...
%     'tests/test_dampingMatrix.m', ...
%     'tests/test_rigidBodyCoriolisMatrix.m', ...
%     'tests/test_rigidBodyMassMatrix.m', ...
%     'tests/test_robotConstraints.m', ...
%     'tests/test_robotMomentum.m', ...
%     'tests/test_trajectory.m', ...
%     });
% assert(~any([results.Failed]));



%%
% Robot parameters
param = bicycleParameters();


% Controller parameters
param.ctrl = controlParameters();



% Controller type
% param.controller = 'none';
param.controller = 'MPC';



% Use gradients to speed up optimisation
% Cannot use at the moment
param.useGradientOpt = false;
if param.useGradientOpt
    switch param.scenario
        case 'track'
            results = runtests({ ...
                'tests/test_robotDynamicsJacobian.m', ...
                'tests/test_errorMPCJacobian.m', ...
                });
        case 'dock'
            results = runtests({ ...
                'tests/test_robotDynamicsJacobian.m', ...
                'tests/test_costMPCGradient.m', ...
                'tests/test_nonlconMPCJacobian.m', ...
                });
        otherwise
            error('Unknown scenario');
    end
    assert(~any([results.Failed]));
end



% Load habitat map
% load('map.mat', 'map', 'Egridlines', 'Ngridlines');
% param.map = map;
% param.Egridlines = Egridlines;
% param.Ngridlines = Ngridlines;

% Set initial conditions
x0 = bicycleInitialConditions(param);

% Simulation parameters
param.dt    = 0.1;      	% Evaluation time interval (simulation may internally use smaller steps) [s]
param.T     = 70;        	% Total simulation time [s]


% Generate trajectory
param.traj = trajectoryDesign(param,x0,'Track_downscaled.png');

u = [0; 0]; % Initial stored control action [sigma_f; accel_x]
options     = odeset('MaxStep', 0.005, 'Events', @(t,x) eventCollision(t, x, u, param, 'Track_downscaled.png'));
tHist       = 0:param.dt:param.T;       % Specify times the output is returned
uHist       = nan(2, length(tHist));        % Sigma_f input



   

%% Run the simulation using NLMPC

switch param.controller
    case 'none'
        param.inputDisturbance = false;
        u = [0; 0];
        func = @(t, x) bicycleDynamics(t, x, u, param);
        [tHist, xHist] = ode45(func, tHist, x0, options);
        tHist = tHist.';
        xHist = xHist.';
        for k = 1:length(tHist)
            uHist(:, k) = u;
        end
        uHist(:, length(tHist)+1:end) = [];
    
    case 'MPC'
        %         param.inputDisturbance = true;          % Enable small input disturbance
        U = [];
        u = [0; 0];                % Initial stored control action [sigma_f; accel_x]
        uHist(:, 1) = u;
        xHist = zeros(6,  length(tHist));
        teHist = zeros(1,  length(tHist));
        xHist(:, 1) = x0;
        delete(findall(0, 'tag', 'TMWWaitbar'));  % Remove any stuck waitbars
        wh = waitbar(0, getStatusMsg, ...
        'Name', 'MCHA4100 motivational waitbar', ...
        'CreateCancelBtn', 'setappdata(gcbf, ''cancelling'', 1)');
        
        
        for k = 1:length(tHist)-1
            if getappdata(wh, 'cancelling')  % Check if waitbar cancel button clicked
                delete(wh);
                error('User cancelled operation.')
            end
            
            try % you can't succeed if you don't
                % Pretend to apply stored control action u
                % PRETENDING
            
                % Simulate one time step with ZOH input u - Used for collision
                % detection
                    func = @(t, x) bicycleDynamics(t, x, u, param);
                    [~, xTemp, te, xe, ie] = ode45(func, [tHist(k) tHist(k+1)], xHist(:, k), options);
                    xTemp = xTemp.';
                    %teHist(:,k) = te;
                    xHist(:, k+1) = xTemp(:, end);
                    if ~isempty(te)
                        warning('Collision detected');
                        uHist(:, k+2:end) = [];
                        uHist(:,end) = uHist(:,end-1);
                        xHist(:, k+2:end) = [];
                        tHist(:, k+2:end) = [];
                        break
                    end


            
                % Pretend to take measurements from sensors
                % Pretend to estimate next state
                % PRETENDING INTENSIFIES

                
                xctrl = xHist(:, k+1);
                
                % Compute next control action
                U = controlMPC(tHist(k+1), xctrl, u, U, param);
            
                % Store next control action to apply
                u = U(1:2);       % Single control action [sigma_f]
            
                uHist(:, k+1) = u; % Save for plotting
            catch hot_potato
                delete(wh);                 % Remove waitbar if error
                rethrow(hot_potato);        % Someone else's problem now
            end
            waitbar(k/length(tHist), wh);    % Update waitbar
        end
        delete(wh);                         % Remove waitbar if we complete successfully
end







%% Plot history
% [Xg, Yg, theta, Vy, r]
NHist   = xHist(2, :);  
EHist   = xHist(1, :);
psiHist = xHist(3, :);
VxHist = xHist(4,:);
VyHist = xHist(5,:);
rHist = xHist(6,:);
gHist   = nan(1, length(tHist));
HHist   = nan(1, length(tHist));
nu3Hist = nan(3, length(tHist));


% for k = 1:length(tHist)
%     nu3Hist(:, k) = robotVelocity(xHist(:, k), param);
%     gHist(k) = Bc.'*nu3Hist(:, k);
%     HHist(k) = robotEnergy(xHist(:, k), param);
% end

Xtraj = nan(2, 2, length(tHist));
for k = 1:length(tHist)
    Xtraj(:, :, k) = trajectoryEval(param.traj, tHist(k));
end
Ntraj = squeeze(Xtraj(1, 1, :)).';
Etraj = squeeze(Xtraj(2, 1, :)).';



% Xtrajectory_ref = [Ntraj; Etraj];
% % 
% % %% Check RMS Error
% rCNn = [xHist(3,:) + param.c*cos(xHist(5,:));    xHist(4,:) + param.c*sin(xHist(5,:))];
% seHist = sum((rCNn - Xtrajectory_ref).^2,1); % (N - Nref)^2 + (E - Eref)^2
% rmsError = realsqrt(trapz(tHist,seHist)/param.T);
% % % 
% fprintf('\tRMS position error: %.3g metres\n', rmsError);
% finalError = realsqrt(seHist(end));


%% Plots
figure(1); clf

subplot(3, 1, 1)
plot(tHist, NHist)
grid on
title('North position')
ylabel('N [m]')
subplot(3, 1, 2)
plot(tHist, EHist)
grid on
title('East position')
ylabel('E [m]')
subplot(3, 1, 3)
plot(tHist, uHist(1, :)*180/pi)
grid on
title('Steering Angle')
ylabel('\theta [\circ]')
xlabel('Time [s]')

figure(2)
title("Longitudinal Velocity, Lateral velocity and yaw rate")
subplot(3, 1, 1)
plot(tHist, VxHist)
grid on
title('Longitudinal Velocity')
ylabel('Longitudinal Velocity')

subplot(3, 1, 2)
plot(tHist, VyHist)
grid on
title('Lateral Velocity')
ylabel('Lateral Velocity')

subplot(3, 1, 3)
plot(tHist, rHist)
grid on
title('Yaw Rate')
ylabel('Yaw Rate')
xlabel('Time [s]')

figure(3);

subplot(2, 1, 1)
plot(tHist, uHist(1, :)*180/pi)
grid on
title('Steering Angle')
ylabel('\theta [\circ]')
subplot(2, 1, 2)
plot(tHist, uHist(2, :))
grid on
title('Vehicle Acceleration')
ylabel('m/s^2')
xlabel('Time [s]')



N = xHist(2,:);
E = xHist(1,:);
theta = xHist(3,:);
t = tHist;

Lf = param.Lf;
Lr = param.Lr;

%% Visualize the vehicle moving on a 2D plane
%% Animation shows center of gravity, back wheel and front wheel positions
outputVideo = false;

if outputVideo
    vid = VideoWriter(['bicycleDynamics' param.controller '.mp4'], 'MPEG-4');
    vid.FrameRate = 1/param.dt;
    vid.Quality = 100;
    open(vid);
end



fig     = 4;
hf      = figure(fig); clf(fig);
hf.Color = 'w';
ax      = axes(hf, 'FontSize', 14);
hold(ax, 'on');

resolution = 5.5;
threshold = 154;
image = imread('Track_downscaled.png');
bwimage = image < threshold;
map = binaryOccupancyMap(bwimage,resolution);
show(map)

hFt     = plot(ax, nan(size(E)), nan(size(N)), 'r');
hRt     = plot(ax, nan(size(E)), nan(size(N)), 'g');
hCt     = plot(ax, nan(size(E)), nan(size(N)), 'b');
hP      = plot(ax, nan, nan, 'k-');
hFLw    = plot(ax, nan, nan, 'k-');
hFRw    = plot(ax, nan, nan, 'k-');
hF      = plot(ax, 0, 0, 'r.');
hR      = plot(ax, 0, 0, 'g.');
hC      = plot(ax, 0, 0, 'b.');

htraj   = plot(ax, Etraj, Ntraj, 'k:');
hP2      = plot(ax, nan, nan, 'ro');

tF      = text(ax, 0, 0, ' F', 'FontSize', 10, 'Color', 'r');
tR      = text(ax, 0, 0, ' R', 'FontSize', 10, 'Color', 'g');
tC      = text(ax, 0, 0, ' C', 'FontSize', 10, 'Color', 'b');

hold(ax, 'off');
axis(ax, 'equal');
axis(ax, [min(E) - 2, max(E) + 2, min(N) - 2, max(N) + 2]);
% grid(ax, 'on');
xlabel(ax, 'East [m]');
ylabel(ax, 'North [m]');


Se3  = skew([0; 0; 1]);

rCNn = nan(3, length(t));
rFNn = nan(3, length(t));
rRNn = nan(3, length(t));
rBNn = nan(3, length(t));
rFLNn = nan(3, length(t));
rFRNn = nan(3, length(t));

rFBb = [Lf; 0; 0];
rRBb = [-Lr; 0; 0];

% Plot the body of the vehicle
r = 0.5;      % Equvalent to the Lf and Lr variables, % North, East, Down
d = 1.5;
% Draw car shape
curve = linspace(3*pi/2, pi/2, 50);

% Plotting and drawing body of the vehicle
rPCb = [ [-d; r; 0], [d; r; 0], [d; -r; 0], [-d; -r; 0], [-d; r; 0], ... % Car Body
    [-d; r+0.25; 0], [-d+1; r+0.25; 0], [-d+1; r; 0],[-d; r; 0] ...  % Car Back Left Wheel
    [-d; -r; 0], [-d; -r-0.25; 0], [-d+1; -r-0.25; 0], [-d+1; -r; 0], ...          % Car Back Right Wheel
    [-d; -r; 0], [-d; r; 0], [d; r; 0], -r*[cos(curve)-2.5; sin(curve); zeros(size(curve))]];     % Mad Aerodynamics

% Plotting Steering Wheels
rPFLwb = [[0.5; 0; 0], [0.5; (0+0.25); 0] [-0.5; (0+0.25); 0] [-0.5; 0; 0],[0.5; 0; 0]]; % Car Front Left Steering Wheel
rPFRwb = [[0.5; 0; 0], [0.5; -(0+0.25); 0] [-0.5; -(0+0.25); 0] [-0.5; 0; 0],[0.5; 0; 0]]; % Car Front Right Steering Wheel
                        

for i = 1:length(tHist)
    % Rotation of Car Body
    Rnb = [sin(theta(i)) cos(theta(i)) 0; cos(theta(i)) -sin(theta(i)) 0; 0 0 1];
    % Rotation of Steering Wheels attached to Car Body
    RnbF = [sin(theta(i) + uHist(1, i)) cos(theta(i)+ uHist(1, i)) 0; cos(theta(i)+ uHist(1, i)) -sin(theta(i)+ uHist(1, i)) 0; 0 0 1];

    rCNn(:, i) = [N(i); E(i); 0];
    rBNn(:, i) = [N(i); E(i); 0];

    % Position of Front and Rear Wheel Base
    rFNn(:, i) = rBNn(:, i) + [Lf*sin(theta(i)); Lf*cos(theta(i)); 0];
    rRNn(:, i) = rBNn(:, i) + [-Lr*sin(theta(i)); -Lr*cos(theta(i)); 0];

    % Position of Front Left and Right Wheel respectively
    rFLNn(:, i) = rFNn(:, i) + [r*cos(theta(i)); -r*sin(theta(i)); 0];
    rFRNn(:, i) = rFNn(:, i) + [-r*cos(theta(i)); r*sin(theta(i)); 0];

    % Draw Car Body
    rPNn = rCNn(:, i) + Rnb*rPCb;
     hP.XData = rPNn(2, :);
     hP.YData = rPNn(1, :);

    % Draw Front Left Steering Wheel
    rFLwNn = rFLNn(:, i) + RnbF*rPFLwb;
     hFLw.XData = rFLwNn(2, :);
     hFLw.YData = rFLwNn(1, :);

    % Draw Front Right Steering Wheel
    rFRwNn = rFRNn(:, i) + RnbF*rPFRwb;
     hFRw.XData = rFRwNn(2, :);
     hFRw.YData = rFRwNn(1, :);
    
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

    hP2.XData = Etraj(i);
    hP2.YData = Ntraj(i);

    pause(0.05);
    drawnow
    
end

