function [t, X] = generateWaypoints(param)

nWaypoints = 4;
t = zeros(1, nWaypoints);
X = zeros(2, 2, nWaypoints);


%% New Waypoints for travelling, bicycle model
% Waypoint 0
t(1) = 0;
X(:, :, 1) = [ ...      
     0, 0;
     0, 0];


% Waypoint 1 - Straight
t(2) = 5;
X(:, :, 2) = [ ...
    0, 0;
    25, 5];


% Waypoint 2 - Corner
t(3) = 10;
X(:, :, 3) = [ ...
    15, 5;
    35, 0];

% Waypoint 2 - Corner
t(4) = 15;
X(:, :, 4) = [ ...
    20, 0;
    15, -5];



% Edit/add waypoints as necessary by adjusting nWaypoints
% and filling out the intercept times and waypoint data
