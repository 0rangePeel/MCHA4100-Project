function [t, X] = generateWaypoints(param)

nWaypoints = 4;
t = zeros(1, nWaypoints);
X = zeros(2, 2, nWaypoints);


%% New Waypoints for travelling, bicycle model
% Waypoint 0
t(1) = 0;
X(:, :, 1) = [ ...      
     6, 0;
     60, 0];


% Waypoint 1 - Straight
t(2) = 4;
X(:, :, 2) = [ ...
    6, 0;
    80, 5];


% Waypoint 2 - Corner
t(3) = 15;
X(:, :, 3) = [ ...
    6, 0;
    130, 5];

% Waypoint 2 - Corner
t(4) = 15;
X(:, :, 4) = [ ...
    15, 5;
    140.5, 0];



% Edit/add waypoints as necessary by adjusting nWaypoints
% and filling out the intercept times and waypoint data
