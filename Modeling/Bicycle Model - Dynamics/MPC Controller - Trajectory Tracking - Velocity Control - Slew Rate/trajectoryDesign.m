function traj = trajectoryDesign(param,x0,imageLine)

% Generate waypoint intercept times and position/velocity/acceleration data
[t, X] = generateWaypoints(param,x0,imageLine);

% Generate trajectory structure from waypoint data
traj = generateTrajectoryFromWaypoints(t, X);
