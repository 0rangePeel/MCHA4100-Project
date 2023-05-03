function [t, X] = generateWaypoints(param,x0,imageLine)

    % nWaypoints = 4;
    % t = zeros(1, nWaypoints);
    % X = zeros(2, 2, nWaypoints);
    % 
    % 
    % %% New Waypoints for travelling, bicycle model
    % % Waypoint 0
    % t(1) = 0;
    % X(:, :, 1) = [ ...      
    %      6, 0;
    %      60, 0];
    % 
    % 
    % % Waypoint 1 - Straight
    % t(2) = 4;
    % X(:, :, 2) = [ ...
    %     6, 0;
    %     80, 5];
    % 
    % 
    % % Waypoint 2 - Corner
    % t(3) = 15;
    % X(:, :, 3) = [ ...
    %     6, 0;
    %     130, 5];
    % 
    % % Waypoint 2 - Corner
    % t(4) = 15;
    % X(:, :, 4) = [ ...
    %     15, 5;
    %     140.5, 0];
    
    
    image = imread(imageLine);
    bwimage = image < param.threshold;
    map = binaryOccupancyMap(bwimage,param.resolution);

    inflate(map,1.4)
    setOccupancy(map,[58 6.8],1)
    setOccupancy(map,[58 6.6],1)
    setOccupancy(map,[58 6.4],1)
    setOccupancy(map,[58 6.2],1)
    setOccupancy(map,[58 6.1],1)
    setOccupancy(map,[58 6],1)
    setOccupancy(map,[58 5.8],1)
    setOccupancy(map,[58 5.6],1)
    setOccupancy(map,[58 5.4],1)
    
    startPose = x0(1:3)';
    %goalPose = [124 40 pi/2]; works with T = 20
    %goalPose = [100 96.5 pi]; works with T = 40;
    goalPose = [57 5.8 0];

    % figure(1);
    % show(map)
    % hold on
    
    traj = codegenPathPlanner(map, startPose, goalPose);
    

    % scatter(traj(:,1),traj(:,2),"cyan","filled")
    
    nWaypoints = length(traj);
    
    t = [linspace(0,5,20) linspace(6,param.T,nWaypoints-20)];
    
    X = zeros(2,2,nWaypoints);
    
    for i=1:nWaypoints
        if i < 20
            X(:,:,i) = [traj(i,2) i*0.05*sin(traj(i,3));
                        traj(i,1) i*0.05*cos(traj(i,3))];
        else
            X(:,:,i) = [traj(i,2) 1*sin(traj(i,3));
                        traj(i,1) 1*cos(traj(i,3))];
        end
    
    end

end
