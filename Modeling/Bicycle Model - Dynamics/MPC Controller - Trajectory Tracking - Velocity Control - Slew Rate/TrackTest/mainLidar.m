clc
clear

resolution = 20;
threshold = 154;

imageLineBlue = 'Track.png';
coneNumBlue = 145;
distanceBlue = 3;
coneStartBlue = [60 4];
flagBlue = 1; % Cones follow outside line

imageLineYellow = 'Track.png';
coneNumYellow = 136;
distanceYellow = 3;
coneStartYellow = [60 8];
flagYellow = 0; % Cones follow inside line

[conePosBlue,coneAngleBlue] = coneGenerator(imageLineBlue,resolution,threshold,coneNumBlue,distanceBlue,coneStartBlue,flagBlue);
[conePosYellow,coneAngleYellow] = coneGenerator(imageLineYellow,resolution,threshold,coneNumYellow,distanceYellow,coneStartYellow,flagYellow);

%% Plot Track
image = imread('Track.png');
grayimage = rgb2gray(image);
bwimage = grayimage > threshold;

grid = binaryOccupancyMap(bwimage,resolution);

grid2 = ~grid.getOccupancy;

grid.setOccupancy(grid2) %grid is map
figure(1);
show(grid)

hold on

%% Plot Cones
for i=1:coneNumBlue
    plot(conePosBlue(i,1),conePosBlue(i,2),'*b','linewidth', 4)
    xlim([0 192])
    ylim([0 108])
    hold on
end

for i=1:coneNumYellow
    plot(conePosYellow(i,1),conePosYellow(i,2),'*g','linewidth', 4)
    xlim([0 192])
    ylim([0 108])
    hold on
end
hold on
plot(conePosBlue(1,1),conePosBlue(1,2),'*r','linewidth', 4)
hold on
plot(conePosYellow(1,1),conePosYellow(1,2),'*r','linewidth', 4)
xlim([0 192])
ylim([0 108])
hold on

%% Reconstruct Cones as binary occupancy map

map = binaryOccupancyMap(grid.XLocalLimits(2),grid.YLocalLimits(2),grid.Resolution);

for i=1:coneNumBlue
    setOccupancy(map,conePosBlue(i,:),1)
end
for i=1:coneNumYellow
    setOccupancy(map,conePosYellow(i,:),1)
end
inflate(map,0.125)

figure(2);
show(map)


%% Lidar
% Cones
% lidar = occupancyMap(grid.XLocalLimits(2),grid.YLocalLimits(2),grid.Resolution);
% obstacles = [conePosBlue;conePosYellow];
% setOccupancy(lidar,obstacles,ones(length(obstacles),1))
% inflate(lidar,0.125)

% Track
image = imread('Track.png');
grayimage = rgb2gray(image);
bwimage = grayimage < threshold;
lidar = occupancyMap(bwimage,resolution);


figure(3);
show(lidar)

maxrange = 20;
rayNum = 100;
angles = linspace(-pi/2,pi/2,rayNum);
poseAngle = deg2rad(-15);
vehiclePose = [50,60,poseAngle];
intsectionPts = rayIntersection(lidar,vehiclePose,angles,maxrange,0.7);

hold on
plot(intsectionPts(:,1),intsectionPts(:,2),'*r') % Intersection points
plot(vehiclePose(1),vehiclePose(2),'ob') % Vehicle pose
for i = 1:rayNum
    if isnan(intsectionPts(i,1))
        plot([vehiclePose(1),vehiclePose(1)-maxrange*sin(angles(i)+(poseAngle - pi/2))],...
        [vehiclePose(2),vehiclePose(2)+maxrange*cos(angles(i)+(poseAngle - pi/2))],'-b') % No intersection ray        
    else
        plot([vehiclePose(1),intsectionPts(i,1)],...
        [vehiclePose(2),intsectionPts(i,2)],'-b') % Plot intersecting rays
    end
end

%% TODO - Use pythagoras to get out distance from the car to the intersection points i.e. cone distance

distance = zeros(1,100);

for i=1:rayNum
    if isnan(intsectionPts(i,1))
        distance(i) = nan;
    else
        distance(i) = sqrt((vehiclePose(1) - intsectionPts(i,1))^2 + (vehiclePose(2) - intsectionPts(i,2))^2);
    end
end

figure(4);
plot(rad2deg(angles),fliplr(distance))
xlim([rad2deg(-pi/2) rad2deg(pi/2)])
ylim([0 20])
xlabel('Angle (degrees)')
ylabel('Distance (m)')