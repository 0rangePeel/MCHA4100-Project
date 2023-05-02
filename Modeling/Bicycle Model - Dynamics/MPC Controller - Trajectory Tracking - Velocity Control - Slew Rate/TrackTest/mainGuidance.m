clc
clear
tic

%resolution = 20;
resolution = 5.5;
threshold = 154;

imageLineBlue = 'Track_downscaled.png';
coneNumBlue = 141;
distanceBlue = 3;
coneStartBlue = [60 4];
flagBlue = 1; % Cones follow outside line

imageLineYellow = 'Track_downscaled.png';
coneNumYellow = 132;
distanceYellow = 3;
coneStartYellow = [60 8];
flagYellow = 0; % Cones follow inside line

[conePosBlue,coneAngleBlue] = coneGenerator(imageLineBlue,resolution,threshold,coneNumBlue,distanceBlue,coneStartBlue,flagBlue);
[conePosYellow,coneAngleYellow] = coneGenerator(imageLineYellow,resolution,threshold,coneNumYellow,distanceYellow,coneStartYellow,flagYellow);

%% Plot Track
image = imread('Track_downscaled.png');
% grayimage = rgb2gray(image);
grayimage = image;
bwimage = grayimage > threshold;

grid = binaryOccupancyMap(bwimage,resolution);

grid2 = ~grid.getOccupancy;

grid.setOccupancy(grid2) %grid is map

setOccupancy(grid,[58 7],1)
setOccupancy(grid,[58 6],1)
setOccupancy(grid,[58 5],1)

inflate(grid,0.75)

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

%%

startPose = [60 6 0];

% goalPose = [80 6 0];
%goalPose = [120 95 pi];
goalPose = [56 6 0];

traj = codegenPathPlanner(grid, startPose, goalPose);

%%
scatter(traj(:,1),traj(:,2),"cyan","filled")
toc


