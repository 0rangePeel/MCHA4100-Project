function [conePos,coneAngle] = coneGenerator(imageLine,resolution,threshold,coneNum,distance,coneStart,flag)
    %% Add Map
    image = imread(imageLine);
    %grayimage = rgb2gray(image);
    grayimage = image;
    bwimage = grayimage > threshold; 
    grid = binaryOccupancyMap(bwimage,resolution);
    grid2 = ~grid.getOccupancy;
    grid.setOccupancy(grid2) %grid is map

    %% Get Cone Position    
    conePos = zeros(coneNum, 2);
    coneAngle = zeros(1,coneNum);
    conePos(1,:) = coneStart;
    offset = 0;

    if flag == 0 
        for i=1:coneNum
            j = 0;
            while j < 180
                theta = deg2rad(j + offset) ;
                conePos(i+1,:) = conePos(i,:) + [distance*sin(theta) distance*cos(theta)];
                j = j + 1;
                if checkOccupancy(grid,conePos(i+1,:)) == 1
                    coneAngle(i) = (j + offset);
                    offset = (j + offset) - 90;
                    j = 360;
                end
            end
        end
    else
        for i=1:coneNum
            j = 180;
            while j > 0
                theta = deg2rad(j + offset) ;
                conePos(i+1,:) = conePos(i,:) + [distance*sin(theta) distance*cos(theta)];
                j = j - 1;
                if checkOccupancy(grid,conePos(i+1,:)) == 1
                    coneAngle(i) = (j + offset);
                    offset = (j + offset) - 90;
                    j = 0;
                end
            end
        end
    end
    conePos(end,:) = []; % Remove overflow value
    %coneAngle(end,:) = []; % Remove overflow value
end

