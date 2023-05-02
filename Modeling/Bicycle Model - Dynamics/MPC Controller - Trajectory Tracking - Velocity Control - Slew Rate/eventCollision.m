function [value, isterminal, direction] = eventCollision(t, x, u, param, imageLine)

    value = 1;
    isterminal = 1;
    direction = 0;

    r = 0.5;      % Equvalent to the Lf and Lr variables, % North, East, Down
    d = 1.5;

    rBNn = [x(1:2);0];

    Rnb = [cos(x(3)), -sin(x(3)), 0;
           sin(x(3)), cos(x(3)), 0;
           0, 0, 1];

    RnbF = [cos(x(3) + u(1)) -sin(x(3) + u(1)) 0; 
            sin(x(3)+ u(1)) cos(x(3) + u(1)) 0; 
            0 0 1];

    rFNn = rBNn + [param.Lf*cos(x(3)); param.Lf*sin(x(3)); 0];

    rFLNn = rFNn + [-r*sin(x(3)); r*cos(x(3)); 0];
    rFRNn = rFNn + [r*sin(x(3)); -r*cos(x(3)); 0];

    image = imread(imageLine);
    bwimage = image < param.threshold; 
    map = binaryOccupancyMap(bwimage,param.resolution);
    
    % persistent collisionCarFrame
    % if isempty(collisionCarFrame)
    %     collisionCarFrame = [[0.1;0.1;0] [0.1;-0.1;0] [-0.1;-0.1;0] [-0.1;0.1;0]];
    % end
    % 
    % collisionCar = Rnb*collisionCarFrame + rBNn;
    % 
    % for i=1:size(collisionCar,2)
    %     testPoint = [collisionCar(1,i) collisionCar(2,i)];
    %     if checkOccupancy(map,testPoint) == 1
    %         value = 0;
    %     end
    % end


    persistent rPCb
    if isempty(rPCb)

        curve = linspace(3*pi/2, pi/2, 50);
        % Drawing body of the vehicle
        rPCb = [ [-d; r; 0], [d; r; 0], [d; -r; 0], [-d; -r; 0], [-d; r; 0], ... % Car Body
                 [-d; r+0.25; 0], [-d+1; r+0.25; 0], [-d+1; r; 0],[-d; r; 0] ...  % Car Back Left Wheel
                 [-d; -r; 0], [-d; -r-0.25; 0], [-d+1; -r-0.25; 0], [-d+1; -r; 0], ...          % Car Back Right Wheel
                 [-d; -r; 0], [-d; r; 0], [d; r; 0], -r*[cos(curve)-2.5; sin(curve); zeros(size(curve))]];     % Mad Aerodynamics
    end

    % Drawing Steering Wheels
    persistent rPFLwb
    if isempty(rPFLwb)
        rPFLwb = [[0.5; 0; 0], [0.5; (0+0.25); 0] [-0.5; (0+0.25); 0] [-0.5; 0; 0],[0.5; 0; 0]]; % Car Front Left Steering Wheel
    end

    persistent rPFRwb
    if isempty(rPFRwb)
        rPFRwb = [[0.5; 0; 0], [0.5; -(0+0.25); 0] [-0.5; -(0+0.25); 0] [-0.5; 0; 0],[0.5; 0; 0]]; % Car Front Right Steering Wheel
    end


    carBody = Rnb*rPCb + rBNn;
    leftWheel = RnbF*rPFLwb + rFLNn;
    rightWheel = RnbF*rPFRwb + rFRNn;

    collisionCar = [carBody leftWheel rightWheel];
    % Plot Car points for testing
    %scatter(collisionCar(1,:),collisionCar(2,:))

    for i=1:size(collisionCar,2)
        testPoint = [collisionCar(1,i) collisionCar(2,i)];
        if checkOccupancy(map,testPoint) == 1
            value = 0;
        end
    end
end