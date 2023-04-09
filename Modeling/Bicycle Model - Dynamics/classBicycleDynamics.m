% Bicycle Dynamics Class

% States are assumed to be:
% x = [xg, yg, theta, vy, r]

% Where:
% xg - X-coordinate of center of gravity of the robot
% yg - Y-coordinate of center of gravity of the robot
% theta - Angle of the car from the positive x-axis anti-clockwise
% vy - Lateral speed of the vehicle
% r - Yaw rate of the vehicle (Theta)

% The input to the system is Steering angle (δf) of the 
% front wheel 
% The vehicle has a 'constant' velocity



classdef classBicycleDynamics
% Properties contains all of the parameters for the Bicycle Dynamics
% Class
properties

    Lf;     % Length of forward wheel to center of gravity (m)
    Lr;     % Length of back wheel to center of gravity (m)
    g;      % Acceleration due to Gravity (m/s/s)
    m;      % Mass of the vehicle (kg)
    Calphaf % Cornering stiffness coefficient for front tyre
    Calphar % Cornering stiffness coefficient for rear tyre
    Iz      % Moment of Inertia about the Z-axis

end



% Methods contain functions related to the Bicycle Model (Dynamic)
methods

%% A, B, C, D, E, F matrices from the full non-linear dynamics equation
    % 'A' matrix from the Dynamics Paper
    function A = A_equ(obj, sigma_f, Vx)

        A = -(obj.Calphaf*cos(sigma_f) + obj.Calphar)/(obj.m*Vx);

    end


    % 'B' matrix from the Dynamics Paper
    function B = B_equ(obj, sigma_f, Vx)

        B = ( (-obj.Lf*obj.Calphaf*cos(sigma_f) + obj.Lr*obj.Calphar) / (obj.m*Vx) ) - Vx;

    end


    % 'C' matrix from the Dynamics Paper
    function C = C_equ(obj, sigma_f, Vx)

        C = (-obj.Lf*obj.Calphaf*cos(sigma_f) + obj.Lr*obj.Calphar)/(obj.Iz*Vx);

    end


    % 'D' matrix from the Dynamics Paper
    function D = D_equ(obj, sigma_f, Vx)

        D = -( (obj.Lf^2)*obj.Calphaf*cos(sigma_f) + (obj.Lr^2)*obj.Calphar ) / (obj.Iz*Vx);

    end


    % 'E' matrix from the Dynamics Paper
    function E = E_equ(obj, sigma_f, Vx)
        
        E = ( obj.Calphaf*cos(sigma_f) )/obj.m;

    end


    % 'F' matrix from the Dynamics Paper
    function F = F_equ(obj, sigma_f, Vx)

        F = ( obj.Lf*obj.Calphaf*cos(sigma_f) )/obj.Iz;

    end



%% Compute state derivatives given current states and inputs
% x = [xg, yg, theta, vy, r]
function dx = state_derivative(obj, x, sigma_f, Vx)

% Unpack states
Xg = x(1);
Yg = x(2);
theta = x(3);
Vy = x(4);
r = x(5);

% Obtain matrices from relevant functions
A = A_equ(obj, sigma_f, Vx);
B = B_equ(obj, sigma_f, Vx);
C = C_equ(obj, sigma_f, Vx);
D = D_equ(obj, sigma_f, Vx);
E = E_equ(obj, sigma_f, Vx);
F = F_equ(obj, sigma_f, Vx);


% Compute expressions for the state derivatives
dXg = Vx*cos(theta) - Vy*sin(theta);
dYg = Vx*sin(theta) + Vy*cos(theta);
dtheta = r;

dVy = A*Vy + C*r + E*sigma_f;
dr = B*Vy + D*r + F*sigma_f;



% Pack state derivative
% x = [xg, yg, theta, vy, r]
dx = [dXg; dYg; dtheta; dVy; dr];
end


end

end
