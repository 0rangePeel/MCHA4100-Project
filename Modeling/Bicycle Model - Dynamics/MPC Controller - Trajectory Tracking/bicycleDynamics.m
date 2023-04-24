function [dxdt, Jx, Ju] = bicycleDynamics(t, x, u, param)


%% Unpack params
% !! Important: These must exactly match the order they were passed as input
% parameters.

% Unpack states
Xg = x(1);
Yg = x(2);
theta = x(3);
Vy = x(4);
r = x(5);


% Unpack parameters
Lf = param.Lf;     % Length of forward wheel to center of gravity (m)
Lr = param.Lr;     % Length of back wheel to center of gravity (m)
g = param.g;      % Acceleration due to Gravity (m/s/s)
m = param.m;      % Mass of the vehicle (kg)
Calphaf = param.Calphaf; % Cornering stiffness coefficient for front tyre
Calphar = param.Calphar; % Cornering stiffness coefficient for rear tyre
Iz = param.Iz;      % Moment of Inertia about the Z-axis

Vx = param.Vx;     % Constant velocity in the direction of the wheels

% Unpack inputs (Steering angle)
sigma_f = u(1);


% Compute matrices
A = -((Calphaf*cos(sigma_f) + Calphar)/(m*Vx));

B = ( (-Lf*Calphaf*cos(sigma_f) + Lr*Calphar) / (Vx*Iz) );

C = ((-Lf*Calphaf*cos(sigma_f) + Lr*Calphar)/(m*Vx)) - Vx;

D = -( (Lf^2)*Calphaf*cos(sigma_f) + (Lr^2)*Calphar ) / (Iz*Vx);

E = ( Calphaf*cos(sigma_f) )/m;

F = ( Lf*Calphaf*cos(sigma_f) )/Iz;





% Compute expressions for the state derivatives
dXg = Vx*cos(theta) - Vy*sin(theta);
dYg = Vx*sin(theta) + Vy*cos(theta);
dtheta = r;

dVy = A*Vy + C*r + E*sigma_f;
dr = B*Vy + D*r + F*sigma_f;


% Pack state derivative
% x = [xg, yg, theta, vy, r]
dxdt = [dXg; dYg; dtheta; dVy; dr];



if nargout >= 2
%     % Reserved for future lab
%     Jx =  [-(2*Km^2*Ng^2)/(Ra*m*rw^2),            -(2*m*x2*(c - l))/((l*(c*m - l*m) - Iz + c*l*m)*(m*l^2 - 2*c*m*l + Iz)), 0, 0,  0;
% -(x2*(c - l))/(m*l^2 - 2*c*m*l + Iz), (Km^2*Ng^2*d^2)/(2*Ra*rw^2*(l*(c*m - l*m) - Iz + c*l*m)) - (x1*(c - l))/(m*l^2 - 2*c*m*l + Iz), 0, 0,  0;
%                            cos(x5)/m,                                                      -(l*sin(x5))/(l*(c*m - l*m) - Iz + c*l*m), 0, 0, - (x1*sin(x5))/m - (l*x2*cos(x5))/(l*(c*m - l*m) - Iz + c*l*m);
%                            sin(x5)/m,                                                       (l*cos(x5))/(l*(c*m - l*m) - Iz + c*l*m), 0, 0,   (x1*cos(x5))/m - (l*x2*sin(x5))/(l*(c*m - l*m) - Iz + c*l*m);
%                                    0,                                                                -1/(l*(c*m - l*m) - Iz + c*l*m), 0, 0,                                                              0];
%  
end
% 
if nargout >= 3
%     % Reserved for future lab
%     Ju = [1750/657, 1750/657;
%  175/438, -175/438;
%        0,        0;
%        0,        0;
%        0,        0];
end
