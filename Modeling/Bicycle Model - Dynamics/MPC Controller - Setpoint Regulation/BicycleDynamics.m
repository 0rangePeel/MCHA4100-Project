% Bicycle Dynamics, continuous time State-Space Equations

function dxdt = BicycleDynamics(x, u, params)

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
Lf = params(1);     % Length of forward wheel to center of gravity (m)
Lr = params(2);     % Length of back wheel to center of gravity (m)
g = params(3);      % Acceleration due to Gravity (m/s/s)
m = params(4);      % Mass of the vehicle (kg)
Calphaf = params(5); % Cornering stiffness coefficient for front tyre
Calphar = params(6); % Cornering stiffness coefficient for rear tyre
Iz = params(7);      % Moment of Inertia about the Z-axis

Vx = params(8);     % Constant velocity in the direction of the wheels

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

end
