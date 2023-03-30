% Bicycle Kinematics, continuous time State-Space Equations

function dxdt = BicycleKinematics(x, u, params)

%% Unpack params
% !! Important: These must exactly match the order they were passed as input
% parameters. It'd be very easy to get this wrong and have bugs.

% Unpack states
N = x(1);
E = x(2);
psi = x(3);

% Unpack parameters
lr = params(1);
lf = params(2);

Vin = u(1);
sigma_f = u(2);


beta = atan( (lr/(lf + lr))*tan(sigma_f) );

% Compute expressions for the state derivatives
dN = Vin*cos(psi + beta);
dE = Vin*sin(psi + beta);
dpsi = (Vin*sin(beta))/lr;

% Pack state derivative
% x = [N, E, psi]
dxdt = [dN; dE; dpsi];

end
