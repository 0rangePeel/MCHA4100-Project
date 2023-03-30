% Bicycle Kinematics Class

% States are assumed to be:
% x = [N E Ψ]

% Where 'N' is the north position of the vehicle, 'E' is the east position
% of the vehicle and 'Ψ' is the yaw angle of the vehicle, positive
% direction counter-clockwise


% The inputs to the system are Steering angle (δf) and the net velocity (V) in the
% inertial (global) frame.



classdef classBicycleKinematics
% Properties contains all of the parameters for the Bicycle Kinematics
% Class
properties

    lf;     % Length of forward wheel to center of gravity (m)
    lr;     % Length of back wheel to center of gravity (m)

end



% Methods contain functions related to the Bicycle Model (Kinematic)
methods
% Computes state derivatives given current states and inputs
function dx = state_derivative(obj,x,Vin,sigma_f)

% Unpack states
N = x(1);
E = x(2);
psi = x(3);

beta = atan( (obj.lr/(obj.lf + obj.lr))*tan(sigma_f) );


% Compute expressions for the state derivatives
dN = Vin*cos(psi + beta);
dE = Vin*sin(psi + beta);
dpsi = (Vin*sin(beta))/obj.lr;

% Pack state derivative
% x = [N, E, psi]
dx = [dN; dE; dpsi];
end


end

end
