% Linearized Kinematics Bicycle Model Class

classdef classBicycleKinematicsLinear
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

beta = atan( (obj.obj.lr/(obj.obj.lf + obj.obj.lr))*tan(sigma_f) );


% Compute expressions for the state derivatives
dN = Vin*cos(psi + beta);
dE = Vin*sin(psi + beta);
dpsi = (Vin*sin(beta))/obj.obj.lr;

% Pack state derivative
% x = [N, E, psi]
dx = [dN; dE; dpsi];
end




  %% Extended Methods - Linearisation

  % Linearised 'A' Matrix for Kinematic Bicycle Model
  function A_lin = A(obj, x, V, sigma_f)
      
        % Unpack states
        N = x(1);
        E = x(2);
        psi = x(3);

        A_lin = [0, 0, -V*sin(psi + atan((obj.obj.lr*tan(sigma_f))/(obj.obj.lf + obj.obj.lr)));
                0, 0,  V*cos(psi + atan((obj.obj.lr*tan(sigma_f))/(obj.obj.lf + obj.obj.lr)));
                0, 0,                                               0];
  end





  % Linearised 'B' Matrix for Kinematic Bicycle Model
  function B_lin = B(obj, x, V, sigma_f)

        % Unpack states
        N = x(1);
        E = x(2);
        psi = x(3);

      B_lin = [cos(psi + atan((obj.lr*tan(sigma_f))/(obj.lf + obj.lr))),    -(V*obj.lr*sin(psi + atan((obj.lr*tan(sigma_f))/(obj.lf + obj.lr)))*(tan(sigma_f)^2 + 1))/((obj.lf + obj.lr)*((obj.lr^2*tan(sigma_f)^2)/(obj.lf + obj.lr)^2 + 1));
               sin(psi + atan((obj.lr*tan(sigma_f))/(obj.lf + obj.lr))),    (V*obj.lr*cos(psi + atan((obj.lr*tan(sigma_f))/(obj.lf + obj.lr)))*(tan(sigma_f)^2 + 1))/((obj.lf + obj.lr)*((obj.lr^2*tan(sigma_f)^2)/(obj.lf + obj.lr)^2 + 1));
               tan(sigma_f)/((obj.lf + obj.lr)*((obj.lr^2*tan(sigma_f)^2)/(obj.lf + obj.lr)^2 + 1)^(1/2)),    (V*(tan(sigma_f)^2 + 1))/((obj.lf + obj.lr)*((obj.lr^2*tan(sigma_f)^2)/(obj.lf + obj.lr)^2 + 1)^(1/2)) - (V*obj.lr^2*tan(sigma_f)^2*(tan(sigma_f)^2 + 1))/((obj.lf + obj.lr)^3*((obj.lr^2*tan(sigma_f)^2)/(obj.lf + obj.lr)^2 + 1)^(3/2))];

  end




% Linearised state space
function dx = state_derivative_linearized(obj, x, V, sigma_f)
        
        % Unpack states
        N = x(1);
        E = x(2);
        psi = x(3);

        x = [N; E; psi];
        u = [V; sigma_f];

        A_lin = A(obj, x, V, sigma_f);
        B_lin = B(obj, x, V, sigma_f);


        dx = A_lin*x + B_lin*u;

end

end

end






