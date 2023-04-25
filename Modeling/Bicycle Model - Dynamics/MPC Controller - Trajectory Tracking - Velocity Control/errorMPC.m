function [e, J] = errorMPC(t1, x1, u0, U, param)

dt = param.dt/param.ctrl.nSubsteps;
Np = param.ctrl.nPredictionHorizon;
Nc = param.ctrl.nControlHorizon;
assert(Np >= Nc, 'Prediction horizon cannot be less than control horizon')

qr      = param.ctrl.qr;
qpsi    = param.ctrl.qpsi;
ru      = param.ctrl.ru;

nx = length(x1);
nu = length(u0);
e  = zeros(5*Np,1);

t = t1;
x = x1;
f = @bicycleDynamics;

if nargout > 1
    J = zeros(nx*Nc,nu*Nc);
    dxdU = zeros(nx,nu*Nc);
    XX = [x, dxdU];
end

for k = 1:Np                 % for each step in the prediction horizon
    if k <= Nc
        u = U(nu*k-1:nu*k);            % Two inputs [sigma_f; accel_x]
    else
        u = [0; 0];
    end

    % Do nSubsteps of Runge-Kutta integration using 3/8-rule
    for j = 1:param.ctrl.nSubsteps
        if nargout < 2
            f1 = f(t,          x,                           u, param);
            f2 = f(t + dt/3,   x + f1*dt/3,                 u, param);
            f3 = f(t + dt*2/3, x - f1*dt/3 + f2*dt,         u, param);
            f4 = f(t + dt,     x + f1*dt   - f2*dt + f3*dt, u, param);
            x = x + (f1 + 3*f2 + 3*f3 + f4)*dt/8;
        else
            dudU = zeros(nu,nu*Nc);
            if k <= Nc
                dudU(:,nu*(k-1)+1:nu*k) = eye(nu);
            end
            UU = [u, dudU];
            F = @(t,X,U,param) augmentGradients(f,t,X,U,param);
            F1 = F(t,          XX,                           UU, param);
            F2 = F(t + dt/3,   XX + F1*dt/3,                 UU, param);
            F3 = F(t + dt*2/3, XX - F1*dt/3 + F2*dt,         UU, param);
            F4 = F(t + dt,     XX + F1*dt   - F2*dt + F3*dt, UU, param);
            XX = XX + (F1 + 3*F2 + 3*F3 + F4)*dt/8;
            x = XX(:,1);
            dxdU = XX(:,2:end);
        end
        t = t + dt;
    end
    
    % Evaluate trajectory
    X = trajectoryEval(param.traj,t);
    rPNn = [X(1:2,1);0];
%     rANn = [x(3) + param.l*cos(x(5));   x(4) + param.l*sin(x(5));   0]; % Old code from Lab 3, replace with rCNn

    % States [Xg, Yg, theta, Vy, r]
    Xg = x(1);
    Yg = x(2);
    theta = x(3);


    % In terms of [North, East, Down]
    rCNn = [Yg;    Xg;   0];

    vPNn = [X(1:2,2); 0];

    if norm(vPNn) == 0
        % Stay on current heading
        psistar = x(3);
    else
        % Head in direction of velocity vector
        psistar = atan2(vPNn(2), vPNn(1));
    end


    % Unwrap heading reference to have same revolution as heading
    psistar = psistar + 2*pi*round((theta - psistar)/(2*pi));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    r_k = rCNn - rPNn;


%     psistar = X(3,1);     % Old code from Lab 3
    
    psi_f = theta - psistar;

    e((k-1)*5+1:k*5) = [(sqrt(qr))*r_k(1);
                        (sqrt(qr))*r_k(2);
                        (sqrt(qpsi))*psi_f;
                        (sqrt(ru))*u(1)             % Sigma_f input
                        (sqrt(ru))*u(2)];           % Accel_x input




    % Theta
%     x5 = x(3);
%     l = param.l;
%     c = param.c;
    
    if nargout > 1
%         if norm(vPNn) == 0
%             % Stay on current heading
%             der    = [0, 0, qr^(1/2),        0, -c*sqrt(qr)*sin(x5);
%             0, 0,        0, sqrt(qr), c*sqrt(qr)*cos(x5)]*dxdU;
%             
%             depsi  = [       0, 0, 0, 0,                   0]*dxdU;
%     
%             deu    = [ru^(1/2), 0; 0, ru^(1/2)]*dudU;
% 
%             J((k-1)*5+1:k*5,:) = [der;depsi;deu];
% 
%         else
%             % Head in direction of velocity vector
%         der    = [0, 0, qr^(1/2),        0, -c*sqrt(qr)*sin(x5);
%             0, 0,        0, sqrt(qr), c*sqrt(qr)*cos(x5)]*dxdU;
% 
%         depsi  = [       0, 0, 0, 0,                   sqrt(qpsi)]*dxdU;
% 
%         deu    = [ru^(1/2), 0; 0, ru^(1/2)]*dudU;
% 
%         J((k-1)*5+1:k*5,:) = [der;depsi;deu];
%         end
    end
end

