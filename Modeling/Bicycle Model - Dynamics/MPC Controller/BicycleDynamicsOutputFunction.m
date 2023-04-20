function [y] = BicycleDynamicsOutputFunction(x,u,Ts)
% Generates a 'measured' output vector from the Bicycle Dynamics where
% States: x = x = [xg, yg, theta, vy, r]'
% Output: y = [xg, yg]'
y = [x(1);
    x(2)];