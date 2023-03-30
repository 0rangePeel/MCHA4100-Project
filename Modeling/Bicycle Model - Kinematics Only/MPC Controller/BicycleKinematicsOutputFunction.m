function [y] = BicycleKinematicsOutputFunction(x,u,Ts)
% Generates a 'measured' output vector from the Bicycle Kinematics where
% States: x = [North, East, Psi]'
% Output: y = [North, East]'
y = [x(1);
    x(2)];