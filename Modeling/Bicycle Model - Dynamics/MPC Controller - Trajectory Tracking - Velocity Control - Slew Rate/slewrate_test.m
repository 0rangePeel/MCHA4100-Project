clc
clear all
% Formative script to help solve slew rate problem
horizon = 4;
delta_u_max = [1 2];
delta_u_min = [-1 -2];

input_size = 2;



%W = eye(horizon) - diag(ones(horizon-1,1), -1)

W = eye(horizon*input_size) - diag(ones(horizon*input_size-2,1), -2);

A = [W; -W];

bMax = [delta_u_max(1);delta_u_max(2)];
bMin = -[delta_u_min(1);delta_u_min(2)];
for i=2:horizon
    bMax = [bMax;[delta_u_max(1);delta_u_max(2)]];
    bMin = [bMin;-[delta_u_min(1);delta_u_min(2)]];
end

b = [bMax;bMin];

u = [1 0 2 0 3 0 4 0]';

inequal = A*u - b