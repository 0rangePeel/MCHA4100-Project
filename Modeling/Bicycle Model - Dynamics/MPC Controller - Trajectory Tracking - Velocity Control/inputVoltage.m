function u = inputVoltage(t, x)

if t < 1
    u = [0;0];
elseif t < 2
    u = [0.5;0.5];
elseif t < 3
    u = [0;0];
elseif t < 4
    u = [3;0];
elseif t < 5
    u = [0;0];
elseif t < 6
    u = [-3;3];
elseif t < 7
    u = [0;0];
elseif t < 8
    u = [0;3];
elseif t < 9
    u = [0;0];
elseif t < 10
    u = [3;-3];
elseif t < 11
    u = [0;0];
elseif t < 12
    u = [6;6];
else
    u = [0;0];
end