% Compute the Jacobian to obtain a Linearized State-Space Model of the 
% Kinematic Bicycle Model
clc;
clear;

syms V psi beta sigma_f lr lf N E

beta = atan((lr / (lf + lr)) * tan(sigma_f));

f = [ V*cos(psi + beta); V*sin(psi + beta); (V/lr)*sin(beta) ];

A = jacobian(f, [N, E, psi])   % Evaluate Jacobian Matrix 'A', Linearized 'A' matrix

B = jacobian(f, [V, sigma_f])   % Evaluate Jacobian Matrix 'B', Linearized 'B' matrix