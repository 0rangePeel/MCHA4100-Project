function x = simulateGaussian(mu, S, m)

%
% Input arguments
%
% mu:       mean vector (n x 1)
% S:        upper-triangular matrix such that S.'*S is the covariance (n x n)
% m:        number of samples to generate
%
% where
%   n is the dimension
%
% Output arguments
%
% x:        samples (n x m)
%

if nargin < 3
    m = size(mu, 2);
end

assert(istriu(S), 'Expected S to be upper triangular');


% Draw m realisations of a Gaussian random variable
% with mean mu and square-root covariance S
x = zeros(n, m);
