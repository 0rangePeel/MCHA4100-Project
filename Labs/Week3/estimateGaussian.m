function [mu, S] = estimateGaussian(x)

%
% Input arguments
%
% x:        samples (n x m)
%
% where
%   n is the dimension
%   m is the number of samples
%
% Output arguments
%
% mu:       mean vector (n x 1)
% S:        upper-triangular matrix such that S.'*S is the covariance (n x n)
%

% Estimate the sample mean and sample square-root covariance of the data x
[n, m] = size(x);
mu = zeros(n,1);
S = zeros(n,n);
