function logPDF = logGaussian(x, mu, S)

%
% Input arguments
%
% x:        points to evaluate (n x 1 or n x m)
% mu:       mean (n x 1 or n x m)
% S:        upper-triangular matrix such that S.'*S is the covariance (n x n)
%
% where
%   n is the dimension
%   m is the number of evaluation points
%
% Output arguments
%
% logPDF: log of Gaussian at evaluation points (1 x m)
%

assert(istriu(S), 'Expected S to be upper triangular');

P = S.'*S;
x_mu = x - mu;
m = size(x_mu, 2);
logPDF = zeros(1, m);
for i = 1:m
    logPDF(i) = reallog( 1/realsqrt(det(2*pi*P))*exp(-0.5*x_mu(:,i).'/P*x_mu(:,i)) );
end
