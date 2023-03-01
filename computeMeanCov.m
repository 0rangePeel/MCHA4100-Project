function [muh,sigmah] = computeMeanCov(z)
    % Computes Mean and Covariance of MVN samples
    N = size(z,2);
    muh = sum(z,2)/N;
    sigmah = (z-muh)*(z-muh).'/N;
end