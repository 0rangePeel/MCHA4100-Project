function [out] = randn(varargin)
%RANDN Summary of this function goes here
%   Detailed explanation goes here
    if(nargin == 1)
        nx  = varargin{1};
        ny  = nx;
    elseif(nargin == 2)
        nx  = varargin{1};
        ny  = varargin{2};
    end
    out 	= zeros(nx, ny);
end

