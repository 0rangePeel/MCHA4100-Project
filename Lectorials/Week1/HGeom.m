function P = HGeom(k,w,b,n)
% Example from Lectorial 2 Probability

% Set conditions
if (k >= 0) && (w >= k) && (n-k >= 0) && (b >= n-k)
    P = (nchoosek(w,k)*nchoosek(b,n-k))/nchoosek(w+b,n);
else
    P = 0;
end

end

