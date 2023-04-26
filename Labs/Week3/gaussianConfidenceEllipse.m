function x = gaussianConfidenceEllipse(mu, S, k)

c = nan;                        % Probability mass enclosed by k standard deviations
t = linspace(0,2*pi,100);       % Sampling angles for circle
r = nan;                        % Circle radius in w coords
w = r*[cos(t);sin(t)];          % Circle sampling points in w coords
x = nan(size(w));               % Points on ellipse in x coords