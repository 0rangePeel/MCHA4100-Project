% Ensure no unit tests fail before continuing
results = runtests('tests');
assert(~any([results.Failed]));

%% Problem 2

mux = 3;   
muy = 0;
sigmax = 1;
sigmay = 1; % Uncomment for (i)
% sigmay = 10; % Uncomment for (ii)

mu = [muy; mux];
S = diag([sigmay sigmax]);

N = 10000;     % Number of samples

yx = simulateGaussian(mu,S,N);  % Draw N realisations of [y;x]
z = nan(1,N);
for k = 1:N
    z(:,k) = cartesianToAngle(yx(:,k));
end

% Sample mean and standard deviation
[muz_mc, sigmaz_mc] = estimateGaussian(z);

% Affine mean and standard deviation
[muz_at, sigmaz_at] = affineTransform(mu, S, @cartesianToAngle);

% Unscented mean and standard deviation
[muz_ut, sigmaz_ut] = unscentedTransform(mu, S, @cartesianToAngle);

zi = linspace(-pi,pi,1000);

logf_at = logGaussian(zi, muz_at, sigmaz_at);
logf_ut = logGaussian(zi, muz_ut, sigmaz_ut);
logf_mc = logGaussian(zi, muz_mc, sigmaz_mc);

%% Plots

% Generate boundary ellipses of Gaussian confidence regions
s = 2;                  % Number of standard deviations

xx = gaussianConfidenceEllipse(mu, S, s);

figure(1);clf
subplot(1,2,1)
hold on
plot(yx(2,:), yx(1,:), 'b+')
plot(mux, muy, 'r.', 'MarkerSize', 30)
plot(xx(2,:),xx(1,:),'r', 'LineWidth', 2)
xlabel('x [m]')
ylabel('y [m]')
hold off

subplot(1,2,2)
hold on
h1 = histogram(z,'Normalization','pdf');
h2 = plot(zi,exp(logf_mc),'LineWidth',2);
h3 = plot(zi,exp(logf_at),'LineWidth',2);
h4 = plot(zi,exp(logf_ut),'LineWidth',2);
hold off
legend([h1 h2 h3 h4],'p(z) histogram','N(z;\mu,\sigma) (Monte Carlo)','N(z;\mu,\sigma) (Affine transform)','N(z;\mu,\sigma) (Unscented transform)')
xticks([-pi -3*pi/4 -pi/2 -pi/4 0 pi/4 pi/2 3*pi/4 pi])
xticklabels({'-\pi','-3\pi/4','-\pi/2','-\pi/4','0','\pi/4','\pi/2','3\pi/4','\pi'})
xlabel('\theta [rad]')
xlim([-pi,pi])
ylabel('p(\theta) [1/rad]')
grid on


