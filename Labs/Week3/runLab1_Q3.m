% Ensure no unit tests fail before continuing
results = runtests('tests');
assert(~any([results.Failed]));

%% Problem 3

mux = 3;   
muy = 0;
sigmax = 1;
sigmay = 1; % Uncomment for (i)
% sigmay = 10; % Uncomment for (ii)

mu = [muy; mux];
S = diag([sigmay sigmax]);

N = 10000;     % Number of samples

yx = simulateGaussian(mu,S,N);  % Draw N realisations of [y;x]
z = nan(2,N);
for k = 1:N
    z(:,k) = cartesianToPolarNoisy(yx(:,k));
end

% Sample mean and sqrt covariance
[muz_mc, Sz_mc] = estimateGaussian(z);

% Affine mean and sqrt covariance
[muz_at, Sz_at] = affineTransformWithAdditiveNoise(mu, S, @cartesianToPolarNoisy);

% Unscented mean and sqrt covariance
[muz_ut, Sz_ut] = unscentedTransformWithAdditiveNoise(mu, S, @cartesianToPolarNoisy);

%%

% Generate boundary ellipses of Gaussian confidence regions
s = 2;                  % Number of standard deviations

xx    	= gaussianConfidenceEllipse(mu, S, s);
zz_mc	= gaussianConfidenceEllipse(muz_mc, Sz_mc, s);
zz_at 	= gaussianConfidenceEllipse(muz_at, Sz_at, s);
zz_ut 	= gaussianConfidenceEllipse(muz_ut, Sz_ut, s);

% Draw figures
figure(2);clf
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
plot(z(1,:),z(2,:),'b+')
plot(muz_at(1), muz_at(2), 'Color', [0,0.5,0], 'Marker', '.', 'MarkerSize', 30)
h_at=plot(zz_at(1,:),zz_at(2,:), 'Color', [0,0.5,0], 'LineWidth', 2);
plot(muz_ut(1), muz_ut(2), 'm.', 'MarkerSize', 30)
h_ut=plot(zz_ut(1,:),zz_ut(2,:), 'm', 'LineWidth', 2);
plot(muz_mc(1), muz_mc(2), 'r.', 'MarkerSize', 30)
h_mc=plot(zz_mc(1,:),zz_mc(2,:),'r', 'LineWidth', 2);
hold off
xlabel('\theta [rad]')
xticks([-pi -3*pi/4 -pi/2 -pi/4 0 pi/4 pi/2 3*pi/4 pi])
xticklabels({'-\pi','-3\pi/4','-\pi/2','-\pi/4','0','\pi/4','\pi/2','3\pi/4','\pi'})
% xlim([-pi,pi])
ylabel('r [m]')
legend([h_mc h_at h_ut],'2\sigma confidence ellipse (MC)','2\sigma confidence ellipse (AT)','2\sigma confidence ellipse (UT)')

