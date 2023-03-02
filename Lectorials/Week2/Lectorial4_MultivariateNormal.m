clc
clear
%% Task One
n = 2;
N = 100000;

mu = [1;pi/2]; % nx1
sigma = [0.1 0.2; 0.2 1.0]; % nxn
z = MVN_SampleGen(mu,sigma,N);

fig1 = figure('Name','MVN PDF sample plot');
plot(z(1,:),z(2,:),'+')
hold on

%% Task Two
[z_muh, z_sigma] = computeMeanCov(z)
plot(z_muh(1),z_muh(2),'g*')


%% Task Three
g = [z(1,:).*cos(z(2,:));z(1,:).*sin(z(2,:))];
fig2 = figure('Name','Polar Coordinates to Cartesian');
plot(g(1,:),g(2,:),'+')
hold on

[g_muh, g_sigmah] = computeMeanCov(g)
plot(g_muh(1),g_muh(2),'g*')

%% Extended - Plotting g function mean and covariance - see how g function is not MVN
y = MVN_SampleGen(g_muh,g_sigmah,N);

fig3 = figure('Name','G function Mean and Covariance plotted');
plot(y(1,:),y(2,:),'+')
hold on
[y_muh, y_sigmah] = computeMeanCov(y)
plot(y_muh(1),y_muh(2),'g*')