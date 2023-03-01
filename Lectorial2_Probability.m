%% Task One
clc
clear

N = 1000;
w = 7;
b=5;
n=4;
k=1;

pop = [1:12];
pms = ones(1,12)/12;

x = zeros(1,N);
for i=1:N
    y = randsample(pop,n,false);

    %Number of White balls
    x(i) = sum(y < 8);
end



for k = 0:4
    P(k+1) = HGeom(k,7,5,4);
end

fig1 = figure('Name','White Balls PDF');
histogram(x,'normalization','probability')
hold on
plot([0,1,2,3,4],P,'linewidth',4)
hold off


%% Task Two
clc
clear

% mu and sigma for changing Normal Distribution
mu = 0.5;
sigma = 0.1;

% Set the limit for Q and CDF for the Normal distribution
leftLimit = 0.3;
rightLimit = 0.6;

N = 100000;
x = random('norm',mu,sigma,1,N);
xx = linspace(min(x),max(x));

normalFunc = @(x) 1/(sqrt(2*pi*sigma^2))*exp(-0.5 * (x-mu).^2/(sigma*sigma));

% Area between limits
Q = integral(normalFunc,leftLimit,rightLimit)

% Cumulative Density Function
CDF = integral(normalFunc,-inf,rightLimit)

fig2 = figure('Name','Normal Distribution Visual Tool');
histogram(x,round(sqrt(length(x))),'Normalization','pdf')
hold on
plot(xx,1/(sqrt(2*pi*sigma^2))*exp(-0.5 * (xx-mu).^2/(sigma*sigma)),'r-','linewidth',3)
hold on
plot([rightLimit rightLimit],[0 4.5],'-b','linewidth',3)
hold on
plot([leftLimit leftLimit],[0 4.5],'-b','linewidth',3)
hold off



