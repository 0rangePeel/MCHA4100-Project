clc
clear
%% Task One
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

histogram(x,'normalization','probability')
hold on
plot([0,1,2,3,4],P,'linewidth',4)


%% Task Two
clc
clear

mu = 1;
sigma = 0.1;
normalFunc = @(x) 1/(sqrt(2*pi*sigma^2))*exp(-0.5 * (x-mu).^2/sigma*sigma)

Q = integral(normalFunc,-inf,0.8)
