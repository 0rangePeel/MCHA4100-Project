%% Main function to generate tests
function tests = test_logGaussian
tests = functiontests(localfunctions);
end

%% Test Functions

function testNominal(testCase)
x = [1; 2; 3];
mu = [2; 4; 6];
S = diag([1, 2, 3]);
expected = -6.04857506884207;
actual = logGaussian(x,mu,S);
assertEqual(testCase, actual, expected, 'AbsTol', 1e-12);
end

function testNominalMultipleX(testCase)
x = [1; 2; 3].*(1:5);
mu = [2; 4; 6];
S = diag([1, 2, 3]);
expected = [-6.04857506884207, -4.54857506884207, -6.04857506884207, -10.5485750688421, -18.0485750688421];
actual = logGaussian(x,mu,S);
assertEqual(testCase, actual, expected, 'AbsTol', 1e-12);
end

function testNominalMultipleMu(testCase)
x = [2; 4; 6];
mu = [1; 2; 3].*(1:5);
S = diag([1, 2, 3]);
expected = [-6.04857506884207, -4.54857506884207, -6.04857506884207, -10.5485750688421, -18.0485750688421];
actual = logGaussian(x,mu,S);
assertEqual(testCase, actual, expected, 'AbsTol', 1e-12);
end

function testUnderflow(testCase)
x = 0;
mu = sqrt(350*log(10)/pi);  % Approx 16
S = 1/sqrt(2*pi);       % Approx 0.4
expected = -805.904782547916;
actual = logGaussian(x,mu,S);
assertEqual(testCase, actual, expected, 'AbsTol', 1e-10);
end

function testDetOverflow(testCase)
a = 1e4;    % Magnitude of st.dev.
n = 100;    % Dimension
S = a*eye(n);
assumeEqual(testCase, det(S), inf, 'Assume det(S) overflows to inf');
x = zeros(n,1);
mu = zeros(n,1);
expected = -n*reallog(a) - n/2*reallog(2*pi);
actual = logGaussian(x,mu,S);
assertEqual(testCase, actual, expected, 'AbsTol', 1e-5);
end

function testDetUnderflow(testCase)
a = 1e-4;  	% Magnitude of st.dev.
n = 100;    % Dimension
S = a*eye(n);
assumeEqual(testCase, det(S), 0, 'Assume det(S) underflows to zero');
x = zeros(n,1);
mu = zeros(n,1);
expected = -n*reallog(a) - n/2*reallog(2*pi);
actual = logGaussian(x,mu,S);
assertEqual(testCase, actual, expected, 'AbsTol', 1e-5);
end


%% Optional file fixtures  
function setupOnce(testCase)  % do not change function name
    addpath ../
end

function teardownOnce(testCase)  % do not change function name
    rmpath ../
end
%% Optional fresh fixtures  
function setup(testCase)  % do not change function name
end

function teardown(testCase)  % do not change function name
end