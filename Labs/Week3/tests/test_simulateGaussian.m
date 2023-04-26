%% Main function to generate tests
function tests = test_simulateGaussian
    tests = functiontests(localfunctions);
end

%% Test Functions
function testSimulateGaussianMean(testCase)
    rng(1);
    mu          = [3;1];
    S           = [1,0;
                   0,1];
    N           = 1;
    tol         = 2/sqrt(N);
    
    % Set randn to zero
    w = warning;
    warning('off','MATLAB:dispatcher:nameConflict')
    addpath('./mocks/random/','-begin');
    z           = simulateGaussian(mu, S, N);
    rmpath('./mocks/random/');
    warning(w);
    
    mus         = z;
    
    expected    = mu;
    actual      = mus;
    verifyEqual(testCase, actual, expected,'abstol',tol,'Incorrect sample mean from simulateGaussian.');
end

function testSimulateGaussianSqrtCovDiagonal(testCase)
    mu          = [0;0];
    S           = [5,0;
                   0,2];
    N           = 10000;

    s = rng;    % save seed
    rng(1);
    x           = simulateGaussian(mu, S, N);
    rng(s);     % restore seed
    
    expected    = S'*S;
    actual      = cov(x.');
    verifyEqual(testCase, actual, expected, 'AbsTol', 0.1, 'Expected covariance of samples to match S.''*S.');
end

function testSimulateGaussianSqrtCovUpperTriangular(testCase)
    mu          = [0;0];
    S           = [1,-0.3;
                   0,0.1];
    N           = 10000;

    s = rng;    % save seed
    rng(1);
    x           = simulateGaussian(mu, S, N);
    rng(s);     % restore seed
    
    expected    = S'*S;
    actual      = cov(x.');
    verifyEqual(testCase, actual, expected, 'AbsTol', 0.1, 'Expected covariance of samples to match S.''*S.');
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