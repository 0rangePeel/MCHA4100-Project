%% Main function to generate tests
function tests = test_estimateGaussian
    tests = functiontests(localfunctions);
end

%% Test Functions
function testEstimateGaussianMean(testCase)
    z = [ -0.15508     -1.0443      -1.1714       0.92622   -0.55806;
           0.61212     -0.34563     -0.68559     -1.4817    -0.028453];
    [actual,~] = estimateGaussian(z);
    expected = [-0.40052; -0.38585];
    verifyEqual(testCase, actual, expected, 'AbsTol', 1e-5, 'Incorrect sample mean.');
end

function testEstimateGaussianCov(testCase)
    z = [ -0.15508     -1.0443      -1.1714       0.92622   -0.55806;
           0.61212     -0.34563     -0.68559     -1.4817    -0.028453];
    [~,S] = estimateGaussian(z);
    verifyEqual(testCase, S, triu(S), 'Expected S to be upper triangular')
    
    actual = S.'*S;
    expected = [ 0.7135     -0.26502;
                -0.26502     0.60401];
    verifyEqual(testCase, actual, expected, 'AbsTol', 1e-5, 'Incorrect sample covariance.');
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