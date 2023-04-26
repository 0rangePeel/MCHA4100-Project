% Main function to generate tests
function tests = test_cartesianToAngleJacobian
tests = functiontests(localfunctions);
end

% Test Functions
function testcartesianToAngleJacobian(testCase)
yx = [1;1];
[~,actual] = cartesianToAngle(yx);
addpath('./DERIVESTsuite');
[expected, err] = jacobianest(@(x) cartesianToAngle(x), yx);
rmpath('./DERIVESTsuite');
assertEqual(testCase, actual, expected, 'AbsTol', max(100*err,1e-12), ...
    'Expected Jacobian to agree with numerical solution');
end

% Optional file fixtures  
function setupOnce(testCase)  % do not change function name
    addpath ../
end

function teardownOnce(testCase)  % do not change function name
    rmpath ../
end

% Optional fresh fixtures  
function setup(testCase)  % do not change function name
end

function teardown(testCase)  % do not change function name
end
