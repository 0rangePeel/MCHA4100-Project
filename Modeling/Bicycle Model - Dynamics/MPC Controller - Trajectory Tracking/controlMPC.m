function U = controlMPC(t1,x1,u0,U,param)

m = size(u0,1);

if isempty(U)
    U = [u0;zeros(m*(param.ctrl.nControlHorizon-1),1)];
else
    % warm start
    U = [U(m+1:end);zeros(m,1)];
end

options = optimoptions('lsqnonlin',...
            'Display','iter',... % Use 'none' to disable console spam
            'SpecifyObjectiveGradient',param.useGradientOpt,...
            'CheckGradients',false,...
            'MaxFunctionEvaluations',1e5,...
            'MaxIterations',1000);
err = @(U) errorMPC(t1,x1,u0,U,param);
lb = repmat(param.ctrl.uLowerBound, param.ctrl.nControlHorizon,1);
ub = repmat(param.ctrl.uUpperBound, param.ctrl.nControlHorizon,1);
U  = lsqnonlin(err,U,lb,ub,options);
