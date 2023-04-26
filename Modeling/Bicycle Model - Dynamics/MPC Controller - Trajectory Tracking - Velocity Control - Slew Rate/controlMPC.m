function U = controlMPC(t1,x1,u0,U,param)

m = size(u0,1);

if isempty(U)
    U = [u0;zeros(m*(param.ctrl.nControlHorizon-1),1)];
else
    % warm start
    U = [U(m+1:end);zeros(m,1)];
end

% options = optimoptions('lsqnonlin',...
%             'Display','iter',... % Use 'none' to disable console spam
%             'SpecifyObjectiveGradient',param.useGradientOpt,...
%             'CheckGradients',false,...
%             'MaxFunctionEvaluations',1e5,...
%             'MaxIterations',1000);
options = optimoptions('fmincon',...
            'Display','iter',... % Use 'none' to disable console spam
            'SpecifyObjectiveGradient',param.useGradientOpt,...
            'CheckGradients',false,...
            'MaxFunctionEvaluations',1e5,...
            'MaxIterations',250);

%Update slew constraints based on previous input
param.ctrl.b(1:2) = [u0(1);u0(2)] + [param.ctrl.delta_u_max(1);param.ctrl.delta_u_max(2)];
param.ctrl.b(param.ctrl.nControlHorizon*m+1:param.ctrl.nControlHorizon*m+2) = -[u0(1);u0(2)] - [param.ctrl.delta_u_min(1);param.ctrl.delta_u_min(2)];

% Legacy Optimisation - does not work with slew rate
%err = @(U) errorMPC(t1,x1,u0,U,param);
%U  = lsqnonlin(err,U,lb,ub,A,b,[],[],[],options);

err = @(U) norm(errorMPC(t1,x1,u0,U,param));
U  = fmincon(err,U,param.ctrl.A,param.ctrl.b,[],[],param.ctrl.lb,param.ctrl.ub,[],options);