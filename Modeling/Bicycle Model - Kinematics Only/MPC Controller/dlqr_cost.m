function [ad,bd,Qd,Rd,Nd] = dlqr_cost(a,b,q,r,nn,Ts)

% Computes the equivilant descrete-time LQR cost for a continuous time
% plant. Code is slightly adapted from the 'dlqr' function to return
% required arguments.

% Source code is available by calling 'open dlqr'

% Original citation:
%   Clay M. Thompson 7-16-90
%   Revised: P. Gahinet  7-25-96
%   Copyright 1986-2007 The MathWorks, Inc.

% Reference: This routine is based on the routine JDEQUIV.M by Franklin, 
% Powell and Workman and is described on pp. 439-441 of "Digital Control
% of Dynamic Systems".

narginchk(5,6)
[msg,a,b] = abcdchk(a,b); error(msg);
Nx = size(a,1); 
Nu = size(b,2);

% Set Ts and NN properly
if nargin==5,
   Ts = nn;
   nn = zeros(Nx,Nu);
end

% Check dimensions and symmetry
if any(size(q)~=Nx),
    ctrlMsgUtils.error('Control:design:lqr2','lqrd','A','Q')
elseif any(size(r)~=Nu),
    ctrlMsgUtils.error('Control:design:lqr3','lqrd')
elseif ~isequal(size(nn),[Nx Nu]),
    ctrlMsgUtils.error('Control:design:lqr2','lqrd','B','N')
elseif norm(q'-q,1) > 100*eps*norm(q,1),
    ctrlMsgUtils.warning('Control:design:MakeSymmetric','lqrd(...,Q,R,Ts)','Q','Q','Q')
elseif norm(r'-r,1) > 100*eps*norm(r,1),
    ctrlMsgUtils.warning('Control:design:MakeSymmetric','lqrd(...,Q,R,Ts)','R','R','R')
end

% Enforce symmetry and check positivity
q = (q+q')/2;
r = (r+r')/2;
vr = real(eig(r));
vqnr = real(eig([q nn;nn' r]));
if min(vr)<=0,
    ctrlMsgUtils.error('Control:design:lqr3','lqrd')
elseif min(vqnr)<-1e2*eps*max(0,max(vqnr)),
    ctrlMsgUtils.warning('Control:design:MustBePositiveDefinite','[Q N;N'' R]','lqrd')
end

% Determine discrete equivalent of continuous cost function 
% along with Ad,Bd matrices of discretized system
n = Nx+Nu;
Za = zeros(Nx); Zb = zeros(Nx,Nu); Zu = zeros(Nu);
M = [ -a' Zb   q  nn
      -b' Zu  nn'  r
      Za  Zb   a   b
      Zb' Zu  Zb' Zu];
phi = expm(M*Ts);
phi12 = phi(1:n,n+1:2*n);
phi22 = phi(n+1:2*n,n+1:2*n);
QQ = phi22'*phi12;
QQ = (QQ+QQ')/2;        % Make sure QQ is symmetric
Qd = QQ(1:Nx,1:Nx);
Rd = QQ(Nx+1:n,Nx+1:n);
Nd = QQ(1:Nx,Nx+1:n);
ad = phi22(1:Nx,1:Nx);
bd = phi22(1:Nx,Nx+1:n);

end
