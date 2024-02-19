function [u, cval1, cval2] = implicitDoubleInterpolatingControl(varargin)
%IMPLICITDOUBLEINTERPOLATINGCONTROL Returns control value according 
%to implicit improved interpolating control law
%
% OUTPUT: u - value of control according to interpolating control law
%         cval - optimal value of c
%         rvk - linear program variable
% INPUT:  LTISystem, x - linear time invariant system, current state of
%         system
%         LTISystem, x, Pu - union with vertex control law
%         functions
%         LTISystem, x, Pu, InvSet - invariant N-step controlled set
%         LTISystem, x, Pu, InvSet, OmegaS - invariant S-controlled set
%         LTISystem, x, Pu, InvSet, OmegaS, LQRSet - invariant controlled set
%         LTISystem, x, Pu, InvSet, OmegaS, LQRSet, PuS - union with vertex
%         control law for OmegaS
%         

if nargin < 2
    u = NaN;
    warning('wrong input!')
    return;
elseif nargin == 2
    system = varargin{1};
    x = varargin{2};
    InvSet = system.invariantSet();
    LQRSet = system.LQRSet();
    OmegaS = invariantControlledSet(system,3,LQRSet);
    Vcenter = zeros(1,system.nx);
    Pu = vertexControl(system, InvSet, Vcenter);
    PuS = vertexControl(system, OmegaS , Vcenter);
elseif nargin == 3
    system = varargin{1};
    x = varargin{2};
    InvSet = system.invariantSet();
    LQRSet = system.LQRSet();
    Vcenter = zeros(1,system.nx);
    OmegaS = invariantControlledSet(system,3,LQRSet);
    Pu = varargin{3};
    PuS = vertexControl(system, OmegaS , Vcenter);
elseif nargin == 4
    system = varargin{1};
    x = varargin{2};
    Pu = varargin{3};
    Vcenter = zeros(1,system.nx);
    InvSet = varargin{4};
    LQRSet = system.LQRSet();
    OmegaS = invariantControlledSet(system,3,LQRSet);
    PuS = vertexControl(system, OmegaS , Vcenter);
elseif nargin == 5
    system = varargin{1};
    x = varargin{2};
    Pu = varargin{3};
    InvSet = varargin{4};
    Vcenter = zeros(1,system.nx);
    OmegaS = varargin{5};
    LQRSet = system.LQRSet();
    PuS = vertexControl(system, OmegaS , Vcenter);
elseif nargin == 6
    system = varargin{1};
    x = varargin{2};
    Pu = varargin{3};
    InvSet = varargin{4};
    Vcenter = zeros(1,system.nx);
    OmegaS = varargin{5};
    LQRSet = varargin{6};
    PuS = vertexControl(system, OmegaS , Vcenter);
elseif nargin == 7
    system = varargin{1};
    x = varargin{2};
    Pu = varargin{3};
    InvSet = varargin{4};
    OmegaS = varargin{5};
    LQRSet = varargin{6};
    PuS = varargin{7};
elseif nargin > 7
    system = varargin{1};
    InvSet = system.invariantSet();
    LQRSet = system.LQRSet();
    Vcenter = zeros(1,system.nx);
    OmegaS = invariantControlledSet(system,3,LQRSet);
    Pu = vertexControl(system, InvSet, Vcenter);
    PuS = vertexControl(system, OmegaS , Vcenter);
end

options = sdpsettings('solver','linprog','verbose',0,'cachesolvers',1);  % setup LP-solver

if LQRSet.contains(x)
    % if x is in set for LQ control
    u = system.LQRGain*x;
    cval1 = 0; cval2 = 0;
elseif OmegaS.contains(x)
    % if x is in addtional set for vertex control
    c2 = sdpvar(1, 1);
    rs = sdpvar(system.nx,1);
    J2 = c2;
    C2 = [OmegaS.A*rs <= c2*OmegaS.b; LQRSet.A*(x-rs) <= (1-c2)*LQRSet.b; 0 <= c2 <= 1];
    
    optimize(C2, J2, options);
    
    rsk = value(rs);
    r0 = x-rsk;
    usk = PuS.feval(rsk,'uv');
    u = system.LQRGain*r0 + usk(1:system.nu,1);
    
    cval2 = value(c2);
    cval1 = 0;
elseif InvSet.contains(x)
    % if x is in invariant N controlled set for vertex control
    c1 = sdpvar(1, 1);
    rv = sdpvar(system.nx,1);
    J1 = c1;
    C1 = [InvSet.A*rv <= c1*InvSet.b; OmegaS.A*(x-rv) <= (1-c1)*OmegaS.b; 0 <= c1 <= 1];
    
    optimize(C1, J1, options);
    
    rvk = value(rv);
    rsk = x - rvk;
    uvk = Pu.feval(rvk,'uv');
    usk = PuS.feval(rsk,'uv');
    u = uvk(1:system.nu,1) + usk(1:system.nu,1);
    
    cval1 = value(c1);
    cval2 = 0;
else
    % x isnt in controlled space
    u = NaN;
end

end

