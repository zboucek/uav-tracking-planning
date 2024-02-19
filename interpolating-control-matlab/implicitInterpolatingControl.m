function [u, cval, rvk] = implicitInterpolatingControl(varargin)
%IMPLICITINTERPOLATINGCONTROL Returns control value according to implicit
%interpolating control law
%
% OUTPUT: u - value of control according to interpolating control law
%         cval - optimal value of c
%         rvk - linear program variable
% INPUT:  LTISystem, x - linear time invariant system, current state of
%         system
%         LTISystem, x, Pu - union with vertex control law
%         functions
%         LTISystem, x, Pu, InvSet - invariant N-step controlled set
%         LTISystem, x, Pu, InvSet, LQRSet - invariant controlled set
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
    Vcenter = zeros(1,system.nx);
    Pu = vertexControl(system, InvSet, Vcenter);
elseif nargin == 3
    system = varargin{1};
    x = varargin{2};
    InvSet = system.invariantSet();
    LQRSet = system.LQRSet();
    Pu = varargin{3};
elseif nargin == 4
    system = varargin{1};
    x = varargin{2};
    Pu = varargin{3};
    InvSet = varargin{4};
    LQRSet = system.LQRSet();
elseif nargin == 5
    system = varargin{1};
    x = varargin{2};
    Pu = varargin{3};
    InvSet = varargin{4};
    LQRSet = varargin{5};
elseif nargin > 5
    system = varargin{1};
    InvSet = system.invariantSet();
    LQRSet = system.LQRSet();
    Vcenter = zeros(1,system.nx);
    Pu = vertexControl(system, InvSet, Vcenter);
end

if LQRSet.contains(x)
    % if x is in set for LQ control
    u = system.LQRGain*x;
    cval = 0;
elseif InvSet.contains(x)
    % optimization variables
    c = sdpvar(1, 1);
    rv = sdpvar(system.nx,1);
    J = c;

    C = [InvSet.A*rv <= c*InvSet.b; LQRSet.A*(x-rv) <= (1-c)*LQRSet.b; 0 <= c <= 1];

    options = sdpsettings('solver','linprog','verbose',0,'cachesolvers',1);  % setup LP-solver

    optimize(C, J, options);

    rvk = value(rv);

    r0 = x - rvk;
    uvk=Pu.feval(rvk, 'uv');
    u = system.LQRGain*r0 + uvk(1:system.nu,1);
    cval = value(c);
else
    u = NaN;
    cval = NaN;
end
end

