function [U] = admisControl(varargin)
%ADMISCONTROL Calculate admissible values of control for corresponding
%vertices
% OUTPUT: matrix U [nu nV], where nu is dim of control and nV is number of
% vertices
% INPUT:  LTISystem
%         LTISystem, Polyhedron - polyh. with invariant controlled set

if nargin < 1
    U = NaN;
    warning('wrong input!')
    return;
elseif nargin == 1
    system = varargin{1};
    InvSet = system.invariantSet();
elseif nargin == 2
    system = varargin{1};
    InvSet = varargin{2};
elseif nargin > 2
    system = varargin{1};
    InvSet = system.invariantSet();
end

% find admissible control values for vertices
options = sdpsettings('verbose',0,'cachesolvers',1);  % setup LP-solver
N = size(InvSet.V,1);

% solve one big LP program
% u = sdpvar(system.nu,N);   % variable of linear program
% Ftemp = InvSet.A*(system.A*InvSet.V'+system.B*u);   % conditions on dynamics
% optimize([Ftemp(:)<=repmat(InvSet.b,N,1);repmat(system.u.min,N,1)<=u(:)<=repmat(system.u.max,N,1)], -norm(u,1)), options);
% U = value(u);

% solve a lot of small LP programs
U = zeros(system.nu,N);
for i = 1:N
    u = sdpvar(system.nu,1);   % variable of linear program
    Ftemp = InvSet.A*(system.A*InvSet.V(i,:)'+system.B*u);   % conditions on dynamics
    optimize([Ftemp<=InvSet.b;system.u.min<=u(:)<=system.u.max], -norm(u,1), options);
    U(:,i) = value(u);
end

end