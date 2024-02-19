function [Pu] = vertexControl(varargin)
%VERTEXCONTROL Returns Union of Polyhedrons for vertex control
%Each set in union contains admissible control in Data structure and
%function of control uv
% OUTPUT: Pu - Union of Polyhedrons
%         Pu.Set(i).Data.U(:,j) - admissible control for set i and vertex j 
%                                 (Pu.Set(i).V(j,:))
%         Pu.Set(i).Data.K - vertex congrol gain for Polyhedron i
%         Pu.feval(x) - get the vertex controller value u for the state x
% INPUT:  LTISystem
%         LTISystem, Polyhedron - polyh. with invariant controlled set
%         LTISystem, Polyhedron, Vcenter - polyh. with invariant controlled
%         set and origin of control [x10, x20, ..., xn0]

if nargin < 1
    Pu = NaN;
    warning('wrong input!')
    return;
elseif nargin == 1
    system = varargin{1};
    InvSet = system.invariantSet();
    Vcenter = zeros(1,system.nx);
    U = admisControl(system, InvSet);
elseif nargin == 2
    system = varargin{1};
    InvSet = varargin{2};
    Vcenter = zeros(1,system.nx);
    U = admisControl(system, InvSet);
elseif nargin == 3
    system = varargin{1};
    InvSet = varargin{2};
    U = varargin{3};
    Vcenter = zeros(1,system.nx);
elseif nargin == 4
    system = varargin{1};
    InvSet = varargin{2};
    U = varargin{3};
    Vcenter = varargin{4};
elseif nargin > 4
    system = varargin{1};
    InvSet = system.invariantSet();
    Vcenter = zeros(1,system.nx);
    U = admisControl(system, InvSet);
end

M = abs(InvSet.A*InvSet.V' - InvSet.b) < 1e-8; % matrix of conditions on vertices

parfor i = 1:size(M,1)
    index = find(M(i,:)==1);    % index of vertices making the vertex set
    P(i) = Polyhedron([InvSet.V(index,:);Vcenter]); % create polyhedron with center and vertices
    P(i).Data.U = U(:,index);   % assign admissible control
    P(i).Data.K = P(i).Data.U*pinv(InvSet.V(index,:)');   % calculate the vertex controller params
end

% create vertex control funs
for i = 1:length(P)
    P(i).addFunction(AffFunction(P(i).Data.K,zeros(system.nu,1)), 'uv');
end

% put Polyhedrons into the union
Pu = PolyUnion('Set', P, 'Convex', true, 'Overlaps', false, 'Bounded', true, 'FullDim', true, 'Connected', true);

end

