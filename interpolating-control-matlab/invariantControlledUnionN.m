function P = invariantControlledUnionN(varargin)
%INVARIANTCONTROLLEDUNIONN Computes N-step inveriant controlled set and
%returns all sets (1:N)
%   dependent on MPT3 (www.mpt3.org)
%   SYSTEM LTI system with defined dynamics and conditions
%   N number of steps
%   CN polyhedron CN for system


if nargin < 0
    error('Missing input!')
elseif nargin == 1
    system = varargin{1};
    N = Inf;
    lqrset = system.LQRSet;
    file = NaN;
elseif nargin == 2
    system = varargin{1};
    N = varargin{2};
    lqrset = system.LQRSet;
    file = NaN;
elseif nargin == 3
    system = varargin{1};
    N = varargin{2};
    lqrset = varargin{3};
    file = NaN;
elseif nargin == 4
    system = varargin{1};
    N = varargin{2};
    lqrset = varargin{3};
    file = varargin{4};
elseif nargin >= 5
    system = varargin{1};
    N = varargin{2};
    lqrset = varargin{3};
    file = NaN;
end
   
P = lqrset.copy;

i = 1;
while 1
    % expand set to Ptemp
    Ptemp = Polyhedron([P(i).A*([system.A,system.B]);eye(system.nx+system.nu);-eye(system.nx+system.nu)],[P(i).b;system.x.max;system.u.max;-system.x.min;-system.u.min]);
    Ptemp = Ptemp.projection(1:system.nx);
    if ~Ptemp.irredundantHRep
        % check redundancy and correct
        Ptemp.minHRep;
    end
    if P(i) == Ptemp || i == N+1
        % check if set was expanded, if not, we have maximal inv.set, break
        break;
    else
        P(i+1) = Ptemp.copy;
        if isstring(file)
            save(file+"_"+i+".mat",'P');
        end
    end
    sprintf('iteration %d...',i);
    i = i + 1;
end

P(i) = Ptemp.copy;
P.minHRep;
if isstring(file)
    save(file+"_"+i+".mat",'P');
end
end