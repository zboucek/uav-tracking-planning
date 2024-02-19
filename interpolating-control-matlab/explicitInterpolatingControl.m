function [solution] = explicitInterpolatingControl(varargin)
%EXPLICITDOUBLEINTERPOLATINGCONTROL Returns explicit solution to the
%interpolating control
%
% OUTPUT: explicit solution to the interpolating control: solution.Set
% is polyhedron array with state-space solution and affine fuction 'u' with
% control law and c as interpolating coefficients
% INPUT:  LTISystem, N - Linear Time Invariant system, N is number of N-step invariant controlled set 

if nargin < 1
    solution = NaN;
    warning('wrong input!')
    return;
elseif nargin == 1
    system = varargin{1};
    InvSet = system.invariantSet();
    LQRSet = system.LQRSet;
elseif nargin == 2
    system = varargin{1};
    InvSet = varargin{2};
    LQRSet = system.LQRSet;
elseif nargin == 3
    system = varargin{1};
    InvSet = varargin{2};
    LQRSet = varargin{3};
elseif nargin > 3
    system = varargin{1};
    InvSet = system.invariantSet();
    LQRSet = system.LQRSet;
end

% clean optimizer variables and set solver
yalmip('clear');
mptopt('plpsolver','PLCP');

% optimization variables
c = sdpvar(1, 1);
rv = sdpvar(system.nx,1);

% parameter
x = sdpvar(system.nx,1);

% objective function
J = c;

% constraints
C = [InvSet.A*rv <= c*InvSet.b; LQRSet.A*(x-rv) <= (1-c)*LQRSet.b; 0 <= c <= 1, system.x.min <= x <= system.x.max];

% solve parametric linear program
PArray = solveICpLP(C,J,x,[rv;c],InvSet);

% calc admissible control on vertices of invariant set InvSet
U = admisControl(system, InvSet);

for i = 1:length(PArray)
    if LQRSet == PArray(i)
        % set with LQ as controller
        PArray(i).addFunction(AffFunction(system.LQRGain,zeros(system.nu,1)), 'u');
        PArray(i).addFunction(AffFunction(zeros(1,system.nx),zeros(1,1)), 'c');
    else
        PArray(i).Data.U = [];   % create structure for admissible control
        PArray(i).Data.c = [];
        for j = 1:size(PArray(i).V,1)
            if LQRSet.contains(PArray(i).V(j,:)')
                % calc admissible control according to LQ law
                PArray(i).Data.U = [PArray(i).Data.U, system.LQRGain*PArray(i).V(j,:)'];
                PArray(i).Data.c = [PArray(i).Data.c, 0];
            else
                % find admissible control for the vertex on C_N
                M = sum(abs(InvSet.V - PArray(i).V(j,:)),2)<1e-8;
                index = M == 1;
                if sum(index) > 0
                    PArray(i).Data.U = [PArray(i).Data.U, U(:,index)];
                    PArray(i).Data.c = [PArray(i).Data.c, 1];
                end
            end
        end
        % calc the control function for C_N\Omega_max
        ctrl = PArray(i).Data.U*pinv([PArray(i).V';ones(1,size(PArray(i).V,1))]);
        PArray(i).addFunction(AffFunction(ctrl(1:system.nu,1:system.nx),ctrl(1:system.nu,system.nx+1)), 'u');
        cval = PArray(i).Data.c*pinv([PArray(i).V';ones(1,size(PArray(i).V,1))]);
        PArray(i).addFunction(AffFunction(cval(1,1:system.nx),cval(1,system.nx+1)), 'c');    
    end
end

% create union from array of polyhedrons
solution = PolyUnion('Set', PArray, 'Convex', true, 'Overlaps', false, 'Bounded', true, 'FullDim', true, 'Connected', true);
end