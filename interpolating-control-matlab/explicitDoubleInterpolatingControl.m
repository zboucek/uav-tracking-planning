function [solution] = explicitDoubleInterpolatingControl(varargin)
%EXPLICITINTERPOLATINGCONTROL Returns explicit solution to the
%improved interpolating control
%
% OUTPUT: explicit solution to the improved interpolating control:
% solution.Set is polyhedron array with state-space solution and affine 
% fuction 'u' with
% control law and c1 and c2 as interpolating coefficients
% INPUT:  LTISystem, N - Linear Time Invariant system, N is number of N-step invariant controlled set 

if nargin < 1
    solution = NaN;
    warning('wrong input!')
    return;
elseif nargin == 1
    system = varargin{1};
    InvSet = system.invariantSet();
    LQRSet = system.LQRSet;
    OmegaS = invariantControlledSetN(system,3,LQRSet);
    US = admisControl(system,OmegaS);
    U = admisControl(system,InvSet);
elseif nargin == 2
    system = varargin{1};
    InvSet = varargin{2};
    LQRSet = system.LQRSet;
    OmegaS = invariantControlledSetN(system,3,LQRSet);
    US = admisControl(system,OmegaS);
    U = admisControl(system,InvSet);
elseif nargin == 3
    system = varargin{1};
    InvSet = varargin{2};
    OmegaS = varargin{3};
    LQRSet = system.LQRSet;
    US = admisControl(system,OmegaS);
    U = admisControl(system,InvSet);
elseif nargin == 4
    system = varargin{1};
    InvSet = varargin{2};
    OmegaS = varargin{3};
    LQRSet = varargin{4};
    US = admisControl(system,OmegaS);
    U = admisControl(system,InvSet);
elseif nargin == 5
    system = varargin{1};
    InvSet = varargin{2};
    OmegaS = varargin{3};
    LQRSet = varargin{4};
    US = admisControl(system,OmegaS);
    U = varargin{5};
elseif nargin > 5
    system = varargin{1};
    InvSet = system.invariantSet();
    LQRSet = system.LQRSet;
    OmegaS = invariantControlledSetN(system,3,LQRSet);
    US = admisControl(system,OmegaS);
    U = admisControl(system,InvSet);
end
yalmip('clear');
mptopt('plpsolver','PLCP');
% optimization variables
c1 = sdpvar(1, 1);
c2 = sdpvar(1, 1);
rv = sdpvar(system.nx,1);
rs = sdpvar(system.nx,1);
% parameter
x = sdpvar(system.nx,1);
% cost function
J1 = c1;
J2 = c2;

C1 = [InvSet.A*rv <= c1*InvSet.b; OmegaS.A*(x-rv) <= (1-c1)*OmegaS.b; 0 <= c1 <= 1];
C2 = [OmegaS.A*rs <= c2*OmegaS.b; LQRSet.A*(x-rs) <= (1-c2)*LQRSet.b; 0 <= c2 <= 1];

% solve parametric linear program
PuV = solveICpLP(C1,J1,x,[rv;c1],InvSet);
PuS = solveICpLP(C2, J2, x, [rs;c2],OmegaS);

index = [];
for i = 1:length(PuV)
    if PuV(i) ~= OmegaS
        index = [index, i];
    end
end

Parray = [PuS.copy, PuV(index).copy];
for i = 1:length(Parray)
    if LQRSet == Parray(i)
        % set with LQ as controller
        Parray(i).addFunction(AffFunction(system.LQRGain,zeros(system.nu,1)), 'u');
        Parray(i).addFunction(AffFunction(zeros(1,system.nx),zeros(1,1)), 'c1');
        Parray(i).addFunction(AffFunction(zeros(1,system.nx),zeros(1,1)), 'c2');
    else
        Parray(i).Data.U = [];   % create structure for admissible control
        Parray(i).Data.c1 = [];
        Parray(i).Data.c2 = [];
        for j = 1:size(Parray(i).V,1)
            if LQRSet.contains(Parray(i).V(j,:)')
                % calc admissible control according to LQ law
                Parray(i).Data.U = [Parray(i).Data.U, system.LQRGain*Parray(i).V(j,:)'];
                Parray(i).Data.c1 = [Parray(i).Data.c1, 0];
                Parray(i).Data.c2 = [Parray(i).Data.c2, 0];
            else
                % find admissible control for the vertex on OmegaS
                M = sum(abs(OmegaS.V - Parray(i).V(j,:)),2)<1e-8;
                index = M == 1;
                if sum(index) > 0
                    Parray(i).Data.U = [Parray(i).Data.U, US(:,index)];
                    Parray(i).Data.c1 = [Parray(i).Data.c1, 0];
                    Parray(i).Data.c2 = [Parray(i).Data.c2, 1];
                else
                
                    % find admissible control for the vertex on C_N
                    M = sum(abs(InvSet.V - Parray(i).V(j,:)),2)<1e-8;
                    index = M == 1;
                    if sum(index) > 0
                        Parray(i).Data.U = [Parray(i).Data.U, U(:,index)];
                        Parray(i).Data.c1 = [Parray(i).Data.c1, 1];
                        Parray(i).Data.c2 = [Parray(i).Data.c2, 0];
                    end
                end
            end
        end
        % calc the control function for C_N\Omega_max
        ctrl = Parray(i).Data.U*pinv([Parray(i).V';ones(1,size(Parray(i).V,1))]);
        Parray(i).addFunction(AffFunction(ctrl(1:system.nu,1:system.nx),ctrl(1:system.nu,system.nx+1)), 'u');
        c1val = Parray(i).Data.c1*pinv([Parray(i).V';ones(1,size(Parray(i).V,1))]);
        c2val = Parray(i).Data.c2*pinv([Parray(i).V';ones(1,size(Parray(i).V,1))]);
        Parray(i).addFunction(AffFunction(c1val(1,1:system.nx),c1val(1,system.nx+1)), 'c1');
        Parray(i).addFunction(AffFunction(c2val(1,1:system.nx),c2val(1,system.nx+1)), 'c2');
    end
end

% create union from array of polyhedrons
solution = PolyUnion('Set', Parray, 'Convex', true, 'Overlaps', false, 'Bounded', true, 'FullDim', true, 'Connected', true);

