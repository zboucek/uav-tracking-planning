function [e2IC_out, relJ] = getE2IC(varargin)
%GETE2IC find improved interpolating controller that satisfy quadratic
%criterion best
%
% OUTPUT: explicit solution to the improved interpolating control:
% solution.Set is polyhedron array with state-space solution and affine 
% function 'u' with control law
%         relJ - relative error compared to MPC
% INPUT:  LTISystem, mpcFun - explicitMPC for comparison, file - string
% with name of file for save data
if nargin < 2
    e2IC_out = NaN;
    relJ = NaN;
    warning('wrong input!')
    return;
elseif nargin == 2
    system = varargin{1};
    mpcFun = varargin{2};
    file = NaN;
elseif nargin == 3
    system = varargin{1};
    mpcFun = varargin{2};
    file = varargin{3};
elseif nargin > 3
    system = varargin{1};
    mpcFun = varargin{2};
end

InvSet = system.invariantSet;

x0 = InvSet.V';
Nx0 = length(x0);
Nsim = 501;
xk = zeros(system.nx,Nsim,Nx0);
uk = zeros(system.nu,Nsim,Nx0);
J_mpc = zeros(1,Nx0);
% calc criterion for MPC
for i = 1:Nx0
    xk(:,1,i) = x0(:,i);
    for j = 2:Nsim
        u_mpc = mpcFun(xk(:,j-1,i));
        uk(1:system.nu,j-1,i) = u_mpc(1,:);
        xk(:,j,i) = system.A*xk(:,j-1,i) + system.B*uk(1:system.nu,j-1,i);
        J_mpc(i) = J_mpc(i) + xk(:,j,i)'*system.x.penalty.weight*xk(:,j,i) + uk(1:system.nu,j-1,i)'*system.u.penalty.weight*uk(1:system.nu,j-1,i);
    end
end
J_mpc_mean = mean(J_mpc);

% get array of additional set for vertex control
OmegaS = invariantControlledUnionN(system);
for i = length(OmegaS):-1:1
    if InvSet <= OmegaS(i)
        OmegaS(i) = [];
    else
        break;
    end
end

U = admisControl(system,InvSet); % admissible control for invariant set
LQRSet = system.LQRSet;
e2IC = [];  % array for saving controllers
idxOmegaS = []; % array for saving controllers' index
for i = 1:length(OmegaS)
    sprintf('Controller %d/%d...',i,length(OmegaS))
    try
        % try to find controller
        e2IC_temp = explicitDoubleInterpolatingControl(system, InvSet, OmegaS(i),LQRSet,U);
        e2IC = [e2IC, e2IC_temp];
        idxOmegaS = [idxOmegaS, i];
    catch
        warning('Problem with calculation of e2IC.  Controller omitted, searching for the next one.');
    end
end

xk = zeros(system.nx,Nsim,Nx0);
uk = zeros(system.nx,Nsim,Nx0);
J_2ic = zeros(length(OmegaS),Nx0);
% test controllers (evaluate on criterion) with linear system
for k = 1:length(OmegaS)
    try
        for i = 1:Nx0
                xk(:,1,i) = x0(:,i);
                for j = 2:Nsim
                    uk(1:system.nu,j-1,i) = e2IC(k).feval(xk(:,j-1,i),'u','tiebreak', 'u');
                    xk(:,j,i) = system.A*xk(:,j-1,i) + system.B*uk(1:system.nu,j-1,i);
                    J_2ic(k,i) = J_2ic(k,i) + xk(:,j,i)'*system.x.penalty.weight*xk(:,j,i) + uk(1:system.nu,j-1,i)'*system.u.penalty.weight*uk(1:system.nu,j-1,i);
                end
        end
    catch
        warning('Problem with e2IC, trying next.');
        J_2ic(k,:) = Inf*ones(1,Nx0);
    end
end

% mean of criterion
J_2ic_mean = mean(J_2ic,2);
[J_ic2_mean_val,index] = min(J_2ic_mean); % find best controller

e2IC_out = e2IC(index).copy;    % copy controller to output variable
relJ = ((J_ic2_mean_val/J_mpc_mean)-1)*100; % compute relative error to MPC

if isstring(file)
    %save output to file
    save(file,'e2IC','index','J_2ic','idxOmegaS','J_mpc','xk','uk');
end
