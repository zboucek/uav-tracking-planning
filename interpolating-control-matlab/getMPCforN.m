function [eMPC, idxN] = getMPCforN(varargin)
%GETMPCFORN Retrun explicit MPC with control horizon 1:50
%
% OUTPUT: eMPC explicit MPCs - structures with controllers from MPT3
%         idxN - N of MPC's horizon 
%
% INPUT:  LTISystem
%         string with file name - save there eMPC and generate m-file with
%         controller function
if nargin < 1
    eMPC = NaN;
    idxN = NaN;
    warning('wrong input!')
    return;
elseif nargin == 1
    system = varargin{1};
    file = NaN;
elseif nargin == 2
    system = varargin{1};
    file = varargin{2};
elseif nargin > 2
    system = varargin{1};
    file = NaN;
end

Nmpc = 1:50;

eMPC(Nmpc(end)) = EMPCController(system,1);
idxN = NaN*zeros(1,Nmpc(end));
parfor i = 1:Nmpc(end)
    fprintf('Controller %d/%d...\n',i,Nmpc(end));
    try
        eMPC(i) = EMPCController(system,Nmpc(i));
        idxN(i) = i;
    catch
        warning('Problem with calculation of eMPC.  Controller omitted, searching for the next one.');
    end
end

for i = 1:Nmpc(end)
    try
        if isstring(file)
            eMPC(i).feedback.toMatlab(char(file+"_mpc_"+i),'primal','first-region');
        end
    catch
        warning('Problem with genereting of eMPC function.');
    end
end

if isstring(file)
    save(file,'eMPC','idxN');
end