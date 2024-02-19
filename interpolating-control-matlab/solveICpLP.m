function [PolyArray] = solveICpLP(C,J,param,var,Set)
%SOLVE2ICPLP Solve multi-parametric Linear Program (mpLP) for Interpolating
%Control
% Check if final solution PolyArray covers same area as Set
% if not solves mpLP for difference between solution and Set
% Input:
% C - constraints of mpLP
% J - criterion function
% param - parameter of mpLP
% var - mpLP variable
% Set - final solution area
% 
% Output:
% PolyArray - Array of solution partitions
%

plp = Opt(C, J, param, var);    % define mpLP
solution = plp.solve();         % solve mpLP
PolyArray = solution.xopt.Set.copy; % copy partitions of solution to output variable 
PuDiff = Set\PolyArray;         % calc difference of Set and solution
j = 1;
while sum(PuDiff.isFullDim) ~= 0    % while solution does not cover full area
    PuDiff = mergeNeighbours(PuDiff);   % merge neighbours of set difference
    for i = 1:length(PuDiff)
        % solve mpLP for missing solution
        C2 = [C;PuDiff(i).A*param <= PuDiff(i).b];
        plp2 = Opt(C2, J, param, var);
        solution2 = plp2.solve();
        PolyArray = [PolyArray, solution2.xopt.Set];
    end
    PuDiff = Set\PolyArray; % calc new difference
    j = j+1;
    if j == 3
        % if too much tries, try to change solver
        mptopt('plpsolver','MPLP');
    end
    if j == 50
        % not feasible, return partial solution
        mptopt('plpsolver','PLCP');
        break;
    end
end

if j >= 3
    mptopt('plpsolver','PLCP');
end

end

