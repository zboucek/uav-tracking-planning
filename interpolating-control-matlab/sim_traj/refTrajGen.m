function [ref] = refTrajGen(t)
%REF_TRAJ_GEN Summary of this function goes here
%   Detailed explanation goes here
Nc = 100;
k = t/Ts;
ref = zeros(system.nx,1:Nc+1);
end