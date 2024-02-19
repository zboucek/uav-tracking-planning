function [ref] = refStepTrajGen(t, Ts, N)
%REF_TRAJ_GEN Summary of this function goes here
%   Detailed explanation goes here

step_max = 10;
step_min = -10;
system.nx = 2;
k = t/Ts;
ref = zeros(system.nx,length(t)+N+1);
ref(1,:) = step_min;
ref(1,round(10/Ts):end) = step_max;
% for i = 1:N+1
%     if k+i > (N+1)
%         ref(1,i) = step_max;
%     end
% end

end