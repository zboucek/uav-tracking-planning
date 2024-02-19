function [ref] = refTriTrajGen(t, Ts, N)
%REF_TRAJ_GEN Summary of this function goes here
%   Detailed explanation goes here

amp = 10;
system.nx = 2;
k = t/Ts;
ref = zeros(system.nx,length(t)+N+1);
ref(1,1:length(k)) = amp-20*abs((2*k/(0.5*length(t)))-1);
ref(1,length(k):end) = ref(1,length(k));

end