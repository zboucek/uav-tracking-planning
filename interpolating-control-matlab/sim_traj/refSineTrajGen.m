function [ref] = refSineTrajGen(t, Ts, N)
%REF_TRAJ_GEN Summary of this function goes here
%   Detailed explanation goes here

amp = 10;
freq = pi/(0.5*length(t));
system.nx = 2;
k = t/Ts;
ref = zeros(system.nx,length(t)+N+1);
ref(1,1:length(k)) = amp*sin(freq*k);

end