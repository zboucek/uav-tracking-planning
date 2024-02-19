function [K, L] = LQTrajStabilized2(system, N)
%LQ_TRAJ Returns parameters of LQ controller for following of N-step trajectory
%   based on  B. D. O. Anderson and J. B. Moore, Optimal Control. pp. 81:
%   Discrete-time tracking
%
% Input:
%   system - LTISystem with quadratic weights for state and input
%   N - number of elements in reference trajectory
% Output:
%   parameters K and L for controller where u(k) = K*x(k) + L*[xr(k);xr(k+1);...xr(N)]
%

% [K, S, P] = LQN(system, N);
[K,P,E] = dlqr(system.A,system.B,system.x.penalty.weight,system.u.penalty.weight);

K = -K;
L = zeros(system.nu,system.nx*(N-1));
lambda = system.A' + K'*system.B';
Lpart = -(system.B'*P*system.B + system.u.penalty.weight)\system.B';
% for i = 1:N-1
% 	L(:,system.nx*(i-1)+1:system.nx*(i)) = -(system.B'*P*system.B + system.u.penalty.weight)\...
%           (system.B'*((system.A' + K'*system.B')^(i-1)*(-system.x.penalty.weight)));
% end

for i = 1:N-1
	L(:,system.nx*(i-1)+1:system.nx*(i)) = Lpart*(-lambda^(i-1)*system.x.penalty.weight);
end
% L(:,:,i) = -pinv(system.B'*S(:,:,i+1)*system.B + system.u.penalty.weight)*...
%           system.B'*(-temp*system.x.penalty.weight);

end