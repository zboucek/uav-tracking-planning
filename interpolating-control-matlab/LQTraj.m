function [K, L] = LQTraj(system, N)
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

[K, P] = LQN(system, N);
% [K,S,E] = dlqr(system.A,system.B,system.x.penalty.weight,system.u.penalty.weight);

% K = -K;
% L = zeros(system.nu,system.nx,N);
% for i = 1:N-1
% 	L(:,:,i) = -pinv(system.B'*S(:,:,i)*system.B + system.u.penalty.weight)*...
%           system.B'*((system.A' + K(:,:,i)'*system.B')^(i-1)*(-system.x.penalty.weight));
% end
% L(:,:,N) = -pinv(system.B'*S(:,:,i)*system.B + system.u.penalty.weight)*...
%           system.B'*((system.A' + K(:,:,i)'*system.B')^(i-1)*(-system.x.penalty.weight));
% end
L=zeros(system.nu,system.nx,N-1);
for i = 1:N-1
    temp  = -eye(size(system.A));
    if i>1
        for j = 2:i
            temp = temp * (system.A' + K(:,:,j-1)'*system.B');
        end
    end
    L(:,:,i) = -(system.B'*P(:,:,i+1)*system.B + system.u.penalty.weight)\...
          system.B'*(temp-system.x.penalty.weight);
end