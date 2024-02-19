function [K, L] = LQReg(system)
%LQ_TRAJ Returns parameters of LQ controller for control to setpoint

%   based on  B. D. O. Anderson and J. B. Moore, Optimal Control. pp. 81:
%   Discrete-time tracking
%
% Input:
%   system - LTISystem with quadratic weights for state and input
%   N - number of elements in reference trajectory
% Output:
%   parameters K and L for controller where u(k) = K*x(k) + L*xr
%

% [K, S, P] = LQN(system, N);
[K,S,E] = dlqr(system.A,system.B,system.x.penalty.weight,system.u.penalty.weight);

K = -K;
L = zeros(system.nx,system.nx);

% L = [sum_i=1^N (A'+K*B')^(i-1)]
i = 1; N = 1e6;
while i<N
    Ltemp = (system.A' + K'*system.B')^(i-1);
    if sum(sum(abs(Ltemp)))==0%<1e-8
        break;
    end
	L = L + Ltemp;
    i = i + 1;
end
% final modification to gain compensation
% S  = S + system.x.penalty.weight;
L = -(system.B'*S*system.B + system.u.penalty.weight)\...
          (system.B'*(L*(-system.x.penalty.weight)));
end