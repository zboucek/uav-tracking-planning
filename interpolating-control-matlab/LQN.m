function [K, P] = LQN(system, N)
%LQ_TRAJ Returns parameters of LQ controller for of N-step control
%   based on  B. D. O. Anderson and J. B. Moore, Optimal Control. pp. 31:
%   Discrete-time regulator
%
% Input:
%   system - LTISystem with quadratic weights for state and input
%   N - number of elements in reference trajectory
% Output:
%   parameters K(k) for controller where u(k) = K(k)*x(k)
%

% [K,S,E] = dlqr(system.A,system.B,system.x.penalty.weight,system.u.penalty.weight);

P = zeros(system.nx,system.nx,N);
% S = zeros(system.nx,system.nx,N);
% P_N = Q_N     (from OPS pres.)
P(:,:,N) = system.x.penalty.weight;
% S_N = Q_N + P_N   (2.4-11)
% S(:,:,N) = P(:,:,N)-system.x.penalty.weight;
for i = N-1:-1:1
    % P(t) = A'(t)[S(t+1)-S(t+1)*B(t)(B'(t)*S(t+1)*B(t)+R(t+1)]^-1...
    % *B'(t)*S(t+1)]A(t)    (2.4-12)
    P(:,:,i) = system.A'*P(:,:,i+1)*system.A + system.x.penalty.weight - (system.A'*P(:,:,i+1)*system.B)*pinv(system.B'*P(:,:,i+1)*system.B + system.u.penalty.weight)*(system.A'*P(:,:,i+1)*system.B)';
%     P(:,:,i) = system.A'*(S(:,:,i+1)-S(:,:,i+1)*system.B*pinv(system.B'*S(:,:,i+1)*system.B + system.u.penalty.weight)*system.B'*S(:,:,i+1))*system.A;
	% S(t)=Q(t)+P(t)    (2.4-11)
%     S(:,:,i) = P(:,:,i)-system.x.penalty.weight;
end

K = zeros(system.nu,system.nx,N-1);
for i = 1:N-1
    % K(t) = -[B'(t)*S(t+1)*B(t)+R(t+1)]^-1 *B'(t)S(t+1)A(t)    (2.4-10)
% 	K(:,:,i) = -pinv(system.B'*P(:,:,i+1)*system.B + system.u.penalty.weight)*system.B'*P(:,:,i+1)*system.A;
	K(:,:,i) = -(system.B'*P(:,:,i+1)*system.B + system.u.penalty.weight)\(system.A'*P(:,:,i+1)*system.B)';
end

end