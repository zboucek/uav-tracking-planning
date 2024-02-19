% LTI time-discrete planar model of quadrotor Crazyflie by Bitcraze
%
% dynamics in rotation with torque as input
%
% continuos dynamics:
% dx/dt = [dphi, tau/Ix]'
%
% state variable:
% x = [phi, dphi]'

run("uav_crazyflie_params.m")

Ts = 1e-3;
Nmpc = 47;

% set matrix of dynamics and input
A = zeros(2,2);
A(1,2) = 1;
B = zeros(2,1);
B(2,1) = 1/Ix;

% weights for LQ control
Q = diag([1/angle_max^2, 1/deg2rad(100)^2]);
R = diag(1/tau_max^2);

% create discrete LTI system of quadrotor
quad_sys = c2d(ss(A,B,eye(size(A)),0),Ts,'zoh');
system = LTISystem(quad_sys);
% fill in limits
system.x.min = [angle_min; arate_min];
system.x.max = [angle_max; arate_max];
system.u.min = tau_min;
system.u.max = tau_max;
system.x.penalty = QuadFunction(Q);
system.u.penalty = QuadFunction(R);
system.x.with('reference');
system.x.reference = 'free';

% K = system.LQRGain;         % LQ controler
LQRSet = invLQSet(system);    % and its invariant set
InvSet = system.invariantSet;

[K,S,E] = dlqr(system.A,system.B,Q,R);
% K1*x + K2*x_d(k) + K3*x_d(k+1)
K1 = -pinv(system.B'*S*system.B + R)*system.B'*(S*system.A);
K2 = -pinv(system.B'*S*system.B + R)*system.B'*(-(system.A' + K1'*system.B')*Q);
K3 = -pinv(system.B'*S*system.B + R)*system.B'*(-Q);

figure;
InvSet.plot
hold on
LQRSet.plot('Color','b')
% mpc with reference treat as
%mpc_phi.evaluate(x0, 'x.reference', xref);
