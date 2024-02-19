% LTI time-discrete planar model of quadrotor Crazyflie by Bitcraze
%
% dynamics in z-plane with thrust as input
%
% continuos dynamics:
% dx/dt = [dz, az]'
%
% state variable:
% x = [z, dz]'

run("uav_crazyflie_params.m")

N = 35;
Nmpc = 100;

% set matrix of dynamics and input
A = zeros(2,2);
A(1,2) = 1;
B = zeros(2,1);
B(2,1) = 1;

% accz_max = f_max/m-g;
% accz_min = -g+f_min/m;
accz_max = 5;
accz_min = -5;

% weights for LQ control
Q = diag([1/0.1*(z_max)^2, 1/(0.5*vz_max)^2]);
R = diag(1/(accz_max)^2);

% create discrete LTI system of quadrotor
quad_sys = c2d(ss(A,B,eye(size(A)),0),Ts,'zoh');
system = LTISystem(quad_sys);
% fill in limits
system.x.min = [z_min; vz_min];
system.x.max = [z_max; vz_max];
system.u.min = accz_min;
system.u.max = accz_max;
system.x.penalty = QuadFunction(Q);
system.u.penalty = QuadFunction(R);

K = system.LQRGain;         % LQ controler
LQRSet = invLQSet(system);    % and its invariant set
% LQRSet == system.LQRSet   % inv set test
InvSet = system.invariantSet;

figure;
InvSet.plot
hold on
LQRSet.plot('Color','b')