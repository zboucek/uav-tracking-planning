% LTI time-discrete planar model of quadrotor Crazyflie by Bitcraze
%
% dynamics in y-plane with angle phi as input
%
% continuos dynamics:
% dx/dt = [dy, u]'
%
% state variable:
% x = [y, dy]'

run("uav_crazyflie_params.m")

N = 18;
Nmpc = 100;

% set matrix of dynamics and input
A = zeros(2,2);
A(1,2) = 1;
B = zeros(2,1);
B(2,1) = 1;
acc_max = 0.5*f_max/m*sin(angle_max);
acc_min = -acc_max;

% weights for LQ control
Q = diag([1/0.5*(y_max)^2, 1/(0.5*vy_max)^2]);
R = diag(1/(acc_max)^2);

% create discrete LTI system of quadrotor
quad_sys = c2d(ss(A,B,eye(size(A)),0),Ts,'zoh');
system = LTISystem(quad_sys);
% fill in limits
system.x.min = [y_min; vy_min];
system.x.max = [y_max; vy_max];
system.u.min = acc_min;
system.u.max = acc_max;
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