% LTI time-discrete planar model of quadrotor UAV Ascending Technologies
% Hummingbird
%
% dynamics in y-plane with angle phi as input
%
% continuos dynamics:
% dx/dt = [dy, -g*phi]'
%
% state variable:
% x = [y, dy]'

N = 18;

kT = 3.2032e-06;    % rotor thrust coefficient
ktau = 1.33191e-7;  % rotor torque coefficient
rad_max = (2*pi*64*200)/60; % maximal angular rate of rotor
rad_min = (2*pi*64*18)/60;  % minimal angular rate of rotor

g = 9.81;   % gravitational acceleration
m = 0.5;    % mass of quadrotor
Ix = 6.4*1e-3;  % inertia of quadrotor aroud x-axis
Ts = 1e-2;  % sampling period

% limit thrust
f_max = 0.7*4*kT*rad_max^2;
f_min = 1.2*4*kT*rad_min^2;
% limit torque
tau_max = 0.5*kT*(-rad_min^2+rad_max^2);
tau_min = -tau_max;

% limit angle
angle_max = deg2rad(30);
angle_min = -angle_max;
% unlimited angular rate
arate_max = Inf;
arate_min = -arate_max;

% limit area of operation with floor
y_max = 2;
y_min = -y_max;
z_max = 2.5/2;
z_min = -z_max;

% limit speed
vy_max = 5;
vy_min = -vy_max;
Dz = 0.5*1.29*0.2*0.1;               % drag coefficient in z
vz_max = 5;  % calculated according to drag
vz_min = -5;

% set matrix of dynamics and input
A = zeros(2,2);
A(1,2) = 1;
B = zeros(2,1);
B(2,1) = -g;

% weights for LQ control
Q = diag([1/0.5^2, 1/(0.5*vy_max)^2]);
R = diag(1/(deg2rad(30))^2);

% create discrete LTI system of quadrotor
quad_sys = c2d(ss(A,B,eye(size(A)),0),Ts,'zoh');
system = LTISystem(quad_sys);
% fill in limits
system.x.min = [y_min; vy_min];
system.x.max = [y_max; vy_max];
system.u.min = angle_min;
system.u.max = angle_max;
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