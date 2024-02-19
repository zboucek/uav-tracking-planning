% LTI time-discrete planar model of quadrotor UAV Ascending Technologies
% Hummingbird
%
% continuos dynamics:
% dx/dt = [x4, x5, x6, -g*x3, -g + u1/m, u2/Ix]'
%
% state variable:
% x = [y, z, phi, dy, dz, dphi]'

kT = 3.2032e-06;    % rotor thrust coefficient
ktau = 1.33191e-7;  % rotor torque coefficient
rad_max = (2*pi*64*200)/60; % maximal angular rate of rotor
rad_min = (2*pi*64*18)/60;  % minimal angular rate of rotor

g = 9.81;   % gravitational acceleration
m = 0.5;    % mass of quadrotor
Ix = 6.4*1e-3;  % inertia of quadrotor aroud x-axis
Ts = 1e-1;  % sampling period

% limit thrust
f_max = 0.8*4*kT*rad_max^2;
f_min = 1.1*4*kT*rad_min^2;
% limit torque
tau_max = 0.5*kT*(-rad_min^2+rad_max^2);
tau_min = -tau_max;

% limit angle
angle_max = deg2rad(52);
angle_min = -angle_max;
% unlimited angular rate
arate_max = Inf;
arate_min = -arate_max;

% limit area of operation with floor
y_max = 100;
y_min = -y_max;
z_max = 100;
z_min = -z_max;

% limit speed
vy_max = 15;
vy_min = -vy_max;
Dz = 0.5*1.29*0.2*0.1;               % drag coefficient in z
vz_max = sqrt(1/Dz*(-m*g + f_max));  % calculated according to drag
vz_min = -sqrt(-1/Dz*(-m*g + f_min));

% set matrix of dynamics and input
A = zeros(6,6);
A(1,4) = 1;
A(2,5) = 1;
A(3,6) = 1;
A(4,3) = -g;
B = zeros(6,2);
B(5,1) = 1/m;
B(6,2) = 1/Ix;

% weights for LQ control
Q = diag([1/0.5^2, 1/0.5^2, 1/deg2rad(5)^2, 1/(0.5*vy_max)^2, 1/(0.5*vz_max)^2, 1/deg2rad(10)^2]);
R = diag([1/f_max^2, 1/tau_max^2]);

% Q = 1e5*eye(6);
% R = 1e-5*eye(size(B,2));

% create discrete LTI system of quadrotor
quad_sys = c2d(ss(A,B,eye(size(A)),0),Ts,'zoh');
system = LTISystem(quad_sys);
% fill in limits
system.x.min = [y_min; z_min; angle_min; vy_min; vz_min; arate_min];
system.x.max = [y_max; z_max; angle_max; vy_max; vz_max; arate_max];
system.u.min = [-m*g+f_min; tau_min];
system.u.max = [f_max-m*g; tau_max];
system.x.penalty = QuadFunction(Q);
system.u.penalty = QuadFunction(R);

K = system.LQRGain;         % LQ controler
LQRSet = invLQSet(system);    % and its invariant set
% LQRSet == system.LQRSet   % inv set test
