% LTI time-discrete planar model of quadrotor UAV Ascending Technologies
% Hummingbird
%
% dynamics in rotation with torque as input
%
% continuos dynamics:
% dx/dt = [dphi, tau/Ix]'
%
% state variable:
% x = [phi, dphi]'

N = 11;
Nmpc = 11;

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
B(2,1) = 1/Ix;

% weights for LQ control
Q = diag([1/deg2rad(30)^2, 1/deg2rad(100)^2]);
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
