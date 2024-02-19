% LTI time-discrete planar model of quadrotor Crazyflie by Bitcraze
%
% dynamics in rotation with torque as input
%
% continuos dynamics:
% dx/dt = [dphi, tau/Ix]'
%
% state variable:
% x = [phi, dphi]'

kT = 3.16e-10;    % rotor thrust coefficient
ktau = 7.94e-12;  % rotor torque coefficient
% rad_max = (2*pi*64*200)/60; % maximal angular rate of rotor
% rad_min = (2*pi*64*0)/60;  % minimal angular rate of rotor

arm = 0.0397;

g = 9.81;   % gravitational acceleration
m = 0.03;    % mass of quadrotor
Ix = 2.3951e-5;  % inertia of quadrotor aroud x-axis
Ts = 1e-2;  % sampling period

% limit thrust
f_max = 0.15*4;
f_min = 0.0;
% limit torque
tau_max = f_max*arm - f_min*arm;
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
vz_max = 5;  % calculated according to drag
vz_min = -5;
