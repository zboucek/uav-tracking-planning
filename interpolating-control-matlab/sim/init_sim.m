clc; clear; close all;
% x = [y, z, phi, dy, dz, dphi]'

cd ..
addpath('data/explicit_ic');
addpath('data/explicit_2ic');
addpath('data/explicit_mpc');
addpath('data/vertex_ctrl');
addpath('sim');
addpath('.');

load('data/vertex_ctrl/vertex_sets.mat')
load('data/vertex_ctrl/quad_y_vc.mat')
load('data/vertex_ctrl/quad_z_vc.mat')
load('data/vertex_ctrl/quad_phi_vc.mat')
load('data/model/lqr_phi.mat')
cd sim

LQR_phi.gain = K;
LQR_phi.comp = L;

InvSet_phiA = InvSet_phi.A;
InvSet_yA = InvSet_y.A;
InvSet_zA = InvSet_z.A;

InvSet_phib = InvSet_phi.b;
InvSet_yb = InvSet_y.b;
InvSet_zb = InvSet_z.b;

LQRSet_phiA = LQRSet_phi.A;
LQRSet_yA = LQRSet_y.A;
LQRSet_zA = LQRSet_z.A;

LQRSet_phib = LQRSet_phi.b;
LQRSet_yb = LQRSet_y.b;
LQRSet_zb = LQRSet_z.b;

pid_att.Kp = 0.3;
pid_att.Kd = 0.003;
pid_att.Ki = 0.0001;

K = [K_y(1),K_z(1),K_phi(1),K_y(2),K_z(2),K_phi(2)];
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
x_min = [y_min;z_min;angle_min;vy_min;vz_min;arate_min];
x_max = [y_max;z_max;angle_max;vy_max;vz_max;arate_max];

u_min = [f_min; tau_min];
u_max = [f_max; tau_max];

i = 1;
j = 2;
k = 3;
x0 = 0.9*[InvSet_y.V(i,1)'; InvSet_z.V(j,1)'; InvSet_phi.V(k,1)'; InvSet_y.V(i,2)'; InvSet_z.V(j,2)'; InvSet_phi.V(k,2)'] ;

x0(3:end,:) = 0;
tsim = 5;