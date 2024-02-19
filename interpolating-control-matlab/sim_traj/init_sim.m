clc; clear; close all;
% x = [y, z, phi, dy, dz, dphi]'

cd ..
addpath('data/model');
addpath('sim_traj');
addpath('.');

load('data/model/lqr_phi_0.01_params_v2.mat')
load('data/model/uav_0.01_model_data.mat')
load('data/model/lqr_0.01_params.mat','Nlq_y','Nlq_z')

cd sim_traj

tsim = 20; % duration of simulation
x0 = zeros(1,system_y.nx+system_z.nx+system_phi.nx);
t_y = 0:Ts_y:tsim;
t_z = 0:Ts_z:tsim;


ref_y = refStepTrajGen(t_y, Ts_y, 2*Nlq_y);
ref_z = refStepTrajGen(t_z, Ts_z, 2*Nlq_z);
