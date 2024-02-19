%% Analysis of simulation data and computational time tests in case of UAV trajectory tracking
clc; clear; close all;
% x = [y, z, phi, dy, dz, dphi]'

cd ..
addpath('data/model');
cd sim_traj

name = ["step","sine"];
files = ["sim_traj_ic","sim_traj_mpc","sim_traj_mpc_block"];

load('uav_y_0.01_model_data.mat','system','vy_max')
system_y = system.copy();
load('uav_z_0.01_model_data.mat','system','vz_max')
system_z = system.copy();
load('uav_phi_0.01_model_data.mat','system')
system_phi = system.copy();
Q = diag([1/0.5^2,1/0.5^2,1/deg2rad(10)^2,1/(0.5*vy_max)^2,1/(0.5*vz_max)^2,1/deg2rad(100)^2]);
R = diag([system_z.u.penalty.weight;system_phi.u.penalty.weight]);

for j = 1:length(files)
    for n = 1:length(name)
        % 'state','control','comp_time_y','comp_time_z','ref_y','ref_z',
        % 'control_y','control_z','control_phi'
        load("output/"+files{j}+"_"+n+"_out.mat");
        ref = zeros(size(state.signals.values));
        for k=1:length(ref_y)
            ref((k-1)*10+1:k*10,1) = ref_y(1,k);
            ref((k-1)*10+1:k*10,2) = ref_z(1,k);
        end
        J = 0;
        e = zeros(size(state.signals.values));
        for k = 1:length(state.signals.values)
            e(k,:) = state.signals.values(k,:) - ref(k,:);
            J = J + e(k,:)*Q*e(k,:)' + control.signals.values(k,:)*R*control.signals.values(k,:)';
        end
        t = state.time;
        save("eval/"+files{j}+"_"+n+"_eval.mat",'J','e','t');
    end
end
