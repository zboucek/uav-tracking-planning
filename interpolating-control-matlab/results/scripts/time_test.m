%% test time consuption of UAV controllers
cd ../..
clear; clc;
addpath data/explicit_ic
addpath data/explicit_2ic
addpath data/explicit_mpc

load('data/vertex_ctrl/vertex_sets.mat')
load('data/vertex_ctrl/quad_y_vc.mat')
load('data/vertex_ctrl/quad_z_vc.mat')
load('data/vertex_ctrl/quad_phi_vc.mat')
load('data/vertex_ctrl/InvSet.mat')

N = 1e4;
load('data/vertex_ctrl/random_invset_points.mat');

% x = zeros(6,N);
% parfor i = 1:N
%     x(:,i) = InvSet.randomPoint;
% end
% save('data/vertex_ctrl/random_invset_points.mat','x');

tIC = tic;
for i = 1:N
    quad_ic(x(:,i));
end
timeIC = toc(tIC);
timeIC = round(timeIC,2,'decimals');

t2IC = tic;
for i = 1:N
    quad_2ic(x(:,i));
end
time2IC = toc(t2IC);
time2IC = round(time2IC,2,'decimals');

tMPC = tic;
for i = 1:N
    quad_mpc(x(:,i));
end
timeMPC = toc(tMPC);
timeMPC = round(timeMPC,2,'decimals');

percIC = round(((timeIC/timeMPC)-1)*100,2,'decimals');
perc2IC = round(((time2IC/timeMPC)-1)*100,2,'decimals');
save('results/timex','timeIC','timeMPC','time2IC','percIC','perc2IC');