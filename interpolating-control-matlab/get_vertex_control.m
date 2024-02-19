%% run the implicit interpolating control with the model from script

clc; clear; close all;

% load('data/model/uav_y_model_data.mat');
load('data/model/uav_y_0.01_room_crazyflie_model_data.mat');
tic;
U = admisControl(system, InvSet);
vertex_y = vertexControl(system, InvSet, U);

save('data/vertex_ctrl/quad_y_vc','vertex_y');
toc;

% load('data/model/uav_z_model_data.mat');
load('data/model/uav_z_0.01_room_crazyflie_model_data.mat');
tic;
U = admisControl(system, InvSet);
vertex_z = vertexControl(system, InvSet, U);

save('data/vertex_ctrl/quad_z_vc','vertex_z');
toc;

% load('data/model/uav_phi_model_data.mat');
load('data/model/uav_phi_0.01_room_crazyflie_model_data.mat');
tic;
U = admisControl(system, InvSet);
vertex_phi = vertexControl(system, InvSet, U);

save('data/vertex_ctrl/quad_phi_vc','vertex_phi');
toc;

% vertex_phi.toMatlab

p = profile('info');
save myprofiledata p
% 
% cmap = hsv(Pu.Num);
% 
% if system.nx == 2
% figure;
% hold on;
% InvSet.projection([1,2]).plot;
% for i = 1:Pu.Num
% Pu.Set(i).projection([1,2]).plot('Color',cmap(i,:));
% end
% end
% 
% if system.nx == 3
% figure;
% hold on;
% InvSet.projection([1,3]).plot;
% for i = 1:Pu.Num
% Pu.Set(i).projection([1,3]).plot('Color',cmap(i,:));
% end
% figure;
% hold on;
% InvSet.projection([1,2]).plot;
% for i = 1:Pu.Num
% Pu.Set(i).projection([1,2]).plot('Color',cmap(i,:));
% end
% end
% 
% if system.nx == 4
% figure;
% hold on;
% InvSet.projection([1,3]).plot;
% for i = 1:Pu.Num
% Pu.Set(i).projection([1,3]).plot('Color',cmap(i,:));
% end
% figure;
% hold on;
% InvSet.projection([2,4]).plot;
% for i = 1:Pu.Num
% Pu.Set(i).projection([2,4]).plot('Color',cmap(i,:));
% end
% end
% 
% if system.nx == 6
% figure;
% hold on;
% InvSet.projection([1,4]).plot;
% for i = 1:Pu.Num
% Pu.Set(i).projection([1,4]).plot('Color',cmap(i,:));
% end
% figure;
% hold on;
% InvSet.projection([2,5]).plot;
% for i = 1:Pu.Num
% Pu.Set(i).projection([2,5]).plot('Color',cmap(i,:));
% end
% figure;
% hold on;
% InvSet.projection([3,6]).plot;
% for i = 1:Pu.Num
% Pu.Set(i).projection([3,6]).plot('Color',cmap(i,:));
% end
% end