
load('data/model/uav_phi_0.01_room_crazyflie_model_data.mat');
InvSet_phi = InvSet;
K_phi = K;
K_phi1 = K1;
K_phi2 = K2;
K_phi3 = K3;
LQRSet_phi = LQRSet;

load('data/model/uav_y_0.01_room_crazyflie_model_data.mat');
InvSet_y = InvSet;
K_y = K;
LQRSet_y = LQRSet;

load('data/model/uav_z_0.01_room_crazyflie_model_data.mat');
InvSet_z = InvSet;
K_z = K;
LQRSet_z = LQRSet;

save('data/vertex_ctrl/vertex_sets','InvSet_phi','InvSet_y','InvSet_z','K_phi','K_phi1','K_phi2','K_phi3','K_y','K_z','LQRSet_phi','LQRSet_y','LQRSet_z');