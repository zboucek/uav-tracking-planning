folder = "data/model";
% names = ["phi_0.01_room","y_0.01_room","z_0.01_room"];
% names = ["phi_0.01_room_hummingbird","y_0.01_room_hummingbird","z_0.01_room_hummingbird"];
names = ["phi_0.01_room_crazyflie","y_0.01_room_crazyflie","z_0.01_room_crazyflie"];
type = "lqr";

load(folder+"/"+type+"_"+names(1)+"_params.mat")
set_h_f_A = LQ1.invSet.A;
set_h_f_b = LQ1.invSet.b;
% set_l_f_A = LQ2.invSet.A;
% set_l_f_b = LQ2.invSet.b;
load(folder+"/"+type+"_"+names(2)+"_params.mat")
set_h_y_A = LQ1.invSet.A;
set_h_y_b = LQ1.invSet.b;
set_l_y_A = LQ2.invSet.A;
set_l_y_b = LQ2.invSet.b;
load(folder+"/"+type+"_"+names(3)+"_params.mat")
set_h_z_A = LQ1.invSet.A;
set_h_z_b = LQ1.invSet.b;
set_l_z_A = LQ2.invSet.A;
set_l_z_b = LQ2.invSet.b;

clearvars ans LQ1 LQ2 Nlq type names
save(folder+"/"+"sets_0.01_room_crazyflie.mat")