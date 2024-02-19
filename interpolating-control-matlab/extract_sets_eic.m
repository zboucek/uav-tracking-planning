folder = "data/model";
% names = ["phi_0.01_room","y_0.01_room","z_0.01_room"];
% names = ["phi_0.01_room_hummingbird","y_0.01_room_hummingbird","z_0.01_room_hummingbird"];
names = ["phi_0.01_room_crazyflie","y_0.01_room_crazyflie","z_0.01_room_crazyflie"];
type = "eic_lqr";

load(folder+"/"+type+"_"+names(1)+"_params.mat")
set_h_f_A = LQ1.invSet.A;
set_h_f_b = LQ1.invSet.b;
load(folder+"/"+type+"_"+names(2)+"_params.mat")
set_h_y_A = LQ1.invSet.A;
set_h_y_b = LQ1.invSet.b;
set_m_y_A = LQ2.invSet.A;
set_m_y_b = LQ2.invSet.b;
set_l_y_A = LQ3.invSet.A;
set_l_y_b = LQ3.invSet.b;
load(folder+"/"+type+"_"+names(3)+"_params.mat")
set_h_z_A = LQ1.invSet.A;
set_h_z_b = LQ1.invSet.b;
set_m_z_A = LQ2.invSet.A;
set_m_z_b = LQ2.invSet.b;
set_l_z_A = LQ3.invSet.A;
set_l_z_b = LQ3.invSet.b;

clearvars ans LQ1 LQ2 LQ3 Nlq type names
save(folder+"/"+"eic_sets_0.01_room_crazyflie.mat")