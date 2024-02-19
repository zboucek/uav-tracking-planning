clear; clc;

load('data/model/uav_phi_model_data.mat')
system.x.with('reference');
system.x.reference = 'free';

for i = 1:5
    Nmpc = i;
    mpc_phi = MPCController(system,Nmpc);
    expmpc_phi = mpc_phi.toExplicit;
    tree = BinTreePolyUnion(expmpc_phi.feedback);
    tree.toMatlab(convertStringsToChars("quad_mpc_tree_phi"+"_"+i),'primal','obj')
end
