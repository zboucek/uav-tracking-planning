cd tbxmanager;
tbxmanager restorepath;
cd ..
cd interpolating_control;
mkdir output;

MetaParPool('open');
run('uav_planar_model.m');
InvSet = system.invariantSet;
save('output/quad_c_n','InvSet');

N = 15;
P = invariantControlledSetN(system,N,LQRSet,"output/quad_c");
save('output/quad_invSet10','P');

boolproj = zeros(1,N);
for i = 1:N
    load("output/quad_c_"+i+".mat");
    boolproj(i) = P.projection([3,6]) == InvSet.projection([3,6])
end
save('output/quad_boolproj','boolproj');

MetaParPool('close');
quit;
