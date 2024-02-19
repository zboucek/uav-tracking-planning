clear; clc; close all
% name = ["phi","y","z"];
% name = ["phi_0.01","y_0.01","z_0.01"];
% name = ["phi_0.01_room","y_0.01_room","z_0.01_room"];
% name = ["phi_0.01_room_hummingbird","y_0.01_room_hummingbird","z_0.01_room_hummingbird"];
name = ["phi_0.01_room_crazyflie","y_0.01_room_crazyflie","z_0.01_room_crazyflie"];
% name = ["y_0.01_room_crazyflie"];
% name = ["","y_0.01_weights","z_0.01_weights"];
for i = 1:length(name)
    load('data/model/uav_'+name(i)+'_model_data.mat');
    Nlq = 8/Ts;
    timeLQ1 = tic;
    if i == 1
        [K,L] = LQReg(system);
    else
        [K,L] = LQTrajStabilized(system,Nlq+1);
    end
    LQ1.time = toc(timeLQ1);
    L = reshape(L,1,[]);
    LQ1.K = K;
    LQ1.L = L;
    LQ1.system = system.copy;
    timeLQ1set = tic;
    LQ1.invSet = invLQSet(LQ1.system);
    LQ1.timeSet = toc(timeLQ1set);
    if i > 1
        LQ2.system = system.copy;
        LQ2.system.x.penalty = QuadFunction(diag((system.x.max).^-2));
        LQ2.system.u.penalty = QuadFunction(diag((system.u.max).^-2));
        timeLQ2 = tic;
        [K,L] = LQReg(LQ2.system);
        LQ2.time = toc(timeLQ2);
        L = reshape(L,1,[]);
        LQ2.K = K;
        LQ2.L = L;
        timeLQ2set = tic;
        LQ2.invSet = invLQSet(LQ2.system);
        LQ2.timeSet = toc(timeLQ2set);
        
        LQ3.system = system.copy;
        LQ3.system.x.penalty = QuadFunction(diag((system.x.max).^-2));
        LQ3.system.u.penalty = QuadFunction(10*diag((system.u.max).^-2));
        timeLQ3 = tic;
        [K,L] = LQReg(LQ3.system);
        LQ3.time = toc(timeLQ3);
        L = reshape(L,1,[]);
        LQ3.K = K;
        LQ3.L = L;
        timeLQ3set = tic;
        LQ3.invSet = invLQSet(LQ3.system);
        LQ3.timeSet = toc(timeLQ3set);
        figure
        InvSet.plot
        hold on
        LQ3.invSet.plot
        LQ2.invSet.plot
        LQ1.invSet.plot
    end
    if i == 1
        save('data/model/eic_lqr_'+name(i)+'_params.mat','LQ1','Nlq');
    else
        save('data/model/eic_lqr_'+name(i)+'_params.mat','LQ1','LQ2','LQ3','Nlq');
    end
end