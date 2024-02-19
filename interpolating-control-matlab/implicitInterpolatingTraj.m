function [u, cval, rvk] = implicitInterpolatingTraj(x, LQ1, LQ2, traj, k)
%IMPLICITINTERPOLATINGREGULATOR Returns control value according to implicit
%interpolating control law with regulation to setpoint
%
% OUTPUT: u - value of control according to interpolating control law
%         cval - optimal value of c
%         rvk - linear program variable
% INPUT:  LTISystem, x, xr - linear time invariant system, current state of
%         system and setpoint
%         LTISystem, x, xk, Ntraj - trajectory length
%         LTISystem, x, xk, Ntraj, Pu - union with vertex control law
%         functions
%         LTISystem, x, xk, Ntraj, Pu, InvSet - invariant N-step controlled set
%         LTISystem, x, xk, Ntraj, Pu, InvSet, LQRSet - invariant controlled set
%         LTISystem, x, xk, Ntraj, Pu, InvSet, LQRSet, LQpar - LQ parameters
%         

setpoint = traj(1:LQ1.system.nx);
if LQ1.invSet(k).contains(x-setpoint)
    % if x is in set for LQ control
    u = LQ1.K(:,:,k)*x + LQ1.L*traj;
    cval = 0;
    rvk = 0;
    if u < LQ1.system.u.min
        u = LQ1.system.u.min;
    elseif u > LQ1.system.u.max
        u = LQ1.system.u.max;
    end
else
    % optimization variables
    c = sdpvar(1, 1);
    rv = sdpvar(LQ1.system.nx,1);
    J = c;

    C = [LQ2.invSet(k).A*rv <= c*LQ2.invSet(k).b; LQ1.invSet(k).A*(x-rv) <= (1-c)*(LQ1.invSet(k).b+LQ1.invSet(k).A*setpoint); 0 <= c <= 1];

%     options = sdpsettings('solver','linprog','verbose',0,'cachesolvers',1);  % setup LP-solver
    options = sdpsettings('solver','sedumi','sedumi.eps',1e-16,'sedumi.bigeps',1e-6,'verbose',0,'cachesolvers',1);  % setup LP-solver
    
    optimize(C, J, options);

    rvk = value(rv);
    
    r0 = x - rvk;
    cval = value(c);
    uvk= LQ2.K(:,:,k)*rvk + (cval)*LQ2.L*(traj);
    if uvk < LQ2.system.u.min
        uvk = LQ2.system.u.min;
    elseif uvk > LQ2.system.u.max
        uvk = LQ2.system.u.max;
    end
    u0k = LQ1.K(:,:,k)*r0 + (1-cval)*LQ1.L*(traj);
    if u0k < LQ1.system.u.min
        u0k = LQ1.system.u.min;
    elseif u0k > LQ1.system.u.max
        u0k = LQ1.system.u.max;
    end
%     x0 = (x-rvk)/(1-cval);
%     u0k = (1-cval)*LQPar.K*(x0 - setpoint);
    u = u0k + uvk;
    if u < LQ1.system.u.min
        u = LQ1.system.u.min;
    elseif u > LQ1.system.u.max
        u = LQ1.system.u.max;
    end
% else
%     u = NaN;
%     cval = NaN;
% end
end

