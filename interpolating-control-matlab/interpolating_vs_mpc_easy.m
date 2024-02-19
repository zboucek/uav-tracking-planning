%% Compare MPC, IC and improved IC, based on Example 3.4 (p62)
% 
% discrete-time LTI system:
%   x(k+1) = [1 1; 0 1] x(k) + [1; 0.3] u(k)
% N = 14, Q = I and R = 1
% constraints:
%  -10<= x1 <= 10, -5 <= x2 <= 5, -1 <= u <= 1
%

clc; clear; close all;

load('results/test/easy_model_data.mat');

Nsim = 30;

x0 = InvSet.V';
x0(:,10) = [];
Nx0 = length(x0);

xk = zeros(system.nx,Nsim,Nx0);
rk = zeros(system.nx,Nsim,Nx0);
uk = zeros(1,Nsim,Nx0);
ck = zeros(1,Nsim,Nx0);
for i = 1:Nx0
    xk(:,1,i) = x0(:,i);

    options = sdpsettings('solver','linprog','verbose',0);

    for j = 2:Nsim
        [uk(1,j-1,i), ck(1,j-1,i), rk(:,j-1,i)] = implicitInterpolatingControl(system, xk(:,j-1,i), vertex, InvSet, LQRSet);
        xk(:,j,i) = A*xk(:,j-1,i) + B*uk(1,j-1,i);
    end
end

save('results/test/ic_easy','xk','ck','uk','rk');

xk = zeros(system.nx,Nsim,Nx0);
uk = zeros(1,Nsim,Nx0);
for i = 1:Nx0
    xk(:,1,i) = x0(:,i);
    for j = 2:Nsim
        uk(1,j-1,i) = mpc.evaluate(xk(:,j-1,i));
        xk(:,j,i) = A*xk(:,j-1,i) + B*uk(1,j-1,i);
    end
end

save('results/test/mpc_easy','xk','uk');

xk = zeros(system.nx,Nsim,Nx0);
uk = zeros(1,Nsim,Nx0);
ck1 = zeros(1,Nsim,Nx0);
ck2 = zeros(1,Nsim,Nx0);
for i = 1:Nx0
    xk(:,1,i) = x0(:,i);

    options = sdpsettings('solver','linprog','verbose',0);

    for j = 2:Nsim
        [uk(1,j-1,i), ck1(1,j-1,i), ck2(1,j-1,i),] = implicitDoubleInterpolatingControl(system, xk(:,j-1,i), vertex, InvSet, OmegaS, LQRSet, vertexS);
        xk(:,j,i) = A*xk(:,j-1,i) + B*uk(1,j-1,i);
    end
end

save('results/test/ic2_easy','xk','ck1','ck2','uk');

eIC = explicitDoubleInterpolatingControl(system, InvSet, OmegaS);
xk = zeros(system.nx,Nsim,Nx0);
uk = zeros(1,Nsim,Nx0);
ck1 = zeros(1,Nsim,Nx0);
ck2 = zeros(1,Nsim,Nx0);
for i = 1:Nx0
    xk(:,1,i) = x0(:,i);

    for j = 2:Nsim
        uk(1,j-1,i) = eIC.feval(xk(:,j-1,i),'u','tiebreak', 'u');
        ck1(1,j-1,i) = eIC.feval(xk(:,j-1,i),'c1','tiebreak', 'c1');
        ck2(1,j-1,i) = eIC.feval(xk(:,j-1,i),'c2','tiebreak', 'c2');
        xk(:,j,i) = A*xk(:,j-1,i) + B*uk(1,j-1,i);
    end
end

save('results/test/ic2e_easy','xk','ck1','ck2','uk');
