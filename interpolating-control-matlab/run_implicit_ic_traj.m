
clc; clear;
fig_x = figure(1);
fig_u = figure(2);
fig_c = figure(3);

clf(fig_x);
clf(fig_u);
clf(fig_c);

load data/model/uav_y_model_data.mat

N = 150;
% Ntraj = 301;
[K,L] = LQTraj(system,N+1);
L = reshape(L,1,[]);
LQ1.K = K;
LQ1.L = L;
LQ1.system = system.copy;
LQ1.invSet = invLQSets(LQ1.system,K);
LQ2.system = system.copy;
LQ2.system.x.penalty = QuadFunction(diag((system.x.max).^-2));
LQ2.system.u.penalty = QuadFunction(10*system.u.max);
[K,L] = LQTraj(LQ2.system,N+1);
% [K,L] = LQTraj(LQ2.system,N+1);
L = reshape(L,1,[]);
LQ2.K = K;
LQ2.L = L;
LQ2.invSet = invLQSet(LQ2.system);
% LQ2.invSet = invLQSets(LQ2.system,K);
% load lq_2nd_stable_n_150.mat

% reftraj = repmat(setpoint,N+Ntraj,1); %const
Nref = 10000;
reftraj = zeros(Nref,2);
% %sinewave x_1
% reftraj(:,1) = 4.5*sin((1:Nref)/5)+5;
% %sinewave x_2
% reftraj(:,2) = 0.9*cos((1:Nref)/5);
% % for j = 1:length(LQRSet.V)
% % x0 = LQRSet.V(j,:);
% % x0 = zeros(system.nx,1);
reftraj(1:floor(N/2),1) = system.x.min(1);
reftraj(floor(N/2)+1:end,1) = system.x.max(1);

reftraj = reshape(reftraj',1,[]);

x0 = LQ2.invSet.V(18,:)';
% x0 = InvSet.V(19,:)';
xk = zeros(system.nx,N);
xk(:,1) = x0;
uk = zeros(system.nu,N);
ck = zeros(1,N);

for i = 2:N
    traj = reftraj((i-1)*system.nx+1:(i-1)*system.nx+(N)*system.nx)';
    [uk(:,i-1),ck(:,i-1)] = implicitInterpolatingTraj(xk(:,i-1), LQ1, LQ2, traj, i-1);
    if sum(uk(:,i-1)<system.u.min)>0 || sum(uk(:,i-1)>system.u.max)>0
        disp('porušeno omezení na řízení!');
        i
    end
    xk(:,i) = system.A*xk(:,i-1) + system.B*uk(:,i-1);
    if sum(xk(:,i-1)<system.x.min)>0 || sum(xk(:,i-1)>system.x.max)>0
        disp('porušen stav systému!');
        i
    end
end
% end


fig_x = figure(1);
hold on
stairs(xk');
% stairs(reftraj(1:2:2*(N),:));
% stairs(reftraj(2:2:2*(N)+1,:));
for i = 1:system.nx
    stairs(reftraj(i:system.nx:system.nx*(N)+i-1));
end
xlabel('k');
ylabel('x_k');
legend('x_1','x_2','x_{1r}','x_{2r}');
hold off

fig_u = figure(2);
stairs(uk');
xlabel('k');
ylabel('u_k');

fig_c = figure(3);
stairs(ck);
xlabel('k');
ylabel('c_k');