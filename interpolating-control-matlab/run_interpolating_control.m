%% run the implicit interpolating control with the model from script

clc; clear; close all;

run('uav_planar_model.m');

% x0 = [-20;20;-pi/4;0;0;0];
% x0 = [5;2;angle_min;2;0;0.5*arate_min];

% computes a control invariant set for LTI system x^+ = A*x+B*u
load('data/vertex_ctrl/InvSet.mat');
% InvSet = P.copy;
% InvSet = system.invariantSet;
Omega = LQRSet;
Pu = vertexControl(system, InvSet);
% load('data/vertex_ctrl/quad_vertex_control_unionn.mat')

x0 = InvSet.V(1,:);
N = 50;
xk = zeros(N,system.nx);
rk = zeros(N,system.nx);
xk(1,:) = x0';
uk = zeros(N,system.nu);
uvk = zeros(N,system.nx);
ck = zeros(N,1);

options = sdpsettings('verbose',0,'cachesolvers',1);  % setup LP-solver

for j = 2:N
    %[uk(j-1,:), ck(j-1), rk(j-1,:)] = implicitInterpolatingControl(system, xk(j-1,:), Pu, InvSet, Omega);
    % optimization variables
c = sdpvar(1, 1);
rv = sdpvar(system.nx,1);
J = c;

C = [InvSet.A*rv <= c*InvSet.b; LQRSet.A*(xk(j-1,:)'-rv) <= (1-c)*LQRSet.b; 0 <= c <= 1];

options = sdpsettings('solver','linprog','verbose',0,'cachesolvers',1);  % setup LP-solver
    
optimize(C, J, options);

rvk = value(rv);

r0 = xk(j-1,:)' - rvk;

ck(j-1,:) = value(c);
if ck(j-1,:) == 0
    uvk = zeros(system.nu,1);
else
    uvk=Pu.feval(rvk, 'uv');
end

uk(j-1,:) = system.LQRGain*r0 + uvk(1:system.nu,1);
    
    xk(j,:) = system.A*xk(j-1,:)' + system.B*uk(j-1,:)';
end

save('output/quad_output_implicit.mat','xk', 'uk', 'ck', 'rk');
% figure;
% hold on;
% stairs(xk)
% xlabel('k');
% ylabel('state');
% % for i = 1:system.nx
% %     stairs(xk(:,i))
% % end

figure;
hold on;
stairs(ck)
xlabel('k');
ylabel('c(k)');

% % figure;
% % hold on;
% % stairs(rk(:,1));
% % stairs(rk(:,2));
% % xlabel('k');
% % ylabel('r(k)');
% 
% figure;
% hold on;
%     stairs(uk+[0,0])
% xlabel('k');
% ylabel('u(k)');
