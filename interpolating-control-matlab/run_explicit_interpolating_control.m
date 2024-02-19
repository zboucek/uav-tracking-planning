clc; clear; close all;

% sys params
run('uav_planar_model.m');

% load maximal invarinat n-step controlled set
load('output/quad_c_1');
InvSet = P.copy;
sol = explicitInterpolatingControl(system, InvSet, LQRSet);

save('quad_explicit_sol','sol');

N = 30;
x = zeros(N,system.nx);
x(1,:) = x0';
uk = zeros(N,system.nu);
ck = zeros(N,1);
rk = zeros(N,system.nx+1);

for j = 2:N
    ck(j-1) = sol.xopt.feval(x(j-1,:)', 'obj');
    rk(j-1,:) = sol.xopt.feval(x(j-1,:)', 'primal');
    uk(j-1,:) = sol.xopt.feval(x(j-1,:)', 'u');
    x(j,:) = system.A*x(j-1,:)' + system.B*uk(j-1,:);
end

save('quad_output_explicit.mat','xk', 'uk', 'ck', 'rk','sol');

% PolyUnion(solution.xopt.Set).toMatlab('quad_explicit_interpolating_controller','u','first-region');