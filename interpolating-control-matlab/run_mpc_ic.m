clc; clear;
name = ["phi","y","z"];
n = 2;
load('data/model/uav_'+name(n)+'_model_data.mat')
load('data/model/lqr_'+name(n)+'_params.mat')

% rk = [sine_dt_traj(1,:);zeros(2,length(sine_dt_traj))];

rk = [zeros(1,100),ones(1,4900);zeros(1,5000)];
% rk = sine_traj;
% rk = step_traj;

% x0 = InvSet.V(1,:)';
x0 = zeros(system.nx,1);
Nsims = 1;
N = 80;
xk = zeros(system.nx,N);
xk(:,1) = x0';
uk = zeros(system.nu,N);
ck = zeros(1,N);

reftraj = reshape(rk,1,[]);

setpoint = sdpvar(system.nx,1);
x = sdpvar(system.nx,1);
c = sdpvar(1,1);
rv = sdpvar(system.nx,1);
% setpoint = traj(1:LQ1.system.nx);

% optimization variables
J = c;

C = [LQ2.invSet.A*rv <= c*LQ2.invSet.b; LQ1.invSet.A*(x-rv) <= (1-c)*(LQ1.invSet.b+LQ1.invSet.A*setpoint); 0 <= c; c <= 1];

ic_parameters_in = {x, setpoint};
ic_solutions_out = {c, rv};
options = sdpsettings('solver','gurobi','verbose',0,'cachesolvers',1);  % setup LP-solver
ic_controller = optimizer(C, J, options, ic_parameters_in, ic_solutions_out);
%     options = sdpsettings('solver','linprog','verbose',0,'cachesolvers',1);  % setup LP-solver
%     options = sdpsettings('solver','sedumi','sedumi.eps',1e-16,'sedumi.bigeps',1e-10,'verbose',0,'cachesolvers',1);  % setup LP-solver
time  = zeros(N-1,Nsims);
tic;
for j = 1:Nsims
    x = x0;
for i = 2:N
    i
    traj = reftraj((i-1)*system.nx+1:(i-1)*system.nx+(N)*system.nx)';
    setpoint = traj(1:LQ1.system.nx);
%     if LQ1.invSet.contains(x-setpoint)
%         % if x is in set for LQ control
%         u = LQ1.K*x + LQ1.L*traj;
%         cval = 0;
%         rvk = 0;
%         if u < LQ1.system.u.min
%             u = LQ1.system.u.min;
%         elseif u > LQ1.system.u.max
%             u = LQ1.system.u.max;
%         end
%     else
%         if LQ2.invSet.contains(x)
        ic_inputs = {x,setpoint};
        [ic_solutions,diagnostics] = ic_controller{ic_inputs};
        rvk = ic_solutions{2};
    
        r0 = x - rvk;
        cval = ic_solutions{1};
        uvk= LQ2.K*rvk + cval*LQ2.L*(traj);
        if uvk < LQ2.system.u.min
            uvk = LQ2.system.u.min;
        elseif uvk > LQ2.system.u.max
            uvk = LQ2.system.u.max;
        end
        % projection of trajectory point into the set of LQ1: LQ1.invSet.project(traj(5:6,:)-setpoint).x
        u0k = LQ1.K*r0 + (1-cval)*LQ1.L*(traj);
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
    %%LP
%     else
%         u= LQ2.K*x + LQ2.L*(traj);
%         if u < LQ2.system.u.min
%             u = LQ2.system.u.min;
%         elseif u > LQ2.system.u.max
%             u = LQ2.system.u.max;
%         end
%         cval = 0;
%     end
    uk(:,i-1) = u;
    ck(:,i-1) = cval;
%     [uk(:,i-1),ck(:,i-1)] = implicitInterpolatingTrajStab(xk(:,i-1), LQ1, LQ2, traj);
    x = system.A*x + system.B*uk(:,i-1);
    xk(:,i) = x;
	time(i-1,j) = toc;
end
j
end

save('results/data/icr_step_time.mat','xk','uk','ck','rk','time','Nsims','N');

% Model data
A = system.A;
B = system.B;

nx = system.nx; % Number of states
nu = system.nu; % Number of inputs

% MPC data
N = 80;
Q = system.x.penalty.weight;
R = system.u.penalty.weight;

u = sdpvar(repmat(nu,1,N),ones(1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
r = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));

constraints = [];
objective = 0;
for k = 1:N
    objective = objective + (x{k}-r{k})'*Q*(x{k}-r{k}) + u{k}'*R*u{k};
    constraints = [constraints, x{k+1} == A*x{k}+B*u{k}];
    constraints = [constraints, system.u.min <= u{k}; u{k} <= system.u.max, system.x.min<=x{k+1}; x{k+1} <=system.x.max];
end
objective = objective + (x{N+1}-r{N+1})'*Q*(x{N+1}-r{N+1});

parameters_in = {x{1},[r{:}]};
solutions_out = {[u{:}], [x{:}]};
options = sdpsettings('solver','gurobi','verbose',0,'cachesolvers',1);  % setup LP-solver
controller = optimizer(constraints, objective,options,parameters_in,solutions_out);
x = x0;

xk = zeros(system.nx,N);
xk(:,1) = x;
uk = zeros(1,N);
time  = zeros(N-1,Nsims);
tic;
for j = 1:Nsims
x = x0;
xk(:,1) = x;
for i = 1:N-1
    i
    future_r = rk(:,i:i+N);%[10*sin(9*pi*(i:i+N)/N);3/5*pi*cos(9*pi*(i:i+N)/(N/3))];    
    inputs = {x,future_r};
    [solutions,diagnostics] = controller{inputs};    
    U = solutions{1};
    X = solutions{2};
    x = A*x + B*U(1);
    uk(:,i) = U(1);
    xk(:,i+1) = x;
    time(i,j) = toc;
end
j
end

save('results/data/mpc_step_time.mat','xk','uk','rk','time','Nsims','N');

% figure(1);
% stairs(xk');
% xlabel('k');
% ylabel('x_k');
% 
% figure(2);
% stairs(uk);
% xlabel('k');
% ylabel('u_k');
% 
% figure(3);
% stairs(ck);
% xlabel('k');
% ylabel('c_k');

% cd results/scripts