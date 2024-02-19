yalmip('clear')
clear all

load data/model/uav_y_model_data.mat

% Model data
A = system.A;
B = system.B;
eps = 1;

nx = system.nx; % Number of states
nu = system.nu; % Number of inputs

% MPC data
N = 100;
Q = system.x.penalty.weight;
R = system.u.penalty.weight;

u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
r = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));

constraints = [];
objective = 0;
for k = 1:N
    objective = objective + (x{k}-r{k})'*Q*(x{k}-r{k}) + u{k}'*u{k};
    constraints = [constraints, x{k+1} == A*x{k}+B*u{k}];
    constraints = [constraints, -1 <= u{k}<= 1, -[10;5]<=x{k+1}<=[10;5]];
end
objective = objective + (x{N+1}-r{N+1})'*Q*(x{N+1}-r{N+1});

parameters_in = {x{1},[r{:}]};
solutions_out = {[u{:}], [x{:}]};
options = sdpsettings('solver','sedumi','sedumi.eps',1e-16,'sedumi.bigeps',1e-6,'verbose',0,'cachesolvers',1);  % setup LP-solver
controller = optimizer(constraints, objective,options,parameters_in,solutions_out);
x = [0;0];
clf;
hold on
xk = zeros(system.nx,N);
xk(:,1) = x;
uk = zeros(1,N);
for i = 1:N
    future_r = [sin(9*pi*(i:i+N)/N);3/5*pi*cos(9*pi*(i:i+N)/(N/3))];    
    future_r(2,:) = 0;
    inputs = {x,future_r};
    [solutions,diagnostics] = controller{inputs};    
    U = solutions{1};
    X = solutions{2};
    if diagnostics == 1
        error('The problem is infeasible');
    end
    x = A*x + B*U(1);
    if sum(U(1)<eps*system.u.min)>0 || sum(U(1)>eps*system.u.max)>0
        disp('porušeno omezení na řízení!');
        i
    end
    if sum(x<eps*system.x.min)>0 || sum(x>eps*system.x.max)>0
        disp('porušen stav systému!');
        i
    end
    uk(:,i) = U(1);
    xk(:,i+1) = x;
end