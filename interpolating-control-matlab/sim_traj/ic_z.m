function ut = ic_z(currentx,reftraj)

persistent Controller
persistent LQ1
persistent LQ2

tstart = tic;
if isempty(Controller)
    % init the controller with data from files
    load('data/model/lqr_z_0.01_params.mat','LQ1','LQ2')

    % Avoid explosion of internally defined variables in YALMIP
    yalmip('clear')

    setpoint = sdpvar(LQ1.system.nx,1);
    x = sdpvar(LQ1.system.nx,1);
    c = sdpvar(1,1);
    rv = sdpvar(LQ1.system.nx,1);

    % optimization variables
    J = c;
    % optimization constraints
    C = [LQ2.invSet.A*rv <= c*LQ2.invSet.b; LQ1.invSet.A*(x-rv) <= (1-c)*(LQ1.invSet.b+LQ1.invSet.A*setpoint); 0 <= c; c <= 1];

    ic_parameters_in = {x, setpoint};
    ic_solutions_out = {c, rv};
    options = sdpsettings('solver','gurobi','verbose',0,'cachesolvers',1);  % setup LP-solver
    Controller = optimizer(C, J, options, ic_parameters_in, ic_solutions_out);
end
ic_inputs = {currentx,reftraj(1:LQ1.system.nx)};
[ic_solutions] = Controller{ic_inputs};
rvk = ic_solutions{2};

r0 = currentx - rvk;
cval = ic_solutions{1};
uvk = LQ2.K*rvk + cval*LQ2.L*(reftraj);
u0k = LQ1.K*r0 + (1-cval)*LQ1.L*(reftraj);
uout = u0k + uvk;
t = toc(tstart);
ut = [uout, t];

end
