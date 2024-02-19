function [dx] = uavPlanarNonlin(x,u)
%UAVPLANARNONLIN nonlinear dynamic planar model of quadrotor UAV
%   input:
%       x - state vector
%       u - control vector
%   output:
%       dx - derivative of state

if length(x) ~= 6
    error('wrong dimension of state vector');
elseif length(u) ~= 2
    error('wrong dimension of control vector');
end

g = 9.81;   % gravitational acceleration

% % asctec hummingbird
% m = 0.5;    % mass of quadrotor
% Ix = 6.4*1e-3;  % inertia of quadrotor aroud x-axis

% bitcraze crazyflie
m = 0.03;    % mass of quadrotor
Ix = 2.3951e-5;  % inertia of quadrotor aroud x-axis

f = [x(4);x(5);x(6);0;-g;0];
h = zeros(length(x),length(u));

h(4,1) = -(1/m)*sin(x(3));
h(5,1) = (1/m)*cos(x(3));
h(6,2) = 1/Ix;

dx = f + h*u;

end