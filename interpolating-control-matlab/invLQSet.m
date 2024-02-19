function [P] = invLQSet(sys)
% INVLQSET search an invariant controlled set for LQ controller

% transform conditions to LMI
Fx = [eye(sys.nx);-eye(sys.nx)];
gx = [sys.x.max; -sys.x.min];
Fu = [eye(sys.nu);-eye(sys.nu)];
gu = [sys.u.max; -sys.u.min];

K = sys.LQRGain;        % LQ controler for system sys

Ac = sys.A + sys.B*K;   % closed loop dynamics

% LMI conditions for closed loop system
Fc = [Fx; Fu*K];
gc = [gx; gu];

% initialize algorithm for invariant controlled set
X0 = Polyhedron(Fc,gc);
X0.minHRep;

while 1
    P = Polyhedron([X0.A;X0.A*Ac],[X0.b;X0.b]); % next iteration
    % check if set is found
    if P == X0
        break;
    else
        % eliminate redundant conditions from half-space representation
        if ~P.irredundantHRep
            P.minHRep;
        end
        % set new polyhedron and go to start
        X0 = P.copy;
    end
end

end

