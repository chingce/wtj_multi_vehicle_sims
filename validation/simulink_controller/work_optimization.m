function [opt_accel,a_star] = work_optimization(al, au, vn, c)
% work_optimization  Copied from the Simulink model.
%
% Source: cs3892-2026F-wtj/simulink/wtj.slx
%   WTJ-Controller/Work Optimization function
% The body below is exactly the block's code. Only the function name was
% changed (Simulink calls it fcn). If the block code changes, update this too.

    % al - lower bound on acceleration given by the longer time gap
    % au - upper bound on the acceleration given by the shorter time gap
    % vn - current ego velocity
    % c1 - weighting fuel efficiency
    % c2 - weighting speed
    % cost function f(a) = a*(vn+0.25)*c1 + c2/a
    % +0.25 adjustment to make push to accelerate from 0 velocity less
    % agressive

    a_star = 0;

    % If we are stopped, preference going - avoid divide by zero
    % (f(a) = c2/a is decreasing, so go as hard as allowed)
    if vn <= 0
        opt_accel = al;
        return
    end

    % Otherwise, we are in fuel conserve mode
    % d(cost)/da = (vn + 0.25)*c1  - c2/(a^2) = 0
    % a = sqrt(c2/((vn + 0.25)*c1)) -- We care about the positive side only
    % c = sqrt(c2/c1)
    % a = c * sqrt(1/(vn + 0.25))
    % never accelerate to above 30, so prevent that possibility
    a_star = c*sqrt(1/(vn + 0.25)) - (c*sqrt(1/(30 + 0.25)));

    % Clamp the unconstrained minimizer to the feasible interval [al, au]
    opt_accel = min(max(al,a_star), au);
end
