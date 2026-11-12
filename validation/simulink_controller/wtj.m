function [opt_acc, max_acc, min_acc, a_star] = wtj(sn,vn,sndot,alpha,tau_min,tau_max,gmin,lambda,c)
% wtj  The full WTJ-Controller subsystem from the Simulink model, as one function.
%
% Source: cs3892-2026F-wtj/simulink/wtj.slx, subsystem WTJ-Controller.
% This file only copies the wiring between the blocks:
%
%   sn, vn, sndot, alpha, gmin, lambda --+--> accel_min (tau = tau_max) --> al --+--> Saturation1 [-3, 1.5] --> min_acc (monitor only)
%                                        |                                       |
%                                        +--> accel_max (tau = tau_min) --> au --+--> Saturation2 [-3, 1.5] --> max_acc (monitor only)
%                                                                                |
%                     al, au (NOT saturated), vn, c --> Work Optimization function --> Saturation [-3, 1.5] --> opt_acc
%
% The top-level model then applies one more Saturation [-3.0, 1.5] to opt_acc
% before publishing it. It has the same limits, so it doesn't change the value.
%
% Argument order matches wtj() in controllers.py and the subsystem's input
% ports 1-9 (sn, vn, sndot, alpha, tau_min, tau_max, gmin, lambda, c).

ACC_LOWER = -3;
ACC_UPPER = 1.5;
sat = @(x) min(max(x, ACC_LOWER), ACC_UPPER);

al = timeGap(sn, vn, sndot, alpha, tau_max, gmin, lambda);   % accel_min block
au = timeGap(sn, vn, sndot, alpha, tau_min, gmin, lambda);   % accel_max block

[opt, a_star] = work_optimization(al, au, vn, c);

opt_acc = sat(opt);       % Saturation (inside WTJ-Controller) + top-level Saturation
max_acc = sat(au);        % Saturation2 -> max_acc output
min_acc = sat(al);        % Saturation1 -> min_acc output
end
