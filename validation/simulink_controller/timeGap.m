function cmd_accel = timeGap(sn,vn,sndot,alpha,tau,gmin,lambda)
% timeGap  Time-gap controller, copied from the Simulink model.
%
% Source: cs3892-2026F-wtj/simulink/wtj.slx
%   WTJ-Controller/accel_min/myController/MATLAB Function   (called with tau = tau_max -> al)
%   WTJ-Controller/accel_max/myController/MATLAB Function   (called with tau = tau_min -> au)
% Both blocks contain exactly this code. Only the function name was changed
% (Simulink calls it fcn). If the block code changes, update this file too.

cmd_accel = alpha * (sn - tau*vn - gmin) + lambda*min(0,sndot);
