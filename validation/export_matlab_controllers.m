% export_matlab_controllers.m
%
% Runs the MATLAB timeGap and wtj controllers on a shared set of test inputs
% and writes every input and output to a CSV. Then compare_controllers.py
% runs the Python controllers on the same rows and compares the results.
%
% The MATLAB controller is the WTJ-Controller subsystem in the Simulink model
% cs3892-2026F-wtj/simulink/wtj.slx. Its MATLAB Function block code was copied
% into the .m files in simulink_controller/ (timeGap.m, work_optimization.m,
% wtj.m). This script adds that folder to the path itself.
%
% Usage:
%   1. In MATLAB, cd into this validation/ folder and run:
%        export_matlab_controllers
%   2. In a terminal, from the validation/ folder:
%        python3 compare_controllers.py matlab_controller_outputs.csv

clear; clc;

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'simulink_controller'), '-begin');
fprintf('Using wtj from:     %s\n', which('wtj'));
fprintf('Using timeGap from: %s\n', which('timeGap'));

outFile = fullfile(here, 'matlab_controller_outputs.csv');
rng(0);  % fixed seed so the test set is reproducible

%% Default parameters used for the edge-case grid
% These are the initial values of the Get Parameter blocks in wtj.slx.
% The model has no separate "tau" for timeGap, so the midpoint is used.
P.alpha   = 1;
P.tau_min = 2.0;
P.tau_max = 3.0;
P.tau     = 2.5;
P.gmin    = 3;
P.lam     = 0.1;    % called "lambda" in the Simulink model
P.c       = 0.1;

%% 1) Random rows: random states AND random parameters
% Parameter ranges cover the model's values above, plus some room around them.
N = 10000;
R.sn      = 0   + 100*rand(N,1);        % gap [m]
R.vn      = -2  + 37*rand(N,1);         % follower speed [m/s], includes vn <= 0
R.sndot   = -10 + 20*rand(N,1);         % gap rate [m/s], both signs
R.alpha   = 0.2 + 1.3*rand(N,1);        % 0.2 - 1.5
R.tau_min = 1.0 + 2.0*rand(N,1);        % 1 - 3
R.tau_max = R.tau_min + 0.1 + 1.9*rand(N,1);   % always tau_max > tau_min
R.tau     = R.tau_min + (R.tau_max - R.tau_min).*rand(N,1);
R.gmin    = 1   + 4*rand(N,1);          % 1 - 5
R.lam     = 0.01 + 0.49*rand(N,1);      % 0.01 - 0.5
R.c       = 0.01 + 0.99*rand(N,1);      % 0.01 - 1
R.case_id = zeros(N,1);                 % 0 = random row

%% 2) Edge-case grid: hits every branch of wtj on purpose
% vn: negative, exactly 0, tiny positive, typical, exactly 30 (a_star = 0), above 30
% sndot: negative, exactly 0, positive (checks the min(0, sndot) term)
% sn: small gaps (clamped low / saturated at -3) through large gaps (saturated at 1.5)
sn_e    = [0 2 5 10 20 35 50 60 75 100];
vn_e    = [-1 0 1e-9 0.5 5 15 29.999 30 30.001 35];
sndot_e = [-5 -1e-9 0 1e-9 5];
[SN, VN, SD] = ndgrid(sn_e, vn_e, sndot_e);
M = numel(SN);
E.sn      = SN(:);
E.vn      = VN(:);
E.sndot   = SD(:);
E.alpha   = P.alpha   * ones(M,1);
E.tau_min = P.tau_min * ones(M,1);
E.tau_max = P.tau_max * ones(M,1);
E.tau     = P.tau     * ones(M,1);
E.gmin    = P.gmin    * ones(M,1);
E.lam     = P.lam     * ones(M,1);
E.c       = P.c       * ones(M,1);
E.case_id = ones(M,1);                  % 1 = edge-case row

%% Combine
cols = {'case_id','sn','vn','sndot','alpha','tau','tau_min','tau_max','gmin','lam','c'};
X = zeros(N + M, numel(cols));
for k = 1:numel(cols)
    X(:,k) = [R.(cols{k}); E.(cols{k})];
end
col = @(name) X(:, strcmp(cols, name));

%% Run the MATLAB controllers
n = size(X,1);
tg_out  = zeros(n,1);
wtj_out = zeros(n,1);
sn = col('sn'); vn = col('vn'); sndot = col('sndot');
alpha = col('alpha'); tau = col('tau'); tau_min = col('tau_min');
tau_max = col('tau_max'); gmin = col('gmin'); lam = col('lam'); c = col('c');
for i = 1:n
    tg_out(i)  = timeGap(sn(i), vn(i), sndot(i), alpha(i), tau(i), gmin(i), lam(i));
    wtj_out(i) = wtj(sn(i), vn(i), sndot(i), alpha(i), tau_min(i), tau_max(i), ...
                     gmin(i), lam(i), c(i));   % first output = opt_acc (saturated)
end

%% Write CSV at full double precision (%.17g so no rounding is introduced)
header = [cols, {'timeGap_matlab','wtj_matlab'}];
data = [X, tg_out, wtj_out];
fid = fopen(outFile, 'w');
fprintf(fid, '%s\n', strjoin(header, ','));
fmt = [repmat('%.17g,', 1, numel(header)-1), '%.17g\n'];
fprintf(fid, fmt, data.');
fclose(fid);

fprintf('Wrote %d rows (%d random, %d edge cases) to %s\n', n, N, M, outFile);
