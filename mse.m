clear all; close all; clc;


NUM_RUNS = 50;
NUM_STEPS = 2000;
TRACK_STATES = 12;

% OWNSHIP
ownship_error_dt = zeros(1,NUM_STEPS);
ownship_error_omni = zeros(1,NUM_STEPS);
ownship_error_ci = zeros(1,NUM_STEPS);
ownship_uncertainty_dt = zeros(1,NUM_STEPS);
ownship_uncertainty_omni = zeros(1,NUM_STEPS);
ownship_uncertainty_ci = zeros(1,NUM_STEPS);

% BLUE Team
blue_error_dt = zeros(1,NUM_STEPS);
blue_error_omni = zeros(1,NUM_STEPS);
blue_error_ci = zeros(1,NUM_STEPS);
blue_uncertainty_dt = zeros(1,NUM_STEPS);
blue_uncertainty_omni = zeros(1,NUM_STEPS);
blue_uncertainty_ci = zeros(1,NUM_STEPS);

% Red Team
red_error_dt = zeros(1,NUM_STEPS);
red_error_omni = zeros(1,NUM_STEPS);
red_error_ci = zeros(1,NUM_STEPS);
red_uncertainty_dt = zeros(1,NUM_STEPS);
red_uncertainty_omni = zeros(1,NUM_STEPS);
red_uncertainty_ci = zeros(1,NUM_STEPS);

% ownship_uncertainty = [];
ownship_tf = zeros(8,24);
ownship_tf(1:4,1:4) = eye(4);
ownship_tf(5:8, 13:16) = eye(4);
ownship_tf_P= zeros(24,12);
ownship_tf_P(1:4,1:4) = eye(4);
ownship_tf_P(TRACK_STATES+5:TRACK_STATES+8,5:8) = eye(4);
ownship_tf_P = repmat(ownship_tf_P, 1, NUM_STEPS);
ownship_tf_P(:, 1:48);


blue_tf = zeros(8,24);
blue_tf(1:4,5:8) = eye(4);
blue_tf(5:8, 17:20) = eye(4);

red_tf = zeros(8,24);
red_tf(1:4,9:12) = eye(4);
red_tf(5:8, 21:24) = eye(4);


for i = 1:NUM_RUNS
    % DT
    filename = "monte_carlos/DT_" + int2str(i) + "_x.txt";
    error = dlmread(filename);
    ownship_error_dt_run = ownship_tf * error;
    ownship_error_dt_run_norm = vecnorm(ownship_error_dt_run) * (1/NUM_RUNS);
    ownship_error_dt = ownship_error_dt + ownship_error_dt_run_norm;
    % filename = "monte_carlos/DT_" + int2str(i) + "_P.txt";
    % P_history = dlmread(filename);
    % ownship_P_history = ownship_tf_P .* P_history;
    % for i = 1:NUM_STEPS
    %     P = ownship_P_history(1:TRACK_STATES, TRACK_STATES*(i-1)+1:TRACK_STATES*i);
    %     P2 = P_history(TRACK_STATES+1:end, TRACK_STATES*(i-1)+1:TRACK_STATES*i)
    %     ownship_P1 = ownship_tf_P * P1 * ownship_tf_P'
    %     ownship_P2 = ownship_tf_P * P2 * ownship_tf_P'
    % end
    % ownship_tf

    % OMNI
    filename = "monte_carlos/OMNI_" + int2str(i) + "_x.txt";
    error = dlmread(filename);
    ownship_error_omni_run = ownship_tf * error;
    ownship_error_omni_run_norm = vecnorm(ownship_error_omni_run) * (1/NUM_RUNS);
    ownship_error_omni = ownship_error_omni + ownship_error_omni_run_norm;

    % CI
    filename = "monte_carlos/CI_" + int2str(i) + "_x.txt";
    error = dlmread(filename);
    ownship_error_ci_run = ownship_tf * error;
    ownship_error_ci_run_norm = vecnorm(ownship_error_ci_run) * (1/NUM_RUNS);
    ownship_error_ci = ownship_error_ci + ownship_error_ci_run_norm;
end

for i = 1:NUM_RUNS
    % DT
    filename = "monte_carlos/DT_" + int2str(i) + "_x.txt";
    error = dlmread(filename);
    blue_error_dt_run = blue_tf * error;
    blue_error_dt_run_norm = vecnorm(blue_error_dt_run) * (1/NUM_RUNS);
    blue_error_dt = blue_error_dt + blue_error_dt_run_norm;

    % OMNI
    filename = "monte_carlos/OMNI_" + int2str(i) + "_x.txt";
    error = dlmread(filename);
    blue_error_omni_run = blue_tf * error;
    blue_error_omni_run_norm = vecnorm(blue_error_omni_run) * (1/NUM_RUNS);
    blue_error_omni = blue_error_omni + blue_error_omni_run_norm;

    % CI
    filename = "monte_carlos/CI_" + int2str(i) + "_x.txt";
    error = dlmread(filename);
    blue_error_ci_run = blue_tf * error;
    blue_error_ci_run_norm = vecnorm(blue_error_ci_run) * (1/NUM_RUNS);
    blue_error_ci = blue_error_ci + blue_error_ci_run_norm;
end

for i = 1:NUM_RUNS
    % DT
    filename = "monte_carlos/DT_" + int2str(i) + "_x.txt";
    error = dlmread(filename);
    red_error_dt_run = red_tf * error;
    red_error_dt_run_norm = vecnorm(red_error_dt_run) * (1/NUM_RUNS);
    red_error_dt = red_error_dt + red_error_dt_run_norm;

    % OMNI
    filename = "monte_carlos/OMNI_" + int2str(i) + "_x.txt";
    error = dlmread(filename);
    red_error_omni_run = red_tf * error;
    red_error_omni_run_norm = vecnorm(red_error_omni_run) * (1/NUM_RUNS);
    red_error_omni = red_error_omni + red_error_omni_run_norm;

    % CI
    filename = "monte_carlos/CI_" + int2str(i) + "_x.txt";
    error = dlmread(filename);
    red_error_ci_run = red_tf * error;
    red_error_ci_run_norm = vecnorm(red_error_ci_run) * (1/NUM_RUNS);
    red_error_ci = red_error_ci + red_error_ci_run_norm;
end

filename = "results/mc_" + int2str(NUM_RUNS) + "_" + int2str(NUM_STEPS);

figure(1);
plot(ownship_error_dt);
hold on
plot(ownship_error_omni);
plot(ownship_error_ci);
legend("DeltaTier", "Omniscient", "QCI");
title("Ownship Mean Norm Error vs time");
xlabel("Time Step");
ylabel("Mean Norm Error");
saveas(gcf,filename + "_ownship.png")

figure(2);
plot(blue_error_dt);
hold on
plot(blue_error_omni);
plot(blue_error_ci);
legend("DeltaTier", "Omniscient", "QCI");
title("Blue Team Mean Norm Error vs time");
xlabel("Time Step");
ylabel("Mean Norm Error");
saveas(gcf,filename + "_blue.png")

figure(3);
plot(red_error_dt);
hold on
plot(red_error_omni);
plot(red_error_ci);
legend("DeltaTier", "Omniscient", "QCI");
title("Red Agent Mean Norm Error vs time");
xlabel("Time Step");
ylabel("Mean Norm Error");
saveas(gcf,filename + "_red.png")