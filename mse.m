clear all; close all; clc;


filename = "monte_carlos/DT_1_x.txt";
filename2 = "monte_carlos/DT_1_P.txt";

NUM_RUNS = 10;
NUM_STEPS = 2000;

% OWNSHIP
ownship_error_dt = zeros(1,NUM_STEPS);
ownship_error_omni = zeros(1,NUM_STEPS);
ownship_error_ci = zeros(1,NUM_STEPS);

% BLUE Team
blue_error_dt = zeros(1,NUM_STEPS);
blue_error_omni = zeros(1,NUM_STEPS);
blue_error_ci = zeros(1,NUM_STEPS);

% Red Team
red_error_dt = zeros(1,NUM_STEPS);
red_error_omni = zeros(1,NUM_STEPS);
red_error_ci = zeros(1,NUM_STEPS);

% ownship_uncertainty = [];
ownship_tf = zeros(8,24);
ownship_tf(1:4,1:4) = eye(4);
ownship_tf(5:8, 13:16) = eye(4);

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
plot(ownship_error_ci, "m");
legend("DeltaTier", "Omniscient", "QCI");
title("Ownship Mean Norm Error");
xlabel("Time Step");
ylabel("Mean Norm Error");
ax = gca;
ax.TitleFontSizeMultiplier = 2;
ax = gca;
ax.FontSize = 28;
saveas(gcf,filename + "_ownship.png")

figure(2);
plot(blue_error_dt);
hold on
plot(blue_error_omni);
plot(blue_error_ci, "m");
legend("DeltaTier", "Omniscient", "QCI");
title("Blue Team Mean Norm Error");
xlabel("Time Step");
ylabel("Mean Norm Error");
ax = gca;
ax.TitleFontSizeMultiplier = 2;
ax = gca;
ax.FontSize = 28;
saveas(gcf,filename + "_blue.png")

figure(3);
plot(red_error_dt);
hold on
plot(red_error_omni);
plot(red_error_ci, "m");
legend("DeltaTier", "Omniscient", "QCI");
title("Red Agent Mean Norm Error");
xlabel("Time Step");
ylabel("Mean Norm Error");
ax = gca;
ax.TitleFontSizeMultiplier = 2;
ax = gca;
ax.FontSize = 28;
saveas(gcf,filename + "_red.png")