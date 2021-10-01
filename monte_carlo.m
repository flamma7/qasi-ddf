close all; clear all; clc;

total_time = 0.0;
MC_RUNS = 10
tic
    for l = 1:MC_RUNS
        l
        qasi_ddf(l)
    end
toc

