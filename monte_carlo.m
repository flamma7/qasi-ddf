close all; clear all; clc;

total_time = 0.0;
MC_RUNS = 500
tic
    for l = 1:MC_RUNS
        l
        rng(l)
        qasi_ddf(l)
    end
toc

