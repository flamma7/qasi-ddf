


NUM_RUNS = 500;
NUM_STEPS = 2000;

omni_explicit = 0;
dt_explicit = 0;
dt_implicit = 0;

for i = 1:NUM_RUNS
    % DT
    filename = "monte_carlos/DT_" + int2str(i) + "_meas_cnts.txt";
    meas_cnts = dlmread(filename);
    dt_explicit = dt_explicit + meas_cnts(1);
    dt_implicit = dt_explicit + meas_cnts(2);

    % OMNI
    filename = "monte_carlos/OMNI_" + int2str(i) + "_meas_cnts.txt";
    meas_cnts = dlmread(filename);
    omni_explicit = omni_explicit + meas_cnts(1);
end
omni_explicit = omni_explicit / NUM_RUNS;
dt_explicit = dt_explicit / NUM_RUNS;
dt_implicit = dt_implicit / NUM_RUNS;

x = categorical({'Omniscient','DeltaTier'});
x = reordercats(x,{'Omniscient','DeltaTier'});
% x = ["Omniscient", "DeltaTier"];
y = [0, omni_explicit; dt_implicit, dt_explicit];
bar(x,y, "stacked")
legend("Implicit", "Explicit")
title("Mean Measurements Shared")
ylabel("Mean Measurements Shared")