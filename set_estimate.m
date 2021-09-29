function [x_hats, Ps] = set_estimate(x_hats, Ps, NUM_STATES, x_hat, P, agent)
    x_hats(:, agent) = x_hat;
    increment  = size(x_hat, 1);
    Ps(:, (agent-1)*increment+1 : agent*increment) = P;
end