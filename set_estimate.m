function [x_hats, Ps] = set_estimate(x_hats, Ps, NUM_STATES, x_hat, P, agent)
    x_hats(:, agent) = x_hat;
    Ps(:, (agent-1)*NUM_STATES+1 : agent*NUM_STATES) = P;
end