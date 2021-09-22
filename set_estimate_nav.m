function [x_hats, Ps] = set_estimate(x_hats, Ps, STATES, x_hat, P, agent)
    x_hats(:, agent) = x_hat;
    Ps(:, (agent-1)*STATES+1 : agent*STATES) = P;
end