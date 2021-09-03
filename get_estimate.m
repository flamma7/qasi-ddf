function [x_hat, P] = get_estimate(x_hats, Ps, NUM_STATES, agent)
    x_hat = x_hats(:,agent);
    P = Ps(:, (agent-1)*NUM_STATES+1 : agent*NUM_STATES);
end
