function [x_hat, P] = get_estimate(x_hats, Ps, STATES, agent)
    x_hat = x_hats(:,agent);
    P = Ps(:, (agent-1)*STATES+1 : agent*STATES);
end
