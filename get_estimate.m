function [x_hat, P] = get_estimate(x_hats, Ps, STATES, NUM_AGENTS, agent)
    x_hat = x_hats(:,agent);
    increment = NUM_AGENTS * STATES;
    start_col = (agent-1)*increment + 1;
    P = Ps(:,  start_col: start_col + increment -1 );
end
