function [x_navs_history, P_navs_history] = set_estimate_nav_history(x_navs_history, P_navs_history, x_nav, P_nav, STATES, agent, index)

    % Adds x_nav to x_navs_history for the agent at the time index

    agent_rows = STATES*(agent-1)+1 : STATES*agent;
    indices = STATES*(index-1)+1 : STATES*index;

    x_navs_history(agent_rows, index) = x_nav;
    P_navs_history(agent_rows, indices) = P_nav;
end
