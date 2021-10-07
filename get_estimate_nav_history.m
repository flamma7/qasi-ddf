function [x_nav_history, P_nav_history] = get_estimate_nav_history(x_navs_history, P_navs_history, STATES, agent)

    % Returns the x_nav_history for a particular agent given all of the nav histories

    agent_rows = STATES*(agent-1)+1 : STATES*agent;
    x_nav_history = x_navs_history(agent_rows, :);
    P_nav_history = P_navs_history(agent_rows, :);
end
