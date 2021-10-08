function [x_nav, P_nav] = get_estimate_nav_index(x_nav_history, P_nav_history, STATES, index)

    % Returns the x_nav, P_nav for a given time index in the x_nav_history
    
    x_nav = x_nav_history(:, index);
    P_nav = P_nav_history(:, STATES*(index-1)+1 : STATES*index);
end