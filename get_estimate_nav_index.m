function [x, P] = get_estimate_index(x_history, P_history, STATES, index)

    % Returns the x, P for a given time index in the x_history
    
    x = x_history(:, index);
    P = P_history(:, STATES*(index-1)+1 : STATES*index);
end