function [x, P] = get_estimate_index(x_history, P_history, STATES, index, agent)

    % Returns the x, P for a given time index in the x_history
    
    x = x_history(STATES*(agent-1)+1:STATES*agent, index);
    P = P_history(STATES*(agent-1)+1:STATES*agent, STATES*(index-1)+1 : STATES*index);
end