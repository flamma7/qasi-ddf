function [state] = normalize_state(state, NUM_AGENTS, STATES)
    for a=1:NUM_AGENTS
        theta = state(STATES*(a-1)+3);
        theta = normalize_angle(theta);
        state(STATES*(a-1)+3) = theta;
    end
end