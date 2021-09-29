function [state] = normalize_state(state, NUM_AGENTS, STATES)
    for l=1:size(state,2) % In the case of a history of states
        for a=1:NUM_AGENTS
            theta = state(STATES*(a-1)+3, l);
            theta = normalize_angle(theta);
            state(STATES*(a-1)+3, l) = theta;
        end
    end
end