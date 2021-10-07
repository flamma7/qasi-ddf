function [x_nav, P] = get_estimate_nav(x_navs, P_navs, STATES, agent)
    x_nav = x_navs(:,agent);
    P = P_navs(:, (agent-1)*STATES+1 : agent*STATES);
end
