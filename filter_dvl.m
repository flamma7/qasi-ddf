function [x_nav, P_nav] = filter_dvl(x_nav, P_nav, x_gt, w, w_perceived, NUM_AGENTS, TOTAL_STATES, agent)
    
    STATES = TOTAL_STATES / NUM_AGENTS;
    start_row = STATES * (agent-1) + 4;
    end_row = start_row + 1;
    dvl = x_gt(start_row:end_row, 1) + normrnd(0, w, 2, 1);

    H = zeros(2, STATES);
    H(1,4) = 1;
    H(2,5) = 1;

    K = P_nav * H' * inv(H * P_nav * H' + w_perceived*eye(2));
    x_nav = x_nav + K * (dvl - H * x_nav);
    P_nav = P_nav - K*H*P_nav;
    
end