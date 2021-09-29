function [x_nav, P_nav] = filter_gyro(x_nav, P_nav, x_gt, w, w_perceived, NUM_AGENTS, TOTAL_STATES, agent)
    
    STATES = TOTAL_STATES / NUM_AGENTS;
    start_row = STATES * (agent-1) + 6;
    end_row = start_row;
    meas = x_gt(start_row:end_row, 1) + normrnd(0, w, 1, 1);

    H = zeros(1, STATES);
    H(1,6) = 1;
    K = P_nav * H' * inv(H * P_nav * H' + w_perceived);
    x_nav = x_nav + K * (meas - H * x_nav);
    x_nav = normalize_state(x_nav, 1, STATES);
    P_nav = P_nav - K*H*P_nav;
end