function [x_hat, P] = filter_dvl(x_hat, P, x_gt, w, w_perceived, NUM_AGENTS, NUM_STATES, agent)
    
    states_per_agent = NUM_STATES / NUM_AGENTS;
    start_row = states_per_agent * (agent-1) + 3;
    end_row = start_row + 1;
    dvl = x_gt(start_row:end_row, 1) + normrnd(0, w, 2, 1);

    H = zeros(2, NUM_STATES);
    H(1,start_row) = 1;
    H(2,end_row) = 1;
    K = P * H' * inv(H * P * H' + w_perceived*eye(2));
    x_hat = x_hat + K * (dvl - H * x_hat);
    P = P - K*H*P;
    
end