function [x_hat, P] = filter_gps(x_hat, P, x_gt, w, w_perceived, NUM_AGENTS, NUM_STATES, agent)
    
    states_per_agent = NUM_STATES / NUM_AGENTS;

    if agent == 0 % Fuse GPS for all agents (broadcast)
        for a = 1:NUM_AGENTS
            
            start_row = states_per_agent * (a-1) + 1;
            end_row = start_row + 1;
            gps = x_gt(start_row:end_row, 1) + normrnd(0, w, 2, 1);

            H = zeros(2, NUM_STATES);
            H(1,start_row) = 1;
            H(2,end_row) = 1;
            K = P * H' * inv(H * P * H' + w_perceived*eye(2));
            x_hat = x_hat + K * (gps - H * x_hat);
            P = P - K*H*P;
        end
    else % Just fuse the GPS for this agent
        start_row = states_per_agent * (agent-1) + 1;
        end_row = start_row + 1;
        gps = x_gt(start_row:end_row, 1) + normrnd(0, w, 2, 1);

        H = zeros(2, NUM_STATES);
        H(1,start_row) = 1;
        H(2,end_row) = 1;
        K = P * H' * inv(H * P * H' + w_perceived*eye(2));
        x_hat = x_hat + K * (gps - H * x_hat);
        P = P - K*H*P;
    end
    
end