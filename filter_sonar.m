function [x_hat, P] = filter_sonar(x_hat, P, x_gt, w, w_perceived, NUM_AGENTS, NUM_STATES, prob_detection, sonar_dist)
    states_per_agent = NUM_STATES / NUM_AGENTS;
    agent_states = reshape(x_gt, states_per_agent, NUM_AGENTS);
    
    for a = 1:NUM_AGENTS
        start_row1 = states_per_agent*(a-1)+1;
        end_row1 = start_row1 + 1;
        agent1 = x_gt(start_row1:end_row1,1);
        for a2 = a+1:NUM_AGENTS
            start_row2 = states_per_agent*(a2-1)+1;
            end_row2 = start_row2 + 1;
            agent2 = x_gt(start_row2:end_row2,1);
            
            delta = agent2 - agent1;
            if norm(delta) < sonar_dist && binornd(1,prob_detection)
                
                delta = delta;% + normrnd(0, w, 2, 1);

                H = zeros(2,NUM_STATES);
                H(1, start_row1) = -1;
                H(1, start_row2) = 1;
                H(2, end_row1) = -1;
                H(2, end_row2) = 1;
                K = P * H' * inv(H * P * H' + w_perceived*eye(2));
                x_hat = x_hat + K * (delta - H * x_hat);
                P = P - K*H*P;
            end
        end
end