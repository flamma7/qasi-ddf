function [x_hat, P] = filter_sonar(x_hat, P, x_gt, w, w_perceived_sonar_range, w_perceived_sonar_azimuth, NUM_AGENTS, STATES, prob_detection, sonar_dist, agent, x_nav)

    TOTAL_STATES = STATES * NUM_AGENTS;

    agent_position = x_gt(STATES*(agent-1)+1 : STATES*(agent-1)+2, 1);
    agents_theta_gt = x_gt(STATES*(agent-1)+3,1);
    agents_theta_est = x_nav(3,1);
    
    for a = 1:NUM_AGENTS
        if a  == agent
            continue
        end
        a_position = x_gt(STATES*(a-1)+1 : STATES*(a-1)+2, 1);

        delta = a_position - agent_position;

        %% We're in sonar range & got a detection
        if norm(delta) < sonar_dist && binornd(1, prob_detection)
            
            rel_range_meas = norm(delta) + normrnd(0, w, 1, 1);
            rel_azimuth_meas = atan2(delta(2), delta(1)) - agents_theta_gt + normrnd(0, w, 1, 1);

            rel_azimuth_meas = rel_azimuth_meas + agents_theta_est; % Linearize the azimuth measurement

            start_x1 = 4*(agent-1)+1;
            start_x2 = 4*(a-1)+1;

            x1 = x_hat(start_x1,1);
            y1 = x_hat(start_x1+1,1);
            x2 = x_hat(start_x2,1);
            y2 = x_hat(start_x2+1,1);

            delta_pred = [x2 - x1; y2 - y1];

            pred_range = norm(delta_pred);
            pred_azimuth = atan2(delta_pred(2), delta_pred(1));

            % Partial Derivatives of Range Measurement
            drdx1 = (x1 - x2) / norm(delta);
            drdx2 = (x2 - x1) / norm(delta);
            drdy1 = (y1 - y2) / norm(delta);
            drdy2 = (y2 - y1) / norm(delta);

            % Partial Derivatives of Azimuth Measurement
            dadx1 = (y2 - y1) / norm(delta)^2;
            dadx2 = -(y2 - y1) / norm(delta)^2;
            dady1 = -(x2 - x1) / norm(delta)^2;
            dady2 = (x2 - x1) / norm(delta)^2;
            % dadt1 = -1; % NonLinear filter

            % Construct range measurement update vector
            C = zeros(1, 4*NUM_AGENTS);
            C(1, start_x1) = drdx1;
            C(1, start_x2) = drdx2;
            C(1, start_x1+1) = drdy1;
            C(1, start_x2+1) = drdy2;

            K = P * C' * inv(C * P * C' + w_perceived_sonar_range);
            x_hat = x_hat + K * (rel_range_meas - pred_range);
            P = P - K*C*P;

            % Construct azimuth measurement update vector
            C = zeros(1, 4*NUM_AGENTS);
            C(1, start_x1) = dadx1;
            C(1, start_x2) = dadx2;
            C(1, start_x1+1) = dady1;
            C(1, start_x2+1) = dady2;

            K = P * C' * inv(C * P * C' + w_perceived_sonar_azimuth);
            x_hat = x_hat + K * normalize_angle(rel_azimuth_meas - pred_azimuth);
            P = P - K*C*P;
        end
    end

    % for a = 1:NUM_AGENTS
    %     start_row1 = STATES*(a-1)+1;
    %     end_row1 = start_row1 + 1;
    %     agent1 = x_gt(start_row1:end_row1,1);
    %     for a2 = a+1:NUM_AGENTS
    %         start_row2 = STATES*(a2-1)+1;
    %         end_row2 = start_row2 + 1;
    %         agent2 = x_gt(start_row2:end_row2,1);
            
    %         delta = agent2 - agent1;
            
    %         % Check if in range and simulate the probability the sonar is
    %         % pointing in the right direction
    %         if norm(delta) < sonar_dist && binornd(1,prob_detection)
                
    %             delta = delta + normrnd(0, w, 2, 1);

    %             H = zeros(2,TOTAL_STATES);
    %             H(1, start_row1) = -1;
    %             H(1, start_row2) = 1;
    %             H(2, end_row1) = -1;
    %             H(2, end_row2) = 1;
    %             K = P * H' * inv(H * P * H' + w_perceived*eye(2));
    %             x_hat = x_hat + K * (delta - H * x_hat);
    %             P = P - K*H*P;
    %         end
    %     end
    % end
end