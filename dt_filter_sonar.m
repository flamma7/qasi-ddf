function [x_hat, P, ledger] = dt_filter_sonar(x_hat, P, x_gt, w, w_perceived_sonar_range, w_perceived_sonar_azimuth, NUM_AGENTS, STATES, prob_detection, sonar_dist, agent, x_nav, ledger, index)

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
            [pred_range, C_range] = predict_range(x_hat, start_x1, start_x2);

            % Splitting up to compare results
            innovation = rel_range_meas - pred_range;
            K = P * C_range' * inv(C_range * P * C_range' + w_perceived_sonar_range);
            x_hat = x_hat + K * innovation;
            P = P - K*C_range*P;

            [pred_azimuth, C_azimuth] = predict_azimuth(x_hat, start_x1, start_x2);
            innovation = rel_azimuth_meas - pred_azimuth;
            K = P * C_azimuth' * inv(C_azimuth * P * C_azimuth' + w_perceived_sonar_azimuth);
            x_hat = x_hat + K * innovation;
            P = P - K*C_azimuth*P;

            % meas = [pred_range; pred_azimuth];
            % innovation = [rel_range_meas; rel_azimuth_meas] - meas;
            % innovation(2) = normalize_angle(innovation(2));
            % R = diag([w_perceived_sonar_range, w_perceived_sonar_azimuth]);
            % C = [C_range; C_azimuth];
            % K = P * C' * inv(C * P * C' + R);
            % x_hat = x_hat + K * innovation;
            % P = P - K*C*P;

            % Add measurements to ledger
            ledger = add_meas(ledger, agent, "sonar_range", index, start_x1, start_x2, rel_range_meas);
            ledger = add_meas(ledger, agent, "sonar_azimuth", index, start_x1, start_x2, rel_azimuth_meas);
        end
    end