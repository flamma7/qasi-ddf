function [x_hat, P, x_common, P_common, x_hats, Ps, num_implicit, num_explicit] = filter_sonar_et(x_hat, P, x_gt, w, w_perceived_sonar_range, w_perceived_sonar_azimuth, NUM_AGENTS, BLUE_NUM, STATES, prob_detection, sonar_dist, agent, x_nav, x_common_bar, P_common_bar, x_bars, P_bars, x_common, P_common, delta_range, delta_azimuth, x_hats, Ps, num_implicit, num_explicit)

    % Determine if there is a sonar measurement
    % Update the regular estimate
    % Share measurements with other agents implicit/explicit

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

            %% UPDATE LOCAL ESTIMATE
            start_x1 = 4*(agent-1)+1;
            start_x2 = 4*(a-1)+1;
            [pred_range, C_range] = predict_range(x_hat, start_x1, start_x2);
            [pred_azimuth, C_azimuth] = predict_azimuth(x_hat, start_x1, start_x2);

            meas = [pred_range; pred_azimuth];
            innovation = [rel_range_meas; rel_azimuth_meas] - meas;
            innovation(2) = normalize_angle(innovation(2));
            R = diag([w_perceived_sonar_range, w_perceived_sonar_azimuth]);
            C = [C_range; C_azimuth];
            K = P * C' * inv(C * P * C' + R);
            x_hat = x_hat + K * innovation;
            P = P - K*C*P;

            %%%%%%%%%%%%%%%%%%%
            %% COMMON FILTER %%
            %%%%%%%%%%%%%%%%%%%
            % Do we use x_common or x_common_bar for the following calculations?
            % TODO make the below function calls for organization / debugging?

            % RANGE COMMON ESTIMATE
            [pred_range, C] = predict_range(x_common, start_x1, start_x2); % TODO should this be x_common_bar??
            innovation_range_common = rel_range_meas - pred_range;
            if abs(innovation_range_common) > delta_range % Explicit
                K = P_common * C' * inv(C * P_common * C' + w_perceived_sonar_range);
                x_common = x_common + K * innovation_range_common;
                P_common = P_common - K*C*P_common;
            else % Implicit
                h_x_hat = pred_range;
                [h_x_bar, C_] = predict_range(x_common_bar, start_x1, start_x2);
                h_x_ref = h_x_bar;
                [x_common, P_common] = implicit_fuse(x_common_bar, P_common_bar, x_common, P_common, x_common_bar, C, w_perceived_sonar_range, delta_range, h_x_hat, h_x_bar, h_x_ref, false);
            end
            % AZIMUTH COMMON ESTIMATE
            [pred_azimuth, C] = predict_azimuth(x_common, start_x1, start_x2); % TODO should this be x_common_bar??
            innovation_azimuth_common = normalize_angle( rel_azimuth_meas - pred_azimuth );
            if abs(innovation_azimuth_common) > delta_azimuth % Explicit
                K = P_common * C' * inv(C * P_common * C' + w_perceived_sonar_azimuth);
                x_common = x_common + K * innovation_azimuth_common;
                P_common = P_common - K*C*P_common;
            else % Implicit
                h_x_hat = pred_azimuth;
                [h_x_bar, C_] = predict_azimuth(x_common_bar, start_x1, start_x2);
                h_x_ref = h_x_bar;
                [x_common, P_common] = implicit_fuse(x_common_bar, P_common_bar, x_common, P_common, x_common_bar, C, w_perceived_sonar_azimuth, delta_azimuth, h_x_hat, h_x_bar, h_x_ref, true);
            end

            %%%%%%%%%%%%%%%%%%%
            %% ET SHARING %%
            %%%%%%%%%%%%%%%%%%%

            % Fuse with local estimates at other agents
            for a2 = 1:BLUE_NUM
                if a2 == agent
                    continue
                end

                % "agent" is sharing wth "a2"
                [x_hat_a2, P_a2] = get_estimate(x_hats, Ps, 4, NUM_AGENTS, a2);
                [pred_range, C] = predict_range(x_hat_a2, start_x1, start_x2);
                if abs(innovation_range_common) > delta_range
                    innovation = rel_range_meas - pred_range;
                    K = P_a2 * C' * inv(C * P_a2 * C' + w_perceived_sonar_range);
                    x_hat_a2 = x_hat_a2 + K * innovation;
                    P_a2 = P_a2 - K*C*P_a2;

                    num_explicit = num_explicit + 1;
                else
                    % x_bar, P_bar, x_hat, P, x_ref, C, R, delta, h_x_hat, h_x_bar, h_x_ref, angle_meas
                    disp("Fusing Range Implicitly");
                    [x_bar_a2, P_bar_a2] = get_estimate(x_bars, P_bars, 4, NUM_AGENTS, a2);
                    x_ref = x_common_bar;
                    R = w_perceived_sonar_range;
                    h_x_hat = pred_range;
                    [h_x_bar, C_] = predict_range(x_bar_a2, start_x1, start_x2);
                    [h_x_ref, C_] = predict_range(x_ref, start_x1, start_x2);
                    angle_meas = false;
                    [x_hat_a2, P_a2] = implicit_fuse(x_bar_a2, P_bar_a2, x_hat_a2, P_a2, x_ref, C, R, delta_range, h_x_hat, h_x_bar, h_x_ref, angle_meas);

                    num_implicit = num_implicit + 1;
                end

                [pred_azimuth, C] = predict_azimuth(x_hat_a2, start_x1, start_x2);
                if abs(innovation_azimuth_common) > delta_azimuth
                    innovation = normalize_angle( rel_azimuth_meas - pred_azimuth );
                    K = P_a2 * C' * inv(C * P_a2 * C' + w_perceived_sonar_azimuth);
                    x_hat_a2 = x_hat_a2 + K * innovation;
                    P_a2 = P_a2 - K*C*P_a2;

                    num_explicit = num_explicit + 1;
                else
                    % disp("Fusing Azimuth Implicitly");
                    [x_bar_a2, P_bar_a2] = get_estimate(x_bars, P_bars, 4, NUM_AGENTS, a2);
                    x_ref = x_common_bar;
                    R = w_perceived_sonar_azimuth;
                    h_x_hat = pred_azimuth;
                    [h_x_bar, C_] = predict_azimuth(x_bar_a2, start_x1, start_x2);
                    [h_x_ref, C_] = predict_azimuth(x_ref, start_x1, start_x2);
                    angle_meas = true;
                    [x_hat_a2, P_a2] = implicit_fuse(x_bar_a2, P_bar_a2, x_hat_a2, P_a2, x_ref, C, R, delta_range, h_x_hat, h_x_bar, h_x_ref, angle_meas);

                    num_implicit = num_implicit + 1;
                end

                [x_hats, Ps] = set_estimate(x_hats, Ps, 4, x_hat_a2, P_a2, a2);
            end
        end
    end
end