function [x_hat, P, x_common, P_common, x_hats, Ps] = filter_sonar_et(x_hat, P, x_gt, w, w_perceived_sonar_range, w_perceived_sonar_azimuth, NUM_AGENTS, STATES, prob_detection, sonar_dist, agent, x_nav, x_common_bar, x_bars, P_bars, x_common, P_common, delta_range, delta_azimuth)

    % Determine if there is a sonar measurement
    % Update the regular estimate
    % Share measurements with other agents implicit/explicit

    x_hats = x_bars;
    Ps = P_bars;

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
            % [pred_range, C] = predict_range(x_common, start_x1, start_x2); % TODO should this be x_common_bar??
            % innovation_range = rel_range_meas - pred_range;
            % if abs(innovation_range) > delta_range % Explicit
            %     K = P_common * C' * inv(C * P_common * C' + w_perceived_sonar_range);
            %     x_common = x_common + K * innovation_range;
            %     P_common = P_common - K*C*P_common;
            % else % Implicit
            %     disp("Fusing range implicitly");
            %     [x_hat, P] = implicit_fuse(x_bar, P_bar, x_hat, P, x_ref, C, R, delta, h_x_hat, h_x_bar, h_x_ref)
                
            %     x1 = x_common(start_x1,1); 
            %     y1 = x_common(start_x1+1,1);
            %     x2 = x_common(start_x2,1);
            %     y2 = x_common(start_x2+1,1);
            %     delta_pred = [x2 - x1; y2 - y1];
            %     pred_range = norm(delta_pred);

            %     mu = pred_range - pred_range_bar;
            %     Qe = C * P_common_bar * C' + w_perceived_sonar_range;
            %     alpha = pred_range_bar - pred_range_bar; % Should always be zero

            %     Qf = @(x) 1 - normcdf(x);

            %     nu_minus = (-delta_range + alpha - mu) / sqrt(Qe);
            %     nu_plus = (delta_range + alpha - mu) / sqrt(Qe);
            %     tmp = (normpdf(nu_minus) - normpdf(nu_plus)) / (Qf(nu_minus) - Qf(nu_plus));
            %     z_bar = tmp * sqrt(Qe);
            %     tmp2 = (nu_minus * normpdf(nu_minus) - nu_plus*normpdf(nu_plus)) / (Qf(nu_minus) - Qf(nu_plus));
            %     curly_theta = tmp^2 - tmp2
            %     K = P_common * C' * inv( C * P_common * C' + w_perceived_sonar_range);
            %     x_common = x_common + K * z_bar;
            %     P_common = P_common - curly_theta * K * P_common;
            % end
            
            % % AZIMUTH MEASUREMENT
            % x1 = x_common_bar(start_x1,1); % let's use x_common_bar
            % y1 = x_common_bar(start_x1+1,1);
            % x2 = x_common_bar(start_x2,1);
            % y2 = x_common_bar(start_x2+1,1);
            % delta_pred = [x2 - x1; y2 - y1];
            % pred_azimuth_bar = atan2(delta_pred(2), delta_pred(1));

            % dadx1 = (y2 - y1) / norm(delta_pred)^2;
            % dadx2 = -(y2 - y1) / norm(delta_pred)^2;
            % dady1 = -(x2 - x1) / norm(delta_pred)^2;
            % dady2 = (x2 - x1) / norm(delta_pred)^2;
            % C = zeros(1, 4*NUM_AGENTS);
            % C(1, start_x1) = dadx1;
            % C(1, start_x2) = dadx2;
            % C(1, start_x1+1) = dady1;
            % C(1, start_x2+1) = dady2;
            
            % % RANGE COMMON ESTIMATE
            % innovation_azimuth = normalize_angle( rel_azimuth_meas - pred_azimuth_bar );

            % if abs(innovation_azimuth) > delta_azimuth % Explicit
            %     K = P_common * C' * inv(C * P_common * C' + w_perceived_sonar_azimuth);
            %     x_common = x_common + K * innovation_azimuth;
            %     P_common = P_common - K*C*P_common;
            % else % Implicit
            %     disp("Fusing azimuth implicitly");
            %     x1 = x_common(start_x1,1);
            %     y1 = x_common(start_x1+1,1);
            %     x2 = x_common(start_x2,1);
            %     y2 = x_common(start_x2+1,1);
            %     delta_pred = [x2 - x1; y2 - y1];
            %     pred_azimuth = atan2(delta_pred(2), delta_pred(1))

            %     mu = normalize_angle( pred_azimuth - pred_azimuth_bar );
            %     Qe = C * P_common_bar * C' + w_perceived_sonar_azimuth;
            %     alpha = pred_azimuth_bar - pred_azimuth_bar; % Should always be zero

            %     Qf = @(x) 1 - normcdf(x);

            %     nu_minus = normalize_angle(-delta_azimuth + alpha - mu) / sqrt(Qe);
            %     nu_plus = normalize_angle(delta_azimuth + alpha - mu) / sqrt(Qe);
            %     tmp = (normpdf(nu_minus) - normpdf(nu_plus)) / (Qf(nu_minus) - Qf(nu_plus));
            %     z_bar = tmp * sqrt(Qe);
            %     tmp2 = (nu_minus * normpdf(nu_minus) - nu_plus*normpdf(nu_plus)) / (Qf(nu_minus) - Qf(nu_plus));
            %     curly_theta = tmp^2 - tmp2
            %     K = P_common * C' * inv( C * P_common * C' + w_perceived_sonar_azimuth);
            %     x_common = x_common + K * z_bar;
            %     P_common = P_common - curly_theta * K * P_common;
            % end

            % % Fuse with local estimates at other agents
            % for a2 = 1:BLUE_NUM
            %     if a2 == agent
            %         continue
            %     end

            %     [x_bar, P_bar] = get_estimate(x_bars, P_bars, 4, NUM_AGENTS, a2);

            %     % RANGE
            %     x1 = x_bar(start_x1,1); % let's use x_common_bar
            %     y1 = x_bar(start_x1+1,1);
            %     x2 = x_bar(start_x2,1);
            %     y2 = x_bar(start_x2+1,1);
            %     delta_pred = [x2 - x1; y2 - y1];
            %     pred_range_bar = norm(delta_pred);
            %     drdx1 = (x1 - x2) / norm(delta_pred);
            %     drdx2 = (x2 - x1) / norm(delta_pred);
            %     drdy1 = (y1 - y2) / norm(delta_pred);
            %     drdy2 = (y2 - y1) / norm(delta_pred);
            %     C = zeros(1, 4*NUM_AGENTS);
            %     C(1, start_x1) = drdx1;
            %     C(1, start_x2) = drdx2;
            %     C(1, start_x1+1) = drdy1;
            %     C(1, start_x2+1) = drdy2;

            %     if abs(innovation_range) > delta_range % Explicit
            %         K = P_bar * C' * inv(C * P_bar * C' + w_perceived_sonar_range);
            %         x_common = x_common + K * innovation_range;
            %         P_bar = P_bar - K*C*P_bar;
            %     else % Implicit
            %         disp("Fusing range implicitly");
            %         x1 = x_common(start_x1,1); 
            %         y1 = x_common(start_x1+1,1);
            %         x2 = x_common(start_x2,1);
            %         y2 = x_common(start_x2+1,1);
            %         delta_pred = [x2 - x1; y2 - y1];
            %         pred_range = norm(delta_pred);
    
            %         mu = pred_range - pred_range_bar;
            %         Qe = C * P_bar_bar * C' + w_perceived_sonar_range;
            %         alpha = pred_range_bar - pred_range_bar; % Should always be zero
    
            %         Qf = @(x) 1 - normcdf(x);
    
            %         nu_minus = (-delta_range + alpha - mu) / sqrt(Qe);
            %         nu_plus = (delta_range + alpha - mu) / sqrt(Qe);
            %         tmp = (normpdf(nu_minus) - normpdf(nu_plus)) / (Qf(nu_minus) - Qf(nu_plus));
            %         z_bar = tmp * sqrt(Qe);
            %         tmp2 = (nu_minus * normpdf(nu_minus) - nu_plus*normpdf(nu_plus)) / (Qf(nu_minus) - Qf(nu_plus));
            %         curly_theta = tmp^2 - tmp2
            %         K = P_bar * C' * inv( C * P_bar * C' + w_perceived_sonar_range);
            %         x_common = x_common + K * z_bar;
            %         P_bar = P_bar - curly_theta * K * P_bar;
            %     end


            %     % AZIMUTH
            % end
        end
    end
end