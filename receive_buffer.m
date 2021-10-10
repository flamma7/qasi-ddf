function [x_hat, P, x_common_debug, P_common_debug] = receive_buffer( ...
                            last_x_hat, last_P, start_index, last_index, share_buffer, mult, ledger, ...
                            x_common, P_common, agent, NUM_AGENTS, q_perceived_tracking, ...
                            delta_range, delta_azimuth, STATES, x_nav_history, P_nav_history, ...
                            w_perceived_modem_range, w_perceived_modem_azimuth, w_perceived_sonar_range, ...
                            w_perceived_sonar_azimuth, MODEM_LOCATION)

    meas_types = ["modem_range", "modem_azimuth", "sonar_range", "sonar_azimuth", "sonar_range_implicit", "sonar_azimuth_implicit"];
    meas_columns = ["type", "index", "start_x1", "start_x2", "data"];
    meas_type_col = find(meas_columns == "type");
    index_col = find(meas_columns == "index");
    startx1_col = find(meas_columns == "start_x1");
    startx2_col = find(meas_columns == "start_x2");
    data_col = find(meas_columns == "data");

    x_hat = last_x_hat;
    P = last_P;
    agent_ledger = get_ledger(ledger, agent);

    % disp("Agent " + int2str(agent) + " receiving buffer from " + int2str(start_index) + " to " + int2str(last_index));
    for index = start_index : last_index
        % PREDICTION COMMON
        [x_common, P_common] = propagate(x_common, P_common, NUM_AGENTS, q_perceived_tracking);
        x_common_bar = x_common;
        P_common_bar = P_common;
        % PREDICTION MAIN ESTIMATE
        [x_hat, P] = propagate(x_hat, P, NUM_AGENTS, q_perceived_tracking);
        x_hat_bar = x_hat;
        P_bar = P;
        rank(P);

        % CORRECTION SHARED ESTIMATES
        shared_measurements = get_measurements(share_buffer, index);
        for i = 1:size(shared_measurements,1)

            meas = shared_measurements(i,:);
            meas_type = meas_types( meas(1,meas_type_col) );
            start_x1 = meas(1,startx1_col);
            start_x2 = meas(1,startx2_col);
            data = meas(data_col);
            
            if meas_type == "modem_range"
                [pred, C] = predict_range_modem(x_common, start_x1, MODEM_LOCATION);
                innovation = data - pred;
                K = P_common * C' * inv(C * P_common * C' + w_perceived_modem_range);
                x_common = x_common + K * innovation;
                P_common = P_common - K*C*P_common;

                % DON'T FUSE WITH MAIN, ITS LEDGER ALREADY HAS THIS MEAS

            elseif meas_type == "modem_azimuth" % ALWAYS SHARE EXPLICITLY (it's already been shared)
                [pred, C] = predict_azimuth_modem(x_common, start_x1, MODEM_LOCATION);
                innovation = normalize_angle( data - pred );
                K = P_common * C' * inv(C * P_common * C' + w_perceived_modem_azimuth);
                x_common = x_common + K * innovation;
                P_common = P_common - K*C*P_common;

                % DON'T FUSE WITH MAIN, ITS LEDGER ALREADY HAS THIS MEAS

            elseif meas_type == "sonar_range"
                % COMMON
                [pred, C] = predict_range(x_common, start_x1, start_x2);
                innovation = data - pred;
                K = P_common * C' * inv(C * P_common * C' + w_perceived_sonar_range);
                x_common = x_common + K * innovation;
                P_common = P_common - K*C*P_common;

                % MAIN
                [pred, C] = predict_range(x_hat, start_x1, start_x2);
                innovation = data - pred;
                K = P * C' * inv(C * P * C' + w_perceived_sonar_range);
                x_hat = x_hat + K * innovation;
                P = P - K*C*P;

            elseif meas_type == "sonar_range_implicit"
                delta = delta_range * mult;
                % COMMON
                [pred, C] = predict_range(x_common, start_x1, start_x2);
                x_ref = x_common; % OR x_common_bar
                h_x_hat = pred;
                [h_x_bar, C_] = predict_range(x_common_bar, start_x1, start_x2);
                [h_x_ref, C_] = predict_range(x_ref, start_x1, start_x2);
                [x_common, P_common] = implicit_fuse(x_common_bar, P_common_bar, x_common, P_common, x_ref, C, w_perceived_sonar_range, delta, h_x_hat, h_x_bar, h_x_ref, false);
                rank(P_common);

                % MAIN
                [pred, C] = predict_range(x_hat, start_x1, start_x2);
                x_ref = x_common;
                R = w_perceived_sonar_range;
                h_x_hat = pred;
                [h_x_bar, C_] = predict_range(x_hat_bar, start_x1, start_x2);
                [h_x_ref, C_] = predict_range(x_ref, start_x1, start_x2);
                angle_meas = false;

                % delta
                % x_hat_bar
                % P_bar
                % x_hat
                % P
                % x_ref
                % C
                % R
                % h_x_hat
                % h_x_bar
                % h_x_ref

                [x_hat, P] = implicit_fuse(x_hat_bar, P_bar, x_hat, P, x_ref, C, R, delta, h_x_hat, h_x_bar, h_x_ref, angle_meas);
                rank(P);

            elseif meas_type == "sonar_azimuth"
                % COMMON                
                [pred, C] = predict_azimuth(x_common, start_x1, start_x2);
                innovation = normalize_angle( data - pred );
                K = P_common * C' * inv(C * P_common * C' + w_perceived_sonar_azimuth);
                x_common = x_common + K * innovation;
                P_common = P_common - K*C*P_common;

                % MAIN
                [pred, C] = predict_azimuth(x_hat, start_x1, start_x2);
                innovation = normalize_angle( data - pred );
                K = P * C' * inv(C * P * C' + w_perceived_sonar_azimuth);
                x_hat = x_hat + K * innovation;
                P = P - K*C*P;

            elseif meas_type == "sonar_azimuth_implicit"
                delta = delta_azimuth * mult;
                % COMMON
                [pred, C] = predict_azimuth(x_common, start_x1, start_x2);
                x_ref = x_common; % OR x_common_bar
                h_x_hat = pred;
                [h_x_bar, C_] = predict_azimuth(x_common_bar, start_x1, start_x2);
                [h_x_ref, C_] = predict_azimuth(x_ref, start_x1, start_x2);
                [x_common, P_common] = implicit_fuse(x_common_bar, P_common_bar, x_common, P_common, x_ref, C, w_perceived_sonar_range, delta, h_x_hat, h_x_bar, h_x_ref, true);
                rank(P_common);

                % MAIN
                [pred, C] = predict_azimuth(x_hat, start_x1, start_x2);
                x_ref = x_common;
                R = w_perceived_sonar_azimuth;
                h_x_hat = pred;
                [h_x_bar, C_] = predict_azimuth(x_hat_bar, start_x1, start_x2);
                [h_x_ref, C_] = predict_azimuth(x_ref, start_x1, start_x2);
                angle_meas = true;
                [x_hat, P] = implicit_fuse(x_hat_bar, P_bar, x_hat, P, x_ref, C, R, delta, h_x_hat, h_x_bar, h_x_ref, angle_meas);
                rank(P);
            else
                disp("Unrecognized measurement type: " + meas_type);
                assert(0);
            end
        end % measurements

        % CORRECTION OWN MEASUREMENTS (ALL EXPLICIT)
        measurements = get_measurements(agent_ledger, index);
        for i = 1:size(measurements,1)

            meas = measurements(i,:);
            meas_type = meas_types( meas(1,meas_type_col) );
            start_x1 = meas(1,startx1_col);
            start_x2 = meas(1,startx2_col);
            data = meas(data_col);

            if meas_type == "modem_range"
                [pred, C] = predict_range_modem(x_hat, start_x1, MODEM_LOCATION);
                innovation = data - pred;
                K = P * C' * inv(C * P * C' + w_perceived_modem_range);
                x_hat = x_hat + K * innovation;
                P = P - K*C*P;

            elseif meas_type == "modem_azimuth" % ALWAYS SHARE EXPLICITLY (it's already been shared)
                [pred, C] = predict_azimuth_modem(x_hat, start_x1, MODEM_LOCATION);
                innovation = normalize_angle( data - pred );
                K = P * C' * inv(C * P * C' + w_perceived_modem_azimuth);
                x_hat = x_hat + K * innovation;
                P = P - K*C*P;

            elseif meas_type == "sonar_range"
                % MAIN
                [pred, C] = predict_range(x_hat, start_x1, start_x2);
                innovation = data - pred;
                K = P * C' * inv(C * P * C' + w_perceived_sonar_range);
                x_hat = x_hat + K * innovation;
                P = P - K*C*P;

            elseif meas_type == "sonar_azimuth"
                % MAIN
                [pred, C] = predict_azimuth(x_hat, start_x1, start_x2);
                innovation = normalize_angle( data - pred );
                K = P * C' * inv(C * P * C' + w_perceived_sonar_azimuth);
                x_hat = x_hat + K * innovation;
                P = P - K*C*P;
            else
                disp("Unrecognized measurement type: " + meas_type);
                assert(0);
            end
        end % measurements

        [x_nav, P_nav] = get_estimate_nav_index(x_nav_history, P_nav_history, STATES, index);
        if sum(x_nav == 0) ~= length(x_nav)
            % FUSE NAVIGATION AND MAIN ESTIMATES
            [x_nav_, P_nav_, x_hat, P] = intersect_estimates(x_nav, P_nav, x_hat, P, agent, STATES); % TODO add
        end
    end 
    x_common_debug = x_common;
    P_common_debug = P_common;
end                            