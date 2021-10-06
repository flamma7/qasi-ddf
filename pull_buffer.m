function [x_common_debug, P_common_debug, mult, share_buffer, explicit_cnt, implicit_cnt] = pull_buffer( ...
                                                                    start_index, last_index, x_common, P_common, ledger, delta_range, delta_azimuth, ...
                                                                    max_num_meas, agent, q_perceived_tracking, w_perceived_modem_range, ...
                                                                    w_perceived_modem_azimuth, w_perceived_sonar_range, w_perceived_sonar_azimuth, ...
                                                                    NUM_AGENTS )

    % Use bisection search to select the optimal delta-multiplier for sharing measurements

    % Start index should be the last time an agent shared
    % last index should be the current loop number
    meas_types = ["modem_range", "modem_azimuth", "sonar_range", "sonar_azimuth", "sonar_range_implicit", "sonar_azimuth_implicit"];
    meas_columns = ["type", "index", "start_x1", "start_x2", "data"];
    meas_type_col = find(meas_columns == "type");
    index_col = find(meas_columns == "index");
    startx1_col = find(meas_columns == "start_x1");
    startx2_col = find(meas_columns == "start_x2");
    data_col = find(meas_columns == "data");

    upper_bound = 10; % Consider using an exponential bound ie 1,2,5,10,20,40,80...
    lower_bound = 0;
    precision_bound = 0.5;
    mult_options = lower_bound : precision_bound : upper_bound;

    mult = mult_options(ceil(end/2)); % Pick middle of array

    agent_ledger = get_ledger(ledger, agent);
    x_common_start = x_common;
    P_common_start = P_common;

    while true % Break out at the bottom

        x_common = x_common_start; % Use this variable for below calculations
        P_common = P_common_start;
        share_buffer = zeros(size(ledger)); % Needs to be of size ledger for add_meas()

        for index = start_index : last_index

            measurements = get_measurements(agent_ledger, index);

            %% PREDICTION
            [x_common, P_common] = propagate(x_common, P_common, NUM_AGENTS, q_perceived_tracking);
            x_common_bar = x_common;
            P_common_bar = P_common;

            %% CORRECTION
            for i = 1:size(measurements,1)

                meas = measurements(i,:);
                meas_type = meas_types( meas(1,meas_type_col) );
                start_x1 = meas(1,startx1_col);
                start_x2 = meas(1,startx2_col);
                data = meas(data_col);
                
                if meas_type == "modem_range" % ALWAYS SHARE EXPLICITLY (it's already been shared)
                    [pred, C] = predict_range_modem(x_common, start_x1);
                    innovation = data - pred;
                    K = P_common * C' * inv(C * P_common * C' + w_perceived_modem_range);
                    x_common = x_common + K * innovation;
                    P_common = P_common - K*C*P_common;
                    share_buffer = add_meas(share_buffer, agent, "modem_range", index, start_x1, start_x2, data);

                elseif meas_type == "modem_azimuth" % ALWAYS SHARE EXPLICITLY (it's already been shared)
                    [pred, C] = predict_azimuth_modem(x_common, start_x1);
                    innovation = normalize_angle( data - pred );
                    K = P_common * C' * inv(C * P_common * C' + w_perceived_modem_azimuth);
                    x_common = x_common + K * innovation;
                    P_common = P_common - K*C*P_common;
                    share_buffer = add_meas(share_buffer, agent, "modem_azimuth", index, start_x1, start_x2, data);

                elseif meas_type == "sonar_range"
                    [pred, C] = predict_range(x_common, start_x1, start_x2);
                    innovation = data - pred;
                    delta = delta_range * mult;

                    if abs(innovation) > delta
                        K = P_common * C' * inv(C * P_common * C' + w_perceived_sonar_range);
                        x_common = x_common + K * innovation;
                        P_common = P_common - K*C*P_common;
                        share_buffer = add_meas(share_buffer, agent, "sonar_range", index, start_x1, start_x2, data);
                    else % Implicit
                        x_ref = x_common; % OR x_common_bar
                        h_x_hat = pred;
                        [h_x_bar, C_] = predict_range(x_common_bar, start_x1, start_x2);
                        [h_x_ref, C_] = predict_range(x_ref, start_x1, start_x2);
                        [x_common, P_common] = implicit_fuse(x_common_bar, P_common_bar, x_common, P_common, x_ref, C, w_perceived_sonar_range, delta, h_x_hat, h_x_bar, h_x_ref, false);
                        rank(P_common);
                        share_buffer = add_meas(share_buffer, agent, "sonar_range_implicit", index, start_x1, start_x2, 0.0);
                    end

                elseif meas_type == "sonar_azimuth"
                    [pred, C] = predict_azimuth(x_common, start_x1, start_x2);
                    innovation = normalize_angle( data - pred );
                    delta = delta_azimuth * mult;

                    if abs(innovation) > delta
                        K = P_common * C' * inv(C * P_common * C' + w_perceived_sonar_azimuth);
                        x_common = x_common + K * innovation;
                        P_common = P_common - K*C*P_common;
                        share_buffer = add_meas(share_buffer, agent, "sonar_azimuth", index, start_x1, start_x2, data);
                    else % Implicit
                        x_ref = x_common; % OR x_common_bar
                        h_x_hat = pred;
                        [h_x_bar, C_] = predict_azimuth(x_common_bar, start_x1, start_x2);
                        [h_x_ref, C_] = predict_azimuth(x_ref, start_x1, start_x2);
                        [x_common, P_common] = implicit_fuse(x_common_bar, P_common_bar, x_common, P_common, x_ref, C, w_perceived_sonar_azimuth, delta, h_x_hat, h_x_bar, h_x_ref, true);
                        rank(P_common);
                        share_buffer = add_meas(share_buffer, agent, "sonar_azimuth_implicit", index, start_x1, start_x2, 0.0);
                    end
                else
                    disp("Unrecognized measurement type: " + meas_type);
                    assert(0);
                end
            end % measurements
        end % indices

        share_buffer = get_ledger(share_buffer, agent);

        % Bisection search next multiple
        ind = find(mult_options == mult);
        [explicit_cnt, implicit_cnt] = get_buffer_size(share_buffer);

        if explicit_cnt > max_num_meas
            if length(mult_options) == 1 % there were no matches
                disp("Buffer upper bound was too low. No deltabands consistent");
                assert(0);
            end
            mult_options = mult_options(ind+1:end);
        else
            mult_options = mult_options(1:ind);
        end
        mult_prior = mult;
        mult = mult_options(ceil(end/2)); % Pick middle of array

        % we've found the optimal deltaband
        if mult == mult_prior
            break 
        end
    end % while
    x_common_debug = x_common;
    P_common_debug = P_common;
end % fxn