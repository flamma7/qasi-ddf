function [x_common_debug, P_common_debug, mult, buffer] = pull_buffer( ...
                                                                    start_index, last_index, x_common, P_common, ledger, delta_range, delta_azimuth, ...
                                                                    max_num_meas, agent, q_perceived_tracking, w_perceived_modem_range, ...
                                                                    w_perceived_modem_azimuth, w_perceived_sonar_range, w_perceived_sonar_azimuth )

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
    lower_bound = 1;
    precision_bound = 0.5;
    mult_options = lower_bound : precision_bound : upper_bound;

    mult = mult_options(ceil(end/2)); % Pick middle of array

    agent_ledger = get_ledger(ledger, agent);
    x_common_start = x_common;
    P_common_start = P_common;

    while mult != upper_bound && mult != lower_bound

        x_common = x_common_start; % Use this variable for below calculations
        P_common = P_common_start;
        share_buffer = zeros(size(ledger)); % Needs to be of size ledger for add_meas()

        for index = start_index : last_index

            measurements = get_measurements(agent_ledger, index);

            %% PREDICTION
            [x_common, P_common] = propagate(x_common, P_common, NUM_AGENTS, q_perceived_tracking);

            %% CORRECTION
            for i = 1:size(measurements,1)

                meas = measurements(i,:);
                meas_type = meas(1,meas_type_col);
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

                    if abs(innovation) > delta_range * mult
                        K = P_common * C' * inv(C * P_common * C' + w_perceived_sonar_range);
                        x_common = x_common + K * innovation;
                        P_common = P_common - K*C*P_common;
                        share_buffer = add_meas(share_buffer, agent, "sonar_range", index, start_x1, start_x2, data);
                    else % Implicit

                    end
                    

                else % sonar azimuth
                    [pred, C] = predict_azimuth(x_common, start_x1, start_x2);
                    innovation = normalize_angle( data - pred );

                    if abs(innovation) > delta_azimuth * mult
                        K = P_common * C' * inv(C * P_common * C' + w_perceived_sonar_azimuth);
                        x_common = x_common + K * innovation;
                        P_common = P_common - K*C*P_common;
                        share_buffer = add_meas(share_buffer, agent, "sonar_azimuth", index, start_x1, start_x2, data);
                    else % Implicit

                    end

                end
                
                
                


            end

        end

    end