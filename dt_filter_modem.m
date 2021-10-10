function [x_hat, P, ledger] = dt_filter_modem(x_hat, P, x_gt, w, w_perceived_modem_range,w_perceived_modem_azimuth, BLUE_NUM, STATES, TRACK_STATES, ledger, agent, index, MODEM_LOCATION)

    for b = 1:BLUE_NUM

        start_gt = STATES*(b-1)+1;

        blue_truth_pos = x_gt(start_gt:start_gt+1,1);
        delta = blue_truth_pos - MODEM_LOCATION;
        truth_range = norm(delta);
        range_meas = truth_range + normrnd(0, w, 1, 1); % Modem located at 0,0,0
        truth_azimuth = atan2(delta(2), delta(1));
        azimuth_meas = truth_azimuth + normrnd(0, w, 1, 1); % Facing 0 degrees

        startx = 4*(b-1)+1;

        % x_hat_prior = x_hat;
        [pred_azimuth, C_azimuth] = predict_azimuth_modem(x_hat, startx, MODEM_LOCATION);
        innovation = normalize_angle( azimuth_meas - pred_azimuth );
        K = P * C_azimuth' * inv(C_azimuth * P * C_azimuth' + w_perceived_modem_azimuth);
        x_hat = x_hat + K * innovation;
        P = P - K*C_azimuth*P;

        % z = zeros(4*BLUE_NUM, TRACK_STATES);
        % z(1:4*BLUE_NUM, 1:4*BLUE_NUM) = eye(4*BLUE_NUM);
        % delta = z * abs(x_hat - x_hat_prior);
        % if sum(delta > 10) > 0
        %     disp("Modem az")
        %     agent
        %     b
        %     x_hat_prior
        %     x_hat
        %     blue_truth_pos
        %     error = norm(blue_truth_pos - x_hat(startx:startx+1,1))
        %     truth_azimuth
        %     azimuth_meas
        %     pred_azimuth
        %     innovation
        %     C_azimuth
        %     K
        %     P
        %     % assert(0)
        % end
        
        % TODOREM
        % x_hat_prior = x_hat;
        [pred_range, C_range] = predict_range_modem(x_hat, startx, MODEM_LOCATION);
        innovation = range_meas - pred_range;
        K = P * C_range' * inv(C_range * P * C_range' + w_perceived_modem_range);
        x_hat = x_hat + K * innovation;
        P = P - K*C_range*P;


        % z = zeros(4*BLUE_NUM, TRACK_STATES);
        % z(1:4*BLUE_NUM, 1:4*BLUE_NUM) = eye(4*BLUE_NUM);
        % delta = z * abs(x_hat - x_hat_prior);
        % if sum(delta > 10) > 0
        %     disp("Modem range")
        %     agent
        %     b
        %     blue_truth_pos
        %     x_hat_prior
        %     x_hat
        %     range_meas
        %     pred_range
        %     K
        %     P
        %     % assert(0)
        % end

        % [pred_range, C_range] = predict_range_modem(x_hat, startx);
        % [pred_azimuth, C_azimuth] = predict_azimuth_modem(x_hat, startx);
        % meas = [pred_range; pred_azimuth];
        % innovation = [range_meas; azimuth_meas] - meas;
        % innovation(2) = normalize_angle(innovation(2));
        % R = diag([w_perceived_modem_range, w_perceived_modem_azimuth]);
        % C = [C_range; C_azimuth];
        % K = P * C' * inv(C * P * C' + R);
        % x_hat = x_hat + K * innovation;
        % P = P - K*C*P;

        % Add meas to ledger
        ledger = add_meas(ledger, agent, "modem_azimuth", index, startx, 0, azimuth_meas);
        ledger = add_meas(ledger, agent, "modem_range", index, startx, 0, range_meas);
    end
end