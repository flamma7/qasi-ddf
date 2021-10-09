function [x_hat, P, ledger] = dt_filter_modem(x_hat, P, x_gt, w, w_perceived_modem_range,w_perceived_modem_azimuth, BLUE_NUM, STATES, TRACK_STATES, ledger, agent, index)

    for b = 1:BLUE_NUM

        start_gt = STATES*(b-1)+1;

        blue_truth_pos = x_gt(start_gt:start_gt+1,1);
        range_meas = norm(blue_truth_pos) + normrnd(0, w, 1, 1); % Modem located at 0,0,0
        azimuth_meas = atan2(blue_truth_pos(2), blue_truth_pos(1))+ normrnd(0, w, 1, 1); % Facing 0 degrees

        startx = 4*(b-1)+1;
        
        [pred_range, C_range] = predict_range_modem(x_hat, startx);
        innovation = range_meas - pred_range;
        K = P * C_range' * inv(C_range * P * C_range' + w_perceived_modem_range);
        x_hat = x_hat + K * innovation;
        P = P - K*C_range*P;

        [pred_azimuth, C_azimuth] = predict_azimuth_modem(x_hat, startx);
        innovation = azimuth_meas - pred_azimuth;
        K = P * C_azimuth' * inv(C_azimuth * P * C_azimuth' + w_perceived_modem_azimuth);
        x_hat = x_hat + K * innovation;
        P = P - K*C_azimuth*P;

        % meas = [pred_range; pred_azimuth];
        % innovation = [range_meas; azimuth_meas] - meas;
        % innovation(2) = normalize_angle(innovation(2));
        % R = diag([w_perceived_modem_range, w_perceived_modem_azimuth]);
        % C = [C_range; C_azimuth];
        % K = P * C' * inv(C * P * C' + R);
        % x_hat = x_hat + K * innovation;
        % P = P - K*C*P;

        % Add meas to ledger
        ledger = add_meas(ledger, agent, "modem_range", index, startx, 0, range_meas);
        ledger = add_meas(ledger, agent, "modem_azimuth", index, startx, 0, azimuth_meas);
    end
end