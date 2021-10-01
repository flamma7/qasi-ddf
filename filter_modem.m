function [x_hat, P] = filter_modem(x_hat, P, x_gt, w, w_perceived_modem_range,w_perceived_modem_azimuth, BLUE_NUM, STATES, TRACK_STATES)

    for b = 1:BLUE_NUM

        start_gt = STATES*(b-1)+1;

        blue_truth_pos = x_gt(start_gt:start_gt+1,1);
        range_meas = norm(blue_truth_pos) + normrnd(0, w, 1, 1); % Modem located at 0,0,0
        azimuth_meas = atan2(blue_truth_pos(2), blue_truth_pos(1))+ normrnd(0, w, 1, 1); % Facing 0 degrees

        startx = 4*(b-1)+1;
        blue_position = x_hat(startx:startx+1,1);
        x = blue_position(1);
        y = blue_position(2);

        pred_range_meas = norm(blue_position);
        pred_azimuth_meas = atan2(blue_position(2), blue_position(1));

        % Partial Derivatives of Range Measurement
        drdx = x / norm(blue_position);
        drdy = y / norm(blue_position);

        % Partial Derivatives of Azimuth Measurement
        dadx = -y / norm(blue_position)^2;
        dady = x / norm(blue_position)^2;

        % Construct azimuth measurement update vector
        C = zeros(1, TRACK_STATES);
        C(1, startx) = dadx;
        C(1, startx+1) = dady;

        innovation = normalize_angle( azimuth_meas - pred_azimuth_meas );
        if abs(innovation) > 1
            innovation
            % pause
        end

        K = P * C' * inv(C * P * C' + w_perceived_modem_azimuth);
        x_hat = x_hat + K * innovation;
        P = P - K*C*P;

        % Construct range measurement update vector
        C = zeros(1, TRACK_STATES);
        C(1, startx) = drdx;
        C(1, startx+1) = drdy;

        K = P * C' * inv(C * P * C' + w_perceived_modem_range);
        x_hat = x_hat + K * (range_meas - pred_range_meas);
        P = P - K*C*P;
    end
end