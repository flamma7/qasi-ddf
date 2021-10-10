function [pred, C] = predict_azimuth_modem(x_hat, start_x1, modem_location)

    % x = x_hat(start_x1,1);
    % y = x_hat(start_x1+1,1);
    % blue_position = [x, y];
    % pred = atan2(blue_position(2), blue_position(1));

    % dadx = -y / norm(blue_position)^2;
    % dady = x / norm(blue_position)^2;
    % C = zeros(1, size(x_hat,1));
    % C(1, start_x1) = dadx;
    % C(1, start_x1+1) = dady;

    x = x_hat(start_x1,1);
    y = x_hat(start_x1+1,1);
    delta_pred = [x;y] - modem_location;
    pred = atan2(delta_pred(2), delta_pred(1));

    dadx = -delta_pred(2) / norm(delta_pred)^2;
    dady = delta_pred(1) / norm(delta_pred)^2;
    C = zeros(1, size(x_hat,1));
    C(1, start_x1) = dadx;
    C(1, start_x1+1) = dady;
end