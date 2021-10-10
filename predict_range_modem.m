function [pred, C] = predict_range_modem(x_hat, start_x1, modem_location)

    % x = x_hat(start_x1,1);
    % y = x_hat(start_x1+1,1);
    % blue_position = [x, y];
    % pred = norm(blue_position);

    % drdx = x / norm(blue_position);
    % drdy = y / norm(blue_position);

    % C = zeros(1, size(x_hat,1));
    % C(1, start_x1) = drdx;
    % C(1, start_x1+1) = drdy;

    x1 = x_hat(start_x1,1);
    y1 = x_hat(start_x1+1,1);
    delta_pred = [x1;y1] - modem_location;
    pred = norm(delta_pred);

    drdx1 = delta_pred(1) / norm(delta_pred);
    drdy1 = delta_pred(2) / norm(delta_pred);
    C = zeros(1, size(x_hat,1));
    C(1, start_x1) = drdx1;
    C(1, start_x1+1) = drdy1;

end