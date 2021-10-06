function [pred, C] = predict_range_modem(x_hat, start_x1)

    x1 = x_hat(start_x1,1);
    y1 = x_hat(start_x1+1,1);
    blue_position = [x1, y1];
    pred = norm(blue_position);

    drdx = x / norm(blue_position);
    drdy = y / norm(blue_position);

    C = zeros(1, size(x_hat,1));
    C(1, start_x1) = drdx;
    C(1, start_x1+1) = drdy;

end