function [pred, C] = predict_azimuth_modem(x_hat, start_x1)

    x1 = x_hat(start_x1,1);
    y1 = x_hat(start_x1+1,1);
    blue_position = [x1, y1];
    pred = atan2(blue_position(2), blue_position(1));

    dadx = -y / norm(blue_position)^2;
    dady = x / norm(blue_position)^2;
    C = zeros(1, size(x_hat,1));
    C(1, start_x1) = dadx;
    C(1, start_x1+1) = dady;
end