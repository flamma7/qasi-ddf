function [pred, C] = predict_azimuth(x_hat, start_x1, start_x2)

    x1 = x_hat(start_x1,1);
    y1 = x_hat(start_x1+1,1);
    x2 = x_hat(start_x2,1);
    y2 = x_hat(start_x2+1,1);
    delta_pred = [x2 - x1; y2 - y1];
    pred = atan2(delta_pred(2), delta_pred(1));

    dadx1 = (y2 - y1) / norm(delta_pred)^2;
    dadx2 = -(y2 - y1) / norm(delta_pred)^2;
    dady1 = -(x2 - x1) / norm(delta_pred)^2;
    dady2 = (x2 - x1) / norm(delta_pred)^2;
    C = zeros(1, size(x_hat,1));
    C(1, start_x1) = dadx1;
    C(1, start_x2) = dadx2;
    C(1, start_x1+1) = dady1;
    C(1, start_x2+1) = dady2;
end