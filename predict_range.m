function [pred, C] = predict_range(x_hat, start_x1, start_x2)

    x1 = x_hat(start_x1,1);
    y1 = x_hat(start_x1+1,1);
    x2 = x_hat(start_x2,1);
    y2 = x_hat(start_x2+1,1);
    delta_pred = [x2 - x1; y2 - y1];
    pred = norm(delta_pred);

    drdx1 = (x1 - x2) / norm(delta_pred);
    drdx2 = (x2 - x1) / norm(delta_pred);
    drdy1 = (y1 - y2) / norm(delta_pred);
    drdy2 = (y2 - y1) / norm(delta_pred);
    C = zeros(1, size(x_hat,1));
    C(1, start_x1) = drdx1;
    C(1, start_x2) = drdx2;
    C(1, start_x1+1) = drdy1;
    C(1, start_x2+1) = drdy2;

end