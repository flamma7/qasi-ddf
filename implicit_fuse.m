function [x_hat, P] = implicit_fuse(x_bar, P_bar, x_hat, P, x_ref, C, R, delta, h_x_hat, h_x_bar, h_x_ref, angle_meas)

    mu = h_x_hat - h_x_bar;
    Qe = C * P_bar * C' + R;
    alpha = h_x_ref - h_x_bar;

    Qf = @(x) 1 - normcdf(x);

    if angle_meas
        nu_minus = normalize_angle(-delta + alpha - mu) / sqrt(Qe);
        nu_plus = normalize_angle(delta + alpha - mu) / sqrt(Qe);
    else
        nu_minus = (-delta + alpha - mu) / sqrt(Qe);
        nu_plus = (delta + alpha - mu) / sqrt(Qe);
    end
    tmp = (normpdf(nu_minus) - normpdf(nu_plus)) / (Qf(nu_minus) - Qf(nu_plus));
    z_bar = tmp * sqrt(Qe);
    tmp2 = (nu_minus * normpdf(nu_minus) - nu_plus*normpdf(nu_plus)) / (Qf(nu_minus) - Qf(nu_plus));
    curly_theta = tmp^2 - tmp2;
    K = P * C' * inv( C * P * C' + R);
    x_hat = x_hat + K * z_bar;
    P = P - curly_theta * K * C * P;
end